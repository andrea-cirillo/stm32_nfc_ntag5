/**
 ******************************************************************************
 * @file    ntag5.h
 * @brief   I2C host driver for the NXP NTAG 5 (NTP5210 / NTP53x2 / NTA53x2).
 *
 *          References: NXP NTP53x2 datasheet Rev. 3.3 (sect. 8.1.3.13, 8.1.4)
 *                      NXP AN12364 "NTAG 5 - pass-through mode" (sect. 6)
 *
 *          I2C-side memory map (BLOCK addresses, 1 block = 4 bytes):
 *            0000h..01FEh  user EEPROM
 *            1000h..109Fh  configuration memory (persistent, EEPROM)
 *            10A0h..10AFh  session registers (volatile, READ/WRITE REGISTER)
 *            2000h..203Fh  256-byte SRAM (only with VCC and SRAM_ENABLE = 1)
 *
 *          Thread safety: every API is protected by a mutex and must be
 *          called from a FreeRTOS task (never from an ISR).
 ******************************************************************************
 */
#ifndef NTAG5_H
#define NTAG5_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32wbxx_hal.h"
#include <stdint.h>
#include <stdbool.h>

/* ------------------------------------------------------------------------- */
/* I2C address                                                                */
/* ------------------------------------------------------------------------- */
#define NTAG5_I2C_ADDR_7BIT          0x54U          /* default, block 3Eh   */
#define NTAG5_I2C_ADDR_HAL           (NTAG5_I2C_ADDR_7BIT << 1)

/* ------------------------------------------------------------------------- */
/* Memory map (I2C-side block addresses)                                      */
/* ------------------------------------------------------------------------- */
#define NTAG5_BLOCK_SIZE             4U

#define NTAG5_USER_MEM_START         0x0000U
#define NTAG5_CFG_MEM_START          0x1000U
#define NTAG5_SRAM_START             0x2000U
#define NTAG5_SRAM_BLOCKS            64U
#define NTAG5_SRAM_SIZE              (NTAG5_SRAM_BLOCKS * NTAG5_BLOCK_SIZE) /* 256 */
#define NTAG5_SRAM_LAST_BLOCK        (NTAG5_SRAM_START + NTAG5_SRAM_BLOCKS - 1U)

/* Persistent configuration (EEPROM) - copied into the session registers at POR */
#define NTAG5_CFG_CONFIG             0x1037U  /* CONFIG_0, CONFIG_1, CONFIG_2, RFU   */
#define NTAG5_CFG_SYNCH_DATA_BLOCK   0x1038U
#define NTAG5_CFG_EH_ED_CONFIG       0x103DU  /* byte0 EH_CONFIG, byte2 ED_CONFIG    */
#define NTAG5_CFG_I2C_SLAVE_ADDR     0x103EU

/* Session registers (volatile, immediate effect) */
#define NTAG5_REG_STATUS             0x10A0U  /* byte0 STATUS0, byte1 STATUS1         */
#define NTAG5_REG_CONFIG             0x10A1U  /* byte0 CONFIG_0, byte1 CONFIG_1, b2 CONFIG_2 */
#define NTAG5_REG_SYNCH_DATA_BLOCK   0x10A2U
#define NTAG5_REG_PWM_GPIO_CONFIG    0x10A3U
#define NTAG5_REG_WDT_CONFIG         0x10A6U
#define NTAG5_REG_EH_CONFIG          0x10A7U
#define NTAG5_REG_ED_CONFIG          0x10A8U  /* byte0 bit3..0 = ED event             */
#define NTAG5_REG_I2C_SLAVE_CONFIG   0x10A9U
#define NTAG5_REG_RESET_GEN          0x10AAU
/* NB: the NTP53x2 datasheet Rev 3.3 gives 10ABh, AN12364 gives 10ACh
 *     (which in the datasheet is I2C_M_S_ADD_REG). The datasheet is the
 *     default; the ED self-test selects the one that actually works. */
#define NTAG5_REG_ED_INTR_CLEAR      0x10ABU

/* ---- STATUS0 (STATUS_REG byte 0) ---- */
#define NTAG5_STATUS0_EEPROM_WR_BUSY     (1U << 7)
#define NTAG5_STATUS0_EEPROM_WR_ERROR    (1U << 6)
#define NTAG5_STATUS0_SRAM_DATA_READY    (1U << 5)
#define NTAG5_STATUS0_SYNCH_BLOCK_WRITE  (1U << 4)
#define NTAG5_STATUS0_SYNCH_BLOCK_READ   (1U << 3)
#define NTAG5_STATUS0_PT_TRANSFER_DIR    (1U << 2)  /* 1 = NFC -> I2C */
#define NTAG5_STATUS0_VCC_SUPPLY_OK      (1U << 1)
#define NTAG5_STATUS0_NFC_FIELD_OK       (1U << 0)

/* ---- STATUS1 (STATUS_REG byte 1) ---- */
#define NTAG5_STATUS1_VCC_BOOT_OK        (1U << 7)
#define NTAG5_STATUS1_NFC_BOOT_OK        (1U << 6)

/* ---- CONFIG_0 ---- */
#define NTAG5_CONFIG0_SRAM_COPY_EN       (1U << 7)
#define NTAG5_CONFIG0_LOCK_SESSION_REG   (1U << 1)
#define NTAG5_CONFIG0_AUTO_STANDBY_EN    (1U << 0)

/* ---- CONFIG_1 ---- */
#define NTAG5_CONFIG1_EH_ARBITER_MODE_EN (1U << 7)
#define NTAG5_CONFIG1_USE_CASE_MASK      (3U << 4)  /* 00b = I2C slave        */
#define NTAG5_CONFIG1_ARBITER_MASK       (3U << 2)
#define NTAG5_CONFIG1_ARBITER_NORMAL     (0U << 2)
#define NTAG5_CONFIG1_ARBITER_MIRROR     (1U << 2)
#define NTAG5_CONFIG1_ARBITER_PASSTHRU   (2U << 2)
#define NTAG5_CONFIG1_ARBITER_PHDC       (3U << 2)
#define NTAG5_CONFIG1_SRAM_ENABLE        (1U << 1)
#define NTAG5_CONFIG1_PT_TRANSFER_DIR    (1U << 0)  /* 1 = NFC -> I2C */

/* RF -> I2C pass-through configuration: ARBITER=10b, SRAM_ENABLE=1, DIR=1 */
#define NTAG5_CONFIG1_PT_MASK   (NTAG5_CONFIG1_ARBITER_MASK | NTAG5_CONFIG1_SRAM_ENABLE | \
                                 NTAG5_CONFIG1_PT_TRANSFER_DIR)
#define NTAG5_CONFIG1_PT_NFC2I2C (NTAG5_CONFIG1_ARBITER_PASSTHRU | NTAG5_CONFIG1_SRAM_ENABLE | \
                                  NTAG5_CONFIG1_PT_TRANSFER_DIR)          /* = 0x0B */

/* Normal mode: "normal" arbiter, SRAM enabled, EEPROM accessible from I2C */
#define NTAG5_CONFIG1_NORMAL    (NTAG5_CONFIG1_ARBITER_NORMAL | NTAG5_CONFIG1_SRAM_ENABLE | \
                                 NTAG5_CONFIG1_PT_TRANSFER_DIR)           /* = 0x03 */

typedef enum {
  NTAG5_MODE_NORMAL = 0,      /* EEPROM readable/writable from I2C             */
  NTAG5_MODE_PT_NFC_TO_I2C,   /* SRAM pass-through, the arbiter serves RF side */
} ntag5_mode_t;

/* ---- ED_CONFIG (bit 3..0) ---- */
#define NTAG5_ED_MASK                    0x0FU
typedef enum {
  NTAG5_ED_DISABLED          = 0x0,
  NTAG5_ED_NFC_FIELD_DETECT  = 0x1,
  NTAG5_ED_PWM               = 0x2,
  NTAG5_ED_PT_I2C_TO_NFC     = 0x3,
  NTAG5_ED_PT_NFC_TO_I2C     = 0x4,  /* ED ON when NFC writes the last SRAM byte */
  NTAG5_ED_ARBITER_LOCK      = 0x5,
  NTAG5_ED_NDEF_TLV_LENGTH   = 0x6,
  NTAG5_ED_STANDBY           = 0x7,
  NTAG5_ED_WRITE_CMD         = 0x8,
  NTAG5_ED_READ_CMD          = 0x9,
  NTAG5_ED_START_OF_CMD      = 0xA,  /* ED ON at the start of every RF command   */
  NTAG5_ED_SYNCH_BLOCK_READ  = 0xB,
  NTAG5_ED_SYNCH_BLOCK_WRITE = 0xC,
  NTAG5_ED_SW_INTERRUPT      = 0xD,  /* ED ON immediately, until ED_INTR_CLEAR    */
} ntag5_ed_event_t;

/* ------------------------------------------------------------------------- */
/* API                                                                        */
/* ------------------------------------------------------------------------- */
typedef enum {
  NTAG5_OK = 0,
  NTAG5_ERR_PARAM,
  NTAG5_ERR_NACK,      /* no answer, or area locked (e.g. arbiter on the NFC side) */
  NTAG5_ERR_BUS,       /* bus error / arbitration lost / overrun                    */
  NTAG5_ERR_TIMEOUT,
  NTAG5_ERR_VERIFY,    /* read-back differs from what was written                   */
} ntag5_status_t;

typedef struct {
  uint8_t status0;
  uint8_t status1;
} ntag5_status_reg_t;

/* Bus diagnostics (cumulative counters + last HAL error) */
#define NTAG5_DIAG_START_FAIL   0x80000000UL  /* top bit: HAL_xxx_IT/DMA did not start */
typedef struct {
  uint32_t last_hal_err;     /* HAL_I2C_ERROR_xxx (BERR=1 ARLO=2 AF=4 OVR=8 DMA=0x10 TIMEOUT=0x20 SIZE=0x40) */
  uint32_t last_hal_state;   /* HAL_I2C_StateTypeDef when the error occurred */
  uint32_t bus_errors;
  uint32_t timeouts;
  uint32_t dma_ok;           /* chunks read successfully with DMA          */
  uint32_t dma_fallback_it;  /* chunks failed with DMA, recovered with IT  */
} ntag5_diag_t;

/** Initializes the driver (mutex, semaphores). Call before the scheduler starts. */
void ntag5_init(I2C_HandleTypeDef *hi2c);

/** Checks that the NTAG 5 acknowledges its I2C address. */
ntag5_status_t ntag5_probe(void);

/** Memory read (user EEPROM, configuration or SRAM). len must be a multiple
 *  of 4. From 16 bytes upward the transfer uses DMA (I2C1_RX, DMA1_Channel1). */
ntag5_status_t ntag5_read_blocks(uint16_t block_addr, uint8_t *buf, uint16_t len);

/** Writes ONE block (4 bytes) of EEPROM/configuration/SRAM.
 *  For EEPROM it waits for the end of programming. */
ntag5_status_t ntag5_write_block(uint16_t block_addr, const uint8_t data[NTAG5_BLOCK_SIZE]);

/** READ REGISTER: reads byte `reg_byte` of session register `reg_addr`. */
ntag5_status_t ntag5_read_reg(uint16_t reg_addr, uint8_t reg_byte, uint8_t *value);

/** WRITE REGISTER: updates only the `mask` bits of byte `reg_byte`. */
ntag5_status_t ntag5_write_reg(uint16_t reg_addr, uint8_t reg_byte, uint8_t mask, uint8_t value);

/** Reads STATUS0 and STATUS1. */
ntag5_status_t ntag5_read_status(ntag5_status_reg_t *st);

/** Sets the arbiter mode and the ED event (session registers), with read-back.
 *  NB: in NFC -> I2C pass-through the arbiter gives the interface to the RF
 *  side until it writes the SRAM: I2C accesses to the EEPROM get NACKed.
 *  Read the EEPROM only in NTAG5_MODE_NORMAL. */
ntag5_status_t ntag5_set_mode(ntag5_mode_t mode, ntag5_ed_event_t ed_event);

/** Changes the event signalled on the ED pin (ED_CONFIG_REG), with read-back. */
ntag5_status_t ntag5_set_ed_event(ntag5_ed_event_t ed_event);

/** true if CONFIG_1 matches the given mode. */
bool ntag5_mode_is(ntag5_mode_t mode);

/** Releases the ED pin (ED_INTR_CLEAR). */
ntag5_status_t ntag5_clear_ed(void);

/** Selects the ED_INTR_CLEAR register address (default 10ABh; AN12364
 *  reports 10ACh). Set by the ED pin self-test. */
void ntag5_set_ed_clear_reg(uint16_t reg_addr);
#define NTAG5_REG_ED_INTR_CLEAR_ALT  0x10ACU

/** Reads the whole SRAM (256 bytes) with DMA. In pass-through mode, reading
 *  the last byte hands the SRAM back to the NFC side (arbiter handover). */
ntag5_status_t ntag5_read_sram(uint8_t buf[NTAG5_SRAM_SIZE]);

/** Writes mode and ED event into the configuration EEPROM, so they survive
 *  a POR. Other bits are preserved. */
ntag5_status_t ntag5_persist_config(ntag5_mode_t mode, ntag5_ed_event_t ed_event);

/** Copy of the bus diagnostics. */
void ntag5_get_diag(ntag5_diag_t *out);

/** Human-readable text for an error code. */
const char *ntag5_strerror(ntag5_status_t st);

#ifdef __cplusplus
}
#endif

#endif /* NTAG5_H */
