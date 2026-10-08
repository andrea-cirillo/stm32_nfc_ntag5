/**
 ******************************************************************************
 * @file    nfc_app.h
 * @brief   FreeRTOS application: NTAG 5 + P-NUCLEO-WB55.
 *
 *  Detection of the operations performed by an RF reader (phone, ISO 15693
 *  reader) on the NTAG 5:
 *
 *   - NTAG 5 in normal arbiter mode, ED pin = "Start of command":
 *     ED goes low at the start of EVERY RF command and is released at the
 *     end of the response.
 *   - Every falling edge (EXTI) counts one command. A reader "transaction"
 *     ends after NFC_APP_QUIET_MS without commands.
 *   - At the end of a transaction the NDEF memory (only the bytes in use) and
 *     the SRAM are acquired over I2C + DMA and compared with the previous copy:
 *        EEPROM changed  -> TAG_WRITE event  (written by the reader)
 *        SRAM changed    -> SRAM_WRITE event (RF -> I2C data)
 *        nothing changed -> TAG_READ event   (read by the reader)
 *   - Transactions with fewer than NFC_APP_MIN_CMDS commands are the phone's
 *     periodic "presence checks" and are ignored.
 *   - In parallel the RF field (STATUS0.NFC_FIELD_OK) defines the reader
 *     "session": if a session ends without any message, the memory is read
 *     and classified anyway (one message per approach is guaranteed).
 *
 *  Events go through a queue to the report task, which prints them on USART1
 *  (ST-LINK Virtual COM, 115200 8N1).
 ******************************************************************************
 */
#ifndef NFC_APP_H
#define NFC_APP_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32wbxx_hal.h"
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/* ------------------------------------------------------------------------- */
/* Configuration                                                              */
/* ------------------------------------------------------------------------- */

/* NTAG 5 ED (Event Detection) pin: open-drain, active low.
 * NTAG 5 board on the P-NUCLEO-WB55: ED -> Arduino D5 = PA15 (EXTI15).
 * At start-up a self-test drives ED low with the NTAG 5 "software interrupt"
 * event and checks that the pin sees it; if not, the firmware relies on the
 * RF field only. NFC_APP_USE_ED_PIN = 0 excludes the pin. */
#ifndef NFC_APP_USE_ED_PIN
#define NFC_APP_USE_ED_PIN        1
#endif
#define NFC_ED_GPIO_PORT          GPIOA
#define NFC_ED_GPIO_PIN           GPIO_PIN_15
#define NFC_ED_GPIO_CLK_ENABLE()  __HAL_RCC_GPIOA_CLK_ENABLE()
#define NFC_ED_EXTI_IRQn          EXTI15_10_IRQn    /* lines 10..15 */
#define NFC_ED_IRQHandler         EXTI15_10_IRQHandler
#define NFC_ED_NAME               "PA15 (Arduino D5)"

/* RF field (STATUS0.NFC_FIELD_OK): sampling and reader sessions */
#define NFC_APP_FIELD_POLL_MS     50U    /* sampling period                       */
#define NFC_APP_FIELD_OFF_MS      600U   /* field absence that closes a session:
                                            merges the phone's short field gaps */
#define NFC_APP_MIN_SESSION_MS    100U   /* shorter sessions = discovery pulses   */

/* RF silence that closes a reader transaction [ms] */
#define NFC_APP_QUIET_MS          80U
/* Maximum transaction length before it is processed anyway [ms] */
#define NFC_APP_MAX_BURST_MS      1500U
/* Minimum commands for a transaction to be a real read/write */
#define NFC_APP_MIN_CMDS          3U

/* Maximum user-memory bytes acquired (CC + NDEF). Multiple of 4. */
#define NFC_APP_TAG_READ_SIZE     256U

/* 1 = also watch the SRAM (RF writes with WRITE SRAM, D3h) */
#ifndef NFC_APP_WATCH_SRAM
#define NFC_APP_WATCH_SRAM        1
#endif

/* 1 = print a summary at the end of every reader session (duration,
 *     ED edges counted, STATUS1). */
#ifndef NFC_APP_VERBOSE
#define NFC_APP_VERBOSE           1
#endif

/* 1 = also store the configuration in EEPROM (persistent across POR) */
#ifndef NFC_APP_PERSIST_CONFIG
#define NFC_APP_PERSIST_CONFIG    0
#endif

/* ------------------------------------------------------------------------- */
/* Events                                                                     */
/* ------------------------------------------------------------------------- */
#define NFC_EVT_DATA_MAX          256U

typedef enum {
  NFC_EVT_READY = 0,      /* NTAG 5 configured, waiting                       */
  NFC_EVT_TAG_CONTENT,    /* current content (at start-up)                    */
  NFC_EVT_TAG_READ,       /* the reader read the tag                          */
  NFC_EVT_TAG_WRITE,      /* the reader wrote/modified the memory             */
  NFC_EVT_SRAM_WRITE,     /* the reader wrote the SRAM (RF -> I2C data)       */
  NFC_EVT_IGNORED,        /* short transaction (presence check)               */
  NFC_EVT_SESSION_END,    /* end of a reader session (verbose)                */
  NFC_EVT_ERROR,
} nfc_evt_type_t;

typedef struct {
  nfc_evt_type_t type;
  uint32_t       tick;          /* ms since start-up                          */
  uint32_t       seq;           /* progressive RF transaction number          */
  uint8_t        status0;       /* NTAG 5 STATUS0/1                           */
  uint8_t        status1;
  /* RF transaction */
  uint32_t       rf_cmds;       /* RF commands counted on the ED pin          */
  uint32_t       rf_ms;         /* transaction duration                       */
  uint8_t        ed_level;      /* ED pin level (diagnostics)                 */
  uint8_t        s1_or;         /* STATUS1: OR of the values seen in session  */
  uint8_t        s1_and;        /* STATUS1: AND of the values seen in session */
  uint32_t       samples;       /* samples with the field present             */
  /* NFC_EVT_READY: ED pin self-test result */
  bool           ed_found;
  bool           ed_clear_ok;
  uint16_t       ed_clear_reg;
  const char    *ed_name;       /* e.g. "PA15 (Arduino D5)"                   */
  /* Acquisition */
  int            read_err;      /* ntag5_status_t                             */
  uint32_t       read_us;       /* duration of the I2C + DMA transfers        */
  uint32_t       hal_err;       /* driver diagnostics (see ntag5_diag_t)      */
  uint32_t       hal_state;
  uint32_t       dma_ok;
  uint32_t       dma_fallback_it;
  const char    *what;          /* NFC_EVT_ERROR                              */
  /* Data: NDEF memory (TAG_*) or SRAM (SRAM_WRITE) */
  uint16_t       len;
  uint8_t        data[NFC_EVT_DATA_MAX];
} nfc_event_t;

/* ------------------------------------------------------------------------- */
/* API                                                                        */
/* ------------------------------------------------------------------------- */

/** Creates resources and tasks. Call from main() before osKernelStart(). */
void NfcApp_Init(I2C_HandleTypeDef *hi2c, UART_HandleTypeDef *huart);

/** Hook called for every event by the report task (never from an ISR).
 *  Weak definition: by default it prints the report on the serial port
 *  (nfc_report.c). */
void NfcApp_OnEvent(const nfc_event_t *evt);

#ifdef __cplusplus
}
#endif

#endif /* NFC_APP_H */
