/**
 ******************************************************************************
 * @file    ntag5.c
 * @brief   I2C driver for the NXP NTAG 5 - see ntag5.h.
 *
 *  I2C command format (NTAG 5 slave, block addressing, 4 bytes per block):
 *
 *   READ memory    : S | SA+W | MEMA_H | MEMA_L | Sr | SA+R | D0..Dn | P
 *   WRITE memory   : S | SA+W | MEMA_H | MEMA_L | D0 D1 D2 D3 | P   (1 block)
 *   READ REGISTER  : S | SA+W | MEMA_H | MEMA_L | REGA | Sr | SA+R | REGDAT | P
 *   WRITE REGISTER : S | SA+W | MEMA_H | MEMA_L | REGA | MASK | REGDAT | P
 *
 *  MEMA = block address (e.g. 10A1h), REGA = byte index inside the block,
 *  MASK = bits to modify (only the bits set in MASK are written).
 *
 *  All transfers are non-blocking for the CPU: interrupt mode for short
 *  commands, DMA (I2C1_RX on DMA1_Channel1) for memory/SRAM reads.
 *  The calling task blocks on a semaphore released by the HAL callbacks.
 ******************************************************************************
 */
#include "ntag5.h"

#include "FreeRTOS.h"
#include "semphr.h"
#include "task.h"
#include <string.h>

/* ------------------------------------------------------------------------- */
/* Parameters                                                                 */
/* ------------------------------------------------------------------------- */
#define NTAG5_XFER_TIMEOUT_MS      100U   /* 256 B @100 kHz ~ 26 ms             */
#define NTAG5_MUTEX_TIMEOUT_MS     500U
#define NTAG5_EEPROM_WR_TIMEOUT_MS 50U    /* typical EEPROM write ~4..5 ms      */

/* Long reads are split into transactions of NTAG5_READ_CHUNK bytes (multiple
 * of 4, <= 255): this avoids the I2C RELOAD mode in DMA transfers. */
#ifndef NTAG5_READ_CHUNK
#define NTAG5_READ_CHUNK           64U
#endif

/* Bus pins used for the bus-clear sequence (P-NUCLEO-WB55: Arduino D15/D14) */
#define NTAG5_SCL_PORT   GPIOB
#define NTAG5_SCL_PIN    GPIO_PIN_8
#define NTAG5_SDA_PORT   GPIOB
#define NTAG5_SDA_PIN    GPIO_PIN_9

/* ------------------------------------------------------------------------- */
/* Driver state                                                               */
/* ------------------------------------------------------------------------- */
static I2C_HandleTypeDef *s_hi2c;

static SemaphoreHandle_t  s_mutex;
static StaticSemaphore_t  s_mutex_buf;
static SemaphoreHandle_t  s_done;
static StaticSemaphore_t  s_done_buf;
static volatile uint32_t  s_hal_err;
static ntag5_diag_t       s_diag;
static uint16_t           s_ed_clear_reg = NTAG5_REG_ED_INTR_CLEAR;

/* ------------------------------------------------------------------------- */
/* HAL callbacks (ISR context)                                                */
/* ------------------------------------------------------------------------- */
static void xfer_signal_from_isr(void)
{
  BaseType_t woken = pdFALSE;
  if (s_done != NULL) {
    xSemaphoreGiveFromISR(s_done, &woken);
  }
  portYIELD_FROM_ISR(woken);
}

void HAL_I2C_MasterTxCpltCallback(I2C_HandleTypeDef *hi2c)
{
  if (hi2c == s_hi2c) { xfer_signal_from_isr(); }
}

void HAL_I2C_MasterRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
  if (hi2c == s_hi2c) { xfer_signal_from_isr(); }
}

void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c)
{
  if (hi2c == s_hi2c) {
    s_hal_err = HAL_I2C_GetError(hi2c);
    xfer_signal_from_isr();
  }
}

void HAL_I2C_AbortCpltCallback(I2C_HandleTypeDef *hi2c)
{
  if (hi2c == s_hi2c) {
    if (s_hal_err == HAL_I2C_ERROR_NONE) { s_hal_err = HAL_I2C_ERROR_TIMEOUT; }
    xfer_signal_from_isr();
  }
}

/* ------------------------------------------------------------------------- */
/* Bus recovery: 9 clocks on SCL + STOP, then peripheral re-init              */
/* ------------------------------------------------------------------------- */
static void bus_recover(void)
{
  GPIO_InitTypeDef gpio = {0};

  (void)HAL_I2C_DeInit(s_hi2c);

  gpio.Mode  = GPIO_MODE_OUTPUT_OD;
  gpio.Pull  = GPIO_NOPULL;
  gpio.Speed = GPIO_SPEED_FREQ_LOW;
  gpio.Pin   = NTAG5_SCL_PIN;
  HAL_GPIO_WritePin(NTAG5_SCL_PORT, NTAG5_SCL_PIN, GPIO_PIN_SET);
  HAL_GPIO_Init(NTAG5_SCL_PORT, &gpio);
  gpio.Pin   = NTAG5_SDA_PIN;
  HAL_GPIO_WritePin(NTAG5_SDA_PORT, NTAG5_SDA_PIN, GPIO_PIN_SET);
  HAL_GPIO_Init(NTAG5_SDA_PORT, &gpio);

  /* Up to 9 clock pulses until the slave releases SDA */
  for (int i = 0; i < 9; i++) {
    if (HAL_GPIO_ReadPin(NTAG5_SDA_PORT, NTAG5_SDA_PIN) == GPIO_PIN_SET) { break; }
    HAL_GPIO_WritePin(NTAG5_SCL_PORT, NTAG5_SCL_PIN, GPIO_PIN_RESET);
    vTaskDelay(pdMS_TO_TICKS(1));
    HAL_GPIO_WritePin(NTAG5_SCL_PORT, NTAG5_SCL_PIN, GPIO_PIN_SET);
    vTaskDelay(pdMS_TO_TICKS(1));
  }
  /* STOP condition: SDA low -> high while SCL is high */
  HAL_GPIO_WritePin(NTAG5_SDA_PORT, NTAG5_SDA_PIN, GPIO_PIN_RESET);
  vTaskDelay(pdMS_TO_TICKS(1));
  HAL_GPIO_WritePin(NTAG5_SDA_PORT, NTAG5_SDA_PIN, GPIO_PIN_SET);
  vTaskDelay(pdMS_TO_TICKS(1));

  /* HAL_I2C_Init -> HAL_I2C_MspInit restores AF pins, DMA and NVIC */
  if (HAL_I2C_Init(s_hi2c) == HAL_OK) {
    (void)HAL_I2CEx_ConfigAnalogFilter(s_hi2c, I2C_ANALOGFILTER_ENABLE);
    (void)HAL_I2CEx_ConfigDigitalFilter(s_hi2c, 0);
  }
}

/* ------------------------------------------------------------------------- */
/* Elementary transfer                                                        */
/* ------------------------------------------------------------------------- */
static ntag5_status_t map_hal_error(uint32_t err)
{
  if (err == HAL_I2C_ERROR_NONE) { return NTAG5_OK; }
  if ((err & ~HAL_I2C_ERROR_AF) == 0U) { return NTAG5_ERR_NACK; }
  return NTAG5_ERR_BUS;
}

static ntag5_status_t wait_done(void)
{
  if (xSemaphoreTake(s_done, pdMS_TO_TICKS(NTAG5_XFER_TIMEOUT_MS)) != pdTRUE) {
    s_diag.last_hal_err   = HAL_I2C_GetError(s_hi2c);
    s_diag.last_hal_state = (uint32_t)HAL_I2C_GetState(s_hi2c);
    s_diag.timeouts++;
    bus_recover();
    return NTAG5_ERR_TIMEOUT;
  }
  ntag5_status_t st = map_hal_error(s_hal_err);
  if (st == NTAG5_ERR_BUS) {
    s_diag.last_hal_err   = s_hal_err;
    s_diag.last_hal_state = (uint32_t)HAL_I2C_GetState(s_hi2c);
    s_diag.bus_errors++;
    bus_recover();
  }
  return st;
}

/* Synchronous error while starting a transfer (HAL_BUSY / HAL_ERROR) */
static ntag5_status_t start_failed(HAL_StatusTypeDef hs)
{
  s_diag.last_hal_err   = NTAG5_DIAG_START_FAIL | (uint32_t)hs;
  s_diag.last_hal_state = (uint32_t)HAL_I2C_GetState(s_hi2c);
  s_diag.bus_errors++;
  bus_recover();
  return NTAG5_ERR_BUS;
}

/**
 * Writes `tx` and, if rx_len > 0, reads `rx` after a repeated START.
 * Must be called with the mutex held.
 */
static ntag5_status_t xfer(const uint8_t *tx, uint16_t tx_len,
                           uint8_t *rx, uint16_t rx_len, bool rx_dma)
{
  HAL_StatusTypeDef hs;
  ntag5_status_t st;

  s_hal_err = HAL_I2C_ERROR_NONE;
  (void)xSemaphoreTake(s_done, 0);                 /* drop stale signals */

  hs = HAL_I2C_Master_Seq_Transmit_IT(s_hi2c, NTAG5_I2C_ADDR_HAL, (uint8_t *)tx, tx_len,
                                      (rx_len > 0U) ? I2C_FIRST_FRAME : I2C_FIRST_AND_LAST_FRAME);
  if (hs != HAL_OK) {
    return start_failed(hs);
  }
  st = wait_done();
  if ((st != NTAG5_OK) || (rx_len == 0U)) {
    return st;
  }

  s_hal_err = HAL_I2C_ERROR_NONE;
  (void)xSemaphoreTake(s_done, 0);
  if (rx_dma) {
    hs = HAL_I2C_Master_Seq_Receive_DMA(s_hi2c, NTAG5_I2C_ADDR_HAL, rx, rx_len, I2C_LAST_FRAME);
  } else {
    hs = HAL_I2C_Master_Seq_Receive_IT(s_hi2c, NTAG5_I2C_ADDR_HAL, rx, rx_len, I2C_LAST_FRAME);
  }
  if (hs != HAL_OK) {
    return start_failed(hs);
  }
  return wait_done();
}

static bool lock(void)
{
  return xSemaphoreTake(s_mutex, pdMS_TO_TICKS(NTAG5_MUTEX_TIMEOUT_MS)) == pdTRUE;
}

static void unlock(void)
{
  (void)xSemaphoreGive(s_mutex);
}

/* "Unlocked" variants, used internally with the mutex already held */
static ntag5_status_t read_reg_u(uint16_t reg_addr, uint8_t reg_byte, uint8_t *value)
{
  const uint8_t tx[3] = { (uint8_t)(reg_addr >> 8), (uint8_t)reg_addr, reg_byte };
  return xfer(tx, sizeof tx, value, 1U, false);
}

static ntag5_status_t write_reg_u(uint16_t reg_addr, uint8_t reg_byte, uint8_t mask, uint8_t value)
{
  const uint8_t tx[5] = { (uint8_t)(reg_addr >> 8), (uint8_t)reg_addr, reg_byte, mask, value };
  return xfer(tx, sizeof tx, NULL, 0U, false);
}

static ntag5_status_t read_blocks_u(uint16_t block_addr, uint8_t *buf, uint16_t len, bool dma)
{
  ntag5_status_t st = NTAG5_OK;

  while ((len > 0U) && (st == NTAG5_OK)) {
    const uint16_t n = (len > NTAG5_READ_CHUNK) ? (uint16_t)NTAG5_READ_CHUNK : len;
    const uint8_t tx[2] = { (uint8_t)(block_addr >> 8), (uint8_t)block_addr };

    st = xfer(tx, sizeof tx, buf, n, dma);
    if (dma) {
      if (st == NTAG5_OK) {
        s_diag.dma_ok++;
      } else if ((st == NTAG5_ERR_BUS) || (st == NTAG5_ERR_TIMEOUT)) {
        /* Fallback: same chunk in interrupt mode. If it succeeds here, the
         * problem lies in the DMA path, not on the bus or in the NTAG 5. */
        st = xfer(tx, sizeof tx, buf, n, false);
        if (st == NTAG5_OK) { s_diag.dma_fallback_it++; }
      }
    }
    block_addr = (uint16_t)(block_addr + n / NTAG5_BLOCK_SIZE);
    buf += n;
    len  = (uint16_t)(len - n);
  }
  return st;
}

/* ------------------------------------------------------------------------- */
/* Public API                                                                 */
/* ------------------------------------------------------------------------- */
void ntag5_init(I2C_HandleTypeDef *hi2c)
{
  s_hi2c  = hi2c;
  s_mutex = xSemaphoreCreateMutexStatic(&s_mutex_buf);
  s_done  = xSemaphoreCreateBinaryStatic(&s_done_buf);
}

ntag5_status_t ntag5_probe(void)
{
  if (!lock()) { return NTAG5_ERR_TIMEOUT; }
  /* Short polling: no IT transfer in progress, guaranteed by the mutex */
  HAL_StatusTypeDef hs = HAL_I2C_IsDeviceReady(s_hi2c, NTAG5_I2C_ADDR_HAL, 3U, 10U);
  unlock();
  return (hs == HAL_OK) ? NTAG5_OK : NTAG5_ERR_NACK;
}

ntag5_status_t ntag5_read_blocks(uint16_t block_addr, uint8_t *buf, uint16_t len)
{
  if ((buf == NULL) || (len == 0U) || ((len % NTAG5_BLOCK_SIZE) != 0U)) {
    return NTAG5_ERR_PARAM;
  }
  if (!lock()) { return NTAG5_ERR_TIMEOUT; }
  ntag5_status_t st = read_blocks_u(block_addr, buf, len, len >= 16U);
  unlock();
  return st;
}

ntag5_status_t ntag5_write_block(uint16_t block_addr, const uint8_t data[NTAG5_BLOCK_SIZE])
{
  uint8_t tx[2 + NTAG5_BLOCK_SIZE];
  ntag5_status_t st;

  if (data == NULL) { return NTAG5_ERR_PARAM; }
  tx[0] = (uint8_t)(block_addr >> 8);
  tx[1] = (uint8_t)block_addr;
  memcpy(&tx[2], data, NTAG5_BLOCK_SIZE);

  if (!lock()) { return NTAG5_ERR_TIMEOUT; }
  st = xfer(tx, sizeof tx, NULL, 0U, false);

  /* EEPROM (user or configuration): wait for the end of programming.
   * While writing, the NTAG 5 may not answer (NACK): retry. */
  if ((st == NTAG5_OK) && (block_addr < NTAG5_SRAM_START)) {
    TickType_t t0 = xTaskGetTickCount();
    uint8_t s0 = NTAG5_STATUS0_EEPROM_WR_BUSY;
    do {
      vTaskDelay(pdMS_TO_TICKS(1));
      st = read_reg_u(NTAG5_REG_STATUS, 0U, &s0);
      if ((xTaskGetTickCount() - t0) > pdMS_TO_TICKS(NTAG5_EEPROM_WR_TIMEOUT_MS)) {
        st = NTAG5_ERR_TIMEOUT;
        break;
      }
    } while ((st == NTAG5_ERR_NACK) || ((st == NTAG5_OK) && (s0 & NTAG5_STATUS0_EEPROM_WR_BUSY)));

    if ((st == NTAG5_OK) && (s0 & NTAG5_STATUS0_EEPROM_WR_ERROR)) {
      /* the flag must be cleared by software */
      (void)write_reg_u(NTAG5_REG_STATUS, 0U, NTAG5_STATUS0_EEPROM_WR_ERROR, 0U);
      st = NTAG5_ERR_VERIFY;
    }
  }
  unlock();
  return st;
}

ntag5_status_t ntag5_read_reg(uint16_t reg_addr, uint8_t reg_byte, uint8_t *value)
{
  if (value == NULL) { return NTAG5_ERR_PARAM; }
  if (!lock()) { return NTAG5_ERR_TIMEOUT; }
  ntag5_status_t st = read_reg_u(reg_addr, reg_byte, value);
  unlock();
  return st;
}

ntag5_status_t ntag5_write_reg(uint16_t reg_addr, uint8_t reg_byte, uint8_t mask, uint8_t value)
{
  if (!lock()) { return NTAG5_ERR_TIMEOUT; }
  ntag5_status_t st = write_reg_u(reg_addr, reg_byte, mask, value);
  unlock();
  return st;
}

ntag5_status_t ntag5_read_status(ntag5_status_reg_t *out)
{
  ntag5_status_t st;
  if (out == NULL) { return NTAG5_ERR_PARAM; }
  if (!lock()) { return NTAG5_ERR_TIMEOUT; }
  st = read_reg_u(NTAG5_REG_STATUS, 0U, &out->status0);
  if (st == NTAG5_OK) {
    st = read_reg_u(NTAG5_REG_STATUS, 1U, &out->status1);
  }
  unlock();
  return st;
}

ntag5_status_t ntag5_clear_ed(void)
{
  return ntag5_write_reg(s_ed_clear_reg, 0U, 0x01U, 0x01U);
}

void ntag5_set_ed_clear_reg(uint16_t reg_addr)
{
  s_ed_clear_reg = reg_addr;
}

static ntag5_status_t set_ed_u(ntag5_ed_event_t ed_event)
{
  uint8_t ed = 0;
  ntag5_status_t st = write_reg_u(NTAG5_REG_ED_CONFIG, 0U, NTAG5_ED_MASK, (uint8_t)ed_event);
  if (st == NTAG5_OK) { st = read_reg_u(NTAG5_REG_ED_CONFIG, 0U, &ed); }
  if ((st == NTAG5_OK) && ((ed & NTAG5_ED_MASK) != (uint8_t)ed_event)) { st = NTAG5_ERR_VERIFY; }
  return st;
}

ntag5_status_t ntag5_set_ed_event(ntag5_ed_event_t ed_event)
{
  if (!lock()) { return NTAG5_ERR_TIMEOUT; }
  ntag5_status_t st = set_ed_u(ed_event);
  unlock();
  return st;
}

static uint8_t mode_config1(ntag5_mode_t mode)
{
  return (mode == NTAG5_MODE_PT_NFC_TO_I2C) ? NTAG5_CONFIG1_PT_NFC2I2C : NTAG5_CONFIG1_NORMAL;
}

ntag5_status_t ntag5_set_mode(ntag5_mode_t mode, ntag5_ed_event_t ed_event)
{
  ntag5_status_t st;
  const uint8_t want = mode_config1(mode);
  uint8_t cfg1 = 0, s0 = 0;

  if (!lock()) { return NTAG5_ERR_TIMEOUT; }

  /* 1) CONFIG_1: ARBITER_MODE, SRAM_ENABLE, PT_TRANSFER_DIR.
   *    USE_CASE_CONF (I2C slave) and EH settings are left untouched. */
  st = write_reg_u(NTAG5_REG_CONFIG, 1U, NTAG5_CONFIG1_PT_MASK, want);

  /* 2) CONFIG_1 read-back */
  if (st == NTAG5_OK) { st = read_reg_u(NTAG5_REG_CONFIG, 1U, &cfg1); }
  if ((st == NTAG5_OK) && ((cfg1 & NTAG5_CONFIG1_PT_MASK) != want)) {
    st = NTAG5_ERR_VERIFY;
  }

  /* 3) ED pin event (with read-back) */
  if (st == NTAG5_OK) { st = set_ed_u(ed_event); }

  /* 4) In pass-through the direction is also reflected in STATUS0.PT_TRANSFER_DIR */
  if ((st == NTAG5_OK) && (mode == NTAG5_MODE_PT_NFC_TO_I2C)) {
    if ((read_reg_u(NTAG5_REG_STATUS, 0U, &s0) == NTAG5_OK) &&
        ((s0 & NTAG5_STATUS0_PT_TRANSFER_DIR) == 0U)) {
      (void)write_reg_u(NTAG5_REG_STATUS, 0U, NTAG5_STATUS0_PT_TRANSFER_DIR,
                        NTAG5_STATUS0_PT_TRANSFER_DIR);
    }
  }

  unlock();
  return st;
}

bool ntag5_mode_is(ntag5_mode_t mode)
{
  uint8_t cfg1 = 0;
  return (ntag5_read_reg(NTAG5_REG_CONFIG, 1U, &cfg1) == NTAG5_OK) &&
         ((cfg1 & NTAG5_CONFIG1_PT_MASK) == mode_config1(mode));
}

ntag5_status_t ntag5_read_sram(uint8_t buf[NTAG5_SRAM_SIZE])
{
  if (buf == NULL) { return NTAG5_ERR_PARAM; }
  if (!lock()) { return NTAG5_ERR_TIMEOUT; }
  /* Full read of 2000h..203Fh: in pass-through, reading the last byte
   * (block 203Fh) hands the arbiter back to the NFC side and releases ED. */
  ntag5_status_t st = read_blocks_u(NTAG5_SRAM_START, buf, NTAG5_SRAM_SIZE, true);
  unlock();
  return st;
}

ntag5_status_t ntag5_persist_config(ntag5_mode_t mode, ntag5_ed_event_t ed_event)
{
  uint8_t blk[NTAG5_BLOCK_SIZE];
  ntag5_status_t st;
  const uint8_t want = mode_config1(mode);

  /* CONFIG (1037h): byte 1 = CONFIG_1 */
  st = ntag5_read_blocks(NTAG5_CFG_CONFIG, blk, sizeof blk);
  if (st != NTAG5_OK) { return st; }
  if ((blk[1] & NTAG5_CONFIG1_PT_MASK) != want) {
    blk[1] = (uint8_t)((blk[1] & ~NTAG5_CONFIG1_PT_MASK) | want);
    st = ntag5_write_block(NTAG5_CFG_CONFIG, blk);
    if (st != NTAG5_OK) { return st; }
  }

  /* EH/ED (103Dh): byte 2 = ED_CONFIG */
  st = ntag5_read_blocks(NTAG5_CFG_EH_ED_CONFIG, blk, sizeof blk);
  if (st != NTAG5_OK) { return st; }
  if ((blk[2] & NTAG5_ED_MASK) != (uint8_t)ed_event) {
    blk[2] = (uint8_t)((blk[2] & ~NTAG5_ED_MASK) | (uint8_t)ed_event);
    st = ntag5_write_block(NTAG5_CFG_EH_ED_CONFIG, blk);
  }
  return st;
}

void ntag5_get_diag(ntag5_diag_t *out)
{
  if (out != NULL) {
    taskENTER_CRITICAL();
    *out = s_diag;
    taskEXIT_CRITICAL();
  }
}

const char *ntag5_strerror(ntag5_status_t st)
{
  switch (st) {
    case NTAG5_OK:          return "OK";
    case NTAG5_ERR_PARAM:   return "invalid parameter";
    case NTAG5_ERR_NACK:    return "NACK (device absent or area locked by the arbiter)";
    case NTAG5_ERR_BUS:     return "I2C bus error";
    case NTAG5_ERR_TIMEOUT: return "timeout";
    case NTAG5_ERR_VERIFY:  return "verification failed";
    default:                return "?";
  }
}
