/**
 ******************************************************************************
 * @file    nfc_app.c
 * @brief   NTAG 5 FreeRTOS tasks: detection of RF reader reads/writes,
 *          I2C + DMA acquisition, event publishing (see nfc_app.h).
 *
 *  Two complementary mechanisms:
 *   1) ED pin = "Start of command" on EXTI: every falling edge is one RF
 *      command. A transaction ends after NFC_APP_QUIET_MS of silence and is
 *      classified (READ / WRITE) by comparing the memory content.
 *   2) RF field (STATUS0.NFC_FIELD_OK) sampled every NFC_APP_FIELD_POLL_MS:
 *      it defines the reader "session". If a session ends without messages
 *      (ED missing or inactive), the memory is read and classified anyway:
 *      one message per approach is guaranteed.
 *
 *  The arbiter stays in normal mode: in NFC->I2C SRAM pass-through the NTAG 5
 *  denies I2C access to the EEPROM (NACK, STATUS1 bit0 = 1).
 *
 *  Tasks:
 *   - nfcTask   (AboveNormal): ED, RF field, acquisitions, classification
 *   - nfcReport (Normal)     : drains the event queue -> NfcApp_OnEvent()
 *  All RTOS resources are static (project FreeRTOS heap = 4 KB).
 ******************************************************************************
 */
#include "nfc_app.h"
#include "nfc_report.h"
#include "ntag5.h"
#include "ndef.h"
#include "app_log.h"

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "cmsis_os2.h"
#include <string.h>

/* ------------------------------------------------------------------------- */
/* Internal parameters                                                        */
/* ------------------------------------------------------------------------- */
#define NFC_TASK_STACK_WORDS     512U
#define REPORT_TASK_STACK_WORDS  768U
#define NFC_QUEUE_DEPTH          4U
#define NFC_READ_RETRIES         5U
#define NFC_READ_RETRY_MS        20U
#define NFC_HEADER_BYTES         16U     /* CC (4/8 B) + start of the NDEF TLV  */
#define NFC_SUPERVISE_MS         5000U   /* configuration check while idle     */
#define NFC_ED_STUCK_MS          1000U   /* ED low without field -> reconfigure */

#if (NFC_APP_TAG_READ_SIZE > NFC_EVT_DATA_MAX) || ((NFC_APP_TAG_READ_SIZE % 4U) != 0U) || \
    (NFC_APP_TAG_READ_SIZE < NFC_HEADER_BYTES)
#error "NFC_APP_TAG_READ_SIZE: multiple of 4, between 16 and NFC_EVT_DATA_MAX"
#endif

/* ------------------------------------------------------------------------- */
/* Resources                                                                  */
/* ------------------------------------------------------------------------- */
static StaticTask_t  s_nfc_tcb;
static StackType_t   s_nfc_stack[NFC_TASK_STACK_WORDS];
static StaticTask_t  s_rep_tcb;
static StackType_t   s_rep_stack[REPORT_TASK_STACK_WORDS];

static QueueHandle_t s_queue;
static StaticQueue_t s_queue_buf;
static uint8_t       s_queue_storage[NFC_QUEUE_DEPTH * sizeof(nfc_event_t)];

static SemaphoreHandle_t s_ed_sem;
static StaticSemaphore_t s_ed_sem_buf;

/* RF command counter (updated by the ED pin ISR) */
static volatile uint32_t   s_cmd_count;
static volatile TickType_t s_last_cmd_tick;
static bool                s_ed_ok;          /* self-test result */

/* Static buffers (DMA destination in SRAM1, outside the task stacks) */
static nfc_event_t s_evt;
static nfc_event_t s_rep_evt;
static uint8_t     s_mem[NFC_APP_TAG_READ_SIZE];
static uint8_t     s_mem_snap[NFC_APP_TAG_READ_SIZE];
static uint16_t    s_mem_len, s_mem_snap_len;
static bool        s_mem_snap_valid;
#if NFC_APP_WATCH_SRAM
static uint8_t     s_sram[NTAG5_SRAM_SIZE];
static uint8_t     s_sram_snap[NTAG5_SRAM_SIZE];
static bool        s_sram_snap_valid;
static bool        s_sram_disabled;
#endif

static uint32_t    s_seq;
static uint32_t    s_dropped;

/* Reader session (RF field) */
static struct {
  bool       active;
  bool       reported;      /* at least one READ/WRITE message published */
  TickType_t t_on;
  TickType_t t_last_on;
  uint32_t   cmds0;
  uint32_t   samples;
  uint8_t    s1_or, s1_and;
} s_sess;

/* ------------------------------------------------------------------------- */
/* Timing (DWT cycle counter)                                                 */
/* ------------------------------------------------------------------------- */
static void cyccnt_init(void)
{
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CYCCNT = 0U;
  DWT->CTRL  |= DWT_CTRL_CYCCNTENA_Msk;
}

static uint32_t cyc_to_us(uint32_t cycles)
{
  return cycles / (SystemCoreClock / 1000000U);
}

/* ------------------------------------------------------------------------- */
/* ED pin on EXTI: one falling edge = start of one RF command                */
/* ------------------------------------------------------------------------- */
static uint8_t ed_level(void)
{
#if NFC_APP_USE_ED_PIN
  return (HAL_GPIO_ReadPin(NFC_ED_GPIO_PORT, NFC_ED_GPIO_PIN) == GPIO_PIN_SET) ? 1U : 0U;
#else
  return 1U;
#endif
}

#if NFC_APP_USE_ED_PIN
static void ed_pin_init(void)
{
  GPIO_InitTypeDef gpio = {0};

  NFC_ED_GPIO_CLK_ENABLE();
  gpio.Pin  = NFC_ED_GPIO_PIN;
  gpio.Mode = GPIO_MODE_IT_FALLING;
  gpio.Pull = GPIO_PULLUP;            /* ED open-drain */
  HAL_GPIO_Init(NFC_ED_GPIO_PORT, &gpio);

  /* Priority >= configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY (5) */
  HAL_NVIC_SetPriority(NFC_ED_EXTI_IRQn, 6, 0);
  HAL_NVIC_EnableIRQ(NFC_ED_EXTI_IRQn);
}

void NFC_ED_IRQHandler(void)
{
  HAL_GPIO_EXTI_IRQHandler(NFC_ED_GPIO_PIN);
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if ((GPIO_Pin == NFC_ED_GPIO_PIN) && (s_ed_sem != NULL)) {
    BaseType_t woken = pdFALSE;
    s_cmd_count++;
    s_last_cmd_tick = xTaskGetTickCountFromISR();
    xSemaphoreGiveFromISR(s_ed_sem, &woken);
    portYIELD_FROM_ISR(woken);
  }
}
#else
static void ed_pin_init(void) {}
#endif

/* ------------------------------------------------------------------------- */
/* Event publishing                                                           */
/* ------------------------------------------------------------------------- */
static nfc_event_t *evt_begin(nfc_evt_type_t type)
{
  memset(&s_evt, 0, offsetof(nfc_event_t, data));
  s_evt.type     = type;
  s_evt.tick     = (uint32_t)xTaskGetTickCount() * portTICK_PERIOD_MS;
  s_evt.seq      = s_seq;
  s_evt.ed_level = ed_level();
  return &s_evt;
}

static void evt_fill_diag(nfc_event_t *e)
{
  ntag5_diag_t d;
  ntag5_get_diag(&d);
  e->hal_err         = d.last_hal_err;
  e->hal_state       = d.last_hal_state;
  e->dma_ok          = d.dma_ok;
  e->dma_fallback_it = d.dma_fallback_it;
}

static void evt_publish(void)
{
  if (xQueueSend(s_queue, &s_evt, pdMS_TO_TICKS(50)) != pdTRUE) {
    s_dropped++;
  }
}

__attribute__((unused)) static void publish_error(const char *what, ntag5_status_t err)
{
  nfc_event_t *e = evt_begin(NFC_EVT_ERROR);
  e->what = what;
  e->read_err = (int)err;
  evt_fill_diag(e);
  evt_publish();
}

/* ------------------------------------------------------------------------- */
/* I2C + DMA acquisitions                                                     */
/* ------------------------------------------------------------------------- */

/** Read with retries: NACK = arbiter busy on the RF side, BUS/TIMEOUT = bus
 *  already recovered by the driver. */
static ntag5_status_t read_retry(uint16_t block, uint8_t *buf, uint16_t len)
{
  ntag5_status_t st = NTAG5_ERR_NACK;
  for (uint32_t i = 0; i < NFC_READ_RETRIES; i++) {
    st = ntag5_read_blocks(block, buf, len);
    if ((st != NTAG5_ERR_NACK) && (st != NTAG5_ERR_BUS) && (st != NTAG5_ERR_TIMEOUT)) {
      break;
    }
    vTaskDelay(pdMS_TO_TICKS(NFC_READ_RETRY_MS));
  }
  return st;
}

/** Reads the NDEF area into s_mem: first the header (CC + TLV), then only
 *  the bytes actually used by the message. */
static ntag5_status_t read_ndef_area(uint32_t *us)
{
  ndef_t5t_info_t info;
  ndef_result_t r;
  uint32_t need = NFC_HEADER_BYTES;
  uint32_t c0 = DWT->CYCCNT;
  ntag5_status_t st;

  s_mem_len = 0;
  st = read_retry(NTAG5_USER_MEM_START, s_mem, NFC_HEADER_BYTES);
  if (st != NTAG5_OK) { return st; }

  r = ndef_t5t_parse(s_mem, NFC_HEADER_BYTES, &info);
  if (((r == NDEF_OK) || (r == NDEF_ERR_TRUNCATED)) && (info.msg_len > 0U)) {
    need = (uint32_t)info.msg_offset + (uint32_t)info.msg_len + 1U;   /* + terminator */
    need = (need + 3U) & ~3U;
    if (need > NFC_APP_TAG_READ_SIZE) { need = NFC_APP_TAG_READ_SIZE; }
  }
  if (need > NFC_HEADER_BYTES) {
    st = read_retry((uint16_t)(NTAG5_USER_MEM_START + NFC_HEADER_BYTES / NTAG5_BLOCK_SIZE),
                    &s_mem[NFC_HEADER_BYTES], (uint16_t)(need - NFC_HEADER_BYTES));
    if (st != NTAG5_OK) { return st; }
  }
  s_mem_len = (uint16_t)need;
  *us += cyc_to_us(DWT->CYCCNT - c0);
  return NTAG5_OK;
}

static bool mem_changed(void)
{
  return !s_mem_snap_valid || (s_mem_len != s_mem_snap_len) ||
         (memcmp(s_mem, s_mem_snap, s_mem_len) != 0);
}

static void mem_take_snapshot(void)
{
  memcpy(s_mem_snap, s_mem, s_mem_len);
  s_mem_snap_len   = s_mem_len;
  s_mem_snap_valid = true;
}

#if NFC_APP_WATCH_SRAM
/** Reads the SRAM; *changed = true if it differs from the previous copy. */
static void sram_check(uint32_t *us, bool *changed)
{
  uint32_t c0 = DWT->CYCCNT;
  ntag5_status_t st;

  *changed = false;
  if (s_sram_disabled) { return; }
  st = read_retry(NTAG5_SRAM_START, s_sram, NTAG5_SRAM_SIZE);
  if (st != NTAG5_OK) {
    s_sram_disabled = true;
    publish_error("SRAM not readable over I2C, SRAM watch disabled", st);
    return;
  }
  *us += cyc_to_us(DWT->CYCCNT - c0);
  *changed = s_sram_snap_valid && (memcmp(s_sram, s_sram_snap, NTAG5_SRAM_SIZE) != 0);
  memcpy(s_sram_snap, s_sram, NTAG5_SRAM_SIZE);
  s_sram_snap_valid = true;
}
#endif

/* ------------------------------------------------------------------------- */
/* Classification of a transaction / session                                 */
/* ------------------------------------------------------------------------- */
/** Reads the memory (and SRAM) and publishes READ / WRITE.
 *  Returns true if a message was produced. */
static bool process_transaction(uint32_t cmds, uint32_t rf_ms, bool force_read)
{
  ntag5_status_reg_t sr = {0};
  ntag5_status_t st;
  uint32_t us = 0;
  bool mem_chg, sram_chg = false;
  nfc_event_t *e;

  if ((cmds < NFC_APP_MIN_CMDS) && !force_read) {
    return false;                 /* phone presence check */
  }

  s_seq++;
  (void)ntag5_read_status(&sr);

  st = read_ndef_area(&us);
  if (st != NTAG5_OK) {
    e = evt_begin(NFC_EVT_TAG_READ);
    e->rf_cmds  = cmds;
    e->rf_ms    = rf_ms;
    e->read_err = (int)st;
    evt_fill_diag(e);
    (void)ntag5_read_status(&sr);            /* status after the error */
    e->status0 = sr.status0;
    e->status1 = sr.status1;
    evt_publish();
    return true;
  }
  mem_chg = mem_changed();

#if NFC_APP_WATCH_SRAM
  sram_check(&us, &sram_chg);
#endif

  if (mem_chg || !sram_chg) {
    e = evt_begin(mem_chg ? NFC_EVT_TAG_WRITE : NFC_EVT_TAG_READ);
    e->rf_cmds  = cmds;
    e->rf_ms    = rf_ms;
    e->status0  = sr.status0;
    e->status1  = sr.status1;
    e->read_us  = us;
    e->read_err = (int)NTAG5_OK;
    e->len      = s_mem_len;
    memcpy(e->data, s_mem, s_mem_len);
    evt_fill_diag(e);
    evt_publish();
    mem_take_snapshot();
  }

#if NFC_APP_WATCH_SRAM
  if (sram_chg) {
    e = evt_begin(NFC_EVT_SRAM_WRITE);
    e->rf_cmds  = cmds;
    e->rf_ms    = rf_ms;
    e->status0  = sr.status0;
    e->status1  = sr.status1;
    e->read_us  = us;
    e->len      = NTAG5_SRAM_SIZE;
    memcpy(e->data, s_sram, NTAG5_SRAM_SIZE);
    evt_publish();
  }
#endif
  return true;
}

/* ------------------------------------------------------------------------- */
/* ED pin self-test                                                           */
/* ------------------------------------------------------------------------- */
#if NFC_APP_USE_ED_PIN
/* The "software interrupt" event (1101b) drives ED low as soon as it is
 * configured and keeps it low until ED_INTR_CLEAR: this checks the wiring to
 * the pin and which clear register address works (10ABh datasheet / 10ACh
 * AN12364). */
static void ed_selftest(nfc_event_t *ready)
{
  static const uint16_t clear_regs[] = { NTAG5_REG_ED_INTR_CLEAR, NTAG5_REG_ED_INTR_CLEAR_ALT };

  ready->ed_found     = false;
  ready->ed_clear_ok  = false;
  ready->ed_clear_reg = 0;

  if (ed_level() == 0U) {
    /* already low: try to release it before the test */
    (void)ntag5_set_ed_event(NTAG5_ED_DISABLED);
    (void)ntag5_clear_ed();
    vTaskDelay(pdMS_TO_TICKS(2));
  }
  if (ed_level() == 0U) { return; }              /* stuck low: test not possible */

  if (ntag5_set_ed_event(NTAG5_ED_SW_INTERRUPT) != NTAG5_OK) { return; }
  vTaskDelay(pdMS_TO_TICKS(2));
  ready->ed_found = (ed_level() == 0U);

  for (uint32_t i = 0; (i < 2U) && ready->ed_found; i++) {
    ntag5_set_ed_clear_reg(clear_regs[i]);
    (void)ntag5_clear_ed();
    vTaskDelay(pdMS_TO_TICKS(2));
    if (ed_level() == 1U) {
      ready->ed_clear_ok  = true;
      ready->ed_clear_reg = clear_regs[i];
      break;
    }
  }
  if (!ready->ed_clear_ok) {
    ntag5_set_ed_clear_reg(NTAG5_REG_ED_INTR_CLEAR);
  }
  (void)ntag5_set_ed_event(NTAG5_ED_DISABLED);
  vTaskDelay(pdMS_TO_TICKS(2));

  /* the test produced edges: reset counter and semaphore */
  (void)xSemaphoreTake(s_ed_sem, 0);
  s_cmd_count = 0;
}
#endif

/* ------------------------------------------------------------------------- */
/* Bring-up                                                                   */
/* ------------------------------------------------------------------------- */
static void ntag5_bringup(void)
{
  ntag5_status_t st;
  bool warned = false;
  uint32_t us = 0;
  nfc_event_t *e;

  for (;;) {
    st = ntag5_probe();
    if (st != NTAG5_OK) {
      if (!warned) {
        LOGW("NTAG5 not answering at 0x%02X: check wiring/power supply",
             NTAG5_I2C_ADDR_7BIT);
        warned = true;
      }
      vTaskDelay(pdMS_TO_TICKS(1000));
      continue;
    }
    st = ntag5_set_mode(NTAG5_MODE_NORMAL, NTAG5_ED_DISABLED);
    if (st != NTAG5_OK) {
      LOGE("NTAG5 configuration failed: %s", ntag5_strerror(st));
      vTaskDelay(pdMS_TO_TICKS(1000));
      continue;
    }
    break;
  }

  /* ED self-test, then the final ED event */
  e = evt_begin(NFC_EVT_READY);
  e->ed_name = NFC_APP_USE_ED_PIN ? NFC_ED_NAME : NULL;
#if NFC_APP_USE_ED_PIN
  ed_selftest(e);
  s_ed_ok = e->ed_found;
#else
  s_ed_ok = false;
#endif
  if (s_ed_ok) {
    st = ntag5_set_ed_event(NTAG5_ED_START_OF_CMD);
    if (st != NTAG5_OK) { s_ed_ok = false; }
  }
#if NFC_APP_PERSIST_CONFIG
  st = ntag5_persist_config(NTAG5_MODE_NORMAL,
                            s_ed_ok ? NTAG5_ED_START_OF_CMD : NTAG5_ED_DISABLED);
  if (st != NTAG5_OK) {
    LOGW("persistent configuration write failed: %s", ntag5_strerror(st));
  }
#endif
  e->ed_level = ed_level();
  evt_publish();

  /* Current tag content (reference for the comparisons) */
  st = read_ndef_area(&us);
  e = evt_begin(NFC_EVT_TAG_CONTENT);
  e->read_err = (int)st;
  e->read_us  = us;
  if (st == NTAG5_OK) {
    e->len = s_mem_len;
    memcpy(e->data, s_mem, s_mem_len);
    mem_take_snapshot();
  } else {
    ntag5_status_reg_t sr;
    if (ntag5_read_status(&sr) == NTAG5_OK) {
      e->status0 = sr.status0;
      e->status1 = sr.status1;
    }
  }
  evt_fill_diag(e);
  evt_publish();

#if NFC_APP_WATCH_SRAM
  {
    bool dummy;
    us = 0;
    sram_check(&us, &dummy);
  }
#endif
}

/* ------------------------------------------------------------------------- */
/* Reader session (RF field)                                                  */
/* ------------------------------------------------------------------------- */
/** Samples the RF field. Returns false if the configuration must be redone. */
static bool supervise_field(TickType_t *t_ed_low)
{
  ntag5_status_reg_t sr;
  TickType_t now = xTaskGetTickCount();

  if (ntag5_read_status(&sr) != NTAG5_OK) {
    return ntag5_mode_is(NTAG5_MODE_NORMAL);
  }
  const bool field = (sr.status0 & NTAG5_STATUS0_NFC_FIELD_OK) != 0U;

  if (field) {
    if (!s_sess.active) {
      memset(&s_sess, 0, sizeof s_sess);
      s_sess.active = true;
      s_sess.t_on   = now;
      s_sess.cmds0  = s_cmd_count;
      s_sess.s1_and = 0xFFU;
    }
    s_sess.t_last_on = now;
    s_sess.samples++;
    s_sess.s1_or  |= sr.status1;
    s_sess.s1_and &= sr.status1;
  } else if (s_sess.active &&
             ((now - s_sess.t_last_on) >= pdMS_TO_TICKS(NFC_APP_FIELD_OFF_MS))) {
    /* End of session: no field for NFC_APP_FIELD_OFF_MS */
    uint32_t ms   = (uint32_t)(s_sess.t_last_on - s_sess.t_on) * portTICK_PERIOD_MS +
                    NFC_APP_FIELD_POLL_MS;
    uint32_t cmds = s_cmd_count - s_sess.cmds0;
    s_sess.active = false;

    if ((ms >= NFC_APP_MIN_SESSION_MS) || (cmds > 0U)) {
#if NFC_APP_VERBOSE
      nfc_event_t *e = evt_begin(NFC_EVT_SESSION_END);
      e->rf_cmds = cmds;
      e->rf_ms   = ms;
      e->samples = s_sess.samples;
      e->s1_or   = s_sess.s1_or;
      e->s1_and  = s_sess.s1_and;
      evt_publish();
#endif
      if (!s_sess.reported && (ms >= NFC_APP_MIN_SESSION_MS)) {
        (void)process_transaction(cmds, ms, true);
      }
    }
  }

  /* ED low without RF field for too long: event latched */
  if (s_ed_ok && !s_sess.active && (ed_level() == 0U)) {
    if (*t_ed_low == 0U) {
      *t_ed_low = now;
    } else if ((now - *t_ed_low) >= pdMS_TO_TICKS(NFC_ED_STUCK_MS)) {
      *t_ed_low = 0;
      LOGW("ED pin stuck low without RF field: reconfiguring the ED event");
      (void)ntag5_set_ed_event(NTAG5_ED_DISABLED);
      (void)ntag5_clear_ed();
      (void)ntag5_set_ed_event(NTAG5_ED_START_OF_CMD);
    }
  } else {
    *t_ed_low = 0;
  }
  return true;
}

/* ------------------------------------------------------------------------- */
/* Main loop                                                                  */
/* ------------------------------------------------------------------------- */
static void nfc_loop(void)
{
  uint32_t   done   = s_cmd_count;
  TickType_t t_sup  = xTaskGetTickCount();
  TickType_t t_ed_low = 0;

  memset(&s_sess, 0, sizeof s_sess);

  for (;;) {
    /* 1) RF command (ED edge) or next field sample */
    if (xSemaphoreTake(s_ed_sem, pdMS_TO_TICKS(NFC_APP_FIELD_POLL_MS)) == pdTRUE) {
      TickType_t t_first = xTaskGetTickCount();

      /* 2) Wait for the RF silence that closes the transaction */
      uint32_t last;
      do {
        last = s_cmd_count;
        vTaskDelay(pdMS_TO_TICKS(NFC_APP_QUIET_MS));
      } while ((s_cmd_count != last) &&
               ((xTaskGetTickCount() - t_first) < pdMS_TO_TICKS(NFC_APP_MAX_BURST_MS)));

      /* 3) Close: drain the semaphore, then snapshot the counter */
      (void)xSemaphoreTake(s_ed_sem, 0);
      uint32_t   now   = s_cmd_count;
      uint32_t   cmds  = now - done;
      TickType_t tl    = s_last_cmd_tick;
      uint32_t   rf_ms = (tl > t_first) ? (uint32_t)(tl - t_first) * portTICK_PERIOD_MS : 0U;
      done = now;

      if (process_transaction(cmds, rf_ms, false)) {
        s_sess.reported = true;
      }
    }

    /* 4) RF field + supervision */
    if (!supervise_field(&t_ed_low)) {
      LOGW("NTAG5 unreachable, reconfiguring");
      return;
    }
    if (!s_sess.active && ((xTaskGetTickCount() - t_sup) >= pdMS_TO_TICKS(NFC_SUPERVISE_MS))) {
      t_sup = xTaskGetTickCount();
      if (!ntag5_mode_is(NTAG5_MODE_NORMAL)) {
        LOGW("NTAG5 configuration lost, reconfiguring");
        return;
      }
    }

    if (s_dropped != 0U) {
      LOGW("%lu events dropped (queue full)", (unsigned long)s_dropped);
      s_dropped = 0;
    }
  }
}

static void nfc_task(void *arg)
{
  (void)arg;

  app_log_raw("\r\n");
  LOGI("NFC app: P-NUCLEO-WB55 + NTAG 5 (I2C 0x%02X)", NTAG5_I2C_ADDR_7BIT);

  for (;;) {
    ntag5_bringup();
    nfc_loop();          /* returns only if the configuration must be redone */
  }
}

/* ------------------------------------------------------------------------- */
/* Report task                                                                */
/* ------------------------------------------------------------------------- */
__attribute__((weak)) void NfcApp_OnEvent(const nfc_event_t *evt)
{
  nfc_report_print(evt);
}

static void report_task(void *arg)
{
  (void)arg;
  for (;;) {
    if (xQueueReceive(s_queue, &s_rep_evt, portMAX_DELAY) == pdTRUE) {
      NfcApp_OnEvent(&s_rep_evt);
    }
  }
}

/* ------------------------------------------------------------------------- */
/* Init                                                                       */
/* ------------------------------------------------------------------------- */
void NfcApp_Init(I2C_HandleTypeDef *hi2c, UART_HandleTypeDef *huart)
{
  app_log_init(huart);
  ntag5_init(hi2c);
  cyccnt_init();

  s_ed_sem = xSemaphoreCreateBinaryStatic(&s_ed_sem_buf);
  s_queue  = xQueueCreateStatic(NFC_QUEUE_DEPTH, sizeof(nfc_event_t),
                                s_queue_storage, &s_queue_buf);
  configASSERT(s_ed_sem != NULL);
  configASSERT(s_queue != NULL);

  ed_pin_init();

  (void)xTaskCreateStatic(nfc_task, "nfcTask", NFC_TASK_STACK_WORDS, NULL,
                          (UBaseType_t)osPriorityAboveNormal, s_nfc_stack, &s_nfc_tcb);
  (void)xTaskCreateStatic(report_task, "nfcReport", REPORT_TASK_STACK_WORDS, NULL,
                          (UBaseType_t)osPriorityNormal, s_rep_stack, &s_rep_tcb);
}
