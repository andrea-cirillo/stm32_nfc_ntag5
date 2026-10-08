/**
 ******************************************************************************
 * @file    app_log.c
 * @brief   Thread-safe UART logging (see app_log.h).
 ******************************************************************************
 */
#include "app_log.h"

#include "FreeRTOS.h"
#include "semphr.h"
#include "task.h"
#include <stdarg.h>
#include <stdio.h>

#define LOG_LINE_MAX   160U

static UART_HandleTypeDef *s_huart;
static SemaphoreHandle_t   s_mutex;
static StaticSemaphore_t   s_mutex_buf;
static char                s_line[LOG_LINE_MAX];

void app_log_init(UART_HandleTypeDef *huart)
{
  s_huart = huart;
  s_mutex = xSemaphoreCreateMutexStatic(&s_mutex_buf);
}

static void uart_write(const char *s, size_t n)
{
  if ((s_huart != NULL) && (n > 0U)) {
    (void)HAL_UART_Transmit(s_huart, (const uint8_t *)s, (uint16_t)n, 100U);
  }
}

void app_log(const char *fmt, ...)
{
  va_list ap;
  int n;

  if ((s_mutex == NULL) || (xSemaphoreTake(s_mutex, pdMS_TO_TICKS(200)) != pdTRUE)) {
    return;
  }
  n = snprintf(s_line, sizeof s_line, "%8lu ", (unsigned long)xTaskGetTickCount());
  va_start(ap, fmt);
  n += vsnprintf(&s_line[n], sizeof s_line - (size_t)n, fmt, ap);
  va_end(ap);
  if (n > (int)sizeof s_line - 1) { n = (int)sizeof s_line - 1; }
  uart_write(s_line, (size_t)n);
  (void)xSemaphoreGive(s_mutex);
}

void app_log_raw(const char *s)
{
  size_t n = 0;
  if ((s == NULL) || (s_mutex == NULL) || (xSemaphoreTake(s_mutex, pdMS_TO_TICKS(200)) != pdTRUE)) {
    return;
  }
  while (s[n] != '\0') { n++; }
  uart_write(s, n);
  (void)xSemaphoreGive(s_mutex);
}

void app_log_hexdump(const uint8_t *data, size_t len)
{
  static const char hex[] = "0123456789ABCDEF";

  if ((s_mutex == NULL) || (xSemaphoreTake(s_mutex, pdMS_TO_TICKS(500)) != pdTRUE)) {
    return;
  }
  for (size_t off = 0; off < len; off += 16U) {
    size_t p = 0;
    p += (size_t)snprintf(&s_line[p], sizeof s_line - p, "  %04X: ", (unsigned)off);
    for (size_t i = 0; i < 16U; i++) {
      if (off + i < len) {
        s_line[p++] = hex[data[off + i] >> 4];
        s_line[p++] = hex[data[off + i] & 0x0FU];
        s_line[p++] = ' ';
      } else {
        s_line[p++] = ' '; s_line[p++] = ' '; s_line[p++] = ' ';
      }
    }
    s_line[p++] = ' ';
    s_line[p++] = '|';
    for (size_t i = 0; (i < 16U) && (off + i < len); i++) {
      uint8_t c = data[off + i];
      s_line[p++] = ((c >= 0x20U) && (c < 0x7FU)) ? (char)c : '.';
    }
    s_line[p++] = '|';
    s_line[p++] = '\r';
    s_line[p++] = '\n';
    uart_write(s_line, p);
  }
  (void)xSemaphoreGive(s_mutex);
}
