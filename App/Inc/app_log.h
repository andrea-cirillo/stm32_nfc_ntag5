/**
 ******************************************************************************
 * @file    app_log.h
 * @brief   Thread-safe logging on USART1 (ST-LINK Virtual COM, 115200 8N1).
 ******************************************************************************
 */
#ifndef APP_LOG_H
#define APP_LOG_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32wbxx_hal.h"
#include <stddef.h>
#include <stdint.h>

void app_log_init(UART_HandleTypeDef *huart);
void app_log(const char *fmt, ...) __attribute__((format(printf, 1, 2)));
void app_log_hexdump(const uint8_t *data, size_t len);
void app_log_raw(const char *s);   /* without timestamp */

#define LOGI(fmt, ...) app_log("[I] " fmt "\r\n", ##__VA_ARGS__)
#define LOGW(fmt, ...) app_log("[W] " fmt "\r\n", ##__VA_ARGS__)
#define LOGE(fmt, ...) app_log("[E] " fmt "\r\n", ##__VA_ARGS__)

#ifdef __cplusplus
}
#endif

#endif /* APP_LOG_H */
