/**
  ******************************************************************************
  * @file    debug_uart.c
  * @brief   Traza de diagnostico por USART2. Un mutex estatico serializa el
  *          acceso al buffer y al UART entre las tasks del lazo.
  ******************************************************************************
  */

#include "debug_uart.h"
#include "usart.h"

#include "FreeRTOS.h"
#include "semphr.h"

#include <stdarg.h>
#include <stdio.h>

#define DEBUG_UART_TX_TIMEOUT_MS  50u
#define DEBUG_UART_BUF_LEN        160u

static SemaphoreHandle_t   s_mutex;
static StaticSemaphore_t   s_mutex_cb;
static char                s_buf[DEBUG_UART_BUF_LEN];

void DebugUart_Init(void)
{
  s_mutex = xSemaphoreCreateMutexStatic(&s_mutex_cb);
}

void DebugUart_Print(const char *fmt, ...)
{
  if (s_mutex == NULL) { return; }
  if (xSemaphoreTake(s_mutex, pdMS_TO_TICKS(DEBUG_UART_TX_TIMEOUT_MS)) != pdTRUE) { return; }

  va_list args;
  va_start(args, fmt);
  int len = vsnprintf(s_buf, sizeof(s_buf), fmt, args);
  va_end(args);

  if (len > 0)
  {
    if ((size_t)len >= sizeof(s_buf)) { len = (int)sizeof(s_buf) - 1; }
    HAL_UART_Transmit(&huart2, (uint8_t *)s_buf, (uint16_t)len, DEBUG_UART_TX_TIMEOUT_MS);
  }

  xSemaphoreGive(s_mutex);
}
