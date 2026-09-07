/**
  ******************************************************************************
  * @file    debug_uart.h
  * @brief   Traza de diagnostico por USART2 (115200 8N1, COM virtual del
  *          ST-Link -> COM3 en Windows tipicamente). Rama debug: siempre
  *          activa, sin flags.
  ******************************************************************************
  */

#ifndef DEBUG_UART_H
#define DEBUG_UART_H

#ifdef __cplusplus
extern "C" {
#endif

/** @brief Crea el mutex que serializa el acceso al UART entre tasks. Llamar una vez, antes de crear las tasks. */
void DebugUart_Init(void);

/** @brief printf a USART2, serializado entre tasks. Agregar el propio \r\n en fmt. */
void DebugUart_Print(const char *fmt, ...);

#ifdef __cplusplus
}
#endif

#endif // DEBUG_UART_H
