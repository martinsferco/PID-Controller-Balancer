/**
  ******************************************************************************
  * @file    bsp_types.h
  * @brief   Tipos de par comunes a los ComponentDrivers: agrupan valores que
  *          siempre viajan juntos (un timer con su canal, un puerto con su pin).
  *          Se pasan por valor (8 bytes) en las firmas de los Init; adentro cada
  *          driver los desarma y guarda los campos por separado.
  ******************************************************************************
  */

#ifndef BSP_TYPES_H
#define BSP_TYPES_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include <stdint.h>

/** @brief Un timer con el canal que se usa dentro de el (Input Capture, PWM). */
typedef struct {
    TIM_HandleTypeDef *htim;     /* timer ya inicializado por CubeMX          */
    uint32_t           channel;  /* TIM_CHANNEL_1..4                          */
} TimerChannel_t;

/** @brief Un pin GPIO: su puerto y su numero de pin. */
typedef struct {
    GPIO_TypeDef *port;          /* puerto del pin                            */
    uint16_t      pin;           /* mascara del pin                           */
} GpioPin_t;

#ifdef __cplusplus
}
#endif

#endif /* BSP_TYPES_H */
