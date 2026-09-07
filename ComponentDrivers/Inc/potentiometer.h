/**
  ******************************************************************************
  * @file    potentiometer.h
  * @brief   Driver HAL para leer un potenciometro lineal por ADC (polling).
  *          Pensado para baja frecuencia (ej. cada 200 ms). RTOS-agnostico.
  *
  *          Requiere ADC de 12 bits con Continuous Conversion deshabilitado.
  ******************************************************************************
  */

#ifndef POTENTIOMETER_H
#define POTENTIOMETER_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include <stdint.h>

typedef enum {
    POTENTIOMETER_OK = 0,
    POTENTIOMETER_TIMEOUT,
    POTENTIOMETER_ERROR
} Potentiometer_Status;

typedef struct Potentiometer Potentiometer_HandleTypeDef;

/**
  * @brief  Reserva un handle de un pool estatico interno. Devuelve
  *         NULL si el pool esta agotado.
  */
Potentiometer_HandleTypeDef *Potentiometer_Create(void);

/**
  * @brief  Inicializa el potenciometro.
  * @param  hadc  ADC configurado.
  */
Potentiometer_Status Potentiometer_Init(Potentiometer_HandleTypeDef *p,
                                        ADC_HandleTypeDef *hadc);

/**
  * @brief  Lee el pote y devuelve la posicion normalizada.
  * @param  out (salida) posicion normalizada.
  */
Potentiometer_Status Potentiometer_ReadNormalized(Potentiometer_HandleTypeDef *p, float *out);

#ifdef __cplusplus
}
#endif

#endif // POTENTIOMETER_H
