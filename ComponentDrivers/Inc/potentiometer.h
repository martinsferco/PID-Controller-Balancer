/**
  ******************************************************************************
  * @file    potentiometer.h
  * @brief   Driver HAL para leer un potenciometro lineal por ADC (polling).
  *          SOLO lee: devuelve la posicion NORMALIZADA en 0.0..1.0, sin saber a
  *          que magnitud fisica corresponde. Convertir ese valor a algo util
  *          (cm, corriente, etc.) es responsabilidad del llamador (ver el modulo
  *          linear_map). Pensado para baja frecuencia (ej. cada 200 ms).
  *          RTOS-agnostico.
  *
  *          El struct es opaco (definido en potentiometer.c): el handle se pide
  *          con Potentiometer_Create() y se opera por la interfaz.
  *
  *          Requisitos de CubeMX:
  *            - ADC con el canal del potenciometro habilitado, 12 bits,
  *              Continuous Conversion = Disabled, sampling time alto.
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
  * @brief  Reserva un handle de un pool estatico interno (sin malloc). Devuelve
  *         NULL si el pool esta agotado. No hay Destroy.
  */
Potentiometer_HandleTypeDef *Potentiometer_Create(void);

/**
  * @brief  Inicializa el potenciometro.
  * @param  hadc  ADC configurado en CubeMX (ej. &hadc1)
  */
Potentiometer_Status Potentiometer_Init(Potentiometer_HandleTypeDef *p,
                                        ADC_HandleTypeDef *hadc);

/**
  * @brief  Lee el pote y devuelve la posicion normalizada en [0.0, 1.0]
  *         (raw / full_scale). Independiza al llamador de la resolucion del ADC.
  * @param  out (salida) posicion normalizada 0.0..1.0
  */
Potentiometer_Status Potentiometer_ReadNormalized(Potentiometer_HandleTypeDef *p, float *out);

#ifdef __cplusplus
}
#endif

#endif /* POTENTIOMETER_H */
