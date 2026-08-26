/**
  ******************************************************************************
  * @file    potentiometer.h
  * @brief   Driver HAL para leer un potenciometro lineal por ADC (polling) y
  *          mapear la lectura a una posicion en cm. Pensado para baja frecuencia
  *          (ej. cada 200 ms). RTOS-agnostico.
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

typedef struct {
    ADC_HandleTypeDef *hadc;        /* ADC ya inicializado por CubeMX        */
    uint32_t           full_scale;  /* valor maximo del ADC (def 4095 = 12b) */
    float              min_cm;      /* cm en el extremo bajo del pote (def 0) */
    float              max_cm;      /* cm en el extremo alto del pote (def 100)*/
    uint32_t           timeout_ms;  /* timeout del poll (def 10 ms)          */
} Potentiometer_HandleTypeDef;

/**
  * @brief  Inicializa el potenciometro. El rango de salida arranca en un default
  *         (0..100 cm); ajustalo con Potentiometer_SetRange.
  * @param  hadc  ADC configurado en CubeMX (ej. &hadc1)
  */
Potentiometer_Status Potentiometer_Init(Potentiometer_HandleTypeDef *p,
                                        ADC_HandleTypeDef *hadc);

/**
  * @brief  Define el rango de cm que devuelve ReadPosition: el extremo bajo del
  *         pote mapea a min_cm y el alto a max_cm. Requiere min_cm < max_cm.
  */
Potentiometer_Status Potentiometer_SetRange(Potentiometer_HandleTypeDef *p,
                                            float min_cm, float max_cm);

/**
  * @brief  Lee y mapea la posicion a cm en [min_cm, max_cm].
  * @param  out_cm (salida) posicion en centimetros
  */
Potentiometer_Status Potentiometer_ReadPosition_cm(Potentiometer_HandleTypeDef *p, float *out_cm);

#ifdef __cplusplus
}
#endif

#endif /* POTENTIOMETER_H */
