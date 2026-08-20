/**
  ******************************************************************************
  * @file    potentiometer.h
  * @brief   Driver HAL para leer un potenciometro lineal por ADC (polling).
  *
  *          Lee el canal del ADC por software (sin DMA ni interrupciones) y
  *          mapea el valor crudo (0..resolution) a una posicion fisica en cm.
  *          Pensado para lecturas a baja frecuencia (ej. cada 200 ms).
  *
  *          RTOS-agnostico: solo usa HAL.
  *
  *          Requisitos de CubeMX (ver Manual_CubeMX.md):
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
    POT_OK = 0,
    POT_TIMEOUT,
    POT_ERROR
} Pot_Status;

typedef struct {
    ADC_HandleTypeDef *hadc;        /* ADC ya inicializado por CubeMX        */
    uint32_t           full_scale;  /* valor maximo del ADC (def 4095 = 12b) */
    float              range_cm;    /* largo util de la barra en cm          */
    uint32_t           timeout_ms;  /* timeout del poll (def 10 ms)          */
} Pot_HandleTypeDef;

/**
  * @brief  Inicializa el potenciometro.
  * @param  hadc      ADC configurado en CubeMX (ej. &hadc1)
  * @param  range_cm  largo fisico que representa el recorrido del pote (cm)
  */
Pot_Status Pot_Init(Pot_HandleTypeDef *p,
                    ADC_HandleTypeDef *hadc,
                    float range_cm);

/**
  * @brief  Lee el valor crudo del ADC (0..full_scale) por polling.
  * @param  out_raw (salida) valor convertido
  */
Pot_Status Pot_ReadRaw(Pot_HandleTypeDef *p, uint32_t *out_raw);

/**
  * @brief  Lee y mapea la posicion a cm en [0, range_cm].
  * @param  out_cm (salida) posicion en centimetros
  */
Pot_Status Pot_ReadPosition_cm(Pot_HandleTypeDef *p, float *out_cm);

#ifdef __cplusplus
}
#endif

#endif /* POTENTIOMETER_H */
