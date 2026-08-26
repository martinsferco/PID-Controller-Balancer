/**
  ******************************************************************************
  * @file    potentiometer.c
  * @brief   Implementacion del driver del potenciometro por ADC (polling, HAL).
  ******************************************************************************
  */

#include "potentiometer.h"

/* Definicion del struct opaco (oculta a los usuarios del .h). */
struct Potentiometer {
    ADC_HandleTypeDef *hadc;        /* ADC ya inicializado por CubeMX          */
    uint32_t           full_scale;  /* valor maximo del ADC (def 4095 = 12b)   */
    uint32_t           timeout_ms;  /* timeout del poll (def 10 ms)            */
};

/* Defaults de Init. */
#define POTENTIOMETER_DEFAULT_FULL_SCALE  4095u   /* ADC 12 bits */
#define POTENTIOMETER_DEFAULT_TIMEOUT_MS  10u

/* Pool estatico de handles: el driver es dueno de la memoria (sin malloc). */
#ifndef POTENTIOMETER_MAX_INSTANCES
#define POTENTIOMETER_MAX_INSTANCES  1
#endif

static struct Potentiometer s_pool[POTENTIOMETER_MAX_INSTANCES];
static unsigned             s_pool_count = 0u;

Potentiometer_HandleTypeDef *Potentiometer_Create(void)
{
    if (s_pool_count >= POTENTIOMETER_MAX_INSTANCES) { return 0; }
    return &s_pool[s_pool_count++];
}

Potentiometer_Status Potentiometer_Init(Potentiometer_HandleTypeDef *p,
                                        ADC_HandleTypeDef *hadc)
{
    if (p == NULL || hadc == NULL) {
        return POTENTIOMETER_ERROR;
    }
    p->hadc       = hadc;
    p->full_scale = POTENTIOMETER_DEFAULT_FULL_SCALE;
    p->timeout_ms = POTENTIOMETER_DEFAULT_TIMEOUT_MS;
    return POTENTIOMETER_OK;
}

/* Interno: lee el valor crudo del ADC (0..full_scale) por polling. */
static Potentiometer_Status potentiometer_read_raw(Potentiometer_HandleTypeDef *p, uint32_t *out_raw)
{
    if (p == NULL || out_raw == NULL) {
        return POTENTIOMETER_ERROR;
    }

    if (HAL_ADC_Start(p->hadc) != HAL_OK) {
        return POTENTIOMETER_ERROR;
    }

    HAL_StatusTypeDef st = HAL_ADC_PollForConversion(p->hadc, p->timeout_ms);
    if (st != HAL_OK) {
        HAL_ADC_Stop(p->hadc);
        return (st == HAL_TIMEOUT) ? POTENTIOMETER_TIMEOUT : POTENTIOMETER_ERROR;
    }

    *out_raw = HAL_ADC_GetValue(p->hadc);
    HAL_ADC_Stop(p->hadc);
    return POTENTIOMETER_OK;
}

Potentiometer_Status Potentiometer_ReadNormalized(Potentiometer_HandleTypeDef *p, float *out)
{
    if (p == NULL || out == NULL) {
        return POTENTIOMETER_ERROR;
    }

    uint32_t raw = 0;
    Potentiometer_Status s = potentiometer_read_raw(p, &raw);
    if (s != POTENTIOMETER_OK) {
        return s;
    }

    if (raw > p->full_scale) {
        raw = p->full_scale;   /* seguridad */
    }

    *out = (float)raw / (float)p->full_scale;
    return POTENTIOMETER_OK;
}
