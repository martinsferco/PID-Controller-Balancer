/**
  ******************************************************************************
  * @file    potentiometer.c
  * @brief   Implementacion del driver del potenciometro por ADC (polling, HAL).
  ******************************************************************************
  */

#include "potentiometer.h"

/* Defaults de Init (el rango de salida es overridable con Potentiometer_SetRange). */
#define POTENTIOMETER_DEFAULT_FULL_SCALE  4095u   /* ADC 12 bits */
#define POTENTIOMETER_DEFAULT_TIMEOUT_MS  10u
#define POTENTIOMETER_DEFAULT_MIN_CM      0.0f
#define POTENTIOMETER_DEFAULT_MAX_CM      100.0f

Potentiometer_Status Potentiometer_Init(Potentiometer_HandleTypeDef *p,
                                        ADC_HandleTypeDef *hadc)
{
    if (p == NULL || hadc == NULL) {
        return POTENTIOMETER_ERROR;
    }
    p->hadc       = hadc;
    p->full_scale = POTENTIOMETER_DEFAULT_FULL_SCALE;
    p->min_cm     = POTENTIOMETER_DEFAULT_MIN_CM;
    p->max_cm     = POTENTIOMETER_DEFAULT_MAX_CM;
    p->timeout_ms = POTENTIOMETER_DEFAULT_TIMEOUT_MS;
    return POTENTIOMETER_OK;
}

Potentiometer_Status Potentiometer_SetRange(Potentiometer_HandleTypeDef *p,
                                            float min_cm, float max_cm)
{
    if (p == NULL || max_cm <= min_cm) {
        return POTENTIOMETER_ERROR;
    }
    p->min_cm = min_cm;
    p->max_cm = max_cm;
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

Potentiometer_Status Potentiometer_ReadPosition_cm(Potentiometer_HandleTypeDef *p, float *out_cm)
{
    if (p == NULL || out_cm == NULL) {
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

    *out_cm = p->min_cm + ((float)raw / (float)p->full_scale) * (p->max_cm - p->min_cm);
    return POTENTIOMETER_OK;
}
