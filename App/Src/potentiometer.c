/**
  ******************************************************************************
  * @file    potentiometer.c
  * @brief   Implementacion del driver del potenciometro por ADC (polling, HAL).
  ******************************************************************************
  */

#include "potentiometer.h"

Pot_Status Pot_Init(Pot_HandleTypeDef *p,
                    ADC_HandleTypeDef *hadc,
                    float range_cm)
{
    if (p == NULL || hadc == NULL || range_cm <= 0.0f) {
        return POT_ERROR;
    }
    p->hadc       = hadc;
    p->full_scale = 4095u;   /* 12 bits; ajustar si cambiar la resolucion */
    p->range_cm   = range_cm;
    p->timeout_ms = 10u;
    return POT_OK;
}

Pot_Status Pot_ReadRaw(Pot_HandleTypeDef *p, uint32_t *out_raw)
{
    if (p == NULL || out_raw == NULL) {
        return POT_ERROR;
    }

    if (HAL_ADC_Start(p->hadc) != HAL_OK) {
        return POT_ERROR;
    }

    HAL_StatusTypeDef st = HAL_ADC_PollForConversion(p->hadc, p->timeout_ms);
    if (st != HAL_OK) {
        HAL_ADC_Stop(p->hadc);
        return (st == HAL_TIMEOUT) ? POT_TIMEOUT : POT_ERROR;
    }

    *out_raw = HAL_ADC_GetValue(p->hadc);
    HAL_ADC_Stop(p->hadc);
    return POT_OK;
}

Pot_Status Pot_ReadPosition_cm(Pot_HandleTypeDef *p, float *out_cm)
{
    if (p == NULL || out_cm == NULL) {
        return POT_ERROR;
    }

    uint32_t raw = 0;
    Pot_Status s = Pot_ReadRaw(p, &raw);
    if (s != POT_OK) {
        return s;
    }

    if (raw > p->full_scale) {
        raw = p->full_scale;   /* seguridad */
    }

    *out_cm = ((float)raw / (float)p->full_scale) * p->range_cm;
    return POT_OK;
}
