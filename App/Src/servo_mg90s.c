/**
  ******************************************************************************
  * @file    servo_mg90s.c
  * @brief   Implementacion del driver del servo MG90S por PWM (HAL).
  *
  *  Con el timer configurado a 1 us/tick y ARR=19999 (periodo 20 ms = 50 Hz),
  *  el registro de comparacion (CCR) expresado en cuentas equivale directamente
  *  al ancho del pulso en microsegundos. Por eso Servo_SetPulseUs() escribe el
  *  valor en us directamente en el CCR con __HAL_TIM_SET_COMPARE().
  ******************************************************************************
  */

#include "servo_mg90s.h"

static uint16_t servo_clamp_u16(uint16_t v, uint16_t lo, uint16_t hi)
{
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

Servo_Status Servo_Init(Servo_HandleTypeDef *s,
                        TIM_HandleTypeDef *htim,
                        uint32_t channel)
{
    if (s == NULL || htim == NULL) {
        return SERVO_ERROR;
    }

    s->htim    = htim;
    s->channel = channel;
    s->min_us  = 1000;
    s->max_us  = 2000;
    s->min_deg = 0.0f;
    s->max_deg = 180.0f;
    s->last_us = (uint16_t)((s->min_us + s->max_us) / 2u);  /* centro */

    if (HAL_TIM_PWM_Start(s->htim, s->channel) != HAL_OK) {
        return SERVO_ERROR;
    }

    __HAL_TIM_SET_COMPARE(s->htim, s->channel, s->last_us);
    return SERVO_OK;
}

Servo_Status Servo_SetLimits(Servo_HandleTypeDef *s,
                             uint16_t min_us, uint16_t max_us,
                             float min_deg, float max_deg)
{
    if (s == NULL || max_us <= min_us || max_deg == min_deg) {
        return SERVO_ERROR;
    }
    s->min_us  = min_us;
    s->max_us  = max_us;
    s->min_deg = min_deg;
    s->max_deg = max_deg;
    return SERVO_OK;
}

Servo_Status Servo_SetPulseUs(Servo_HandleTypeDef *s, uint16_t us)
{
    if (s == NULL) {
        return SERVO_ERROR;
    }
    us = servo_clamp_u16(us, s->min_us, s->max_us);
    __HAL_TIM_SET_COMPARE(s->htim, s->channel, us);
    s->last_us = us;
    return SERVO_OK;
}

Servo_Status Servo_SetAngle(Servo_HandleTypeDef *s, float deg)
{
    if (s == NULL) {
        return SERVO_ERROR;
    }

    /* Clampear el angulo al rango configurado */
    float lo = (s->min_deg < s->max_deg) ? s->min_deg : s->max_deg;
    float hi = (s->min_deg < s->max_deg) ? s->max_deg : s->min_deg;
    if (deg < lo) deg = lo;
    if (deg > hi) deg = hi;

    /* Mapeo lineal angulo -> us */
    float frac = (deg - s->min_deg) / (s->max_deg - s->min_deg);
    float us_f = (float)s->min_us + frac * ((float)s->max_us - (float)s->min_us);

    return Servo_SetPulseUs(s, (uint16_t)(us_f + 0.5f));
}

uint16_t Servo_GetPulseUs(const Servo_HandleTypeDef *s)
{
    return (s != NULL) ? s->last_us : 0u;
}
