/**
  ******************************************************************************
  * @file    servo_mg90s.c
  * @brief   Implementacion del driver del servo MG90S por PWM (HAL).
  ******************************************************************************
  */

#include "servo_mg90s.h"

// Definicion del struct opaco
struct Servo {
    TIM_HandleTypeDef *htim;     // timer en modo PWM
    uint32_t           channel;  // TIM_CHANNEL_1..4
    float              min_deg;  // recorrido permitido: piso
    float              max_deg;  // recorrido permitido: techo
};

// Pool estatico de handles
#ifndef SERVO_MAX_INSTANCES
#define SERVO_MAX_INSTANCES  1
#endif

static struct Servo s_pool[SERVO_MAX_INSTANCES];
static unsigned     s_pool_count = 0u;

Servo_HandleTypeDef *Servo_Create(void)
{
    if (s_pool_count >= SERVO_MAX_INSTANCES) { return NULL; }
    return &s_pool[s_pool_count++];
}

// Extremos fisicos del MG90S
#define SERVO_MIN_US        500u
#define SERVO_MAX_US        2500u
#define SERVO_MIN_ANGLE     0.0f
#define SERVO_MAX_ANGLE     180.0f

#define SERVO_US_PER_DEG    (((float)(SERVO_MAX_US - SERVO_MIN_US)) / \
                             (SERVO_MAX_ANGLE - SERVO_MIN_ANGLE))

static uint16_t servo_us_from_deg(float deg)
{
    float us_f = (float)SERVO_MIN_US + (deg - SERVO_MIN_ANGLE) * SERVO_US_PER_DEG;

    if (us_f < (float)SERVO_MIN_US) { return (uint16_t)SERVO_MIN_US; }
    if (us_f > (float)SERVO_MAX_US) { return (uint16_t)SERVO_MAX_US; }
    return (uint16_t)(us_f + 0.5f);
}

static float servo_limit_deg(const Servo_HandleTypeDef *s, float deg)
{
    if (deg < s->min_deg) { return s->min_deg; }
    if (deg > s->max_deg) { return s->max_deg; }
    return deg;
}

Servo_Status Servo_Init(Servo_HandleTypeDef *s,
                        TimerChannel_t pwm)
{
    if (s == NULL || pwm.htim == NULL) {
        return SERVO_ERROR;
    }

    s->htim    = pwm.htim;
    s->channel = pwm.channel;
    s->min_deg = SERVO_MIN_ANGLE;   // sin guarda hasta que la declare la app
    s->max_deg = SERVO_MAX_ANGLE;

    if (HAL_TIM_PWM_Start(s->htim, s->channel) != HAL_OK) {
        return SERVO_ERROR;
    }

    return Servo_SetAngle(s, (SERVO_MIN_ANGLE + SERVO_MAX_ANGLE) * 0.5f);   // centro: posicion segura
}

Servo_Status Servo_SetTravel(Servo_HandleTypeDef *s,
                             float min_deg, float max_deg)
{
    if (s == NULL || max_deg <= min_deg) {
        return SERVO_ERROR;
    }
    if (min_deg < SERVO_MIN_ANGLE || max_deg > SERVO_MAX_ANGLE) {
        return SERVO_ERROR;
    }

    s->min_deg = min_deg;
    s->max_deg = max_deg;
    return SERVO_OK;
}

Servo_Status Servo_SetAngle(Servo_HandleTypeDef *s, float deg)
{
    if (s == NULL) {
        return SERVO_ERROR;
    }

    __HAL_TIM_SET_COMPARE(s->htim, s->channel,
                          servo_us_from_deg(servo_limit_deg(s, deg)));
    return SERVO_OK;
}
