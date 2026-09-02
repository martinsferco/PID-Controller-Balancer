/**
  ******************************************************************************
  * @file    servo_mg90s.c
  * @brief   Implementacion del driver del servo MG90S por PWM (HAL).
  *
  *  Con el timer configurado a 1 us/tick y ARR=19999 (periodo 20 ms = 50 Hz), el
  *  registro de comparacion (CCR) expresado en cuentas equivale directamente al
  *  ancho del pulso en microsegundos. Por eso escribir el CCR es escribir us.
  *
  *  Los us viven aca adentro: la interfaz habla en grados 0..180 y la aplicacion
  *  solo declara cuanto de ese recorrido se permite usar.
  ******************************************************************************
  */

#include "servo_mg90s.h"

/* ============================ Internos ==================================== */

/* Definicion del struct opaco (oculta a los usuarios del .h). */
struct Servo {
    TIM_HandleTypeDef *htim;     /* timer en modo PWM                         */
    uint32_t           channel;  /* TIM_CHANNEL_1..4                          */
    float              min_deg;  /* recorrido permitido: piso (ver SetTravel) */
    float              max_deg;  /* recorrido permitido: techo                */
};

/* Pool estatico de handles: el driver es dueno de la memoria (sin malloc). */
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

/* La recta del componente. Cambia solo si se cambia de servo. 500 y 2500 us son
 * los extremos FISICOS del MG90S: ahi el horn ya esta contra su tope interno, y
 * sostenerlo ahi lo quema. Por eso son a la vez los extremos de la escala
 * angular y el limite duro que el driver nunca cruza. */
#define SERVO_MIN_US        500u
#define SERVO_MAX_US        2500u
#define SERVO_MIN_ANGLE     0.0f
#define SERVO_MAX_ANGLE     180.0f

/* us por grado (11.111 en un MG90S). */
#define SERVO_US_PER_DEG    (((float)(SERVO_MAX_US - SERVO_MIN_US)) / \
                             (SERVO_MAX_ANGLE - SERVO_MIN_ANGLE))

/* Aplica la recta del componente: grados -> us. El recorte final a
 * [SERVO_MIN_US, SERVO_MAX_US] es la red ultima: con un angulo ya saturado al
 * recorrido permitido no actua nunca, pero garantiza que por este driver no
 * salga un pulso capaz de forzar el tope interno del servo. */
static uint16_t servo_us_from_deg(float deg)
{
    float us_f = (float)SERVO_MIN_US + (deg - SERVO_MIN_ANGLE) * SERVO_US_PER_DEG;

    if (us_f < (float)SERVO_MIN_US) { return (uint16_t)SERVO_MIN_US; }
    if (us_f > (float)SERVO_MAX_US) { return (uint16_t)SERVO_MAX_US; }
    return (uint16_t)(us_f + 0.5f);
}

/* Limita un angulo al recorrido permitido (SetTravel garantiza min < max). */
static float servo_limit_deg(const Servo_HandleTypeDef *s, float deg)
{
    if (deg < s->min_deg) { return s->min_deg; }
    if (deg > s->max_deg) { return s->max_deg; }
    return deg;
}

/* ============================== API ======================================= */

Servo_Status Servo_Init(Servo_HandleTypeDef *s,
                        TimerChannel_t pwm)
{
    if (s == NULL || pwm.htim == NULL) {
        return SERVO_ERROR;
    }

    /* Se desarma el par: adentro el driver usa htim/channel sueltos. */
    s->htim    = pwm.htim;
    s->channel = pwm.channel;
    s->min_deg = SERVO_MIN_ANGLE;   /* sin guarda hasta que la declare la app */
    s->max_deg = SERVO_MAX_ANGLE;

    if (HAL_TIM_PWM_Start(s->htim, s->channel) != HAL_OK) {
        return SERVO_ERROR;
    }

    /* Arrancar en el centro: la posicion segura mientras no haya guarda. */
    return Servo_SetAngle(s, (SERVO_MIN_ANGLE + SERVO_MAX_ANGLE) * 0.5f);
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
