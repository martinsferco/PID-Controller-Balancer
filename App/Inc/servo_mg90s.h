/**
  ******************************************************************************
  * @file    servo_mg90s.h
  * @brief   Driver HAL para servo MG90S (y similares de hobby) por PWM.
  *
  *          Controla el ancho de pulso de una senal PWM de 50 Hz (periodo 20 ms).
  *          El angulo del servo es proporcional al ancho del pulso, tipicamente:
  *            ~1000 us  -> extremo (0 grados)
  *            ~1500 us  -> centro
  *            ~2000 us  -> extremo opuesto (180 grados)
  *          Estos limites son calibrables con Servo_SetLimits().
  *
  *          RTOS-agnostico: solo usa HAL. Multi-instancia (varios servos en
  *          distintos canales/timers).
  *
  *          Requisitos de CubeMX (ver Manual_CubeMX.md):
  *            - TIM en PWM Generation, Prescaler=83 (1 us/tick), ARR=19999
  *              (20 ms -> 50 Hz). Asi el valor del CCR en cuentas == us.
  ******************************************************************************
  */

#ifndef SERVO_MG90S_H
#define SERVO_MG90S_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include <stdint.h>

typedef enum {
    SERVO_OK = 0,
    SERVO_ERROR
} Servo_Status;

typedef struct {
    TIM_HandleTypeDef *htim;     /* timer en modo PWM            */
    uint32_t           channel;  /* TIM_CHANNEL_1..4             */
    uint16_t           min_us;   /* ancho de pulso minimo (def 1000) */
    uint16_t           max_us;   /* ancho de pulso maximo (def 2000) */
    float              min_deg;  /* angulo en min_us (def 0)     */
    float              max_deg;  /* angulo en max_us (def 180)   */
    uint16_t           last_us;  /* ultimo pulso aplicado [us]   */
} Servo_HandleTypeDef;

/**
  * @brief  Inicializa el servo y arranca la generacion PWM.
  *         Deja el servo en la posicion central por defecto.
  * @param  htim    timer ya inicializado por CubeMX en PWM
  * @param  channel TIM_CHANNEL_1..4
  */
Servo_Status Servo_Init(Servo_HandleTypeDef *s,
                        TIM_HandleTypeDef *htim,
                        uint32_t channel);

/**
  * @brief  Calibra los limites de pulso (us) y el rango angular (grados).
  *         Ej: Servo_SetLimits(&s, 500, 2500, 0.0f, 180.0f);
  */
Servo_Status Servo_SetLimits(Servo_HandleTypeDef *s,
                             uint16_t min_us, uint16_t max_us,
                             float min_deg, float max_deg);

/**
  * @brief  Fija el ancho de pulso directamente (us). Se clampea a [min_us,max_us].
  */
Servo_Status Servo_SetPulseUs(Servo_HandleTypeDef *s, uint16_t us);

/**
  * @brief  Fija el angulo (grados). Se mapea linealmente a us y se clampea.
  */
Servo_Status Servo_SetAngle(Servo_HandleTypeDef *s, float deg);

/**
  * @brief  Devuelve el ultimo ancho de pulso aplicado en us.
  */
uint16_t Servo_GetPulseUs(const Servo_HandleTypeDef *s);

#ifdef __cplusplus
}
#endif

#endif /* SERVO_MG90S_H */
