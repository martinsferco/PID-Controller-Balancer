/**
  ******************************************************************************
  * @file    servo_mg90s.h
  * @brief   Driver HAL para servo MG90S por PWM. RTOS-agnostico y multi-instancia.
  *
  *          Requiere TIM en PWM Generation a 1 us/tick con ARR=19999 (50 Hz).
  ******************************************************************************
  */

#ifndef SERVO_MG90S_H
#define SERVO_MG90S_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include "bsp_types.h"
#include <stdint.h>

typedef enum {
    SERVO_OK = 0,
    SERVO_ERROR
} Servo_Status;

typedef struct Servo Servo_HandleTypeDef;

/**
  * @brief  Reserva un handle de un pool estatico interno. Devuelve
  *         NULL si el pool esta agotado.
  */
Servo_HandleTypeDef *Servo_Create(void);

/**
  * @brief  Inicializa el servo y arranca la generacion PWM.
  * @param  pwm  timer y canal en modo PWM
  */
Servo_Status Servo_Init(Servo_HandleTypeDef *s,
                        TimerChannel_t pwm);

/**
  * @brief  Declara el recorrido PERMITIDO, en grados, dentro del rango permitido.
  * @retval SERVO_ERROR si max_deg <= min_deg o si alguno cae fuera de 0..180;
  *         en ese caso queda el recorrido anterior.
  */
Servo_Status Servo_SetTravel(Servo_HandleTypeDef *s,
                             float min_deg, float max_deg);

/**
  * @brief  Fija el angulo del servo en grados.
  */
Servo_Status Servo_SetAngle(Servo_HandleTypeDef *s, float deg);

#ifdef __cplusplus
}
#endif

#endif // SERVO_MG90S_H
