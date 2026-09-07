/**
  ******************************************************************************
  * @file    task_motor.h
  * @brief   Task del actuador: aplica en el servo el angulo recibido.
  *          Failsafe: si deja de llegar angulo nuevo, nivela la barra.
  ******************************************************************************
  */

#ifndef TASK_MOTOR_H
#define TASK_MOTOR_H

#ifdef __cplusplus
extern "C" {
#endif

#include "servo_mg90s.h"
#include "FreeRTOS.h"
#include "queue.h"

typedef struct {
    Servo_HandleTypeDef *servo;         // servo ya inicializado
    QueueHandle_t        queue_angulo;  // entrada: angulo a aplicar
} TaskMotorContext;

void MotorTask(void *argument);

#ifdef __cplusplus
}
#endif

#endif // TASK_MOTOR_H
