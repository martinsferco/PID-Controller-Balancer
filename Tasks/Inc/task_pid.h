/**
  ******************************************************************************
  * @file    task_pid.h
  * @brief   Task del control PID: con cada posicion nueva, calcula la accion
  *          de control contra el setpoint vigente y publica el angulo
  *          resultante (IPC en TaskPidContext).
  ******************************************************************************
  */

#ifndef TASK_PID_H
#define TASK_PID_H

#ifdef __cplusplus
extern "C" {
#endif

#include "pid.h"
#include "FreeRTOS.h"
#include "queue.h"

typedef struct {
    PID_HandleTypeDef *pid;             // controlador ya configurado
    QueueHandle_t      queue_pos_fil;   // entrada: pos + vel estimadas
    volatile float    *setpoint;        // entrada: setpoint vigente, sin cola (lo escribe PotTask)
    QueueHandle_t      queue_angulo;    // salida: angulo para el servo
} TaskPidContext;

void PidTask(void *argument);

#ifdef __cplusplus
}
#endif

#endif // TASK_PID_H
