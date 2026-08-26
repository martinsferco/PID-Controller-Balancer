/**
  ******************************************************************************
  * @file    task_pid.h
  * @brief   Task del control PID: QueueSet{QueuePosFil, QueueObjetivo} -> QueueAngulo.
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

/** @brief Todo lo que usa PidTask (handle ya configurado + IPC). */
typedef struct {
    PID_HandleTypeDef *pid;             /* controlador ya configurado         */
    QueueSetHandle_t   queue_set;       /* bloquea en pos_fil + objetivo      */
    QueueHandle_t      queue_pos_fil;   /* entrada: pos + vel estimadas       */
    QueueHandle_t      queue_objetivo;  /* entrada: setpoint (pote)           */
    QueueHandle_t      queue_angulo;    /* salida: angulo para el servo       */
} TaskPidContext;

void PidTask(void *argument);

#ifdef __cplusplus
}
#endif

#endif /* TASK_PID_H */
