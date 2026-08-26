/**
  ******************************************************************************
  * @file    task_pot.h
  * @brief   Task del potenciometro (setpoint): ADC -> QueueObjetivo.
  ******************************************************************************
  */

#ifndef TASK_POT_H
#define TASK_POT_H

#ifdef __cplusplus
extern "C" {
#endif

#include "potentiometer.h"
#include "FreeRTOS.h"
#include "queue.h"

/** @brief Todo lo que usa PotTask (handle ya configurado + IPC). */
typedef struct {
    Potentiometer_HandleTypeDef *pot;             /* pote ya inicializado     */
    QueueHandle_t                queue_objetivo;  /* salida: setpoint          */
} TaskPotContext;

void PotTask(void *argument);

#ifdef __cplusplus
}
#endif

#endif /* TASK_POT_H */
