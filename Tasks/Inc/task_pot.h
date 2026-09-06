/**
  ******************************************************************************
  * @file    task_pot.h
  * @brief   Task del potenciometro (setpoint): ADC -> *setpoint (sin cola).
  ******************************************************************************
  */

#ifndef TASK_POT_H
#define TASK_POT_H

#ifdef __cplusplus
extern "C" {
#endif

#include "potentiometer.h"
#include "FreeRTOS.h"

/** @brief Todo lo que usa PotTask (handle ya configurado + IPC). */
typedef struct {
    Potentiometer_HandleTypeDef *pot;      /* pote ya inicializado                             */
    volatile float              *setpoint; /* salida: setpoint vigente, lo lee PidTask directo */
} TaskPotContext;

void PotTask(void *argument);

#ifdef __cplusplus
}
#endif

#endif /* TASK_POT_H */
