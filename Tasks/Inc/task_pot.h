/**
  ******************************************************************************
  * @file    task_pot.h
  * @brief   Task del potenciometro: lee el pote periodicamente y publica el
  *          setpoint (en cm) directo en *setpoint, sin cola.
  ******************************************************************************
  */

#ifndef TASK_POT_H
#define TASK_POT_H

#ifdef __cplusplus
extern "C" {
#endif

#include "potentiometer.h"
#include "FreeRTOS.h"

typedef struct {
    Potentiometer_HandleTypeDef *pot;      // potenciometro ya inicializado
    volatile float              *setpoint; // salida: setpoint vigente, lo lee PidTask directo
} TaskPotContext;

void PotTask(void *argument);

#ifdef __cplusplus
}
#endif

#endif // TASK_POT_H
