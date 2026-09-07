/**
  ******************************************************************************
  * @file    task_kalman.h
  * @brief   Task del filtro de Kalman: filtra la distancia cruda recibida
  *          (posicion + velocidad estimadas) y publica el resultado (IPC en
  *          TaskKalmanContext).
  ******************************************************************************
  */

#ifndef TASK_KALMAN_H
#define TASK_KALMAN_H

#ifdef __cplusplus
extern "C" {
#endif

#include "kalman.h"
#include "FreeRTOS.h"
#include "queue.h"

typedef struct {
    Kalman_HandleTypeDef *kalman;         // filtro ya inicializado
    QueueHandle_t         queue_pos;      // entrada: distancia cruda
    QueueHandle_t         queue_pos_fil;  // salida: pos + vel estimadas
} TaskKalmanContext;

void KalmanTask(void *argument);

#ifdef __cplusplus
}
#endif

#endif // TASK_KALMAN_H
