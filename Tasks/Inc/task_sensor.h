/**
  ******************************************************************************
  * @file    task_sensor.h
  * @brief   Task del sensor HC-SR04: dispara la medicion con el tick de 100 ms,
  *          espera el aviso de la ISR y publica la distancia medida (IPC en
  *          TaskSensorContext).
  ******************************************************************************
  */

#ifndef TASK_SENSOR_H
#define TASK_SENSOR_H

#ifdef __cplusplus
extern "C" {
#endif

#include "hc_sr04.h"
#include "FreeRTOS.h"
#include "semphr.h"
#include "queue.h"

typedef struct {
    HC_SR04_HandleTypeDef *sensor;      // sensor ya inicializado
    SemaphoreHandle_t      sem_timer;   // tick de 100 ms (da la ISR TIM4)
    SemaphoreHandle_t      sem_sensor;  // echo listo (da la ISR de captura)
    QueueHandle_t          queue_pos;   // salida: distancia cruda (borde)
} TaskSensorContext;

void SensorTask(void *argument);

#ifdef __cplusplus
}
#endif

#endif // TASK_SENSOR_H
