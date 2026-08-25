/**
  ******************************************************************************
  * @file    task_sensor.h
  * @brief   Task del sensor HC-SR04: dispara la medicion con el tick de 100 ms,
  *          espera el aviso de la ISR y publica la distancia en QueuePos.
  ******************************************************************************
  */

#ifndef TASK_SENSOR_H
#define TASK_SENSOR_H

#ifdef __cplusplus
extern "C" {
#endif

/** @brief Cuerpo de la task del sensor. Se pasa a xTaskCreate. */
void SensorTask(void *argument);

#ifdef __cplusplus
}
#endif

#endif /* TASK_SENSOR_H */
