/**
  ******************************************************************************
  * @file    task_sensor.h
  * @brief   Task de FreeRTOS del sensor HC-SR04: dispara, espera el aviso de la
  *          ISR, lee la distancia y la reporta por UART.
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
