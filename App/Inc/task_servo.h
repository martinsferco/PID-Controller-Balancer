/**
  ******************************************************************************
  * @file    task_servo.h
  * @brief   Task de FreeRTOS de prueba del servo: barrido ciclico entre los dos
  *          extremos. Placeholder hasta que el PID gobierne el angulo.
  ******************************************************************************
  */

#ifndef TASK_SERVO_H
#define TASK_SERVO_H

#ifdef __cplusplus
extern "C" {
#endif

/** @brief Cuerpo de la task del servo. Se pasa a xTaskCreate. */
void ServoTask(void *argument);

#ifdef __cplusplus
}
#endif

#endif /* TASK_SERVO_H */
