/**
  ******************************************************************************
  * @file    task_motor.h
  * @brief   Task del actuador (servo): QueueAngulo -> PWM.
  ******************************************************************************
  */

#ifndef TASK_MOTOR_H
#define TASK_MOTOR_H

#ifdef __cplusplus
extern "C" {
#endif

void MotorTask(void *argument);

#ifdef __cplusplus
}
#endif

#endif /* TASK_MOTOR_H */
