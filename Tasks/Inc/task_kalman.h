/**
  ******************************************************************************
  * @file    task_kalman.h
  * @brief   Task del filtro de Kalman: QueuePos -> QueuePosFil.
  ******************************************************************************
  */

#ifndef TASK_KALMAN_H
#define TASK_KALMAN_H

#ifdef __cplusplus
extern "C" {
#endif

void KalmanTask(void *argument);

#ifdef __cplusplus
}
#endif

#endif /* TASK_KALMAN_H */
