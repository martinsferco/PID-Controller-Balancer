/**
  ******************************************************************************
  * @file    task_pid.h
  * @brief   Task del control PID: QueueSet{QueuePosFil, QueueObjetivo} -> QueueAngulo.
  ******************************************************************************
  */

#ifndef TASK_PID_H
#define TASK_PID_H

#ifdef __cplusplus
extern "C" {
#endif

void PidTask(void *argument);

#ifdef __cplusplus
}
#endif

#endif /* TASK_PID_H */
