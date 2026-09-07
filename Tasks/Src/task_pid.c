/**
  ******************************************************************************
  * @file    task_pid.c
  * @brief   Task del control PID: con cada posicion nueva, calcula la accion
  *          de control contra el setpoint vigente (context->setpoint, sin
  *          lock) y publica el angulo resultante.
  ******************************************************************************
  */

#include "task_pid.h"
#include "app.h"          // PosFil_t
#include "app_config.h"

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

void PidTask(void *argument)
{
  TaskPidContext *context = (TaskPidContext *)argument;

  // Estado estimado inicial
  PosFil_t est = { SETPOINT_DEFAULT_CM, 0.0f };

  for (;;)
  {
    if (xQueueReceive(context->queue_pos_fil, &est, pdMS_TO_TICKS(PID_TASK_TIMEOUT_MS)) == pdTRUE)
    {
      float u     = PID_ComputeRate(context->pid, *context->setpoint, est.pos, est.vel);
      float angle = SERVO_CENTER_DEG + (SERVO_DIR * u);
      xQueueOverwrite(context->queue_angulo, &angle);
    }
  }
}
