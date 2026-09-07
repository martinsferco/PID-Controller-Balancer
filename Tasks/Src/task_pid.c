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
#include "debug_uart.h"

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
      float setpoint = *context->setpoint;
      float error    = setpoint - est.pos;
      float u        = PID_ComputeRate(context->pid, setpoint, est.pos, est.vel);
      float angle    = SERVO_CENTER_DEG + (SERVO_DIR * u);
      xQueueOverwrite(context->queue_angulo, &angle);

      DebugUart_Print("[%10lu] PID setpoint=%.2fcm pos=%.2fcm vel=%.2fcm/s error=%.2fcm u=%.2fdeg angle=%.2fdeg\r\n",
                       (unsigned long)HAL_GetTick(), setpoint, est.pos, est.vel, error, u, angle);
    }
    else
    {
      DebugUart_Print("[%10lu] PID timeout\r\n", (unsigned long)HAL_GetTick());
    }
  }
}
