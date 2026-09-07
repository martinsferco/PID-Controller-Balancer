/**
  ******************************************************************************
  * @file    task_kalman.c
  * @brief   Task del filtro de Kalman: recibe la distancia cruda, la filtra
  *          (posicion + velocidad) y publica ambos estados. Se resetea si el
  *          sensor deja de publicar.
  ******************************************************************************
  */

#include "task_kalman.h"
#include "app.h"          // PosFil_t
#include "app_config.h"

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

void KalmanTask(void *argument)
{
  TaskKalmanContext *context = (TaskKalmanContext *)argument;

  int inicializado = 0;

  for (;;)
  {
    float z = 0.0f;
    if (xQueueReceive(context->queue_pos, &z, pdMS_TO_TICKS(KALMAN_TASK_TIMEOUT_MS)) == pdTRUE)
    {
      if (!inicializado) { Kalman_Reset(context->kalman, z); inicializado = 1; }

      PosFil_t est;
      est.pos = Kalman_Update(context->kalman, z);        // posicion estimada
      est.vel = Kalman_GetVelocity(context->kalman);      // vel del update
      xQueueOverwrite(context->queue_pos_fil, &est);
    }
    else
    {
      inicializado = 0;
    }
  }
}
