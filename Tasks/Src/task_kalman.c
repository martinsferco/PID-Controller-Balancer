/**
  ******************************************************************************
  * @file    task_kalman.c
  * @brief   Task del filtro de Kalman (prio 4). Recibe la distancia cruda de
  *          QueuePos, la filtra (2 estados pos/vel) y publica la posicion
  *          estimada en QueuePosFil.
  ******************************************************************************
  */

#include "task_kalman.h"
#include "app.h"
#include "app_config.h"
#include "kalman.h"

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

void KalmanTask(void *argument)
{
  (void)argument;

  Kalman_t kf;
  Kalman_Init(&kf, KALMAN_DT, KALMAN_Q, KALMAN_R, 0.0f);
  int inited = 0;

  for (;;)
  {
    float z = 0.0f;
    if (xQueueReceive(QueuePos, &z, portMAX_DELAY) == pdTRUE)
    {
      if (!inited) { Kalman_Reset(&kf, z); inited = 1; }   /* arrancar en la 1a muestra */
      float pos_fil = Kalman_Update(&kf, z);
      xQueueOverwrite(QueuePosFil, &pos_fil);
    }
  }
}
