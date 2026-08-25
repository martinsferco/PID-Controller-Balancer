/**
  ******************************************************************************
  * @file    task_kalman.c
  * @brief   Task del filtro de Kalman (prio 4). Recibe la distancia cruda de
  *          QueuePos, la filtra (2 estados: posicion y velocidad) y publica
  *          AMBOS estados en QueuePosFil.
  *
  *          La velocidad no es un extra: es la entrada del termino derivativo
  *          del PID. Estimarla en el filtro, y no restando dos posiciones en el
  *          PID, es lo que permite usar un KD util: la cuantizacion del HC-SR04
  *          (escalones de ~0.35 cm) convertida en velocidad por diferencia
  *          finita da varios cm/s de ruido, suficiente para hacer temblar al
  *          servo.
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
  int inicializado = 0;

  for (;;)
  {
    float z = 0.0f;
    if (xQueueReceive(QueuePos, &z, portMAX_DELAY) == pdTRUE)
    {
      /* Arrancar el filtro en la primera muestra real, no en 0 cm. */
      if (!inicializado) { Kalman_Reset(&kf, z); inicializado = 1; }

      PosFil_t est;
      est.pos = Kalman_Update(&kf, z);   /* x0 del update */
      est.vel = kf.x1;                   /* x1 del MISMO update */
      xQueueOverwrite(QueuePosFil, &est);
    }
  }
}
