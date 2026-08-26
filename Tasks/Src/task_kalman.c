/**
  ******************************************************************************
  * @file    task_kalman.c
  * @brief   Task del filtro de Kalman (prio 4). Recibe la distancia cruda de
  *          queue_pos, la filtra (2 estados: posicion y velocidad) y publica
  *          AMBOS estados en queue_pos_fil.
  *
  *          La velocidad no es un extra: es la entrada del termino derivativo
  *          del PID. Estimarla en el filtro, y no restando dos posiciones en el
  *          PID, es lo que permite usar un KD util: la cuantizacion del HC-SR04
  *          (escalones de ~0.35 cm) convertida en velocidad por diferencia
  *          finita da varios cm/s de ruido, suficiente para hacer temblar al
  *          servo.
  *
  *          El filtro ya viene creado e inicializado desde App_Init; lo unico
  *          que depende de runtime, y por eso queda aca, es el Kalman_Reset con
  *          la primera muestra real.
  ******************************************************************************
  */

#include "task_kalman.h"
#include "app.h"          /* PosFil_t */

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
    if (xQueueReceive(context->queue_pos, &z, portMAX_DELAY) == pdTRUE)
    {
      /* Arrancar el filtro en la primera muestra real, no en 0 cm. */
      if (!inicializado) { Kalman_Reset(context->kalman, z); inicializado = 1; }

      PosFil_t est;
      est.pos = Kalman_Update(context->kalman, z);        /* posicion estimada */
      est.vel = Kalman_GetVelocity(context->kalman);      /* vel del MISMO update */
      xQueueOverwrite(context->queue_pos_fil, &est);
    }
  }
}
