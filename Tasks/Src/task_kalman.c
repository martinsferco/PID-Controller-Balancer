/**
  ******************************************************************************
  * @file    task_kalman.c
  * @brief   Task del filtro de Kalman (prio 4). Recibe la distancia cruda de
  *          QueuePos, la filtra (2 estados pos/vel) y publica AMBOS estados en
  *          QueuePosFil.
  *
  *          La velocidad no es un extra: es la entrada del termino derivativo
  *          del PID. Antes se descartaba y el PID se fabricaba su propia
  *          velocidad por diferencia finita de la posicion, que con la
  *          cuantizacion real del HC-SR04 (escalones de ~0.35 cm) daba ~3.5 cm/s
  *          de ruido -- suficiente para que cualquier KD util hiciera temblar al
  *          servo. La del filtro sale de la misma informacion pero pesada por Q
  *          y R, y ademas es consistente con la posicion que la acompana.
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

      PosFil_t est;
      est.pos = Kalman_Update(&kf, z);   /* x0 (lo que devuelve el update) */
      est.vel = kf.x1;                   /* x1 del MISMO update            */
      xQueueOverwrite(QueuePosFil, &est);
    }
  }
}
