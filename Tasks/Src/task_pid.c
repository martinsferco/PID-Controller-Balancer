/**
  ******************************************************************************
  * @file    task_pid.c
  * @brief   Task del control PID (prio 3). Bloquea en un queue set sobre
  *          QueuePosFil (estado estimado) y QueueObjetivo (setpoint del pote).
  *          Con cada posicion nueva recalcula la accion de control, la mapea a
  *          un angulo de servo y lo publica en QueueAngulo; con cada setpoint
  *          nuevo solo actualiza la referencia.
  ******************************************************************************
  */

#include "task_pid.h"
#include "app.h"
#include "app_config.h"
#include "pid.h"

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

void PidTask(void *argument)
{
  (void)argument;

  PID_t pid;
  PID_Init(&pid, PID_KP, PID_KI, PID_KD, PID_DT);
  PID_SetLimits(&pid, PID_OUT_MIN, PID_OUT_MAX);
  PID_SetIntegralBand(&pid, PID_I_BAND);

  /* Referencia hasta que PotTask publique la primera lectura del pote. */
  float setpoint = SETPOINT_DEFAULT_CM;

  /* Estado estimado: arranca en el setpoint y quieto, para que la primera
   * accion de control no sea un salto si todavia no llego nada del Kalman. */
  PosFil_t est = { SETPOINT_DEFAULT_CM, 0.0f };

  for (;;)
  {
    QueueSetMemberHandle_t quien = xQueueSelectFromSet(QueueSetPid, portMAX_DELAY);

    /* Se lee con timeout 0: con xQueueOverwrite sobre miembros de un set puede
     * quedar algun aviso sin dato detras. Un setpoint nuevo solo refresca la
     * referencia; el PID se computa una unica vez por posicion, que es lo que
     * respeta el dt fijo del controlador. */
    if (quien == QueueObjetivo)
    {
      (void)xQueueReceive(QueueObjetivo, &setpoint, 0);
    }
    else if (quien == QueuePosFil)
    {
      if (xQueueReceive(QueuePosFil, &est, 0) == pdTRUE)
      {
        /* PID_ComputeRate y no PID_Compute: la velocidad viene del Kalman, que
         * la estima con su modelo de ruido en vez de restar dos posiciones
         * cuantizadas, y sale del mismo update que la posicion. */
        float u     = PID_ComputeRate(&pid, setpoint, est.pos, est.vel);
        float angle = SERVO_CENTER_DEG + (SERVO_DIR * u);
        xQueueOverwrite(QueueAngulo, &angle);
      }
    }
  }
}
