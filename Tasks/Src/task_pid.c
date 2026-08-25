/**
  ******************************************************************************
  * @file    task_pid.c
  * @brief   Task del control PID (prio 3). Bloquea en un Queue Set sobre
  *          QueuePosFil (posicion filtrada) y QueueObjetivo (setpoint del pote).
  *          Al llegar una posicion nueva recalcula la accion de control, la mapea
  *          a un angulo y la publica en QueueAngulo. Al llegar un setpoint nuevo,
  *          lo actualiza.
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

  /* Setpoint por defecto hasta que PotTask publique el suyo (mitad de la barra). */
  float setpoint = SETPOINT_FIXED_CM;

  /* Estado estimado: arranca en el setpoint y quieto, para que la primera accion
   * de control no sea un salto si todavia no llego nada del Kalman. */
  PosFil_t est = { SETPOINT_FIXED_CM, 0.0f };

  for (;;)
  {
    QueueSetMemberHandle_t who = xQueueSelectFromSet(QueueSetPid, portMAX_DELAY);

    /* Se lee con timeout 0: con xQueueOverwrite sobre miembros de un set puede
     * quedar algun aviso "stale". Un setpoint nuevo solo refresca la variable;
     * el PID se computa una unica vez por tick de posicion (respeta el dt fijo). */
    if (who == QueueObjetivo)
    {
      (void)xQueueReceive(QueueObjetivo, &setpoint, 0);
    }
    else if (who == QueuePosFil)
    {
      if (xQueueReceive(QueuePosFil, &est, 0) == pdTRUE)
      {
        /* PID_ComputeRate y no PID_Compute: la velocidad viene del Kalman, que
         * la estima con el modelo de ruido (Q, R) en vez de restar dos muestras
         * cuantizadas. Misma cantidad, mucho menos ruido, y ninguna fase extra
         * porque sale del mismo filtro que ya suaviza la posicion. */
        float u     = PID_ComputeRate(&pid, setpoint, est.pos, est.vel);
        float angle = SERVO_CENTER_DEG + (SERVO_DIR * u);
        xQueueOverwrite(QueueAngulo, &angle);

#if (APP_LOG_LOOP == 1)
        App_LogTrace(g_dbg_raw, est.pos, est.vel, setpoint, u, pid.integ, angle);
#endif
      }
    }
  }
}
