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

  /* Setpoint por defecto hasta que PotTask publique el suyo (mitad de la barra). */
  float setpoint = SETPOINT_FIXED_CM;
  float pos_fil  = setpoint;

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
      if (xQueueReceive(QueuePosFil, &pos_fil, 0) == pdTRUE)
      {
        float u     = PID_Compute(&pid, setpoint, pos_fil);
        float angle = SERVO_CENTER_DEG + (SERVO_DIR * u);
        xQueueOverwrite(QueueAngulo, &angle);

#if (APP_LOG_LOOP == 1)
        App_LogTrace(g_dbg_raw, pos_fil, setpoint, u, angle);
#endif
      }
    }
  }
}
