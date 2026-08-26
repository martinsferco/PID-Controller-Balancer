/**
  ******************************************************************************
  * @file    task_pid.c
  * @brief   Task del control PID (prio 3). Bloquea en un queue set sobre
  *          queue_pos_fil (estado estimado) y queue_objetivo (setpoint del pote).
  *          Con cada posicion nueva recalcula la accion de control, la mapea a
  *          un angulo de servo y lo publica en queue_angulo; con cada setpoint
  *          nuevo solo actualiza la referencia.
  *
  *          Usa PID_ComputeRate (no PID_Compute) porque el Kalman ya entrega la
  *          velocidad estimada junto con la posicion: se aprovecha esa velocidad
  *          en vez de estimarla por diferencia finita de posiciones cuantizadas.
  *
  *          El PID ya viene creado y configurado (ganancias, limites, banda)
  *          desde App_Init; la task solo recibe el contexto y corre el lazo.
  ******************************************************************************
  */

#include "task_pid.h"
#include "app.h"          /* PosFil_t */
#include "app_config.h"

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

void PidTask(void *argument)
{
  TaskPidContext *context = (TaskPidContext *)argument;

  /* Referencia hasta que PotTask publique la primera lectura del pote. */
  float setpoint = SETPOINT_DEFAULT_CM;

  /* Estado estimado: arranca en el setpoint y quieto, para que la primera
   * accion de control no sea un salto si todavia no llego nada del Kalman. */
  PosFil_t est = { SETPOINT_DEFAULT_CM, 0.0f };

  for (;;)
  {
    QueueSetMemberHandle_t quien = xQueueSelectFromSet(context->queue_set, portMAX_DELAY);

    /* Se lee con timeout 0: con xQueueOverwrite sobre miembros de un set puede
     * quedar algun aviso sin dato detras. Un setpoint nuevo solo refresca la
     * referencia; el PID se computa una unica vez por posicion, que es lo que
     * respeta el dt fijo del controlador. */
    if (quien == context->queue_objetivo)
    {
      (void)xQueueReceive(context->queue_objetivo, &setpoint, 0);
    }
    else if (quien == context->queue_pos_fil)
    {
      if (xQueueReceive(context->queue_pos_fil, &est, 0) == pdTRUE)
      {
        /* PID_ComputeRate y no PID_Compute: la velocidad viene del Kalman, que
         * la estima con su modelo de ruido en vez de restar dos posiciones
         * cuantizadas, y sale del mismo update que la posicion. */
        float u     = PID_ComputeRate(context->pid, setpoint, est.pos, est.vel);
        float angle = SERVO_CENTER_DEG + (SERVO_DIR * u);
        xQueueOverwrite(context->queue_angulo, &angle);
      }
    }
  }
}
