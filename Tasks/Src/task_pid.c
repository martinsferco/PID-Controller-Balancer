/**
  ******************************************************************************
  * @file    task_pid.c
  * @brief   Task del control PID (prio 3). Bloquea en queue_pos_fil (estado
  *          estimado); con cada posicion nueva relee el setpoint vigente
  *          (context->setpoint, escrito por PotTask sin pasar por una cola) y
  *          recalcula la accion de control, la mapea a un angulo de servo y lo
  *          publica en queue_angulo.
  *
  *          context->setpoint no tiene lock: un solo escritor (PotTask), un
  *          solo lector (esta task), y un float alineado se lee/escribe en una
  *          sola instruccion en Cortex-M, asi que no hay tearing posible. Es
  *          `volatile` solo para que el compilador no cachee el valor en un
  *          registro entre iteraciones del for(;;).
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

  /* Estado estimado: arranca en el setpoint default y quieto, para que la
   * primera accion de control no sea un salto si todavia no llego nada del
   * Kalman. */
  PosFil_t est = { SETPOINT_DEFAULT_CM, 0.0f };

  for (;;)
  {
    /* Timeout en vez de portMAX_DELAY: si vence (KalmanTask sin publicar), el
     * if no entra y el loop sigue sin hacer nada -- el integrador del PID se
     * deja congelado a proposito (ver comentario de MotorTask), no se
     * resetea aca. */
    if (xQueueReceive(context->queue_pos_fil, &est, pdMS_TO_TICKS(PID_TASK_TIMEOUT_MS)) == pdTRUE)
    {
      /* PID_ComputeRate y no PID_Compute: la velocidad viene del Kalman, que
       * la estima con su modelo de ruido en vez de restar dos posiciones
       * cuantizadas, y sale del mismo update que la posicion. */
      float u     = PID_ComputeRate(context->pid, *context->setpoint, est.pos, est.vel);
      float angle = SERVO_CENTER_DEG + (SERVO_DIR * u);
      xQueueOverwrite(context->queue_angulo, &angle);
    }
  }
}
