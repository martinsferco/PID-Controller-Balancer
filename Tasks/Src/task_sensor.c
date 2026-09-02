/**
  ******************************************************************************
  * @file    task_sensor.c
  * @brief   Task del sensor (prio 5). Se despierta con sem_timer (tick de 100 ms
  *          de TIM4), dispara el HC-SR04, espera el echo (sem_sensor) y publica
  *          la distancia cruda en queue_pos.
  *
  *          Politica: medir siempre que sea posible, nunca descartar en
  *          silencio. Una lectura que cae fuera de la ventana util pero a
  *          menos de SENSOR_EDGE_GRACE_CM del borde se recorta al borde (la
  *          pelota esta en la punta de la barra o en la zona muerta del
  *          sensor). Una lectura MUY fuera de rango es el eco fantasma de la
  *          zona ciega del HC-SR04 (por debajo de ~2 cm el modulo no recibe
  *          rebote real y el ECHO se queda en alto hasta su propio timeout
  *          interno, que el driver mide igual y convierte en una distancia
  *          enorme): la barra vive dentro de un armazon, asi que ese valor no
  *          puede ser un eco real del ambiente, y se recorta a SENSOR_MIN_CM.
  *          No descartar es lo que importa: si SensorTask deja de publicar,
  *          la cascada de timeouts (Kalman -> Pid -> Motor) termina en el
  *          failsafe de MotorTask, que nivela la barra y no reintenta solo.
  *
  *          El sensor ya viene creado, inicializado y con el callback puesto
  *          desde App_Init; la task solo recibe el contexto y corre el lazo.
  ******************************************************************************
  */

#include "task_sensor.h"
#include "app_config.h"

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "queue.h"

void SensorTask(void *argument)
{
  TaskSensorContext *context = (TaskSensorContext *)argument;

  for (;;)
  {
    /* Tick de 100 ms (lo da la ISR de TIM4). */
    if (xSemaphoreTake(context->sem_timer, portMAX_DELAY) != pdTRUE) { continue; }

    /* Descartar cualquier aviso viejo, para esperar SOLO esta medicion. */
    (void)xSemaphoreTake(context->sem_sensor, 0);

    if (HC_SR04_Trigger(context->sensor) != HC_SR04_OK)
    {
      continue;
    }

    if (xSemaphoreTake(context->sem_sensor, pdMS_TO_TICKS(SENSOR_ECHO_TIMEOUT_MS)) != pdTRUE)
    {
      /* Sin echo: GetDistance detecta el timeout y resetea la FSM a IDLE. */
      float descarte = 0.0f;
      (void)HC_SR04_GetDistance(context->sensor, &descarte);
      continue;
    }

    float dist = 0.0f;
    HC_SR04_Status st = HC_SR04_GetDistance(context->sensor, &dist);

    if (st == HC_SR04_OK)
    {
      xQueueOverwrite(context->queue_pos, &dist);
    }
    else if (st == HC_SR04_INVALID)
    {
      /* La medicion se hizo bien: solo cayo fuera de la ventana util. Nunca se
       * descarta -- ver la politica en el encabezado del archivo. */
      float borde;

      if (dist < SENSOR_MIN_CM)
      {
        /* Por debajo del borde inferior (con o sin gracia): la pelota esta
         * pegada al sensor o en su zona muerta. */
        borde = SENSOR_MIN_CM;
      }
      else if (dist < (SENSOR_MAX_CM + SENSOR_EDGE_GRACE_CM))
      {
        /* A un pelo del borde superior: la pelota esta en la punta de la
         * barra. */
        borde = SENSOR_MAX_CM;
      }
      else
      {
        /* Muy por encima del borde superior: no es un objeto lejano (la
         * barra esta encerrada, no puede rebotar en el ambiente) sino el eco
         * fantasma de la zona ciega. Misma conclusion que el primer caso:
         * pelota pegada al sensor. */
        borde = SENSOR_MIN_CM;
      }

      xQueueOverwrite(context->queue_pos, &borde);
    }
  }
}
