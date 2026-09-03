/**
  ******************************************************************************
  * @file    task_sensor.c
  * @brief   Task del sensor (prio 5). Se despierta con sem_timer (tick de 100 ms
  *          de TIM4), dispara el HC-SR04, espera el echo (sem_sensor) y publica
  *          la distancia cruda (borde del carro que encara al sensor) en
  *          queue_pos. Kalman, PID y el setpoint del pote trabajan todos en
  *          esa misma referencia.
  *
  *          Politica: medir siempre que sea posible, nunca descartar en
  *          silencio, y no tirar informacion real solo porque cae mas cerca
  *          de lo que la app pediria como setpoint. HC_SR04_OK y HC_SR04_INVALID
  *          se tratan distinto porque significan cosas distintas:
  *            - HC_SR04_OK: el driver confirma un eco real (dentro de
  *              [HC_SR04_HW_MIN_CM, HC_SR04_HW_MAX_CM]). Se publica la
  *              distancia medida tal cual, aunque sea menor a SENSOR_MIN_CM
  *              -- es una medicion real, no ambigua, y clamparla a un piso fijo
  *              le esconde al lazo de control el movimiento real del carro
  *              cerca del sensor (eso rompia a Kalman/PID: con el mismo valor
  *              constante en cada muestra, la velocidad estimada colapsa a
  *              cero y el error deja de reflejar la posicion real). Solo se
  *              recorta el extremo lejano (SENSOR_MAX_CM): un eco real mas
  *              alla de ahi no es fisicamente alcanzable (el carro choca
  *              contra su tope mecanico antes), asi que el carro esta en la
  *              punta de la barra.
  *            - HC_SR04_INVALID: el driver NO confirma un eco real, por dos
  *              motivos distintos:
  *                - dist < HC_SR04_HW_MIN_CM: zona ciega sin rebote, el carro
  *                  esta practicamente tocando el sensor. Se publica
  *                  HC_SR04_HW_MIN_CM, el piso fisico real.
  *                - dist > HC_SR04_HW_MAX_CM: distancia enorme tras el
  *                  timeout interno del eco fantasma, sin ninguna relacion
  *                  con la posicion real. Se publica SENSOR_MIN_CM / 2, mas
  *                  conservador que el piso de arriba por la incertidumbre
  *                  extra de este caso.
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

    float borde;

    if (st == HC_SR04_OK)
    {
      /* Eco real: se publica la medicion tal cual, salvo el recorte al
       * extremo lejano (el carro no puede llegar mas alla de SENSOR_MAX_CM,
       * choca contra su tope mecanico antes). No hay piso: un eco real cerca
       * del sensor sigue siendo informacion real, ver la politica arriba. */
      borde = (dist > SENSOR_MAX_CM) ? SENSOR_MAX_CM : dist;
    }
    else if (st == HC_SR04_INVALID)
    {
      /* Sin eco real, por dos motivos distintos -- ver la politica arriba. */
      if (dist < HC_SR04_HW_MIN_CM)
      {
        /* No hay rebote: el carro esta practicamente tocando el sensor. */
        borde = HC_SR04_HW_MIN_CM;
      }
      else
      {
        /* dist > HC_SR04_HW_MAX_CM: eco fantasma tras el timeout interno del
         * driver. Sin ninguna relacion con la distancia real, asi que se
         * asume una proximidad conservadora en vez del piso de arriba. */
        borde = SENSOR_MIN_CM / 2.0f;
      }
    }
    else
    {
      /* BUSY/ERROR no deberian darse aca (recien se confirmo el echo), pero
       * si pasara no hay distancia que publicar. */
      continue;
    }

    xQueueOverwrite(context->queue_pos, &borde);
  }
}
