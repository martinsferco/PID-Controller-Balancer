/**
  ******************************************************************************
  * @file    task_sensor.c
  * @brief   Task del sensor (prio 5). Se despierta con SemTimer (tick de 100 ms
  *          de TIM4), dispara el HC-SR04, espera el echo (SemSensor) y publica
  *          la distancia cruda en QueuePos.
  *
  *          Politica de "medir siempre que sea posible": una lectura que cae
  *          fuera de [SENSOR_MIN_CM, SENSOR_MAX_CM] no se tira sin mas. Si esta
  *          a menos de SENSOR_EDGE_GRACE_CM del borde, la pelota esta en la
  *          punta de la barra (o en la zona muerta del sensor): se recorta al
  *          borde y se publica igual. Solo se descarta el eco que volvio del
  *          ambiente, que si se usara mandaria al PID a inclinar por una pelota
  *          que no esta.
  *
  *          Con APP_LOG_ENABLED = 1 informa por UART cada fallo de medicion,
  *          distinguiendo "no llego el echo" (cableado) de "recortada al borde"
  *          y de "descartada" (apuntado / eco del ambiente).
  ******************************************************************************
  */

#include "task_sensor.h"
#include "app.h"
#include "app_config.h"

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "queue.h"
#include "tim.h"        /* htim2 */

void SensorTask(void *argument)
{
  (void)argument;

  HC_SR04_Init(&g_sensor, &htim2, TIM_CHANNEL_1, TRIG_GPIO_Port, TRIG_Pin);
  HC_SR04_SetRange(&g_sensor, SENSOR_MIN_CM, SENSOR_MAX_CM);
  HC_SR04_SetCompleteCallback(&g_sensor, App_OnSensorComplete_FromISR);

#if (APP_LOG_ENABLED == 1)
  App_LogMsg("SensorTask arrancada (TRIG=PA9, ECHO=PA0/TIM2_CH1)");
#endif

  for (;;)
  {
    /* Espera el tick hard-real-time de 100 ms (lo da la ISR de TIM4). */
    if (xSemaphoreTake(SemTimer, portMAX_DELAY) != pdTRUE) { continue; }

    /* Descartar cualquier aviso viejo para esperar SOLO esta medicion. */
    (void)xSemaphoreTake(SemSensor, 0);

    if (HC_SR04_Trigger(&g_sensor) != HC_SR04_OK)
    {
#if (APP_LOG_LOOP == 1)
      App_LogMsg("SENSOR: trigger rechazado (FSM ocupada)");
#endif
      continue;
    }

    if (xSemaphoreTake(SemSensor, pdMS_TO_TICKS(SENSOR_ECHO_TIMEOUT_MS)) != pdTRUE)
    {
      /* Sin echo: GetDistance detecta el timeout y resetea la FSM a IDLE. */
      float dummy = 0.0f;
      (void)HC_SR04_GetDistance(&g_sensor, &dummy);
#if (APP_LOG_LOOP == 1)
      App_LogMsg("SENSOR: sin echo (revisar TRIG/ECHO/divisor/VCC=5V)");
#endif
      continue;
    }

    float dist = 0.0f;
    HC_SR04_Status st = HC_SR04_GetDistance(&g_sensor, &dist);

    /* El driver escribe la distancia tanto con OK como con INVALID, asi que la
     * traza muestra SIEMPRE lo que el sensor midio de verdad -- incluso cuando
     * la muestra se termina descartando. Sin esto, un rechazo dejaba el z viejo
     * en la traza y parecia que el sensor se habia quedado quieto. */
#if (APP_LOG_ENABLED == 1)
    if (st == HC_SR04_OK || st == HC_SR04_INVALID) { g_dbg_raw = dist; }
#endif

    if (st == HC_SR04_OK)
    {
      xQueueOverwrite(QueuePos, &dist);
    }
    else if (st == HC_SR04_INVALID)
    {
      /* La medicion se hizo bien: solo cayo fuera de la ventana util. Y ahi hay
       * dos situaciones muy distintas que no merecen el mismo trato:
       *
       *  - A un pelo del borde (dentro de SENSOR_EDGE_GRACE_CM): la pelota esta
       *    en la PUNTA de la barra, o pegada al sensor en su zona muerta. Es
       *    informacion real -> se recorta al borde y se PUBLICA. Descartarla
       *    dejaria al PID sin dato nuevo y al servo congelado en la ultima
       *    inclinacion, justo cuando mas autoridad hace falta para recuperarla.
       *
       *  - Lejos de la ventana: no hubo eco de la pelota, volvio del ambiente
       *    (pared, mesa). Recortarla seria mentirle al PID, que volcaria la
       *    barra por una pelota que no esta ahi -> esa si se descarta.        */
      if ((dist > (SENSOR_MIN_CM - SENSOR_EDGE_GRACE_CM)) &&
          (dist < (SENSOR_MAX_CM + SENSOR_EDGE_GRACE_CM)))
      {
        float edge = (dist < SENSOR_MIN_CM) ? SENSOR_MIN_CM : SENSOR_MAX_CM;
        xQueueOverwrite(QueuePos, &edge);
#if (APP_LOG_LOOP == 1)
        App_LogMsgF("SENSOR: borde, recortada a cm=", edge);
#endif
      }
#if (APP_LOG_LOOP == 1)
      else
      {
        /* La unica muestra que se pierde de verdad. Si esto sale seguido, el
         * sensor no esta viendo la pelota: apuntado, o la pelota se fue. */
        App_LogMsgF("SENSOR: descartada (eco del ambiente), raw_cm=", dist);
      }
#endif
    }
#if (APP_LOG_LOOP == 1)
    else
    {
      App_LogMsg("SENSOR: sin dato listo (BUSY/ERROR)");
    }
#endif
  }
}
