/**
  ******************************************************************************
  * @file    task_sensor.c
  * @brief   Task del sensor (prio 5). Se despierta con SemTimer (tick de 100 ms
  *          de TIM4), dispara el HC-SR04, espera el echo (SemSensor) y publica
  *          la distancia cruda en QueuePos.
  *
  *          Con APP_LOG_ENABLED = 1 informa por UART cada fallo de medicion,
  *          distinguiendo "no llego el echo" (cableado) de "fuera de rango"
  *          (apuntado / SENSOR_MIN_CM / SENSOR_MAX_CM).
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

    if (st == HC_SR04_OK)
    {
#if (APP_LOG_ENABLED == 1)
      g_dbg_raw = dist;
#endif
      xQueueOverwrite(QueuePos, &dist);
    }
#if (APP_LOG_LOOP == 1)
    else if (st == HC_SR04_INVALID)
    {
      /* El echo llego y se midio bien, pero cae fuera de [MIN, MAX] y se
       * descarta: apuntado del sensor, o rango mal configurado. */
      App_LogMsgF("SENSOR: fuera de rango, raw_cm=", dist);
    }
    else
    {
      App_LogMsg("SENSOR: sin dato listo (BUSY/ERROR)");
    }
#endif
  }
}
