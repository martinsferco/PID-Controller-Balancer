/**
  ******************************************************************************
  * @file    task_sensor.c
  * @brief   Task del sensor (prio 5). Se despierta con SemTimer (tick de 100 ms
  *          de TIM4), dispara el HC-SR04, espera el echo (SemSensor) y publica
  *          la distancia cruda en QueuePos. Con APP_USE_SYNTHETIC_SENSOR=1
  *          genera una señal sintetica para validar la cadena sin hardware.
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

#if !APP_USE_SYNTHETIC_SENSOR
  HC_SR04_Init(&g_sensor, &htim2, TIM_CHANNEL_1, TRIG_GPIO_Port, TRIG_Pin);
  HC_SR04_SetRange(&g_sensor, SENSOR_MIN_CM, SENSOR_MAX_CM);
  HC_SR04_SetCompleteCallback(&g_sensor, App_OnSensorComplete_FromISR);
#else
  uint32_t k = 0;
#endif

  for (;;)
  {
    /* Espera el tick hard-real-time de 100 ms (lo da la ISR de TIM4). */
    if (xSemaphoreTake(SemTimer, portMAX_DELAY) != pdTRUE) { continue; }

#if APP_USE_SYNTHETIC_SENSOR
    /* Señal sintetica: triangular entre min y max (periodo ~4 s) + un poco de
     * "ruido" deterministico para que el Kalman tenga algo que suavizar. */
    const uint32_t period = 40u, half = 20u;
    uint32_t phase = k % period;
    float frac = (phase < half) ? ((float)phase / (float)half)
                                : ((float)(period - phase) / (float)half);
    float z = SENSOR_MIN_CM + frac * (SENSOR_MAX_CM - SENSOR_MIN_CM);
    z += (k & 1u) ? 0.3f : -0.3f;
    k++;
    g_dbg_raw = z;
    xQueueOverwrite(QueuePos, &z);
#else
    /* Descartar cualquier aviso viejo para esperar SOLO esta medicion. */
    (void)xSemaphoreTake(SemSensor, 0);

    if (HC_SR04_Trigger(&g_sensor) == HC_SR04_OK)
    {
      if (xSemaphoreTake(SemSensor, pdMS_TO_TICKS(SENSOR_ECHO_TIMEOUT_MS)) == pdTRUE)
      {
        float dist = 0.0f;
        if (HC_SR04_GetDistance(&g_sensor, &dist) == HC_SR04_OK)
        {
          g_dbg_raw = dist;
          xQueueOverwrite(QueuePos, &dist);
        }
      }
      else
      {
        /* Sin echo: GetDistance detecta el timeout y resetea la FSM a IDLE. */
        float dummy = 0.0f;
        (void)HC_SR04_GetDistance(&g_sensor, &dummy);
      }
    }
#endif
  }
}
