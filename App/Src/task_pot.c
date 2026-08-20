/**
  ******************************************************************************
  * @file    task_pot.c
  * @brief   Task del potenciometro (prio 1). Cada 200 ms (vTaskDelayUntil) lee
  *          la posicion del pote por ADC (polling) y publica el setpoint en
  *          QueueObjetivo. g_pot ya fue inicializado en App_Init().
  ******************************************************************************
  */

#include "task_pot.h"
#include "app.h"
#include "app_config.h"

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

void PotTask(void *argument)
{
  (void)argument;

  TickType_t last = xTaskGetTickCount();

#if APP_USE_SYNTHETIC_SENSOR
  /* Sin pote real (smoke test): setpoint fijo al centro del rango, para no leer
   * un ADC flotante y tener una traza limpia. La task igual vive (delay). */
  float sp_fijo = (SENSOR_MIN_CM + SENSOR_MAX_CM) * 0.5f;
  xQueueOverwrite(QueueObjetivo, &sp_fijo);
  for (;;)
  {
    vTaskDelayUntil(&last, pdMS_TO_TICKS(POT_PERIOD_MS));
  }
#else
  for (;;)
  {
    vTaskDelayUntil(&last, pdMS_TO_TICKS(POT_PERIOD_MS));

    float cm = 0.0f;
    if (Pot_ReadPosition_cm(&g_pot, &cm) == POT_OK)
    {
      xQueueOverwrite(QueueObjetivo, &cm);
    }
  }
#endif
}
