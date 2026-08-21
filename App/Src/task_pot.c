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

  for (;;)
  {
    vTaskDelayUntil(&last, pdMS_TO_TICKS(POT_PERIOD_MS));

    float cm = 0.0f;
    if (Pot_ReadPosition_cm(&g_pot, &cm) == POT_OK)
    {
      xQueueOverwrite(QueueObjetivo, &cm);
    }
  }
}
