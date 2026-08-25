/**
  ******************************************************************************
  * @file    task_pot.c
  * @brief   Task del potenciometro (prio 1). Cada POT_PERIOD_MS lee la posicion
  *          del pote por ADC (polling) y publica el setpoint en QueueObjetivo.
  *          g_potentiometer ya quedo inicializado en App_Init().
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

  TickType_t ultimo = xTaskGetTickCount();

  for (;;)
  {
    vTaskDelayUntil(&ultimo, pdMS_TO_TICKS(POT_PERIOD_MS));

    float cm = 0.0f;
    if (Potentiometer_ReadPosition_cm(&g_potentiometer, &cm) == POTENTIOMETER_OK)
    {
      xQueueOverwrite(QueueObjetivo, &cm);
    }
  }
}
