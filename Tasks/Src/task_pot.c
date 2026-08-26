/**
  ******************************************************************************
  * @file    task_pot.c
  * @brief   Task del potenciometro (prio 1). Cada POT_PERIOD_MS lee la posicion
  *          del pote por ADC (polling) y publica el setpoint en queue_objetivo.
  *          El pote ya quedo creado e inicializado en App_Init.
  ******************************************************************************
  */

#include "task_pot.h"
#include "app_config.h"

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

void PotTask(void *argument)
{
  TaskPotContext *context = (TaskPotContext *)argument;

  TickType_t ultimo = xTaskGetTickCount();

  for (;;)
  {
    vTaskDelayUntil(&ultimo, pdMS_TO_TICKS(POT_PERIOD_MS));

    float cm = 0.0f;
    if (Potentiometer_ReadPosition_cm(context->pot, &cm) == POTENTIOMETER_OK)
    {
      xQueueOverwrite(context->queue_objetivo, &cm);
    }
  }
}
