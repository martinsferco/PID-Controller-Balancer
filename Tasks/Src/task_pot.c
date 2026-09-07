/**
  ******************************************************************************
  * @file    task_pot.c
  * @brief   Task del potenciometro: cada POT_PERIOD_MS lee el pote y
  *          publica el setpoint (cm) en *context->setpoint, sin cola.
  ******************************************************************************
  */

#include "task_pot.h"
#include "app_config.h"
#include "linear_map.h"
#include "debug_uart.h"

#include "FreeRTOS.h"
#include "task.h"

void PotTask(void *argument)
{
  TaskPotContext *context = (TaskPotContext *)argument;

  TickType_t ultimo = xTaskGetTickCount();

  for (;;)
  {
    vTaskDelayUntil(&ultimo, pdMS_TO_TICKS(POT_PERIOD_MS));

    float norm = 0.0f;
    if (Potentiometer_ReadNormalized(context->pot, &norm) == POTENTIOMETER_OK)
    {
      *context->setpoint = linear_map(norm, 0.0f, 1.0f,
                                      POTENTIOMETER_MIN_CM, POTENTIOMETER_MAX_CM);

      DebugUart_Print("[%10lu] SETPOINT pote=%.3f setpoint=%.2fcm\r\n",
                       (unsigned long)HAL_GetTick(), norm, *context->setpoint);
    }
  }
}
