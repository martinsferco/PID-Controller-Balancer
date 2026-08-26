/**
  ******************************************************************************
  * @file    task_pot.c
  * @brief   Task del potenciometro (prio 1). Cada POT_PERIOD_MS lee la posicion
  *          NORMALIZADA del pote (0.0..1.0) y la mapea al rango de setpoint en cm
  *          con linear_map, publicando el resultado en queue_objetivo.
  *
  *          La lectura y la conversion son responsabilidades separadas: el
  *          driver del pote solo lee (normalizado), y esta task decide a que
  *          magnitud lo convierte. Asi el mismo pote sirve para cualquier
  *          magnitud sin tocar el driver. El pote ya quedo creado e inicializado
  *          en App_Init.
  ******************************************************************************
  */

#include "task_pot.h"
#include "app_config.h"
#include "linear_map.h"

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

    float norm = 0.0f;
    if (Potentiometer_ReadNormalized(context->pot, &norm) == POTENTIOMETER_OK)
    {
      float setpoint = linear_map(norm, 0.0f, 1.0f,
                                  POTENTIOMETER_MIN_CM, POTENTIOMETER_MAX_CM);
      xQueueOverwrite(context->queue_objetivo, &setpoint);
    }
  }
}
