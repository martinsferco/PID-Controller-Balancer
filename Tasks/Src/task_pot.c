/**
  ******************************************************************************
  * @file    task_pot.c
  * @brief   Task del potenciometro (prio 1). Cada 200 ms (vTaskDelayUntil) lee
  *          la posicion del pote por ADC (polling) y publica el setpoint en
  *          QueueObjetivo. g_potentiometer ya fue inicializado en App_Init().
  *
  *          Con APP_USE_FIXED_SETPOINT = 1 (pote todavia no cableado) NO se
  *          toca el ADC: se publica SETPOINT_FIXED_CM con el mismo periodo, asi
  *          el resto del pipeline se comporta igual que con el pote real.
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

#if (APP_USE_FIXED_SETPOINT == 1)
    /* Sin pote: PA4 esta flotando, leer el ADC daria un setpoint aleatorio. */
    float cm = SETPOINT_FIXED_CM;
    xQueueOverwrite(QueueObjetivo, &cm);
#else
    float cm = 0.0f;
    if (Potentiometer_ReadPosition_cm(&g_potentiometer, &cm) == POTENTIOMETER_OK)
    {
      xQueueOverwrite(QueueObjetivo, &cm);
    }
#endif
  }
}
