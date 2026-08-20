/**
  ******************************************************************************
  * @file    task_motor.c
  * @brief   Task del actuador (prio 4). Recibe el angulo de QueueAngulo y lo
  *          aplica al servo. Reemplaza a la task de barrido de prueba de la
  *          Etapa 1. Hace toggle de LD2 (PA5) como heartbeat de que la cadena
  *          llega hasta el motor.
  ******************************************************************************
  */

#include "task_motor.h"
#include "app.h"
#include "app_config.h"

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "tim.h"        /* htim3 */

void MotorTask(void *argument)
{
  (void)argument;

  Servo_Init(&g_servo, &htim3, TIM_CHANNEL_1);   /* arranca el PWM y centra */

  for (;;)
  {
    float angle = 0.0f;
    if (xQueueReceive(QueueAngulo, &angle, portMAX_DELAY) == pdTRUE)
    {
      Servo_SetAngle(&g_servo, angle);
      HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);   /* heartbeat del lazo */
    }
  }
}
