/**
  ******************************************************************************
  * @file    task_motor.c
  * @brief   Task del actuador: aplica en el servo el angulo recibido.
  *          Failsafe: si no llega angulo nuevo, nivela la barra.
  ******************************************************************************
  */

#include "task_motor.h"
#include "app_config.h"
#include "debug_uart.h"

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

void MotorTask(void *argument)
{
  TaskMotorContext *context = (TaskMotorContext *)argument;

  uint8_t perdida = 0u;

  for (;;)
  {
    float angle = 0.0f;
    if (xQueueReceive(context->queue_angulo, &angle, pdMS_TO_TICKS(MOTOR_TASK_TIMEOUT_MS)) == pdTRUE)
    {
      perdida = 0u;
      Servo_SetAngle(context->servo, angle);
      HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);   // heartbeat del lazo

      DebugUart_Print("[%10lu] MOTOR angle=%.2fdeg\r\n", (unsigned long)HAL_GetTick(), angle);
    }
    else if (!perdida)
    {
      perdida = 1u;
      Servo_SetAngle(context->servo, SERVO_LEVEL_DEG);

      DebugUart_Print("[%10lu] MOTOR FAILSAFE angle=%.2fdeg (sin dato nuevo)\r\n",
                       (unsigned long)HAL_GetTick(), SERVO_LEVEL_DEG);
    }
  }
}
