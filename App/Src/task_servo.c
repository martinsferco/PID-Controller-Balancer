/**
  ******************************************************************************
  * @file    task_servo.c
  * @brief   Task de prueba del servo MG90S: barrido ciclico entre los dos
  *          extremos. Placeholder hasta que el PID gobierne el angulo en
  *          funcion de la distancia medida. Comportamiento identico al baseline.
  ******************************************************************************
  */

#include "task_servo.h"
#include "app.h"
#include "app_config.h"

#include "FreeRTOS.h"
#include "task.h"
#include "tim.h"        /* htim3 */

/* Task de prueba del servo: lo mueve ciclicamente entre los dos extremos.
 * Corre EN PARALELO con SensorTask (FreeRTOS las multiplexa). Despues el angulo
 * lo manejara el PID en funcion de la distancia medida. */
void ServoTask(void *argument)
{
  (void)argument;

  Servo_Init(&g_servo, &htim3, TIM_CHANNEL_1);   /* arranca el PWM y centra */

  for (;;)
  {
    Servo_SetAngle(&g_servo, 180.0f);   /* un extremo      */
    vTaskDelay(pdMS_TO_TICKS(SERVO_SWEEP_MS));

    Servo_SetAngle(&g_servo, 0.0f);     /* el otro extremo */
    vTaskDelay(pdMS_TO_TICKS(SERVO_SWEEP_MS));
  }
}
