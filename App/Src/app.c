/**
  ******************************************************************************
  * @file    app.c
  * @brief   Wiring de la aplicacion ball-and-beam. Define las instancias
  *          compartidas, el retarget de printf y el hook del sensor, y crea las
  *          tasks. Es el unico lugar que main.c necesita conocer (App_Init()).
  ******************************************************************************
  */

#include "app.h"
#include "app_config.h"
#include "task_sensor.h"
#include "task_servo.h"
#include "selftest.h"

#include "FreeRTOS.h"
#include "task.h"
#include "usart.h"      /* huart2, para el retarget de printf */

/* --- Instancias compartidas (declaradas extern en app.h) ------------------ */
HC_SR04_HandleTypeDef g_sensor;      /* instancia del sensor        */
SemaphoreHandle_t     g_sensorSem;   /* aviso ISR -> task (binario) */
Servo_HandleTypeDef   g_servo;       /* instancia del servo         */

/* Hook que el driver invoca AL COMPLETAR una medicion (contexto ISR de TIM2).
 * Despierta a SensorTask. Seguro desde ISR porque TIM2 esta en prioridad NVIC 5
 * (>= configMAX_SYSCALL_INTERRUPT_PRIORITY). */
void App_OnSensorComplete_FromISR(HC_SR04_HandleTypeDef *h)
{
  (void)h;
  BaseType_t higherPriorityTaskWoken = pdFALSE;
  xSemaphoreGiveFromISR(g_sensorSem, &higherPriorityTaskWoken);
  portYIELD_FROM_ISR(higherPriorityTaskWoken);
}

/* Retarget de printf hacia USART2 (COM virtual del ST-Link), solo para pruebas. */
int __io_putchar(int ch)
{
  HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
  return ch;
}

/* Crea los objetos de sincronizacion y las tasks. Se invoca desde USER CODE 2
 * de main.c, antes de arrancar el scheduler. */
void App_Init(void)
{
#if APP_RUN_SELFTESTS
  /* Self-tests on-target de los modulos puros, antes de arrancar el lazo.
   * Con APP_RUN_SELFTESTS=0 (produccion) este bloque no se compila. */
  SelfTest_Run();
#endif

  g_sensorSem = xSemaphoreCreateBinary();
  xTaskCreate(SensorTask, "SensorTask", SENSOR_TASK_STACK, NULL, SENSOR_TASK_PRIO, NULL);
  xTaskCreate(ServoTask,  "ServoTask",  SERVO_TASK_STACK,  NULL, SERVO_TASK_PRIO,  NULL);
}
