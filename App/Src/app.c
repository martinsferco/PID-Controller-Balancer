/**
  ******************************************************************************
  * @file    app.c
  * @brief   Wiring de la aplicacion ball-and-beam: instancias de drivers,
  *          IPC (4 colas + queue set + 2 semaforos), hooks de ISR, arranque de
  *          perifericos de tiempo real y creacion de las 5 tasks.
  ******************************************************************************
  */

#include "app.h"
#include "app_config.h"
#include "task_sensor.h"
#include "task_kalman.h"
#include "task_pid.h"
#include "task_motor.h"
#include "task_pot.h"
#include "selftest.h"

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "usart.h"      /* huart2, para el retarget de printf */
#include "tim.h"        /* htim4 */
#include "adc.h"        /* hadc1 */
#include <stdio.h>

/* --- Instancias de drivers ------------------------------------------------ */
HC_SR04_HandleTypeDef g_sensor;
Servo_HandleTypeDef   g_servo;
Pot_HandleTypeDef     g_pot;

/* --- Semaforos ------------------------------------------------------------ */
SemaphoreHandle_t SemTimer;
SemaphoreHandle_t SemSensor;

/* --- Colas + queue set ---------------------------------------------------- */
QueueHandle_t    QueuePos;
QueueHandle_t    QueuePosFil;
QueueHandle_t    QueueObjetivo;
QueueHandle_t    QueueAngulo;
QueueSetHandle_t QueueSetPid;

/* --- Debug -------------------------------------------------------------- */
volatile float g_dbg_raw = 0.0f;

/* ====================== Hooks de ISR ======================= */

/* ISR de TIM2 (Input Capture del HC-SR04): medicion completa -> despierta SensorTask. */
void App_OnSensorComplete_FromISR(HC_SR04_HandleTypeDef *h)
{
  (void)h;
  BaseType_t hpw = pdFALSE;
  xSemaphoreGiveFromISR(SemSensor, &hpw);
  portYIELD_FROM_ISR(hpw);
}

/* ISR de TIM4 (cada 100 ms): tick hard-real-time -> dispara el ciclo del sensor. */
void App_OnTimerTick_FromISR(void)
{
  BaseType_t hpw = pdFALSE;
  xSemaphoreGiveFromISR(SemTimer, &hpw);
  portYIELD_FROM_ISR(hpw);
}

/* ====================== Utilitarios ======================== */

/* Retarget de printf hacia USART2 (COM virtual del ST-Link). */
int __io_putchar(int ch)
{
  HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
  return ch;
}

/* Imprime un float como signo + entero + 3 decimales (newlib-nano sin %f). */
static void print_f(float v)
{
  if (v < 0.0f) { printf("-"); v = -v; }
  int ip = (int)v;
  int fp = (int)((v - (float)ip) * 1000.0f + 0.5f);
  if (fp >= 1000) { ip += 1; fp -= 1000; }
  printf("%d.%03d", ip, fp);
}

void App_LogTrace(float z, float pos_fil, float sp, float u, float angle)
{
  printf("z="); print_f(z);
  printf(" fil="); print_f(pos_fil);
  printf(" sp="); print_f(sp);
  printf(" u="); print_f(u);
  printf(" ang="); print_f(angle);
  printf("\r\n");
}

/* ====================== Inicializacion ===================== */

void App_Init(void)
{
#if APP_RUN_SELFTESTS
  /* Self-tests on-target antes de arrancar el lazo (no se compila con flag 0). */
  SelfTest_Run();
#endif

  /* --- Semaforos binarios --- */
  SemTimer  = xSemaphoreCreateBinary();
  SemSensor = xSemaphoreCreateBinary();
  if (SemTimer == NULL || SemSensor == NULL) { Error_Handler(); }

  /* --- Colas float profundidad 1 --- */
  QueuePos      = xQueueCreate(1, sizeof(float));
  QueuePosFil   = xQueueCreate(1, sizeof(float));
  QueueObjetivo = xQueueCreate(1, sizeof(float));
  QueueAngulo   = xQueueCreate(1, sizeof(float));
  if (QueuePos == NULL || QueuePosFil == NULL ||
      QueueObjetivo == NULL || QueueAngulo == NULL) { Error_Handler(); }

  /* --- Queue set del PID (QueuePosFil + QueueObjetivo) --- */
  QueueSetPid = xQueueCreateSet(2);   /* 1 + 1 (profundidades) */
  if (QueueSetPid == NULL) { Error_Handler(); }
  if (xQueueAddToSet(QueuePosFil,  QueueSetPid) != pdPASS) { Error_Handler(); }
  if (xQueueAddToSet(QueueObjetivo, QueueSetPid) != pdPASS) { Error_Handler(); }

  /* --- Potenciometro (solo init de struct; el ADC ya lo configuro CubeMX) --- */
  if (Pot_Init(&g_pot, &hadc1, POT_RANGE_CM) != POT_OK) { Error_Handler(); }

  /* --- Arrancar el tick de 100 ms (TIM4 en modo base con interrupcion) --- */
  if (HAL_TIM_Base_Start_IT(&htim4) != HAL_OK) { Error_Handler(); }

  /* --- Tasks (una por archivo) --- */
  if (xTaskCreate(SensorTask, "Sensor", SENSOR_TASK_STACK, NULL, SENSOR_TASK_PRIO, NULL) != pdPASS) { Error_Handler(); }
  if (xTaskCreate(KalmanTask, "Kalman", KALMAN_TASK_STACK, NULL, KALMAN_TASK_PRIO, NULL) != pdPASS) { Error_Handler(); }
  if (xTaskCreate(PidTask,    "Pid",    PID_TASK_STACK,    NULL, PID_TASK_PRIO,    NULL) != pdPASS) { Error_Handler(); }
  if (xTaskCreate(MotorTask,  "Motor",  MOTOR_TASK_STACK,  NULL, MOTOR_TASK_PRIO,  NULL) != pdPASS) { Error_Handler(); }
  if (xTaskCreate(PotTask,    "Pot",    POT_TASK_STACK,    NULL, POT_TASK_PRIO,    NULL) != pdPASS) { Error_Handler(); }
}
