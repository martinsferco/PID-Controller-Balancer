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

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "tim.h"        /* htim4 */
#include "adc.h"        /* hadc1 */

#if (APP_LOG_ENABLED == 1)
#include "usart.h"      /* huart2, para el retarget de printf */
#include <stdio.h>
#endif

/* --- Instancias de drivers ------------------------------------------------ */
HC_SR04_HandleTypeDef g_sensor;
Servo_HandleTypeDef   g_servo;
Potentiometer_HandleTypeDef g_potentiometer;

/* --- Semaforos ------------------------------------------------------------ */
SemaphoreHandle_t SemTimer;
SemaphoreHandle_t SemSensor;

/* --- Colas + queue set ---------------------------------------------------- */
QueueHandle_t    QueuePos;
QueueHandle_t    QueuePosFil;
QueueHandle_t    QueueObjetivo;
QueueHandle_t    QueueAngulo;
QueueSetHandle_t QueueSetPid;

/* ====================== Hooks de ISR ======================= */

/* ISR de TIM2 (Input Capture del HC-SR04): medicion completa -> despierta SensorTask. */
void App_OnSensorComplete_FromISR(HC_SR04_HandleTypeDef *h)
{
  (void)h;
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  xSemaphoreGiveFromISR(SemSensor, &xHigherPriorityTaskWoken);
  portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

/* ISR de TIM4 (cada 100 ms): tick hard-real-time -> dispara el ciclo del sensor. */
void App_OnTimerTick_FromISR(void)
{
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  xSemaphoreGiveFromISR(SemTimer, &xHigherPriorityTaskWoken);
  portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

/* ====================== Traza de debug ===================== */
#if (APP_LOG_ENABLED == 1)

volatile float g_dbg_raw = 0.0f;

/* Retarget de printf hacia USART2 (COM virtual del ST-Link). */
int __io_putchar(int ch)
{
  HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
  return ch;
}

/* Imprime un float como signo + entero + 3 decimales (newlib-nano no trae %f). */
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
  printf("z=");    print_f(z);
  printf(" fil="); print_f(pos_fil);
  printf(" sp=");  print_f(sp);
  printf(" u=");   print_f(u);
  printf(" ang="); print_f(angle);
  printf("\r\n");
}

void App_LogMsg(const char *msg)
{
  printf("%s\r\n", msg);
}

void App_LogMsgF(const char *msg, float v)
{
  printf("%s", msg);
  print_f(v);
  printf("\r\n");
}

#endif /* APP_LOG_ENABLED */

/* ====================== Inicializacion ===================== */

void App_Init(void)
{
#if (APP_LOG_ENABLED == 1)
  /* Banner: si esto NO aparece en la terminal, el problema es el UART / la
   * terminal / el flasheo, no el lazo de control. */
  App_LogMsg("\r\n=== BALL & BEAM - DEBUG ===");
  App_LogMsgF("barra_cm=",    BEAM_LENGTH_CM);
  App_LogMsgF("setpoint_cm=", SETPOINT_FIXED_CM);
  App_LogMsgF("servo_dir=",   SERVO_DIR);
  App_LogMsg("formato: z=cruda fil=kalman sp=objetivo u=pid ang=servo");
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
  if (Potentiometer_Init(&g_potentiometer, &hadc1) != POTENTIOMETER_OK) { Error_Handler(); }
  if (Potentiometer_SetRange(&g_potentiometer, POTENTIOMETER_MIN_CM, POTENTIOMETER_MAX_CM) != POTENTIOMETER_OK) { Error_Handler(); }

  /* --- Arrancar el tick de 100 ms (TIM4 en modo base con interrupcion) --- */
  if (HAL_TIM_Base_Start_IT(&htim4) != HAL_OK) { Error_Handler(); }

  /* --- Tasks (una por archivo) --- */
  if (xTaskCreate(SensorTask, "Sensor", SENSOR_TASK_STACK, NULL, SENSOR_TASK_PRIO, NULL) != pdPASS) { Error_Handler(); }
  if (xTaskCreate(KalmanTask, "Kalman", KALMAN_TASK_STACK, NULL, KALMAN_TASK_PRIO, NULL) != pdPASS) { Error_Handler(); }
  if (xTaskCreate(PidTask,    "Pid",    PID_TASK_STACK,    NULL, PID_TASK_PRIO,    NULL) != pdPASS) { Error_Handler(); }
  if (xTaskCreate(MotorTask,  "Motor",  MOTOR_TASK_STACK,  NULL, MOTOR_TASK_PRIO,  NULL) != pdPASS) { Error_Handler(); }
  if (xTaskCreate(PotTask,    "Pot",    POT_TASK_STACK,    NULL, POT_TASK_PRIO,    NULL) != pdPASS) { Error_Handler(); }
}
