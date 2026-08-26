/**
  ******************************************************************************
  * @file    app.c
  * @brief   Wiring de la aplicacion sistema de control PID para balanceo: instancias de drivers, IPC
  *          (4 colas + queue set + 2 semaforos), hooks de ISR, arranque de los
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
HC_SR04_HandleTypeDef       g_sensor;
Servo_HandleTypeDef         g_servo;
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

/* ============================ Hooks de ISR ================================ */

/* ISR de TIM2 (Input Capture del HC-SR04): medicion completa -> SensorTask. */
void App_OnSensorComplete_FromISR(HC_SR04_HandleTypeDef *h)
{
  (void)h;
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  xSemaphoreGiveFromISR(SemSensor, &xHigherPriorityTaskWoken);
  portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

/* ISR de TIM4 (cada 100 ms): tick hard real time del ciclo del sensor. */
void App_OnTimerTick_FromISR(void)
{
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  xSemaphoreGiveFromISR(SemTimer, &xHigherPriorityTaskWoken);
  portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

/* ============================ Traza por UART ============================== */
#if (APP_LOG_ENABLED == 1)

volatile float g_sensor_raw_cm = 0.0f;

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

void App_LogTrace(float z, float pos_fil, float vel, float sp, float u,
                  float integ, float angle)
{
  printf("z=");    print_f(z);
  printf(" fil="); print_f(pos_fil);
  printf(" vel="); print_f(vel);
  printf(" sp=");  print_f(sp);
  printf(" u=");   print_f(u);
  printf(" i=");   print_f(integ);
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

/* ============================ Inicializacion ============================== */

void App_Init(void)
{
#if (APP_LOG_ENABLED == 1)
  App_LogMsg("\r\n=== BALL & BEAM ===");
  App_LogMsg("formato: z=cruda fil=kalman vel=kalman sp=objetivo u=pid i=integral ang=servo");
#endif

  /* --- Semaforos binarios --- */
  SemTimer  = xSemaphoreCreateBinary();
  SemSensor = xSemaphoreCreateBinary();
  if (SemTimer == NULL || SemSensor == NULL) { Error_Handler(); }

  /* --- Colas de profundidad 1 --- */
  QueuePos      = xQueueCreate(1, sizeof(float));
  QueuePosFil   = xQueueCreate(1, sizeof(PosFil_t));
  QueueObjetivo = xQueueCreate(1, sizeof(float));
  QueueAngulo   = xQueueCreate(1, sizeof(float));
  if (QueuePos == NULL || QueuePosFil == NULL ||
      QueueObjetivo == NULL || QueueAngulo == NULL) { Error_Handler(); }

  /* --- Queue set del PID (QueuePosFil + QueueObjetivo) ---------------------
   * Longitud 4, no 2. La regla de FreeRTOS (suma de las profundidades = 1+1)
   * vale para colas normales, pero aca se publica con xQueueOverwrite: cada
   * escritura genera un aviso al set AUNQUE la cola ya tuviera un dato sin
   * leer, asi que los avisos pendientes pueden superar la cantidad de datos. Si
   * el contenedor se llena, FreeRTOS pega en un configASSERT, que en este
   * proyecto es taskDISABLE_INTERRUPTS() + for(;;): un cuelgue mudo. Dos slots
   * de mas cuestan 16 bytes. */
  QueueSetPid = xQueueCreateSet(4);
  if (QueueSetPid == NULL) { Error_Handler(); }
  if (xQueueAddToSet(QueuePosFil,   QueueSetPid) != pdPASS) { Error_Handler(); }
  if (xQueueAddToSet(QueueObjetivo, QueueSetPid) != pdPASS) { Error_Handler(); }

  /* --- Potenciometro (el ADC ya lo configuro CubeMX) --- */
  if (Potentiometer_Init(&g_potentiometer, &hadc1) != POTENTIOMETER_OK) { Error_Handler(); }
  if (Potentiometer_SetRange(&g_potentiometer,
                             POTENTIOMETER_MIN_CM,
                             POTENTIOMETER_MAX_CM) != POTENTIOMETER_OK) { Error_Handler(); }

  /* --- Tick de 100 ms (TIM4 en modo base con interrupcion) --- */
  if (HAL_TIM_Base_Start_IT(&htim4) != HAL_OK) { Error_Handler(); }

  /* --- Tasks (una por archivo, prioridades en app_config.h) --- */
  if (xTaskCreate(SensorTask, "Sensor", SENSOR_TASK_STACK, NULL, SENSOR_TASK_PRIO, NULL) != pdPASS) { Error_Handler(); }
  if (xTaskCreate(KalmanTask, "Kalman", KALMAN_TASK_STACK, NULL, KALMAN_TASK_PRIO, NULL) != pdPASS) { Error_Handler(); }
  if (xTaskCreate(PidTask,    "Pid",    PID_TASK_STACK,    NULL, PID_TASK_PRIO,    NULL) != pdPASS) { Error_Handler(); }
  if (xTaskCreate(MotorTask,  "Motor",  MOTOR_TASK_STACK,  NULL, MOTOR_TASK_PRIO,  NULL) != pdPASS) { Error_Handler(); }
  if (xTaskCreate(PotTask,    "Pot",    POT_TASK_STACK,    NULL, POT_TASK_PRIO,    NULL) != pdPASS) { Error_Handler(); }
}
