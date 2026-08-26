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

/* ============================ Inicializacion ============================== */

void App_Init(void)
{
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
