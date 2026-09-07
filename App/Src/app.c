/**
  ******************************************************************************
  * @file    app.c
  * @brief   Capa de conexion del sistema de control PID para balanceo: crea las
  *          instancias de los drivers, las inicializa y configura, arma la IPC,
  *          crea los contextos de cada task y las crea.
  ******************************************************************************
  */

#include "app.h"
#include "app_config.h"
#include "task_sensor.h"
#include "task_kalman.h"
#include "task_pid.h"
#include "task_motor.h"
#include "task_pot.h"

#include "servo_mg90s.h"
#include "potentiometer.h"
#include "pid.h"
#include "kalman.h"

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "tim.h"        
#include "adc.h"        

static SemaphoreHandle_t      SemTimer;    // ISR de TIM4 (100 ms)
static SemaphoreHandle_t      SemSensor;   // ISR de Input Capture
static HC_SR04_HandleTypeDef *s_sensor;    // handle del sensor (recibe hook)

// ISR de TIM2 (Input Capture del HC-SR04): medicion completa -> SensorTask
void App_OnSensorComplete_FromISR(HC_SR04_HandleTypeDef *h)
{
  (void)h;
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  xSemaphoreGiveFromISR(SemSensor, &xHigherPriorityTaskWoken);
  portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

// ISR de TIM4 (cada 100 ms): tick del sensor
void App_OnTimerTick_FromISR(void)
{
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  xSemaphoreGiveFromISR(SemTimer, &xHigherPriorityTaskWoken);
  portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

void App_Init(void)
{
  // Semaforos binarios (los usan los hooks)
  static StaticSemaphore_t s_sem_timer_cb;
  static StaticSemaphore_t s_sem_sensor_cb;
  SemTimer  = xSemaphoreCreateBinaryStatic(&s_sem_timer_cb);
  SemSensor = xSemaphoreCreateBinaryStatic(&s_sem_sensor_cb);
  if (SemTimer == NULL || SemSensor == NULL) { Error_Handler(); }

  // Colas para comunicar
  static StaticQueue_t s_queue_pos_cb;
  static uint8_t       s_queue_pos_storage[sizeof(float)];
  static StaticQueue_t s_queue_pos_fil_cb;
  static uint8_t       s_queue_pos_fil_storage[sizeof(PosFil_t)];
  static StaticQueue_t s_queue_angulo_cb;
  static uint8_t       s_queue_angulo_storage[sizeof(float)];

  QueueHandle_t QueuePos      = xQueueCreateStatic(1, sizeof(float), s_queue_pos_storage, &s_queue_pos_cb);
  QueueHandle_t QueuePosFil   = xQueueCreateStatic(1, sizeof(PosFil_t), s_queue_pos_fil_storage, &s_queue_pos_fil_cb);
  QueueHandle_t QueueAngulo   = xQueueCreateStatic(1, sizeof(float), s_queue_angulo_storage, &s_queue_angulo_cb);
  if (QueuePos == NULL || QueuePosFil == NULL || QueueAngulo == NULL) { Error_Handler(); }

  // Setpoint sin cola
  static float s_setpoint = SETPOINT_DEFAULT_CM;

  // Sensor HC-SR04
  s_sensor = HC_SR04_Create();
  if (s_sensor == NULL) { Error_Handler(); }

  TimerChannel_t echo = { &htim2, TIM_CHANNEL_1 };
  GpioPin_t      trig = { TRIG_GPIO_Port, TRIG_Pin };

  if (HC_SR04_Init(s_sensor, echo, trig) != HC_SR04_OK) { Error_Handler(); }

  HC_SR04_SetCompleteCallback(s_sensor, App_OnSensorComplete_FromISR);

  // Servo MG90S
  Servo_HandleTypeDef *servo = Servo_Create();
  if (servo == NULL) { Error_Handler(); }

  TimerChannel_t pwm = { &htim3, TIM_CHANNEL_1 };
  if (Servo_Init(servo, pwm) != SERVO_OK) { Error_Handler(); }
  if (Servo_SetTravel(servo, SERVO_MIN_DEG, SERVO_MAX_DEG) != SERVO_OK) { Error_Handler(); }

  Servo_SetAngle(servo, SERVO_LEVEL_DEG);   // barra nivelada antes de las tasks

  // Potenciometro
  Potentiometer_HandleTypeDef *pot = Potentiometer_Create();

  if (pot == NULL) { Error_Handler(); }
  if (Potentiometer_Init(pot, &hadc1) != POTENTIOMETER_OK) { Error_Handler(); }

  // PID
  PID_HandleTypeDef *pid = PID_Create();
  if (pid == NULL) { Error_Handler(); }
  PID_Init(pid, PID_KP, PID_KI, PID_KD, PID_DT);
  PID_SetLimits(pid, PID_OUT_MIN, PID_OUT_MAX);
  PID_SetIntegralBand(pid, PID_I_BAND);

  // Kalman
  Kalman_HandleTypeDef *kalman = Kalman_Create();
  if (kalman == NULL) { Error_Handler(); }
  Kalman_Init(kalman, KALMAN_DT, KALMAN_Q, KALMAN_R, 0.0f);

  // Tick de 100 ms
  if (HAL_TIM_Base_Start_IT(&htim4) != HAL_OK) { Error_Handler(); }

  // Contextos por task
  static TaskSensorContext sensor_ctx;
  sensor_ctx.sensor     = s_sensor;
  sensor_ctx.sem_timer  = SemTimer;
  sensor_ctx.sem_sensor = SemSensor;
  sensor_ctx.queue_pos  = QueuePos;

  static TaskKalmanContext kalman_ctx;
  kalman_ctx.kalman        = kalman;
  kalman_ctx.queue_pos     = QueuePos;
  kalman_ctx.queue_pos_fil = QueuePosFil;

  static TaskPidContext pid_ctx;
  pid_ctx.pid            = pid;
  pid_ctx.queue_pos_fil  = QueuePosFil;
  pid_ctx.setpoint       = &s_setpoint;
  pid_ctx.queue_angulo   = QueueAngulo;

  static TaskMotorContext motor_ctx;
  motor_ctx.servo        = servo;
  motor_ctx.queue_angulo = QueueAngulo;

  static TaskPotContext pot_ctx;
  pot_ctx.pot            = pot;
  pot_ctx.setpoint       = &s_setpoint;

  // Tasks
  static StaticTask_t s_sensor_tcb;
  static StackType_t  s_sensor_stack[SENSOR_TASK_STACK];
  static StaticTask_t s_kalman_tcb;
  static StackType_t  s_kalman_stack[KALMAN_TASK_STACK];
  static StaticTask_t s_pid_tcb;
  static StackType_t  s_pid_stack[PID_TASK_STACK];
  static StaticTask_t s_motor_tcb;
  static StackType_t  s_motor_stack[MOTOR_TASK_STACK];
  static StaticTask_t s_pot_tcb;
  static StackType_t  s_pot_stack[POT_TASK_STACK];

  if (xTaskCreateStatic(SensorTask, "Sensor", SENSOR_TASK_STACK, &sensor_ctx, SENSOR_TASK_PRIO, s_sensor_stack, &s_sensor_tcb) == NULL) { Error_Handler(); }
  if (xTaskCreateStatic(KalmanTask, "Kalman", KALMAN_TASK_STACK, &kalman_ctx, KALMAN_TASK_PRIO, s_kalman_stack, &s_kalman_tcb) == NULL) { Error_Handler(); }
  if (xTaskCreateStatic(PidTask,    "Pid",    PID_TASK_STACK,    &pid_ctx,    PID_TASK_PRIO,    s_pid_stack,    &s_pid_tcb)    == NULL) { Error_Handler(); }
  if (xTaskCreateStatic(MotorTask,  "Motor",  MOTOR_TASK_STACK,  &motor_ctx,  MOTOR_TASK_PRIO,  s_motor_stack,  &s_motor_tcb)  == NULL) { Error_Handler(); }
  if (xTaskCreateStatic(PotTask,    "Pot",    POT_TASK_STACK,    &pot_ctx,    POT_TASK_PRIO,    s_pot_stack,    &s_pot_tcb)    == NULL) { Error_Handler(); }
}
