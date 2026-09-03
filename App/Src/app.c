/**
  ******************************************************************************
  * @file    app.c
  * @brief   Composition root del sistema de control PID para balanceo: crea las
  *          instancias de los drivers (opacos, via Create), las inicializa y
  *          configura, arma la IPC (4 colas + queue set + 2 semaforos),
  *          rellena un contexto por task y las crea. Aloja tambien los hooks
  *          de ISR.
  *
  *          Reparto de memoria (todo persistente, nada en stack automatico):
  *            - Lo que toca la ISR (SemSensor, SemTimer y el handle del sensor
  *              que recibe el hook) -> file-scope en este archivo.
  *            - Los contextos por task -> `static` locales de App_Init: duracion
  *              estatica (sobreviven al return) pero solo App_Init los ve.
  *          No se usan objetos locales de main(): el scheduler pisa ese stack.
  *
  *          Semaforos, colas y tasks se crean con las *Static de FreeRTOS (sin
  *          heap): cada handle lleva su StaticQueue_t/StaticSemaphore_t/
  *          StaticTask_t + storage/stack como `static` local. Unica excepcion:
  *          el queue set del PID (`xQueueCreateSet`), que esta version de
  *          FreeRTOS solo compila con configSUPPORT_DYNAMIC_ALLOCATION==1 (no
  *          existe variante static); por eso el heap sigue habilitado.
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
#include "tim.h"        /* htim2 (echo), htim3 (pwm), htim4 (tick) */
#include "adc.h"        /* hadc1 */

/* --- Lo que cruza a la ISR: file-scope para que lo vean los hooks --------- */
static SemaphoreHandle_t      SemTimer;    /* lo da la ISR de TIM4 (100 ms)   */
static SemaphoreHandle_t      SemSensor;   /* lo da la ISR de Input Capture   */
static HC_SR04_HandleTypeDef *s_sensor;    /* handle del sensor (recibe hook) */

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
  /* --- Semaforos binarios (file-scope: los usan los hooks) ---
   * *Static: el control block es solo para la creacion, no hace falta que lo
   * vea el hook, asi que puede ser static local (igual que los contextos). */
  static StaticSemaphore_t s_sem_timer_cb;
  static StaticSemaphore_t s_sem_sensor_cb;
  SemTimer  = xSemaphoreCreateBinaryStatic(&s_sem_timer_cb);
  SemSensor = xSemaphoreCreateBinaryStatic(&s_sem_sensor_cb);
  if (SemTimer == NULL || SemSensor == NULL) { Error_Handler(); }

  /* --- Colas de profundidad 1 (sus handles se guardan en los contextos) ---
   * *Static: el storage y el control block deben durar tanto como la cola,
   * por eso static (no automatico), aunque solo App_Init los referencia. */
  static StaticQueue_t s_queue_pos_cb;
  static uint8_t       s_queue_pos_storage[1 * sizeof(float)];
  static StaticQueue_t s_queue_pos_fil_cb;
  static uint8_t       s_queue_pos_fil_storage[1 * sizeof(PosFil_t)];
  static StaticQueue_t s_queue_objetivo_cb;
  static uint8_t       s_queue_objetivo_storage[1 * sizeof(float)];
  static StaticQueue_t s_queue_angulo_cb;
  static uint8_t       s_queue_angulo_storage[1 * sizeof(float)];

  QueueHandle_t QueuePos      = xQueueCreateStatic(1, sizeof(float), s_queue_pos_storage, &s_queue_pos_cb);
  QueueHandle_t QueuePosFil   = xQueueCreateStatic(1, sizeof(PosFil_t), s_queue_pos_fil_storage, &s_queue_pos_fil_cb);
  QueueHandle_t QueueObjetivo = xQueueCreateStatic(1, sizeof(float), s_queue_objetivo_storage, &s_queue_objetivo_cb);
  QueueHandle_t QueueAngulo   = xQueueCreateStatic(1, sizeof(float), s_queue_angulo_storage, &s_queue_angulo_cb);
  if (QueuePos == NULL || QueuePosFil == NULL ||
      QueueObjetivo == NULL || QueueAngulo == NULL) { Error_Handler(); }

  /* --- Queue set del PID (QueuePosFil + QueueObjetivo) ---------------------
   * Longitud 4, no 2. La regla de FreeRTOS (suma de las profundidades = 1+1)
   * vale para colas normales, pero aca se publica con xQueueOverwrite: cada
   * escritura genera un aviso al set AUNQUE la cola ya tuviera un dato sin
   * leer, asi que los avisos pendientes pueden superar la cantidad de datos. Si
   * el contenedor se llena, FreeRTOS pega en un configASSERT, que en este
   * proyecto es taskDISABLE_INTERRUPTS() + for(;;): un cuelgue mudo. Dos slots
   * de mas cuestan 16 bytes.
   *
   * Sin variante *Static: xQueueCreateSet() esta compilado solo bajo
   * configSUPPORT_DYNAMIC_ALLOCATION==1 en esta version de FreeRTOS (no existe
   * xQueueCreateSetStatic). Es la unica asignacion dinamica que queda en todo
   * el proyecto; por eso el heap sigue habilitado en FreeRTOSConfig.h.
   *
   * Alternativa que se evaluo y no se uso: reemplazar este queue set por
   * notificaciones de tarea (xTaskNotify con eSetBits en cada productor,
   * xTaskNotifyWait en PidTask). Un bit no tiene "profundidad" -- prender un
   * bit ya prendido no desborda nada, asi que se saca de encima el riesgo del
   * configASSERT de arriba sin depender de un margen empirico, y de paso esta
   * asignacion dinamica dejaria de existir. Se prefirio quedarse con colas
   * (IPC uniforme en todo el proyecto) y aceptar el margen; queda anotado por
   * si el desborde llegara a darse en la practica. */
  QueueSetHandle_t QueueSetPid = xQueueCreateSet(4);
  if (QueueSetPid == NULL) { Error_Handler(); }
  if (xQueueAddToSet(QueuePosFil,   QueueSetPid) != pdPASS) { Error_Handler(); }
  if (xQueueAddToSet(QueueObjetivo, QueueSetPid) != pdPASS) { Error_Handler(); }

  /* --- Sensor HC-SR04: Create + Init + callback ---
   * El rango fisico del sensor es fijo (constantes de hardware en el driver);
   * la ventana util de la barra vive en SensorTask (SENSOR_MIN_CM/MAX_CM). */
  s_sensor = HC_SR04_Create();
  if (s_sensor == NULL) { Error_Handler(); }
  TimerChannel_t echo = { &htim2, TIM_CHANNEL_1 };
  GpioPin_t      trig = { TRIG_GPIO_Port, TRIG_Pin };
  if (HC_SR04_Init(s_sensor, echo, trig) != HC_SR04_OK) { Error_Handler(); }
  HC_SR04_SetCompleteCallback(s_sensor, App_OnSensorComplete_FromISR);

  /* --- Servo MG90S: Create + Init + recorrido + nivelar al arranque --- */
  Servo_HandleTypeDef *servo = Servo_Create();
  if (servo == NULL) { Error_Handler(); }
  TimerChannel_t pwm = { &htim3, TIM_CHANNEL_1 };
  if (Servo_Init(servo, pwm) != SERVO_OK) { Error_Handler(); }
  if (Servo_SetTravel(servo, SERVO_MIN_DEG, SERVO_MAX_DEG) != SERVO_OK) { Error_Handler(); }
  Servo_SetAngle(servo, SERVO_LEVEL_DEG);   /* barra nivelada antes de las tasks */

  /* --- Potenciometro: Create + Init (el ADC ya lo configuro CubeMX) ---
   * El pote solo lee normalizado; el mapeo a cm del setpoint
   * (POTENTIOMETER_MIN_CM/MAX_CM via linear_map) lo hace PotTask. */
  Potentiometer_HandleTypeDef *pot = Potentiometer_Create();
  if (pot == NULL) { Error_Handler(); }
  if (Potentiometer_Init(pot, &hadc1) != POTENTIOMETER_OK) { Error_Handler(); }

  /* --- PID: Create + Init + limites + banda de integracion --- */
  PID_HandleTypeDef *pid = PID_Create();
  if (pid == NULL) { Error_Handler(); }
  PID_Init(pid, PID_KP, PID_KI, PID_KD, PID_DT);
  PID_SetLimits(pid, PID_OUT_MIN, PID_OUT_MAX);
  PID_SetIntegralBand(pid, PID_I_BAND);

  /* --- Kalman: Create + Init (el Reset con la 1a muestra lo hace la task) --- */
  Kalman_HandleTypeDef *kalman = Kalman_Create();
  if (kalman == NULL) { Error_Handler(); }
  Kalman_Init(kalman, KALMAN_DT, KALMAN_Q, KALMAN_R, 0.0f);

  /* --- Tick de 100 ms (TIM4 en modo base con interrupcion) --- */
  if (HAL_TIM_Base_Start_IT(&htim4) != HAL_OK) { Error_Handler(); }

  /* --- Contextos por task (static locales: sobreviven al return) --- */
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
  pid_ctx.queue_set      = QueueSetPid;
  pid_ctx.queue_pos_fil  = QueuePosFil;
  pid_ctx.queue_objetivo = QueueObjetivo;
  pid_ctx.queue_angulo   = QueueAngulo;

  static TaskMotorContext motor_ctx;
  motor_ctx.servo        = servo;
  motor_ctx.queue_angulo = QueueAngulo;

  static TaskPotContext pot_ctx;
  pot_ctx.pot            = pot;
  pot_ctx.queue_objetivo = QueueObjetivo;

  /* --- Tasks (una por archivo, prioridades en app_config.h) ---
   * *Static: TCB y stack de cada task, static locales por la misma razon que
   * las colas de arriba. */
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
