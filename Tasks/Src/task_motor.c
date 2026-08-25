/**
  ******************************************************************************
  * @file    task_motor.c
  * @brief   Task del actuador (prio 4). En APP_MOTOR_MODE_LOOP recibe el angulo
  *          de QueueAngulo y lo aplica al servo, con toggle de LD2 (PA5) como
  *          heartbeat. En cualquier otro modo corre una rutina de banco que no
  *          retorna nunca: el lazo de control no arranca.
  *
  *          Esta task habla SOLO en grados: los microsegundos son asunto del
  *          driver. El pulso efectivo se lee con Servo_GetPulseUs() para la
  *          traza -- observar no es comandar --, que es la unica forma de
  *          distinguir "el firmware no manda" de "el firmware manda y el servo
  *          no responde": la traza del PID muestra el angulo PUBLICADO, no el
  *          APLICADO. El volcado de CEN/CC1E/PSC/ARR si mira los registros: es
  *          un diagnostico del PERIFERICO, no una forma de comandar el servo.
  ******************************************************************************
  */

#include "task_motor.h"
#include "app.h"
#include "app_config.h"

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "tim.h"        /* htim3 */

/* ===================== Utilitarios de banco ================================ */

#if (APP_LOG_ENABLED == 1)
/* Pulso que esta saliendo por el pin, para la traza. Via el driver: la task no
 * tiene por que saber en que registro vive el ancho de pulso. */
static float servo_pulse_us(void)
{
  uint16_t us = 0u;
  (void)Servo_GetPulseUs(&g_servo, &us);
  return (float)us;
}

/* Volcado del estado real del periferico de PWM. Si CEN=0 el timer no corre; si
 * CC1E=0 la salida del canal esta deshabilitada y el pin no conmuta. Aca si se
 * miran los registros a proposito: el diagnostico es DEL PERIFERICO. */
static void servo_dump_pwm_state(void)
{
  App_LogMsg("--- estado real del PWM (TIM3_CH1 -> PA6) ---");
  App_LogMsgF("  CEN  (timer corriendo) = ",
              (htim3.Instance->CR1  & TIM_CR1_CEN)   ? 1.0f : 0.0f);
  App_LogMsgF("  CC1E (salida CH1 on)   = ",
              (htim3.Instance->CCER & TIM_CCER_CC1E) ? 1.0f : 0.0f);
  App_LogMsgF("  PSC  (esperado 15)     = ", (float)htim3.Instance->PSC);
  App_LogMsgF("  ARR  (esperado 19999)  = ", (float)htim3.Instance->ARR);
  App_LogMsgF("  pulso actual us        = ", servo_pulse_us());
}
#endif

#if (APP_MOTOR_MODE != APP_MOTOR_MODE_LOOP)
/* Aplica un angulo, lo reporta con el pulso efectivo y espera. Toda rutina de
 * banco pasa por aca, asi la traza tiene siempre el mismo formato. */
static void servo_goto(float deg, uint32_t hold_ms)
{
  Servo_SetAngle(&g_servo, deg);
  HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
  vTaskDelay(pdMS_TO_TICKS(hold_ms));

#if (APP_LOG_ENABLED == 1)
  /* La traza sale DESPUES de la espera, y a proposito: asi `z_cm` es la posicion
   * de REPOSO de la pelota a este angulo, no la de transito. Esa tabla
   * angulo -> reposo es el dato de calibracion del montaje, y mide de una sola
   * vez las tres cosas que no se pueden deducir:
   *   1) la HORIZONTAL real: el angulo donde la pelota no se va para ningun lado
   *      esté donde esté (si no coincide con SERVO_LEVEL_DEG, se corrige ahi).
   *   2) el UMBRAL DE ARRANQUE a cada lado: cuanto hay que pasarse de la
   *      horizontal para que se despegue. No tiene por que ser simetrico.
   *   3) si la barra es PLANA: si el reposo depende de donde estaba la pelota,
   *      hay hondonadas locales y el umbral es distinto en cada punto -- eso no
   *      lo arregla ninguna ganancia.
   * SensorTask corre en todos los modos, asi que g_dbg_raw esta siempre fresco. */
  App_LogMsgF("ang=", deg);
  App_LogMsgF("  us=", servo_pulse_us());
  App_LogMsgF("  z_cm=", g_dbg_raw);
#endif
}
#endif

/* ===================== Rutinas de banco ==================================== */

#if (APP_MOTOR_MODE == APP_MOTOR_MODE_SWEEP)
/* Barrido chico alrededor del centro. APP_SWEEP_CYCLES = 0 -> infinito. */
static void motor_mode_sweep(void)
{
  const float centro = SERVO_CENTER_DEG;
  const float pasos[4] = { centro + APP_SWEEP_DEG, centro,
                           centro - APP_SWEEP_DEG, centro };

#if (APP_SWEEP_CYCLES == 0)
  for (;;)
#else
  for (uint32_t c = 0u; c < APP_SWEEP_CYCLES; c++)
#endif
  {
    for (uint32_t i = 0u; i < 4u; i++) { servo_goto(pasos[i], APP_SWEEP_STEP_MS); }
  }
}
#endif

#if (APP_MOTOR_MODE == APP_MOTOR_MODE_ENDPOINTS)
/* Extremo a extremo, infinito: el estimulo mas grande posible. */
static void motor_mode_endpoints(void)
{
#if (APP_LOG_ENABLED == 1)
  uint32_t ciclo = 0u;
#endif

  for (;;)
  {
#if (APP_LOG_ENABLED == 1)
    /* El contador es el que da valor al test: un numero que sigue subiendo dice
     * que el firmware no se colgo y que la Nucleo no se reseteo por brownout
     * (un reset arranca el contador de cero y reimprime el banner). */
    App_LogMsgF("-- ciclo ", (float)(++ciclo));
#endif
    servo_goto(APP_ENDPOINT_LO_DEG, APP_ENDPOINT_STEP_MS);
    servo_goto(APP_ENDPOINT_HI_DEG, APP_ENDPOINT_STEP_MS);
  }
}
#endif

#if (APP_MOTOR_MODE == APP_MOTOR_MODE_STEP)
/* Escalera LO -> HI -> LO, infinita. Herramienta de calibracion: se mira la
 * barra y se anota en que angulo queda horizontal y donde toca los topes. */
static void motor_mode_step(void)
{
  const int n = (int)((APP_STEP_HI_DEG - APP_STEP_LO_DEG) / APP_STEP_DEG);

  for (;;)
  {
#if (APP_LOG_ENABLED == 1)
    App_LogMsg("-- subiendo --");
#endif
    for (int k = 0; k <= n; k++)
    {
      servo_goto(APP_STEP_LO_DEG + (float)k * APP_STEP_DEG, APP_STEP_MS);
    }
#if (APP_LOG_ENABLED == 1)
    App_LogMsg("-- bajando (comparar posiciones: revela juego mecanico) --");
#endif
    for (int k = n - 1; k >= 1; k--)
    {
      servo_goto(APP_STEP_LO_DEG + (float)k * APP_STEP_DEG, APP_STEP_MS);
    }
  }
}
#endif

#if (APP_MOTOR_MODE == APP_MOTOR_MODE_HOLD)
/* Clavado en un angulo fijo: para montar el horn o para verificar si el temblor
 * es electrico (el comando no cambia ni un microsegundo). */
static void motor_mode_hold(void)
{
  servo_goto(APP_HOLD_DEG, 1u);

  /* BANCO DEL SENSOR. La barra queda clavada y no se mueve NUNCA mas, asi que
   * todo cambio de z_cm es la pelota -- o lo que el sensor este viendo en su
   * lugar. Sirve para la pregunta que ninguna otra traza contesta: el sensor,
   * ve la pelota? Procedimiento:
   *   1) Movela despacio a mano de una punta a la otra: z_cm tiene que
   *      seguirla en todo el recorrido, sin saltos ni zonas donde se congela.
   *   2) SACALA de la barra: z_cm TIENE que cambiar. Si sigue marcando lo
   *      mismo, el sensor nunca la estuvo viendo y esta enganchado a algo fijo
   *      (lo mas probable: la superficie de la barra, que el cono de ~15 grados
   *      roza y devuelve un eco mucho mas fuerte que el de la pelota). */
  for (;;)
  {
    vTaskDelay(pdMS_TO_TICKS(500));
#if (APP_LOG_ENABLED == 1)
    App_LogMsgF("z_cm=", g_dbg_raw);
#endif
  }
}
#endif

/* =============================== Task ===================================== */

void MotorTask(void *argument)
{
  (void)argument;

  Servo_Status init_st = Servo_Init(&g_servo, &htim3, TIM_CHANNEL_1);  /* arranca el PWM y centra */

  /* Lo unico que declara la aplicacion: la guarda contra los topes, en grados.
   * La recta us <-> grados es asunto del driver. */
  Servo_Status trv_st = Servo_SetTravel(&g_servo, SERVO_MIN_DEG, SERVO_MAX_DEG);

  Servo_SetAngle(&g_servo, SERVO_LEVEL_DEG);   /* arrancar nivelado */

#if (APP_LOG_ENABLED == 1)
  App_LogMsgF("MOTOR: Servo_Init status (0=OK) = ", (float)init_st);
  /* Si trv_st no es 0, el driver RECHAZO la guarda y quedo la anterior (0..180,
   * o sea SIN guarda): sin este aviso el servo podria llegar a sus topes. */
  App_LogMsgF("MOTOR: Servo_SetTravel (0=OK)  = ", (float)trv_st);
  App_LogMsgF("MOTOR: us por grado = ", SERVO_US_PER_DEG);
  App_LogMsgF("MOTOR: guarda min deg = ", SERVO_MIN_DEG);
  App_LogMsgF("MOTOR: guarda max deg = ", SERVO_MAX_DEG);
  App_LogMsgF("MOTOR: APP_MOTOR_MODE = ", (float)APP_MOTOR_MODE);
  servo_dump_pwm_state();
#else
  (void)init_st;
  (void)trv_st;
#endif

#if (APP_MOTOR_MODE == APP_MOTOR_MODE_SWEEP)
#if (APP_LOG_ENABLED == 1)
  App_LogMsg("MODO SWEEP (el lazo NO arranca hasta terminar los ciclos)");
#endif
  motor_mode_sweep();
#elif (APP_MOTOR_MODE == APP_MOTOR_MODE_ENDPOINTS)
#if (APP_LOG_ENABLED == 1)
  App_LogMsg("MODO ENDPOINTS: extremo a extremo, infinito, el lazo NO arranca");
  App_LogMsg("  SERVO DESACOPLADO: 160 grados de horn de golpe en cada salto");
  App_LogMsgF("  extremo bajo deg=", APP_ENDPOINT_LO_DEG);
  App_LogMsgF("  extremo alto deg=", APP_ENDPOINT_HI_DEG);
#endif
  motor_mode_endpoints();
#elif (APP_MOTOR_MODE == APP_MOTOR_MODE_STEP)
#if (APP_LOG_ENABLED == 1)
  App_LogMsg("MODO STEP (calibracion): escalera, el lazo NO arranca");
  App_LogMsg("  anotar: angulo con la barra HORIZONTAL y angulos de tope");
#endif
  motor_mode_step();
#elif (APP_MOTOR_MODE == APP_MOTOR_MODE_HOLD)
#if (APP_LOG_ENABLED == 1)
  App_LogMsg("MODO HOLD: angulo fijo, el lazo NO arranca");
  App_LogMsg("  MONTAJE DEL HORN: aflojar el tornillo central, LEVANTAR el horn");
  App_LogMsg("  del eje estriado y volver a calzarlo con la barra nivelada.");
  App_LogMsg("  NO forzarlo a mano contra el servo alimentado: se comen los");
  App_LogMsg("  dientes del reductor.");
  App_LogMsg("  El estriado tiene 21 dientes = 17.1 grados por diente, asi que");
  App_LogMsg("  a mano se llega a +-8.6 grados; el resto va por software.");
  App_LogMsg("  Si queda desnivelada, ajustar SERVO_LEVEL_DEG en app_config.h.");
  App_LogMsg("  Para saber hacia donde: probar 95, y si empeora ir a 85. No se");
  App_LogMsg("  deduce del numero -- depende de como quedo calzado el horn.");
  App_LogMsgF("  1 grado = us: ", SERVO_US_PER_DEG);
  App_LogMsgF("  angulo pedido ahora = ", APP_HOLD_DEG);
#endif
  motor_mode_hold();
#endif
  /* --- APP_MOTOR_MODE_LOOP: lazo de control normal --- */
#if (APP_LOG_ENABLED == 1)
  uint32_t n = 0u;
#endif

  /* Estado del failsafe. Se usa para actuar SOLO en el flanco: nivelar una vez
   * al perder la pelota, y no reescribir el mismo angulo cada timeout. */
  uint8_t lost = 0u;

  for (;;)
  {
    float angle = 0.0f;
    if (xQueueReceive(QueueAngulo, &angle, pdMS_TO_TICKS(APP_MOTOR_TIMEOUT_MS)) == pdTRUE)
    {
      lost = 0u;
      Servo_SetAngle(&g_servo, angle);
      HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);   /* heartbeat del lazo */

#if (APP_LOG_ENABLED == 1)
      /* Una vez por segundo: angulo APLICADO y el pulso efectivo en el registro. */
      if ((++n % 10u) == 0u)
      {
        App_LogMsgF("MOTOR aplicado ang=", angle);
        App_LogMsgF("  -> us=", servo_pulse_us());
      }
#endif
    }
    else if (!lost)
    {
      /* FAILSAFE: pasaron APP_MOTOR_TIMEOUT_MS sin angulo nuevo, o sea que el
       * sensor dejo de ver la pelota. Antes esta task esperaba con
       * portMAX_DELAY: se quedaba bloqueada para siempre y el servo clavado en
       * la ultima inclinacion, que es exactamente la peor posicion para que la
       * pelota vuelva. Nivelar es lo unico razonable sin medicion.
       *
       * El integrador de PidTask NO se descarga aca, y no hace falta: PidTask
       * solo integra cuando LLEGA una posicion, asi que mientras el sensor no
       * entrega, el integrador queda congelado en vez de acumular. Lo que
       * conserva es la carga que tenia al perder la pelota -- unos pocos grados,
       * acotados por PID_I_BAND -- y la descarga al cruzar el setpoint se la
       * come en el primer cruce. */
      lost = 1u;
      Servo_SetAngle(&g_servo, SERVO_LEVEL_DEG);
#if (APP_LOG_ENABLED == 1)
      App_LogMsg("MOTOR: sin angulo nuevo -> barra NIVELADA (failsafe)");
#endif
    }
  }
}
