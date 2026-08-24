/**
  ******************************************************************************
  * @file    app_config.h
  * @brief   Constantes y flags de configuracion de la aplicacion ball-and-beam.
  *          Un unico lugar para tunear stacks, prioridades, rango del sensor y
  *          periodos.
  ******************************************************************************
  */

#ifndef APP_CONFIG_H
#define APP_CONFIG_H

/* ========================= DEBUG / BANCO ================================== *
 * APP_LOG_ENABLED: traza por USART2 (COM3 @115200 8N1). Ortogonal al modo.    *
 *                                                                            *
 * APP_MOTOR_MODE: QUE hace MotorTask. Un unico selector, asi nunca hay dos    *
 * modos compitiendo. El modo activo se imprime al arrancar.                  *
 *                                                                            *
 *   LOOP      lazo de control normal (entrega)                                *
 *   SWEEP     +-APP_SWEEP_DEG alrededor del centro, APP_SWEEP_CYCLES veces    *
 *   ENDPOINTS LO <-> HI infinito (el estimulo mas grande)                     *
 *   STEP      escalera LO -> HI -> LO en saltos: CALIBRAR                     *
 *   HOLD      clavado en APP_HOLD_DEG: para montar el horn                    *
 *                                                                            *
 * En todo modo != LOOP el lazo de control NO arranca (sensor y PID corren,    *
 * pero sus angulos se descartan). Desacoplar el horn de la barra antes.       */
#define APP_LOG_ENABLED         1

#define APP_MOTOR_MODE_LOOP      0
#define APP_MOTOR_MODE_SWEEP     1
#define APP_MOTOR_MODE_ENDPOINTS 2
#define APP_MOTOR_MODE_STEP      3
#define APP_MOTOR_MODE_HOLD      4

#define APP_MOTOR_MODE          APP_MOTOR_MODE_LOOP

/* APP_LOG_LOOP: traza periodica del LAZO (linea z/fil/sp/u/ang de PidTask y las
 * quejas por medicion de SensorTask). Se apaga sola en los modos de banco: ahi
 * SensorTask y PidTask siguen corriendo, pero sus angulos se descartan, asi que
 * su traza a 10 Hz solo tapa la del modo (y parece "salida vieja"). Los mensajes
 * de arranque y los del propio modo siguen saliendo con APP_LOG_ENABLED. */
#if (APP_LOG_ENABLED == 1) && (APP_MOTOR_MODE == APP_MOTOR_MODE_LOOP)
  #define APP_LOG_LOOP          1
#else
  #define APP_LOG_LOOP          0
#endif

/* --- SWEEP --- */
#define APP_SWEEP_DEG           8.0f    /* amplitud (+-, grados reales)       */
#define APP_SWEEP_CYCLES        3u      /* idas y vueltas; 0 = infinito       */
#define APP_SWEEP_STEP_MS       600u

/* --- ENDPOINTS --- */
#define APP_ENDPOINT_LO_DEG     SERVO_MIN_DEG
#define APP_ENDPOINT_HI_DEG     SERVO_MAX_DEG
#define APP_ENDPOINT_STEP_MS    1000u

/* --- STEP (calibracion) --------------------------------------------------- *
 * Recorre LO -> HI -> LO en saltos de APP_STEP_DEG, esperando APP_STEP_MS en  *
 * cada uno e imprimiendo el angulo y el pulso real. Mirando la barra anotas:  *
 *   1) en que angulo queda HORIZONTAL  -> va a SERVO_LEVEL_DEG               *
 *   2) en que angulos toca los topes   -> acotan SERVO_MIN_DEG/MAX_DEG       *
 * La bajada sirve para ver juego mecanico: si la barra no repite las mismas   *
 * posiciones subiendo y bajando, hay backlash en el acople.                  */
#define APP_STEP_LO_DEG         SERVO_MIN_DEG
#define APP_STEP_HI_DEG         SERVO_MAX_DEG
#define APP_STEP_DEG            5.0f    /* grados por salto                   */
#define APP_STEP_MS             1000u

/* --- HOLD --- */
#define APP_HOLD_DEG            SERVO_LEVEL_DEG  /* barra nivelada            */

/* --- Tasks (prioridades del anteproyecto) --------------------------------- *
 * Stack en WORDS (no bytes). Prioridad mayor = mas urgente.                  *
 * Sensor 5, Kalman 4, Motor 4, PID 3, Pot 1.                                 *
 * Con APP_LOG_ENABLED las tasks que hacen printf necesitan mas stack         *
 * (newlib-nano pide ~300 bytes); en produccion vuelven a los valores chicos.  */
#if (APP_LOG_ENABLED == 1) || (APP_MOTOR_MODE != APP_MOTOR_MODE_LOOP)
  #define SENSOR_TASK_STACK     384u
  #define MOTOR_TASK_STACK      384u
#else
  #define SENSOR_TASK_STACK     256u
  #define MOTOR_TASK_STACK      192u
#endif
#define KALMAN_TASK_STACK       256u
#define PID_TASK_STACK          384u
#define POT_TASK_STACK          192u

#define SENSOR_TASK_PRIO        5u
#define KALMAN_TASK_PRIO        4u
#define MOTOR_TASK_PRIO         4u
#define PID_TASK_PRIO           3u
#define POT_TASK_PRIO           1u

/* --- Barra (montaje fisico) ----------------------------------------------- *
 * Largo util de la barra medido DESDE LA CARA DEL SENSOR. Si el sensor esta   *
 * montado unos cm antes del inicio de la barra, sumarlos aca.                 */
#define BEAM_LENGTH_CM          30.0f

/* --- Sensor HC-SR04 ------------------------------------------------------- */
#define SENSOR_MIN_CM           3.0f    /* zona muerta real (2 cm es optimista) */
#define SENSOR_MAX_CM           32.0f   /* barra 30 cm + margen; mas lejos se descarta */
#define SENSOR_ECHO_TIMEOUT_MS  80u     /* guarda < periodo (100 ms), > timeout interno */

/* --- Potenciometro (setpoint) --------------------------------------------- */
#define POTENTIOMETER_MIN_CM             0.0f            /* cm en el extremo bajo del pote  */
#define POTENTIOMETER_MAX_CM             BEAM_LENGTH_CM  /* cm en el extremo alto           */
#define POT_PERIOD_MS           200u    /* lectura del setpoint cada 200 ms    */

/* --- Setpoint fijo (mientras no exista el pote) --------------------------- *
 * Con el pote sin cablear, PA4 queda flotando y el ADC devuelve basura: el    *
 * setpoint bailaria entre 0 y BEAM_LENGTH_CM. Con el flag en 1, PotTask NO    *
 * lee el ADC y publica SETPOINT_FIXED_CM (las 5 tasks siguen existiendo).     *
 * Volver a 0 cuando el potenciometro este cableado a 3.3 V / PA4.             */
#define APP_USE_FIXED_SETPOINT  1
#define SETPOINT_FIXED_CM       (BEAM_LENGTH_CM * 0.5f)  /* mitad de la barra  */

/* --- Filtro de Kalman ----------------------------------------------------- */
#define KALMAN_DT               0.1f    /* 100 ms (tick del sensor)            */
#define KALMAN_Q                0.05f   /* densidad ruido de proceso (tuning)  */
#define KALMAN_R                0.09f   /* varianza HC-SR04 ~ (0.3 cm)^2       */

/* --- Controlador PID ------------------------------------------------------ *
 * Primer test: SOLO proporcional (KI = KD = 0) y rango de accion chico. Subir *
 * KD para amortiguar y KI al final (chico, 0.05-0.1).                         */
#define PID_DT                  0.1f
#define PID_KP                  0.8f
#define PID_KI                  0.0f
#define PID_KD                  0.0f
#define PID_OUT_MIN             (-8.0f)  /* grados relativos al centro; +-20 es *
                                          * demasiada pendiente en 30 cm       */
#define PID_OUT_MAX             (8.0f)
/* SERVO_CENTER_DEG (= la horizontal) esta en el bloque de calibracion del servo:
 * el lazo manda SERVO_CENTER_DEG + SERVO_DIR*u, con u acotado por PID_OUT_*.
 * Con 90 +- 8 se pide 82..98 grados: bien adentro de la guarda de 10..170.     */

/* Sentido del actuador: +1 si un error positivo (pelota mas cerca que el
 * setpoint) debe inclinar la barra para alejarla del sensor. Si en el primer
 * test la barra inclina al reves, poner -1.0f (no hace falta desarmar nada). */
#define SERVO_DIR               (+1.0f)

/* --- Recorrido del servo --------------------------------------------------- *
 * La recta us <-> grados NO esta aca: vive en el driver (servo_mg90s.h), que es
 * su dueno -- un MG90S recorre 0..180 grados entre 500 y 2500 us, y eso es un
 * hecho del componente. Aca solo se declara lo que SI es decision de la
 * aplicacion, y todo en GRADOS:
 *
 *  - SERVO_MIN_DEG / SERVO_MAX_DEG: la GUARDA contra los topes fisicos. El
 *    driver la aplica a todo llamador via Servo_SetTravel(). VERIFICADO EN
 *    HARDWARE 2026-08-23: 10 y 170 se alcanzan sin zumbido, y el test ENDPOINTS
 *    aguanta extremo a extremo sin resets.
 *  - SERVO_LEVEL_DEG: el angulo con la barra HORIZONTAL, o sea la referencia del
 *    lazo. VERIFICADO EN HARDWARE 2026-08-23 en modo HOLD: con 90 grados
 *    (1500 us) la barra queda horizontal, el horn quedo montado en el diente
 *    justo. Si algun dia hay que remontarlo, se re-mide en HOLD; un grado vale
 *    SERVO_US_PER_DEG us (lo define el driver).                                */
#define SERVO_MIN_DEG           10.0f   /* recorrido permitido: piso           */
#define SERVO_MAX_DEG           170.0f  /* recorrido permitido: techo          */
#define SERVO_LEVEL_DEG         90.0f   /* barra HORIZONTAL (referencia)       */

/* Centro del movimiento de la barra = la horizontal. Es el 0 del PID: el lazo
 * manda SERVO_CENTER_DEG + SERVO_DIR*u, con u en [PID_OUT_MIN, PID_OUT_MAX]. */
#define SERVO_CENTER_DEG        SERVO_LEVEL_DEG

#endif /* APP_CONFIG_H */
