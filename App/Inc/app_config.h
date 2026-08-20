/**
  ******************************************************************************
  * @file    app_config.h
  * @brief   Constantes y flags de configuracion de la aplicacion ball-and-beam.
  *          Un unico lugar para tunear stacks, prioridades, rango del sensor y
  *          periodos. Ampliable en etapas siguientes (Kalman/PID/pote).
  ******************************************************************************
  */

#ifndef APP_CONFIG_H
#define APP_CONFIG_H

/* --- Tasks ---------------------------------------------------------------- *
 * Mismos valores que el baseline (Etapa 0): el tuning fino de prioridades y
 * stacks llega en las etapas del pipeline RTOS. Stack en WORDS (no bytes).   */
#define SENSOR_TASK_STACK       256u
#define SERVO_TASK_STACK        256u
#define SENSOR_TASK_PRIO        1u
#define SERVO_TASK_PRIO         1u

/* --- Sensor HC-SR04 ------------------------------------------------------- */
#define SENSOR_MIN_CM           2.0f
#define SENSOR_MAX_CM           50.0f   /* ajustar al largo de la barra        */
#define SENSOR_PERIOD_MS        50u     /* ~20 Hz de muestreo                  */
#define SENSOR_ECHO_TIMEOUT_MS  80u     /* guarda > timeout interno del driver */

/* --- Servo MG90S (barrido de prueba) -------------------------------------- */
#define SERVO_SWEEP_MS          1000u   /* tiempo en cada extremo              */

/* --- Flags de test (opt-in; en produccion van en 0) ----------------------- */
#define APP_RUN_SELFTESTS       0       /* 1 = corre self-tests al arrancar    */

/* --- Filtro de Kalman ----------------------------------------------------- */
#define KALMAN_DT               0.1f    /* 100 ms (tick del sensor)            */
#define KALMAN_Q                0.05f   /* densidad ruido de proceso (tuning)  */
#define KALMAN_R                0.09f   /* varianza HC-SR04 ~ (0.3 cm)^2       */

/* --- Controlador PID (ganancias iniciales; tuning fino en Etapa 5) -------- */
#define PID_DT                  0.1f
#define PID_KP                  1.2f
#define PID_KI                  0.4f
#define PID_KD                  0.15f
#define PID_OUT_MIN             (-20.0f) /* grados relativos al centro          */
#define PID_OUT_MAX             (20.0f)
#define SERVO_CENTER_DEG        90.0f    /* angulo neutro de la barra           */

#endif /* APP_CONFIG_H */
