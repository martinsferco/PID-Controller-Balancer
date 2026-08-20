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

#endif /* APP_CONFIG_H */
