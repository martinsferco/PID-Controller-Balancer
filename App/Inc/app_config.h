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

/* --- Tasks (prioridades del anteproyecto) --------------------------------- *
 * Stack en WORDS (no bytes). Prioridad mayor = mas urgente.                  *
 * Sensor 5, Kalman 4, Motor 4, PID 3, Pot 1.                                 *
 * Stacks dimensionados por rol; verificar margen real con                    *
 * uxTaskGetStackHighWaterMark (INCLUDE ya activo) y ajustar. La PID lleva mas *
 * porque hace printf (App_LogTrace) cuando el log esta activo.               */
#define SENSOR_TASK_STACK       256u
#define KALMAN_TASK_STACK       256u
#define PID_TASK_STACK          384u
#define MOTOR_TASK_STACK        192u
#define POT_TASK_STACK          192u

#define SENSOR_TASK_PRIO        5u
#define KALMAN_TASK_PRIO        4u
#define MOTOR_TASK_PRIO         4u
#define PID_TASK_PRIO           3u
#define POT_TASK_PRIO           1u

/* --- Sensor HC-SR04 ------------------------------------------------------- */
#define SENSOR_MIN_CM           2.0f
#define SENSOR_MAX_CM           50.0f   /* ajustar al largo de la barra        */
#define SENSOR_ECHO_TIMEOUT_MS  80u     /* guarda < periodo (100 ms), > timeout interno */

/* --- Potenciometro (setpoint) --------------------------------------------- */
#define POT_RANGE_CM            50.0f   /* largo util de la barra (mapea el pote)*/
#define POT_PERIOD_MS           200u    /* lectura del setpoint cada 200 ms    */

/* --- Flags de test / diagnostico (opt-in; produccion en 0 salvo LOG) ------ */
#define APP_RUN_SELFTESTS       0       /* 1 = corre self-tests al arrancar    */
#define APP_USE_SYNTHETIC_SENSOR 0      /* 1 = sensor sintetico (valida cadena sin HW) */
#define APP_LOG_ENABLED         0       /* 1 = traza del lazo por UART (subir para tuning) */

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

/* --- Calibracion del servo (AJUSTAR al recorrido mecanico real, Etapa 5) --- *
 * El mapeo por defecto del driver (1000-2000us = 0-180deg) puede no coincidir  *
 * con la mecanica; se recalibra con Servo_SetLimits en MotorTask. Empezar con   *
 * estos valores y ajustar contra los topes fisicos de la barra.                */
#define SERVO_MIN_US            1000u
#define SERVO_MAX_US            2000u
#define SERVO_MIN_DEG           0.0f
#define SERVO_MAX_DEG           180.0f

#endif /* APP_CONFIG_H */
