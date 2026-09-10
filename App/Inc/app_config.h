/**
  ******************************************************************************
  * @file    app_config.h
  * @brief   Configuracion de la aplicacion sistema de control PID para balanceo.
  *          Se definen todas las constantes de la maqueta, las ganancias del lazo
  *          de control y los parametros de las tasks de FreeRTOS.
  ******************************************************************************
  */

#ifndef APP_CONFIG_H
#define APP_CONFIG_H

#include "hc_sr04.h"

// Barra
#define BEAM_LENGTH_CM          30.0f   // largo util, desde la cara del sensor

// Sensor HC-SR04
#define SENSOR_SAFETY_MARGIN_CM 2.0f
#define SENSOR_MIN_CM           (HC_SR04_HW_MIN_CM + SENSOR_SAFETY_MARGIN_CM)
#define SENSOR_MAX_CM           20.0f   // carrito contra su limite mecanico
#define SENSOR_ECHO_TIMEOUT_MS  50u     

// Potenciometro
#define POTENTIOMETER_MIN_CM    SENSOR_MIN_CM
#define POTENTIOMETER_MAX_CM    SENSOR_MAX_CM
#define SETPOINT_DEFAULT_CM     ((POTENTIOMETER_MIN_CM + POTENTIOMETER_MAX_CM) * 0.5f)

// Servo MG90S
#define SERVO_MIN_DEG           10.0f
#define SERVO_MAX_DEG           170.0f
#define SERVO_LEVEL_DEG         90.0f   // barra horizontal
#define SERVO_CENTER_DEG        SERVO_LEVEL_DEG
#define SERVO_DIR               (-1.0f)

// Filtro de Kalman
#define KALMAN_DT               0.1f    // 100 ms: el tick del sensor
#define KALMAN_Q                20.0f   // densidad de ruido de proceso
#define KALMAN_R                0.04f   // varianza de medicion (sigma 0.2 cm)

// Controlador PID
#define PID_DT                  0.1f
#define PID_KP                  8.0f
#define PID_KI                  1.0f
#define PID_KD                  3.6f
#define PID_I_BAND              4.0f
#define PID_OUT_MIN             (-80.0f)
#define PID_OUT_MAX             (80.0f)

// Tasks y tiempos de FreeRTOS
#define SENSOR_TASK_STACK       400u
#define KALMAN_TASK_STACK       300u
#define PID_TASK_STACK          400u
#define MOTOR_TASK_STACK        400u
#define POT_TASK_STACK          300u

#define SENSOR_TASK_PRIO        5u
#define KALMAN_TASK_PRIO        4u
#define MOTOR_TASK_PRIO         4u
#define PID_TASK_PRIO           3u
#define POT_TASK_PRIO           1u

#define POT_PERIOD_MS           200u

#define MOTOR_TASK_TIMEOUT_MS   500u    // MotorTask: nivela la barra
#define KALMAN_TASK_TIMEOUT_MS  500u    // KalmanTask: se re-arma para Kalman_Reset
#define PID_TASK_TIMEOUT_MS     500u    // PidTask: sigue esperando, sin resetear

// Coherencia entre el lazo y el servo
_Static_assert((SERVO_CENTER_DEG + SERVO_DIR * PID_OUT_MAX) <= SERVO_MAX_DEG,
               "el angulo del lazo con PID_OUT_MAX pasa SERVO_MAX_DEG");
_Static_assert((SERVO_CENTER_DEG + SERVO_DIR * PID_OUT_MAX) >= SERVO_MIN_DEG,
               "el angulo del lazo con PID_OUT_MAX pasa SERVO_MIN_DEG");
_Static_assert((SERVO_CENTER_DEG + SERVO_DIR * PID_OUT_MIN) <= SERVO_MAX_DEG,
               "el angulo del lazo con PID_OUT_MIN pasa SERVO_MAX_DEG");
_Static_assert((SERVO_CENTER_DEG + SERVO_DIR * PID_OUT_MIN) >= SERVO_MIN_DEG,
               "el angulo del lazo con PID_OUT_MIN pasa SERVO_MIN_DEG");

#endif // APP_CONFIG_H
