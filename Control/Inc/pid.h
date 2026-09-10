/**
  ******************************************************************************
  * @file    pid.h
  * @brief   PID discreto con derivada sobre la medicion, banda de
  *          integracion, anti-windup por integracion condicional y
  *          saturacion de salida.
  ******************************************************************************
  */

#ifndef PID_H
#define PID_H

#ifdef __cplusplus
extern "C" {
#endif

typedef struct PID PID_HandleTypeDef;

/**
  * @brief  Reserva un handle de un pool estatico interno. Devuelve
  *         NULL si el pool esta agotado.
  */
PID_HandleTypeDef *PID_Create(void);

/**
  * @brief  Inicializa el PID. Arranca sin saturacion practica, sin banda de
  *         integracion y con anti-windup activo.
  */
void  PID_Init(PID_HandleTypeDef *pid, float kp, float ki, float kd, float dt);

/** @brief Fija los limites de saturacion de la salida. */
void  PID_SetLimits(PID_HandleTypeDef *pid, float out_min, float out_max);

/**
  * @brief  Banda de integracion: el integrador solo acumula mientras
  *         |setpoint - meas| <= band. Con band no positivo, se integra siempre.
  *         Ademas se descarga cuando el error cambia de signo.
  */
void  PID_SetIntegralBand(PID_HandleTypeDef *pid, float band);

/**
  * @brief  Salida de control para el estado actual, con la velocidad de la
  *         medicion dada desde afuera.
  * @param  rate  velocidad de la MEDICION.
  */
float PID_ComputeRate(PID_HandleTypeDef *pid, float setpoint, float meas, float rate);

#ifdef __cplusplus
}
#endif

#endif // PID_H
