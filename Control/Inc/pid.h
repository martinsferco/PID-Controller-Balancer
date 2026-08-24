/**
  ******************************************************************************
  * @file    pid.h
  * @brief   PID discreto con derivada sobre la medicion (sin derivative-kick),
  *          anti-windup por integracion condicional (clamping) y saturacion de
  *          salida. Modulo PURO: solo <float>, sin HAL ni FreeRTOS.
  ******************************************************************************
  */

#ifndef PID_H
#define PID_H

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    float kp, ki, kd;     /* ganancias                                  */
    float dt;             /* paso de tiempo [s]                         */
    float out_min;        /* saturacion inferior de la salida           */
    float out_max;        /* saturacion superior de la salida           */
    float integ;          /* termino integral acumulado                 */
    float prev_meas;      /* medicion anterior (derivada sobre medida)  */
    int   first;          /* 1 = primer Compute (evita spike derivativo)*/
    int   anti_windup;    /* 1 = clamping condicional del integrador     */
} PID_t;

/**
  * @brief  Inicializa el PID. Limites por defecto muy amplios (sin saturar).
  *         Anti-windup habilitado por defecto.
  */
void  PID_Init(PID_t *pid, float kp, float ki, float kd, float dt);

/**
  * @brief  Fija los limites de saturacion de la salida.
  */
void  PID_SetLimits(PID_t *pid, float out_min, float out_max);

/**
  * @brief  Calcula la salida de control para el (setpoint, medicion) actuales.
  *         Derivada sobre la medicion (no sobre el error) => sin kick al mover
  *         el setpoint. Salida saturada a [out_min, out_max].
  */
float PID_Compute(PID_t *pid, float setpoint, float meas);

/**
  * @brief  Reinicia el estado interno (integral, derivada) sin tocar ganancias.
  */
void  PID_Reset(PID_t *pid);

#ifdef __cplusplus
}
#endif

#endif /* PID_H */
