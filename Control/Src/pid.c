/**
  ******************************************************************************
  * @file    pid.c
  * @brief   Implementacion del PID discreto con anti-windup y saturacion.
  ******************************************************************************
  */

#include "pid.h"

void PID_Init(PID_t *pid, float kp, float ki, float kd, float dt)
{
    if (pid == 0) { return; }
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->dt = dt;
    pid->out_min = -1.0e9f;   /* sin saturar hasta que se llame SetLimits */
    pid->out_max =  1.0e9f;
    pid->integ = 0.0f;
    pid->prev_meas = 0.0f;
    pid->first = 1;
    pid->anti_windup = 1;
}

void PID_SetLimits(PID_t *pid, float out_min, float out_max)
{
    if (pid == 0) { return; }
    pid->out_min = out_min;
    pid->out_max = out_max;
}

void PID_Reset(PID_t *pid)
{
    if (pid == 0) { return; }
    pid->integ = 0.0f;
    pid->prev_meas = 0.0f;
    pid->first = 1;
}

float PID_Compute(PID_t *pid, float setpoint, float meas)
{
    if (pid == 0) { return 0.0f; }

    const float err = setpoint - meas;

    /* --- Proporcional --- */
    const float prop = pid->kp * err;

    /* --- Derivada sobre la MEDICION (evita el kick ante escalon de setpoint) --- */
    float d_meas = 0.0f;
    if (!pid->first) {
        d_meas = (meas - pid->prev_meas) / pid->dt;
    }
    pid->prev_meas = meas;
    pid->first = 0;
    const float deriv = -pid->kd * d_meas;

    /* --- Integral tentativa --- */
    const float integ_new = pid->integ + pid->ki * err * pid->dt;

    /* --- Salida sin saturar --- */
    float out = prop + integ_new + deriv;

    /* --- Saturacion + anti-windup (integracion condicional / clamping) ---
     * Si la salida satura y el error empujaria mas hacia la saturacion, se
     * congela el integrador (no se acumula). Si el error apunta a salir de la
     * saturacion, se permite integrar. Sin anti-windup, siempre acumula. */
    if (out > pid->out_max) {
        out = pid->out_max;
        if (!pid->anti_windup || err < 0.0f) {
            pid->integ = integ_new;   /* permitir (o forzar si no hay AW) */
        }
    } else if (out < pid->out_min) {
        out = pid->out_min;
        if (!pid->anti_windup || err > 0.0f) {
            pid->integ = integ_new;
        }
    } else {
        pid->integ = integ_new;       /* zona lineal: siempre integra */
    }

    return out;
}
