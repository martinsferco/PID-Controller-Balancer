/**
  ******************************************************************************
  * @file    pid.c
  * @brief   Implementacion del PID discreto con banda de integracion,
  *          anti-windup y saturacion de salida.
  ******************************************************************************
  */

#include "pid.h"

/* Definicion del struct opaco (oculta a los usuarios del .h). */
struct PID {
    float kp, ki, kd;     /* ganancias                                        */
    float dt;             /* paso de tiempo [s]                               */
    float out_min;        /* saturacion inferior de la salida                 */
    float out_max;        /* saturacion superior de la salida                 */
    float integ;          /* termino integral acumulado                       */
    float i_band;         /* solo integra con |err| <= i_band (0 = siempre)   */
    float prev_meas;      /* medicion anterior (derivada por diferencia)      */
    float prev_err;       /* error anterior (deteccion de cruce por cero)     */
    int   first;          /* 1 = primer Compute (evita el spike derivativo)   */
    int   anti_windup;    /* 1 = clamping condicional del integrador          */
};

/* Pool estatico de handles: el driver es dueno de la memoria (sin malloc). */
#ifndef PID_MAX_INSTANCES
#define PID_MAX_INSTANCES  1
#endif

static struct PID s_pool[PID_MAX_INSTANCES];
static unsigned   s_pool_count = 0u;

PID_HandleTypeDef *PID_Create(void)
{
    if (s_pool_count >= PID_MAX_INSTANCES) { return 0; }
    return &s_pool[s_pool_count++];
}

void PID_Init(PID_HandleTypeDef *pid, float kp, float ki, float kd, float dt)
{
    if (pid == 0) { return; }
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->dt = dt;
    pid->out_min = -1.0e9f;   /* sin saturar hasta que se llame SetLimits */
    pid->out_max =  1.0e9f;
    pid->integ = 0.0f;
    pid->i_band = 0.0f;       /* 0 = sin banda: integra siempre */
    pid->prev_meas = 0.0f;
    pid->prev_err = 0.0f;
    pid->first = 1;
    pid->anti_windup = 1;
}

void PID_SetLimits(PID_HandleTypeDef *pid, float out_min, float out_max)
{
    if (pid == 0) { return; }
    pid->out_min = out_min;
    pid->out_max = out_max;
}

void PID_SetIntegralBand(PID_HandleTypeDef *pid, float band)
{
    if (pid == 0) { return; }
    pid->i_band = band;
}

void PID_Reset(PID_HandleTypeDef *pid)
{
    if (pid == 0) { return; }
    pid->integ = 0.0f;
    pid->prev_meas = 0.0f;
    pid->prev_err = 0.0f;
    pid->first = 1;
}

float PID_Compute(PID_HandleTypeDef *pid, float setpoint, float meas)
{
    if (pid == 0) { return 0.0f; }

    /* En la primera llamada no hay muestra anterior: la derivada arranca en 0
     * para no meter un spike. */
    float rate = 0.0f;
    if (!pid->first) {
        rate = (meas - pid->prev_meas) / pid->dt;
    }

    return PID_ComputeRate(pid, setpoint, meas, rate);
}

float PID_ComputeRate(PID_HandleTypeDef *pid, float setpoint, float meas, float rate)
{
    if (pid == 0) { return 0.0f; }

    const float err = setpoint - meas;

    /* --- Proporcional --- */
    const float prop = pid->kp * err;

    /* --- Derivada sobre la MEDICION (sin kick ante un escalon de setpoint) ---
     * rate es d(meas)/dt y el error es (setpoint - meas), asi que
     * d(err)/dt = -rate: de ahi el signo. */
    const float deriv = -pid->kd * rate;

    /* --- Descarga al CRUZAR el setpoint --------------------------------------
     * Si el error cambio de signo, la medicion paso por el setpoint y toda la
     * carga que el integrador junto para llegar hasta aca ahora empuja para el
     * lado contrario.
     *
     * OJO al orden: esto tiene que leer prev_err ANTES de que se actualice mas
     * abajo. Si se actualiza primero, la comparacion queda err*err >= 0 y el
     * cruce no se detecta nunca (y el sintoma es no tener sintoma: el PID anda,
     * solo que la guarda no existe). */
    if (!pid->first && (err * pid->prev_err) < 0.0f) {
        pid->integ = 0.0f;
    }

    /* --- Integral tentativa, solo dentro de la banda ---
     * Sin <math.h> a proposito: el modulo no depende de nada. */
    const float abs_err = (err < 0.0f) ? -err : err;
    const int en_banda = (pid->i_band <= 0.0f) || (abs_err <= pid->i_band);
    const float integ_new = en_banda ? (pid->integ + pid->ki * err * pid->dt)
                                     : pid->integ;

    /* prev_meas se mantiene fresco aunque esta entrada no lo use, para que
     * alternar entre PID_Compute() y PID_ComputeRate() no genere un salto
     * derivativo con una muestra vieja. */
    pid->prev_meas = meas;
    pid->prev_err = err;
    pid->first = 0;

    float out = prop + integ_new + deriv;

    /* --- Saturacion + anti-windup (integracion condicional) ---
     * Si la salida satura y el error empuja aun mas hacia la saturacion, el
     * integrador se congela; si el error apunta a salir de ella, se permite
     * integrar. */
    if (out > pid->out_max) {
        out = pid->out_max;
        if (!pid->anti_windup || err < 0.0f) {
            pid->integ = integ_new;
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
