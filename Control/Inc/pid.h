/**
  ******************************************************************************
  * @file    pid.h
  * @brief   PID discreto con derivada sobre la medicion (sin derivative kick),
  *          banda de integracion, anti-windup por integracion condicional
  *          (clamping) y saturacion de salida. Modulo PURO: sin HAL ni FreeRTOS.
  ******************************************************************************
  */

#ifndef PID_H
#define PID_H

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
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
} PID_t;

/**
  * @brief  Inicializa el PID. Arranca sin saturacion practica (usar
  *         PID_SetLimits), sin banda de integracion y con anti-windup activo.
  */
void  PID_Init(PID_t *pid, float kp, float ki, float kd, float dt);

/** @brief Fija los limites de saturacion de la salida. */
void  PID_SetLimits(PID_t *pid, float out_min, float out_max);

/**
  * @brief  Banda de integracion: el integrador solo acumula mientras
  *         |setpoint - meas| <= band. Con band <= 0 integra siempre (default).
  *
  *         Lejos del setpoint el proporcional ya pide todo lo que el actuador
  *         puede dar, asi que ahi el integrador no aporta autoridad: solo se
  *         carga durante el transitorio para sobrepasar despues. El anti-windup
  *         no alcanza para eso, porque frena la carga recien cuando la salida YA
  *         satura. Elegir la banda apenas por encima del error estacionario que
  *         se quiere limpiar.
  *
  *         Ademas el integrador se DESCARGA cuando el error cambia de signo (la
  *         medicion cruzo el setpoint): con friccion seca, la carga que hizo
  *         falta para despegar la masa un instante despues la empuja de mas.
  */
void  PID_SetIntegralBand(PID_t *pid, float band);

/**
  * @brief  Salida de control para el (setpoint, medicion) actuales, estimando la
  *         velocidad de la medicion por diferencia finita. Es lo mejor que se
  *         puede hacer con un solo numero por tick; si el llamador tiene un
  *         estimador de estado, usar PID_ComputeRate().
  */
float PID_Compute(PID_t *pid, float setpoint, float meas);

/**
  * @brief  Igual que PID_Compute(), pero con la velocidad de la medicion dada
  *         desde afuera. Sirve cuando hay un estimador (aca el Kalman) que ya
  *         modela el ruido de la senal: da la misma cantidad mejor estimada, y
  *         evita filtrar dos veces, que costaria fase justo en el termino cuyo
  *         trabajo es aportarla.
  * @param  rate  velocidad de la MEDICION (no del error) y con su mismo signo:
  *               meas creciente => rate positivo. El termino D vale -kd*rate.
  *               Tiene que corresponder a la `meas` de esta misma llamada; el
  *               PID no puede verificarlo.
  */
float PID_ComputeRate(PID_t *pid, float setpoint, float meas, float rate);

/** @brief Reinicia el estado interno (integral, derivada) sin tocar ganancias. */
void  PID_Reset(PID_t *pid);

#ifdef __cplusplus
}
#endif

#endif /* PID_H */
