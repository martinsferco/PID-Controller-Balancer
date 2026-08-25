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
    float i_band;         /* solo integra con |err| <= i_band (0 = siempre) */
    float prev_meas;      /* medicion anterior (derivada sobre medida)  */
    float prev_err;       /* error anterior (deteccion de cruce por cero)*/
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
  * @brief  Banda de integracion: el integrador solo acumula mientras
  *         |setpoint - meas| <= band. Con band <= 0 integra siempre (default).
  *
  *         Para que sirve: lejos del setpoint el proporcional ya pide todo lo
  *         que el actuador puede dar, asi que el integrador no aporta autoridad
  *         -- lo unico que hace es cargarse durante el transitorio para
  *         sobrepasar cuando la medicion por fin llega. El anti-windup solo
  *         frena la carga cuando la salida YA esta saturada; la banda la frena
  *         tambien en la franja de error grande pero todavia sin saturar, que es
  *         de donde sale la mayor parte del sobrepaso.
  *
  *         Como elegirla: apenas mas grande que el error estacionario que se
  *         quiere limpiar. Ni cerca del rango completo de la medicion.
  *
  *         Ademas, el integrador se DESCARGA cuando el error cambia de signo
  *         (la medicion cruzo el setpoint). Con friccion seca eso es lo que
  *         corta el ciclo limite: la carga que hizo falta para despegar la
  *         pelota, un instante despues la empuja para el lado contrario.
  */
void  PID_SetIntegralBand(PID_t *pid, float band);

/**
  * @brief  Calcula la salida de control para el (setpoint, medicion) actuales.
  *         Derivada sobre la medicion (no sobre el error) => sin kick al mover
  *         el setpoint. Salida saturada a [out_min, out_max].
  *
  *         La velocidad de la medicion la estima el propio PID por diferencia
  *         finita, que es lo mejor que se puede hacer con un solo numero por
  *         tick. Si el llamador tiene un estimador de estado (un Kalman, un
  *         encoder), usar PID_ComputeRate(): da la MISMA magnitud pero mejor
  *         estimada, sin apilar un segundo filtro.
  */
float PID_Compute(PID_t *pid, float setpoint, float meas);

/**
  * @brief  Igual que PID_Compute(), pero con la velocidad de la medicion dada
  *         desde afuera (d(meas)/dt, en unidades de meas por segundo).
  *
  *         Para que sirve: la calidad de una derivada depende del modelo de
  *         ruido de la senal, y ese modelo no vive en el PID. Cuando hay un
  *         estimador de estado que ya lo tiene (aca el Kalman, con su Q y su R),
  *         su velocidad es la misma cantidad que el PID calcularia solo, pero
  *         estimada bien -- y encima sale del mismo update que la posicion, o
  *         sea que es CONSISTENTE con la `meas` de esta misma llamada. La
  *         alternativa (filtrar la derivada adentro del PID) filtra dos veces la
  *         misma senal y cuesta fase dos veces, que es carisimo justo en el
  *         termino cuyo unico trabajo es aportar adelanto de fase.
  *
  *         El riesgo de esta firma es pasar un `rate` que no corresponda a la
  *         `meas` de la misma llamada (otro dt, otra muestra, signo invertido):
  *         el PID no puede detectarlo. Por eso los dos viajan juntos en la misma
  *         estructura (PosFil_t) y por la misma cola, no por caminos separados.
  *
  * @param  rate  velocidad de la MEDICION, no del error, y con su mismo signo
  *               (meas creciente => rate positivo). El termino D vale -kd*rate.
  */
float PID_ComputeRate(PID_t *pid, float setpoint, float meas, float rate);

/**
  * @brief  Reinicia el estado interno (integral, derivada) sin tocar ganancias.
  */
void  PID_Reset(PID_t *pid);

#ifdef __cplusplus
}
#endif

#endif /* PID_H */
