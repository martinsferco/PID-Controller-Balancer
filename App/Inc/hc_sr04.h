/**
  ******************************************************************************
  * @file    hc_sr04.h
  * @brief   Driver HAL para el sensor ultrasonico HC-SR04 (medicion por Input
  *          Capture, basado en interrupciones, sin busy-waiting).
  *
  *          Caracteristicas:
  *            - Multi-instancia (varios sensores con distinto TRIG/canal de TIM).
  *            - Sin variables globales compartidas (salvo el registro interno de
  *              instancias necesario por el callback global del HAL).
  *            - Manejo de timeout y descarte de lecturas fuera de rango.
  *            - RTOS-agnostico: no llama a FreeRTOS. Expone un hook (puntero a
  *              funcion) que se invoca al completar una medicion; desde ahi vos
  *              podes dar un semaforo con xSemaphoreGiveFromISR(), etc.
  *
  *          Requisitos de CubeMX (ver Manual_CubeMX.md):
  *            - TIM en Input Capture direct mode, Prescaler=83 (1 us/tick).
  *            - Preferentemente un timer de 32 bits (TIM2/TIM5) para no preocuparse
  *              por el overflow. Igual el driver maneja el wrap-around.
  *            - Pin TRIG como GPIO_Output push-pull.
  ******************************************************************************
  */

#ifndef HC_SR04_H
#define HC_SR04_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"   /* trae stm32f4xx_hal.h y los tipos HAL */
#include <stdint.h>

/* Estados de retorno de la API */
typedef enum {
    HC_SR04_OK = 0,     /* hay una medicion valida disponible            */
    HC_SR04_BUSY,       /* medicion en curso, todavia no hay dato         */
    HC_SR04_TIMEOUT,    /* no llego el echo dentro del tiempo esperado    */
    HC_SR04_INVALID,    /* medicion fuera de rango (descartada)           */
    HC_SR04_ERROR       /* parametros invalidos / fallo de HAL            */
} HC_SR04_Status;

/* Estado interno de la maquina de captura */
typedef enum {
    HC_SR04_STATE_IDLE = 0,
    HC_SR04_STATE_WAIT_RISE,
    HC_SR04_STATE_WAIT_FALL
} HC_SR04_FsmState;

typedef struct HC_SR04_Handle HC_SR04_HandleTypeDef;

/* Hook opcional invocado (en contexto de ISR) al completar una medicion.
 * Usalo para dar un semaforo a tu TaskSensor: ...GiveFromISR(). */
typedef void (*HC_SR04_CompleteCb)(HC_SR04_HandleTypeDef *h);

struct HC_SR04_Handle {
    /* --- Configuracion (seteada en Init) --- */
    TIM_HandleTypeDef *htim;       /* timer en modo Input Capture            */
    uint32_t           channel;    /* TIM_CHANNEL_1..4                       */
    uint32_t           active_ch;  /* HAL_TIM_ACTIVE_CHANNEL_x (uso interno) */
    GPIO_TypeDef      *trig_port;  /* puerto del pin TRIG                    */
    uint16_t           trig_pin;   /* pin TRIG                               */

    /* --- Parametros ajustables (tienen default en Init) --- */
    float    min_cm;               /* lectura minima valida (def 2.0)        */
    float    max_cm;               /* lectura maxima valida (def 400.0)      */
    uint32_t timeout_ms;           /* timeout de medicion (def 60 ms)        */

    /* --- Estado en runtime --- */
    volatile HC_SR04_FsmState state;
    volatile uint32_t t_rise;      /* captura del flanco de subida [us]      */
    volatile uint32_t t_fall;      /* captura del flanco de bajada [us]      */
    volatile float    distance_cm; /* ultima distancia calculada             */
    volatile uint8_t  data_ready;  /* hay dato nuevo sin leer (0/1)          */
    uint32_t          trigger_tick;/* HAL_GetTick() al disparar (timeout)    */

    /* --- Hook opcional --- */
    HC_SR04_CompleteCb on_complete;
};

/**
  * @brief  Inicializa una instancia del sensor y la registra para el dispatch.
  * @param  h         puntero al handle (lo provee el usuario, sin malloc)
  * @param  htim      timer ya inicializado por CubeMX en Input Capture
  * @param  channel   TIM_CHANNEL_1..4 usado para el ECHO
  * @param  trig_port puerto del pin TRIG (ej. TRIG_GPIO_Port)
  * @param  trig_pin  pin TRIG (ej. TRIG_Pin)
  * @retval HC_SR04_OK / HC_SR04_ERROR
  */
HC_SR04_Status HC_SR04_Init(HC_SR04_HandleTypeDef *h,
                            TIM_HandleTypeDef *htim,
                            uint32_t channel,
                            GPIO_TypeDef *trig_port,
                            uint16_t trig_pin);

/**
  * @brief  Setea el hook que se llama al completar una medicion (contexto ISR).
  *         Pasar NULL para deshabilitarlo.
  */
void HC_SR04_SetCompleteCallback(HC_SR04_HandleTypeDef *h, HC_SR04_CompleteCb cb);

/**
  * @brief  Ajusta el rango valido de medicion (cm). Por defecto 2..400.
  */
void HC_SR04_SetRange(HC_SR04_HandleTypeDef *h, float min_cm, float max_cm);

/**
  * @brief  Dispara una nueva medicion: pulso TRIG de 10 us y arma la captura
  *         del ECHO por interrupcion. No bloquea.
  * @retval HC_SR04_OK si se disparo / HC_SR04_BUSY si ya hay una en curso
  */
HC_SR04_Status HC_SR04_Trigger(HC_SR04_HandleTypeDef *h);

/**
  * @brief  Obtiene la ultima distancia. No bloquea.
  * @param  out_cm  (salida) distancia en cm si retorna HC_SR04_OK
  * @retval HC_SR04_OK     hay dato nuevo valido (queda consumido)
  *         HC_SR04_BUSY   medicion en curso, todavia sin dato
  *         HC_SR04_TIMEOUT venció el timeout sin echo (resetea a IDLE)
  *         HC_SR04_INVALID dato fuera de rango (queda consumido)
  */
HC_SR04_Status HC_SR04_GetDistance(HC_SR04_HandleTypeDef *h, float *out_cm);

/**
  * @brief  Manejador de la captura para ESTA instancia. Llamalo desde
  *         HAL_TIM_IC_CaptureCallback si manejas el dispatch vos mismo.
  */
void HC_SR04_TIM_IC_Callback(HC_SR04_HandleTypeDef *h);

/**
  * @brief  Dispatcher global: recorre las instancias registradas y atiende la
  *         que corresponde a (htim, canal activo). Llamalo asi desde tu codigo:
  *
  *             void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim) {
  *                 HC_SR04_HandleInterrupt(htim);
  *             }
  */
void HC_SR04_HandleInterrupt(TIM_HandleTypeDef *htim);

#ifdef __cplusplus
}
#endif

#endif /* HC_SR04_H */
