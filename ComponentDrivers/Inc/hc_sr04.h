/**
  ******************************************************************************
  * @file    hc_sr04.h
  * @brief   Driver HAL para el sensor ultrasonico HC-SR04: mide distancia.
  *          No bloqueante, RTOS-agnostico y multi-instancia. Al completar una
  *          medicion invoca un hook opcional (ver HC_SR04_SetCompleteCallback).
  *
  *          Requisitos de CubeMX:
  *            - TIM en Input Capture direct mode, con el Prescaler que de 1 us/tick
  *              (PSC = MHz del timer - 1; en este proyecto 16 MHz -> PSC=15).
  *            - Preferentemente un timer de 32 bits (TIM2/TIM5): la medicion y el
  *              delay del TRIG usan resta unsigned del contador, que maneja el
  *              wrap-around solo en 32 bits (o en 16 bits con ARR=0xFFFF).
  *            - Pin TRIG como GPIO_Output push-pull.
  ******************************************************************************
  */

#ifndef HC_SR04_H
#define HC_SR04_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"   /* trae stm32f4xx_hal.h y los tipos HAL */
#include "bsp_types.h"
#include <stdint.h>

/* Estados de retorno de la API */
typedef enum {
    HC_SR04_OK = 0,     /* hay una medicion valida disponible            */
    HC_SR04_BUSY,       /* medicion en curso, todavia no hay dato         */
    HC_SR04_TIMEOUT,    /* no llego el echo dentro del tiempo esperado    */
    HC_SR04_INVALID,    /* medicion fuera de rango (descartada)           */
    HC_SR04_ERROR       /* parametros invalidos / fallo de HAL            */
} HC_SR04_Status;

typedef struct HC_SR04_Handle HC_SR04_HandleTypeDef;

/* Hook opcional invocado (en contexto de ISR) al completar una medicion.
 * Usalo para dar un semaforo a tu TaskSensor: ...GiveFromISR(). */
typedef void (*HC_SR04_CompleteCallback)(HC_SR04_HandleTypeDef *h);

/**
  * @brief  Reserva un handle de un pool estatico interno (sin malloc). Devuelve
  *         NULL si el pool esta agotado. No hay Destroy.
  */
HC_SR04_HandleTypeDef *HC_SR04_Create(void);

/**
  * @brief  Inicializa una instancia del sensor y la registra para su uso.
  * @param  h     handle obtenido de HC_SR04_Create()
  * @param  echo  timer + canal de Input Capture del ECHO
  * @param  trig  pin de salida del TRIG
  * @retval HC_SR04_OK / HC_SR04_ERROR
  */
HC_SR04_Status HC_SR04_Init(HC_SR04_HandleTypeDef *h,
                            TimerChannel_t echo,
                            GpioPin_t trig);

/**
  * @brief  Setea el hook que se llama al completar una medicion (contexto ISR).
  *         Pasar NULL para deshabilitarlo.
  */
void HC_SR04_SetCompleteCallback(HC_SR04_HandleTypeDef *h, HC_SR04_CompleteCallback cb);

/**
  * @brief  Ajusta el rango valido de medicion en cm.
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
  * @param  out_cm  (salida) distancia en cm. Se escribe con HC_SR04_OK y
  *                 tambien con HC_SR04_INVALID (valor crudo rechazado, util
  *                 para diagnostico); no se toca con BUSY/TIMEOUT/ERROR.
  * @retval HC_SR04_OK     hay dato nuevo valido (queda consumido)
  *         HC_SR04_BUSY   medicion en curso, todavia sin dato
  *         HC_SR04_TIMEOUT venció el timeout sin echo (resetea a IDLE)
  *         HC_SR04_INVALID dato fuera de rango (queda consumido)
  */
HC_SR04_Status HC_SR04_GetDistance(HC_SR04_HandleTypeDef *h, float *out_cm);

/**
  * @brief  Dispatcher global de la captura: recorre las instancias registradas y
  *         atiende la que corresponde a (htim, canal activo). Llamalo desde
  *         HAL_TIM_IC_CaptureCallback.
  */
void HC_SR04_HandleInterrupt(TIM_HandleTypeDef *htim);


#ifdef __cplusplus
}
#endif

#endif /* HC_SR04_H */
