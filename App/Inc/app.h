/**
  ******************************************************************************
  * @file    app.h
  * @brief   Capa de wiring de la aplicacion sistema de control PID para balanceo.
  *          Punto de entrada: App_Init() (composition root). Expone tambien los
  *          hooks de ISR y el tipo de mensaje que comparten Kalman y PID.
  ******************************************************************************
  */

#ifndef APP_H
#define APP_H

#ifdef __cplusplus
extern "C" {
#endif

#include "hc_sr04.h"   /* tipo del handle que recibe el hook del sensor */

/* --- Estado estimado que el Kalman le pasa al PID ------------------------- *
 * Los dos estados del filtro viajan JUNTOS, en un solo item de una sola cola,
 * y eso es a proposito: el termino D del PID consume `vel`, asi que tiene que
 * ser la velocidad del MISMO update que produjo `pos`. Con dos colas separadas
 * el PID podria leer la posicion de la muestra N con la velocidad de la N-1,
 * que es exactamente una derivada desfasada. */
typedef struct {
    float pos;   /* posicion filtrada [cm]                                    */
    float vel;   /* velocidad estimada [cm/s], positiva si pos crece          */
} PosFil_t;

/**
  * @brief  Composition root: crea drivers + IPC + contextos y las tasks, y
  *         arranca los perifericos de tiempo real. Llamar una sola vez desde
  *         USER CODE 2 de main.c.
  */
void App_Init(void);

/** @brief Hook del sensor (ISR de TIM2): despierta a SensorTask (da SemSensor). */
void App_OnSensorComplete_FromISR(HC_SR04_HandleTypeDef *h);

/** @brief Hook del TIM4 (ISR cada 100 ms): da SemTimer (tick del sensor). */
void App_OnTimerTick_FromISR(void);

#ifdef __cplusplus
}
#endif

#endif /* APP_H */
