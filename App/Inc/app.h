/**
  ******************************************************************************
  * @file    app.h
  * @brief   Capa de conexion de la aplicacion sistema de control PID para balanceo.
  *          Expone el punto de entrada App_Init() y tambien los
  *          hooks de ISR y el tipo de mensaje que comparten Kalman y PID.
  ******************************************************************************
  */

#ifndef APP_H
#define APP_H

#ifdef __cplusplus
extern "C" {
#endif

#include "hc_sr04.h"   // handle del sensor

// Estado estimado que Kalman le pasa al PID
typedef struct {
    float pos;   // cm
    float vel;   // cm/s
} PosFil_t;

/**
  * @brief  Crea drivers + IPC + contextos y las tasks, e inicia los perifericos
  *         de tiempo real.
  */
void App_Init(void);

/** @brief Hook del sensor (ISR de TIM2): despierta a SensorTask (da SemSensor). */
void App_OnSensorComplete_FromISR(HC_SR04_HandleTypeDef *h);

/** @brief Hook del TIM4 (ISR cada 100 ms): da SemTimer (tick del sensor). */
void App_OnTimerTick_FromISR(void);

#ifdef __cplusplus
}
#endif

#endif // APP_H
