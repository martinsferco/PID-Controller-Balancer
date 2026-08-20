/**
  ******************************************************************************
  * @file    app.h
  * @brief   Capa de wiring de la aplicacion. Expone el punto de entrada
  *          App_Init() (llamado desde USER CODE 2 de main.c) y el hook de
  *          completado del sensor. Concentra las instancias compartidas.
  ******************************************************************************
  */

#ifndef APP_H
#define APP_H

#ifdef __cplusplus
extern "C" {
#endif

#include "FreeRTOS.h"
#include "semphr.h"
#include "hc_sr04.h"
#include "servo_mg90s.h"

/* --- Instancias compartidas de la aplicacion (definidas en app.c) --------- */
extern HC_SR04_HandleTypeDef g_sensor;      /* sensor de distancia            */
extern Servo_HandleTypeDef   g_servo;       /* servo del brazo                */
extern SemaphoreHandle_t     g_sensorSem;   /* aviso ISR -> SensorTask        */

/**
  * @brief  Inicializa la aplicacion: crea el semaforo y las tasks.
  *         Llamar una sola vez desde USER CODE 2 de main.c, antes del scheduler.
  */
void App_Init(void);

/**
  * @brief  Hook invocado por el driver del sensor al completar una medicion
  *         (contexto ISR de TIM2). Despierta a SensorTask. Se registra en
  *         SensorTask via HC_SR04_SetCompleteCallback().
  */
void App_OnSensorComplete_FromISR(HC_SR04_HandleTypeDef *h);

#ifdef __cplusplus
}
#endif

#endif /* APP_H */
