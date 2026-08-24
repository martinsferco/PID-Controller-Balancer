/**
  ******************************************************************************
  * @file    app.h
  * @brief   Capa de wiring de la aplicacion ball-and-beam. Concentra las
  *          instancias de drivers, los objetos de IPC (colas, queue set,
  *          semaforos) y los hooks de ISR. Punto de entrada: App_Init().
  ******************************************************************************
  */

#ifndef APP_H
#define APP_H

#ifdef __cplusplus
extern "C" {
#endif

#include "app_config.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "queue.h"
#include "hc_sr04.h"
#include "servo_mg90s.h"
#include "potentiometer.h"

/* --- Instancias de drivers (definidas en app.c) --------------------------- */
extern HC_SR04_HandleTypeDef g_sensor;   /* sensor de distancia            */
extern Servo_HandleTypeDef   g_servo;    /* servo del brazo                */
extern Potentiometer_HandleTypeDef g_potentiometer;  /* potenciometro (setpoint) */

/* --- Semaforos binarios --------------------------------------------------- */
extern SemaphoreHandle_t SemTimer;   /* lo da la ISR de TIM4 (cada 100 ms) */
extern SemaphoreHandle_t SemSensor;  /* lo da la ISR de Input Capture (echo)*/

/* --- Colas float profundidad 1 (xQueueOverwrite / xQueueReceive) ---------- */
extern QueueHandle_t QueuePos;       /* sensor  -> kalman  (distancia cruda)  */
extern QueueHandle_t QueuePosFil;    /* kalman  -> pid     (posicion filtrada)*/
extern QueueHandle_t QueueObjetivo;  /* pot     -> pid     (setpoint)         */
extern QueueHandle_t QueueAngulo;    /* pid     -> motor   (angulo)           */

/* --- Queue set del PID (bloquea en QueuePosFil + QueueObjetivo a la vez) --- */
extern QueueSetHandle_t QueueSetPid;

/**
  * @brief  Crea IPC + tasks y arranca los perifericos de tiempo real.
  *         Llamar una sola vez desde USER CODE 2 de main.c.
  */
void App_Init(void);

/** @brief Hook del sensor (ISR de TIM2): despierta a SensorTask (da SemSensor). */
void App_OnSensorComplete_FromISR(HC_SR04_HandleTypeDef *h);

/** @brief Hook del TIM4 (ISR cada 100 ms): da SemTimer (tick del sensor). */
void App_OnTimerTick_FromISR(void);

/* --- Traza de debug (solo con APP_LOG_ENABLED = 1) ------------------------ */
#if (APP_LOG_ENABLED == 1)

/** @brief Ultima distancia cruda medida, para que la traza del PID la muestre. */
extern volatile float g_dbg_raw;

/** @brief Una linea de traza del lazo: z / fil / sp / u / ang. */
void App_LogTrace(float z, float pos_fil, float sp, float u, float angle);

/** @brief Mensaje suelto (banner, estados del sensor, etc.). */
void App_LogMsg(const char *msg);

/** @brief Mensaje + un float (ej. "SENSOR INVALID raw=" 45.230). */
void App_LogMsgF(const char *msg, float v);

#endif /* APP_LOG_ENABLED */

#ifdef __cplusplus
}
#endif

#endif /* APP_H */
