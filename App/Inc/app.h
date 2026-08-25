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

/* --- Estado estimado que el Kalman le pasa al PID ------------------------- *
 * Los dos estados del filtro viajan JUNTOS, en un solo item de una sola cola, y
 * eso es a proposito. El termino D del PID consume `vel`, asi que tiene que ser
 * la velocidad del MISMO update que produjo `pos`: si fueran dos colas (o dos
 * miembros del queue set) el PID podria leer la posicion de la muestra N con la
 * velocidad de la N-1, que es exactamente un termino derivativo desfasado, y
 * ademas cada muestra generaria dos avisos al queue set en vez de uno. */
typedef struct {
    float pos;   /* posicion filtrada [cm]                                  */
    float vel;   /* velocidad estimada [cm/s], mismo signo que pos creciente */
} PosFil_t;

/* --- Colas profundidad 1 (xQueueOverwrite / xQueueReceive) ---------------- */
extern QueueHandle_t QueuePos;       /* sensor  -> kalman  (distancia cruda)  */
extern QueueHandle_t QueuePosFil;    /* kalman  -> pid     (PosFil_t: pos+vel)*/
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

/** @brief Una linea de traza del lazo: z / fil / vel / sp / u / i / ang.
  *        `vel` es la entrada del termino D: si el servo tiembla con la pelota
  *        quieta, es esta columna la que hay que mirar (y el remedio esta en
  *        KALMAN_Q, no en PID_KD).
  *        `i` es el integrador. Es la unica forma de ver si KI esta haciendo lo
  *        que se espera: tiene que crecer despacio con la pelota clavada, y
  *        volver a 0 cuando cruza el setpoint. Si crece sin parar o se queda
  *        cargado del lado equivocado, es el ciclo limite empezando. */
void App_LogTrace(float z, float pos_fil, float vel, float sp, float u, float integ, float angle);

/** @brief Mensaje suelto (banner, estados del sensor, etc.). */
void App_LogMsg(const char *msg);

/** @brief Mensaje + un float (ej. "SENSOR INVALID raw=" 45.230). */
void App_LogMsgF(const char *msg, float v);

#endif /* APP_LOG_ENABLED */

#ifdef __cplusplus
}
#endif

#endif /* APP_H */
