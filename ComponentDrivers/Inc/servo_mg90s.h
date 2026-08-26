/**
  ******************************************************************************
  * @file    servo_mg90s.h
  * @brief   Driver HAL para servo MG90S (y similares de hobby) por PWM.
  *          Interfaz en grados (escala 0..180); los microsegundos son asunto del
  *          driver. La aplicacion solo declara el recorrido permitido con
  *          Servo_SetTravel, y Servo_SetAngle satura a ese recorrido.
  *          RTOS-agnostico y multi-instancia.
  *
  *          Requisitos de CubeMX:
  *            - TIM en PWM Generation, con el Prescaler que de 1 us/tick
  *              (PSC = MHz del timer - 1; en este proyecto 16 MHz -> PSC=15) y
  *              ARR=19999 (20 ms -> 50 Hz). Asi el CCR en cuentas equivale a us.
  ******************************************************************************
  */

#ifndef SERVO_MG90S_H
#define SERVO_MG90S_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include "bsp_types.h"
#include <stdint.h>

typedef enum {
    SERVO_OK = 0,
    SERVO_ERROR
} Servo_Status;

typedef struct Servo Servo_HandleTypeDef;

/**
  * @brief  Reserva un handle de un pool estatico interno (sin malloc). Devuelve
  *         NULL si el pool esta agotado. No hay Destroy.
  */
Servo_HandleTypeDef *Servo_Create(void);

/**
  * @brief  Inicializa el servo y arranca la generacion PWM. Recorrido inicial:
  *         todo 0..180 (sin guarda), y queda parado en 90 grados, que es la
  *         posicion segura mientras la aplicacion no declare la suya.
  * @param  pwm  timer + canal en modo PWM
  */
Servo_Status Servo_Init(Servo_HandleTypeDef *s,
                        TimerChannel_t pwm);

/**
  * @brief  Declara el recorrido PERMITIDO, en grados, dentro de 0..180.
  *         Ej. dejar 10 grados de guarda a cada punta:
  *           Servo_SetTravel(&s, 10.0f, 170.0f);
  * @retval SERVO_ERROR si max_deg <= min_deg o si alguno cae fuera de 0..180;
  *         en ese caso queda el recorrido anterior.
  */
Servo_Status Servo_SetTravel(Servo_HandleTypeDef *s,
                             float min_deg, float max_deg);

/**
  * @brief  Fija el angulo en grados. Unico comando del driver: el pulso sale de
  *         la recta del componente y el angulo se satura al recorrido permitido.
  */
Servo_Status Servo_SetAngle(Servo_HandleTypeDef *s, float deg);

/**
  * @brief  Ancho de pulso que esta saliendo por el pin, en us. Solo para
  *         diagnostico: es lo que distingue "el firmware no manda" de "el
  *         firmware manda y el servo no responde".
  */
Servo_Status Servo_GetPulseUs(const Servo_HandleTypeDef *s, uint16_t *us);

#ifdef __cplusplus
}
#endif

#endif /* SERVO_MG90S_H */
