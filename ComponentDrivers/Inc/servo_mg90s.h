/**
  ******************************************************************************
  * @file    servo_mg90s.h
  * @brief   Driver HAL para servo MG90S (y similares de hobby) por PWM.
  *
  *          INTERFAZ EN GRADOS, escala 0..180:
  *              0 grados -> horn en un extremo   (SERVO_MIN_US)
  *            180 grados -> horn en el otro       (SERVO_MAX_US)
  *             90 grados -> centro                (1500 us)
  *
  *          La recta us <-> grados NO se expone: es parte del driver, igual que
  *          el nombre del archivo. Un MG90S recorre sus 180 grados entre 500 y
  *          2500 us, y eso es un hecho del componente, no una decision de la
  *          aplicacion. Los microsegundos no aparecen en ninguna llamada.
  *
  *          Lo unico que la aplicacion declara es el RECORRIDO PERMITIDO en
  *          grados (Servo_SetTravel), porque eso si es una decision suya: cuanta
  *          guarda quiere dejar contra los topes fisicos del servo. Servo_SetAngle
  *          satura a ese recorrido, asi que el limite vale para todo llamador y
  *          ninguna task necesita repetir el clamp.
  *
  *          RTOS-agnostico: solo usa HAL. Multi-instancia (varios servos en
  *          distintos canales/timers).
  *
  *          Requisitos de CubeMX (ver Manual_CubeMX_Servo.md):
  *            - TIM en PWM Generation, con el Prescaler que de 1 us/tick
  *              (PSC = MHz del timer - 1; en este proyecto 16 MHz -> PSC=15) y
  *              ARR=19999 (20 ms -> 50 Hz). Asi el valor del CCR en cuentas == us.
  ******************************************************************************
  */

#ifndef SERVO_MG90S_H
#define SERVO_MG90S_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include <stdint.h>

/* ---- La recta del componente. Cambia solo si se cambia de servo. ---------- *
 * 500 y 2500 us son los EXTREMOS FISICOS del MG90S: ahi el horn ya esta contra
 * su tope interno, y sostenerlo ahi lo quema. Por eso son a la vez los extremos
 * de la escala angular (0 y 180) y el limite duro que el driver nunca cruza: la
 * aplicacion deja su guarda con Servo_SetTravel(), en grados. */
#define SERVO_MIN_US        500u
#define SERVO_MAX_US        2500u
#define SERVO_MIN_ANGLE     0.0f
#define SERVO_MAX_ANGLE     180.0f

/* us por grado (11.111 en un MG90S). Informativo para el llamador: sirve para
 * saber cuanto vale un grado al ajustar una referencia, sin recalcular la recta. */
#define SERVO_US_PER_DEG    (((float)(SERVO_MAX_US - SERVO_MIN_US)) / \
                             (SERVO_MAX_ANGLE - SERVO_MIN_ANGLE))

typedef enum {
    SERVO_OK = 0,
    SERVO_ERROR
} Servo_Status;

typedef struct {
    TIM_HandleTypeDef *htim;     /* timer en modo PWM                        */
    uint32_t           channel;  /* TIM_CHANNEL_1..4                         */
    float              min_deg;  /* recorrido permitido: piso (ver SetTravel) */
    float              max_deg;  /* recorrido permitido: techo               */
} Servo_HandleTypeDef;

/**
  * @brief  Inicializa el servo y arranca la generacion PWM.
  *         Recorrido permitido inicial: todo 0..180 (sin guarda). Queda parado
  *         en 90 grados (1500 us), el centro, que es la posicion segura mientras
  *         la aplicacion no haya declarado su guarda.
  * @param  htim    timer ya inicializado por CubeMX en PWM
  * @param  channel TIM_CHANNEL_1..4
  */
Servo_Status Servo_Init(Servo_HandleTypeDef *s,
                        TIM_HandleTypeDef *htim,
                        uint32_t channel);

/**
  * @brief  Declara el recorrido PERMITIDO, en grados, dentro de 0..180.
  *         Servo_SetAngle() satura todo pedido a [min_deg, max_deg], asi que el
  *         limite del actuador vive aca y vale para cualquier llamador (el lazo,
  *         un modo de banco, un PID mal tuneado).
  *         Ej. dejar 10 grados de guarda a cada punta:
  *           Servo_SetTravel(&s, 10.0f, 170.0f);
  * @retval SERVO_ERROR si max_deg <= min_deg o si alguno cae fuera de 0..180.
  *         En ese caso queda el recorrido anterior (no se toca nada).
  */
Servo_Status Servo_SetTravel(Servo_HandleTypeDef *s,
                             float min_deg, float max_deg);

/**
  * @brief  Fija el angulo en grados. UNICO comando del driver. El pulso se
  *         deriva de la recta del componente; el angulo se satura al recorrido
  *         permitido.
  */
Servo_Status Servo_SetAngle(Servo_HandleTypeDef *s, float deg);

/**
  * @brief  Ancho de pulso que esta saliendo por el pin, en us. Solo para
  *         traza/diagnostico: observar no es comandar. Es lo que distingue "el
  *         firmware no manda" de "el firmware manda y el servo no responde".
  */
Servo_Status Servo_GetPulseUs(const Servo_HandleTypeDef *s, uint16_t *us);

#ifdef __cplusplus
}
#endif

#endif /* SERVO_MG90S_H */
