/**
  ******************************************************************************
  * @file    kalman.h
  * @brief   Filtro de Kalman 1D con 2 estados [posicion, velocidad], modelo de
  *          velocidad constante. Modulo PURO: solo <float>, sin HAL ni FreeRTOS
  *          (compilable en host si algun dia se instala gcc).
  *
  *          El struct es opaco (definido en kalman.c): el handle se pide con
  *          Kalman_Create() y se opera por la interfaz.
  ******************************************************************************
  */

#ifndef KALMAN_H
#define KALMAN_H

#ifdef __cplusplus
extern "C" {
#endif

typedef struct Kalman Kalman_HandleTypeDef;

/**
  * @brief  Reserva un handle de un pool estatico interno (sin malloc). Devuelve
  *         NULL si el pool esta agotado. No hay Destroy.
  */
Kalman_HandleTypeDef *Kalman_Create(void);

/**
  * @brief  Inicializa el filtro.
  * @param  dt  paso de tiempo en segundos (ej. 0.1)
  * @param  q   densidad de ruido de proceso (subir = mas responsivo)
  * @param  r   varianza de medicion (HC-SR04 ~ (0.3 cm)^2 ~ 0.09)
  * @param  x0  posicion inicial estimada
  */
void  Kalman_Init(Kalman_HandleTypeDef *kf, float dt, float q, float r, float x0);

/**
  * @brief  Un ciclo predict + update con la medicion z. Devuelve la posicion
  *         estimada.
  */
float Kalman_Update(Kalman_HandleTypeDef *kf, float z);

/**
  * @brief  Velocidad estimada [cm/s] del ultimo update (positiva si la posicion
  *         crece). Es la entrada del termino derivativo del PID.
  */
float Kalman_GetVelocity(const Kalman_HandleTypeDef *kf);

/**
  * @brief  Reinicia el estado a x0 (velocidad 0) y la covarianza.
  */
void  Kalman_Reset(Kalman_HandleTypeDef *kf, float x0);

#ifdef __cplusplus
}
#endif

#endif /* KALMAN_H */
