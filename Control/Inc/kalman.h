/**
  ******************************************************************************
  * @file    kalman.h
  * @brief   Filtro de Kalman 1D con 2 estados [posicion, velocidad], modelo de
  *          velocidad constante. 
  ******************************************************************************
  */

#ifndef KALMAN_H
#define KALMAN_H

#ifdef __cplusplus
extern "C" {
#endif

typedef struct Kalman Kalman_HandleTypeDef;

/**
  * @brief  Reserva un handle de un pool estatico interno. Devuelve
  *         NULL si el pool esta agotado.
  */
Kalman_HandleTypeDef *Kalman_Create(void);

/**
  * @brief  Inicializa el filtro.
  * @param  dt  paso de tiempo en segundos
  * @param  q   densidad de ruido de proceso
  * @param  r   varianza de medicion
  * @param  x0  posicion inicial estimada
  */
void  Kalman_Init(Kalman_HandleTypeDef *kf, float dt, float q, float r, float x0);

/**
  * @brief  Un ciclo prediccion y actualizacion con la medicion z. Devuelve la posicion
  *         estimada.
  */
float Kalman_Update(Kalman_HandleTypeDef *kf, float z);

/**
  * @brief  Velocidad estimada del ultimo update (positiva si la posicion
  *         crece). 
  */
float Kalman_GetVelocity(const Kalman_HandleTypeDef *kf);

/**
  * @brief  Reinicia el estado a x0 y la covarianza.
  */
void  Kalman_Reset(Kalman_HandleTypeDef *kf, float x0);

#ifdef __cplusplus
}
#endif

#endif // KALMAN_H
