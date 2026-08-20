/**
  ******************************************************************************
  * @file    kalman.h
  * @brief   Filtro de Kalman 1D con 2 estados [posicion, velocidad], modelo de
  *          velocidad constante. Modulo PURO: solo <float>, sin HAL ni FreeRTOS
  *          (compilable en host si algun dia se instala gcc).
  ******************************************************************************
  */

#ifndef KALMAN_H
#define KALMAN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Estado x = [pos, vel]^T. Modelo de transicion (velocidad constante):
 *   F = [[1, dt],[0, 1]]      H = [1, 0]
 * Q (ruido de proceso, aceleracion como white noise, densidad q):
 *   Q = q * [[dt^3/3, dt^2/2],[dt^2/2, dt]]
 * R = varianza de medicion (escalar). */
typedef struct {
    float dt;                 /* paso de tiempo [s]                     */
    float q;                  /* densidad de ruido de proceso           */
    float r;                  /* varianza de medicion                   */
    float x0;                 /* estado: posicion estimada              */
    float x1;                 /* estado: velocidad estimada             */
    float P00, P01, P10, P11; /* matriz de covarianza del error         */
} Kalman_t;

/**
  * @brief  Inicializa el filtro.
  * @param  dt  paso de tiempo en segundos (ej. 0.1)
  * @param  q   densidad de ruido de proceso (subir = mas responsivo)
  * @param  r   varianza de medicion (HC-SR04 ~ (0.3 cm)^2 ~ 0.09)
  * @param  x0  posicion inicial estimada
  */
void  Kalman_Init(Kalman_t *kf, float dt, float q, float r, float x0);

/**
  * @brief  Un ciclo predict + update con la medicion z. Devuelve la posicion
  *         estimada (x0).
  */
float Kalman_Update(Kalman_t *kf, float z);

/**
  * @brief  Reinicia el estado a x0 (velocidad 0) y la covarianza.
  */
void  Kalman_Reset(Kalman_t *kf, float x0);

#ifdef __cplusplus
}
#endif

#endif /* KALMAN_H */
