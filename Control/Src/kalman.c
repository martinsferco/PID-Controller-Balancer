/**
  ******************************************************************************
  * @file    kalman.c
  * @brief   Implementacion del filtro de Kalman 2 estados (pos, vel).
  ******************************************************************************
  */

#include "kalman.h"

/* Estado x = [pos, vel]^T. Modelo de transicion (velocidad constante):
 *   F = [[1, dt],[0, 1]]      H = [1, 0]
 * Q (ruido de proceso, aceleracion como white noise, densidad q):
 *   Q = q * [[dt^3/3, dt^2/2],[dt^2/2, dt]]
 * R = varianza de medicion (escalar). Struct opaco: se define aca. */
struct Kalman {
    float dt;                 /* paso de tiempo [s]                     */
    float q;                  /* densidad de ruido de proceso           */
    float r;                  /* varianza de medicion                   */
    float x0;                 /* estado: posicion estimada              */
    float x1;                 /* estado: velocidad estimada             */
    float P00, P01, P10, P11; /* matriz de covarianza del error         */
};

/* Pool estatico de handles: el modulo es dueno de la memoria (sin malloc). */
#ifndef KALMAN_MAX_INSTANCES
#define KALMAN_MAX_INSTANCES  1
#endif

static struct Kalman s_pool[KALMAN_MAX_INSTANCES];
static unsigned      s_pool_count = 0u;

Kalman_HandleTypeDef *Kalman_Create(void)
{
    if (s_pool_count >= KALMAN_MAX_INSTANCES) { return 0; }
    return &s_pool[s_pool_count++];
}

static void kalman_reset_cov(Kalman_HandleTypeDef *kf)
{
    /* Covarianza inicial: incertidumbre moderada, sin correlacion. */
    kf->P00 = 1.0f; kf->P01 = 0.0f;
    kf->P10 = 0.0f; kf->P11 = 1.0f;
}

void Kalman_Init(Kalman_HandleTypeDef *kf, float dt, float q, float r, float x0)
{
    if (kf == 0) { return; }
    kf->dt = dt;
    kf->q  = q;
    kf->r  = r;
    kf->x0 = x0;
    kf->x1 = 0.0f;
    kalman_reset_cov(kf);
}

void Kalman_Reset(Kalman_HandleTypeDef *kf, float x0)
{
    if (kf == 0) { return; }
    kf->x0 = x0;
    kf->x1 = 0.0f;
    kalman_reset_cov(kf);
}

float Kalman_Update(Kalman_HandleTypeDef *kf, float z)
{
    if (kf == 0) { return 0.0f; }

    const float dt = kf->dt;

    /* ---- Prediccion: x = F x ---- */
    kf->x0 = kf->x0 + dt * kf->x1;
    /* kf->x1 = kf->x1;  (velocidad constante) */

    /* ---- Prediccion de covarianza: P = F P F^T + Q ---- */
    /* F P F^T con F = [[1,dt],[0,1]] */
    const float p00 = kf->P00 + dt * (kf->P10 + kf->P01) + dt * dt * kf->P11;
    const float p01 = kf->P01 + dt * kf->P11;
    const float p10 = kf->P10 + dt * kf->P11;
    const float p11 = kf->P11;

    /* Q = q * [[dt^3/3, dt^2/2],[dt^2/2, dt]] */
    const float dt2 = dt * dt;
    const float dt3 = dt2 * dt;
    const float q00 = kf->q * (dt3 / 3.0f);
    const float q01 = kf->q * (dt2 / 2.0f);
    const float q10 = q01;
    const float q11 = kf->q * dt;

    kf->P00 = p00 + q00;
    kf->P01 = p01 + q01;
    kf->P10 = p10 + q10;
    kf->P11 = p11 + q11;

    /* ---- Update con H = [1, 0] ---- */
    const float y = z - kf->x0;               /* innovacion               */
    const float S = kf->P00 + kf->r;          /* covarianza de innovacion */
    if (S <= 0.0f) { return kf->x0; }         /* guarda numerica          */

    const float K0 = kf->P00 / S;             /* ganancia de Kalman       */
    const float K1 = kf->P10 / S;

    kf->x0 = kf->x0 + K0 * y;
    kf->x1 = kf->x1 + K1 * y;

    /* P = (I - K H) P,  con K H = [[K0,0],[K1,0]] */
    const float n00 = (1.0f - K0) * kf->P00;
    const float n01 = (1.0f - K0) * kf->P01;
    const float n10 = kf->P10 - K1 * kf->P00;
    const float n11 = kf->P11 - K1 * kf->P01;

    kf->P00 = n00; kf->P01 = n01;
    kf->P10 = n10; kf->P11 = n11;

    return kf->x0;
}

float Kalman_GetVelocity(const Kalman_HandleTypeDef *kf)
{
    if (kf == 0) { return 0.0f; }
    return kf->x1;
}
