/**
  ******************************************************************************
  * @file    hc_sr04.c
  * @brief   Implementacion del driver HAL del HC-SR04 (Input Capture, IT-based).
  *
  *  Como mide:
  *   1) HC_SR04_Trigger() emite un pulso de 10 us en TRIG y arma la captura del
  *      ECHO por flanco de SUBIDA (HAL_TIM_IC_Start_IT).
  *   2) Al llegar el flanco de subida, se guarda t_rise y se cambia la captura a
  *      flanco de BAJADA.
  *   3) Al llegar el flanco de bajada, se guarda t_fall. El ancho del pulso es
  *      (t_fall - t_rise) en us (porque el timer corre a 1 us/tick). La distancia
  *      en cm es ancho_us / 58 (ida y vuelta del sonido a ~343 m/s).
  *
  *  El timer NO debe resetearse entre flancos: usamos captura libre y restamos.
  *  La resta unsigned maneja el wrap-around del contador automaticamente
  *  (32 bits en TIM2/TIM5; tambien funciona si el ARR es 0xFFFF en 16 bits).
  ******************************************************************************
  */

#include "hc_sr04.h"

/* Velocidad del sonido: 343 m/s -> 29.15 us/cm (ida) -> 58.3 us/cm (ida+vuelta) */
#define HC_SR04_US_PER_CM      58.0f
#define HC_SR04_TRIG_PULSE_US  10u

/* Registro interno de instancias para el dispatch del callback global del HAL. */
#ifndef HC_SR04_MAX_INSTANCES
#define HC_SR04_MAX_INSTANCES  4
#endif

static HC_SR04_HandleTypeDef *s_instances[HC_SR04_MAX_INSTANCES] = {0};

/* ------------------------------------------------------------------------- */
/* Utilidades internas                                                       */
/* ------------------------------------------------------------------------- */

/* Mapea TIM_CHANNEL_x al codigo HAL_TIM_ACTIVE_CHANNEL_x para identificar el
 * canal que disparo la interrupcion (htim->Channel). */
static uint32_t hc_sr04_active_channel(uint32_t channel)
{
    switch (channel) {
        case TIM_CHANNEL_1: return HAL_TIM_ACTIVE_CHANNEL_1;
        case TIM_CHANNEL_2: return HAL_TIM_ACTIVE_CHANNEL_2;
        case TIM_CHANNEL_3: return HAL_TIM_ACTIVE_CHANNEL_3;
        case TIM_CHANNEL_4: return HAL_TIM_ACTIVE_CHANNEL_4;
        default:            return 0xFFFFFFFFu;
    }
}

/* Pequena demora bloqueante de microsegundos SOLO para el pulso de TRIG (10 us).
 * No se usa para medir el ECHO (eso es 100% por interrupcion). */
static void hc_sr04_delay_us(uint32_t us)
{
    /* Aproximacion por busy-loop calibrada para ~16 MHz (HSI, sin PLL). Es para
     * 10 us, su exactitud no afecta la medicion. Si cambias el reloj del CPU,
     * ajusta el factor (= MHz del CPU). Si tenes un timer libre, podes usar
     * __HAL_TIM_GET_COUNTER en su lugar. */
    volatile uint32_t cycles = us * 16u / 4u; /* ~4 ciclos por iteracion */
    while (cycles--) {
        __NOP();
    }
}

/* ------------------------------------------------------------------------- */
/* API publica                                                               */
/* ------------------------------------------------------------------------- */

HC_SR04_Status HC_SR04_Init(HC_SR04_HandleTypeDef *h,
                            TIM_HandleTypeDef *htim,
                            uint32_t channel,
                            GPIO_TypeDef *trig_port,
                            uint16_t trig_pin)
{
    if (h == NULL || htim == NULL || trig_port == NULL) {
        return HC_SR04_ERROR;
    }

    h->htim       = htim;
    h->channel    = channel;
    h->active_ch  = hc_sr04_active_channel(channel);
    h->trig_port  = trig_port;
    h->trig_pin   = trig_pin;

    /* Defaults razonables */
    h->min_cm      = 2.0f;
    h->max_cm      = 400.0f;
    h->timeout_ms  = 60u;          /* echo maximo ~38 ms + margen */

    h->state        = HC_SR04_STATE_IDLE;
    h->t_rise       = 0;
    h->t_fall       = 0;
    h->distance_cm  = 0.0f;
    h->data_ready   = false;
    h->trigger_tick = 0;
    h->on_complete  = NULL;

    /* TRIG en bajo al iniciar */
    HAL_GPIO_WritePin(h->trig_port, h->trig_pin, GPIO_PIN_RESET);

    /* Registrar la instancia para el dispatcher */
    for (int i = 0; i < HC_SR04_MAX_INSTANCES; i++) {
        if (s_instances[i] == h) {           /* ya estaba */
            return HC_SR04_OK;
        }
    }
    for (int i = 0; i < HC_SR04_MAX_INSTANCES; i++) {
        if (s_instances[i] == NULL) {
            s_instances[i] = h;
            return HC_SR04_OK;
        }
    }
    return HC_SR04_ERROR;  /* sin lugar en el registro */
}

void HC_SR04_SetCompleteCallback(HC_SR04_HandleTypeDef *h, HC_SR04_CompleteCb cb)
{
    if (h != NULL) {
        h->on_complete = cb;
    }
}

void HC_SR04_SetRange(HC_SR04_HandleTypeDef *h, float min_cm, float max_cm)
{
    if (h != NULL && max_cm > min_cm) {
        h->min_cm = min_cm;
        h->max_cm = max_cm;
    }
}

HC_SR04_Status HC_SR04_Trigger(HC_SR04_HandleTypeDef *h)
{
    if (h == NULL) {
        return HC_SR04_ERROR;
    }
    if (h->state != HC_SR04_STATE_IDLE) {
        return HC_SR04_BUSY;
    }

    /* Preparar la maquina de estados para esperar el flanco de subida */
    h->data_ready = false;
    h->state      = HC_SR04_STATE_WAIT_RISE;

    /* Configurar captura por flanco de SUBIDA y arrancar la IC con interrupcion */
    __HAL_TIM_SET_CAPTUREPOLARITY(h->htim, h->channel, TIM_INPUTCHANNELPOLARITY_RISING);
    if (HAL_TIM_IC_Start_IT(h->htim, h->channel) != HAL_OK) {
        h->state = HC_SR04_STATE_IDLE;
        return HC_SR04_ERROR;
    }

    /* Pulso de TRIG: 10 us en alto */
    HAL_GPIO_WritePin(h->trig_port, h->trig_pin, GPIO_PIN_SET);
    hc_sr04_delay_us(HC_SR04_TRIG_PULSE_US);
    HAL_GPIO_WritePin(h->trig_port, h->trig_pin, GPIO_PIN_RESET);

    h->trigger_tick = HAL_GetTick();
    return HC_SR04_OK;
}

HC_SR04_Status HC_SR04_GetDistance(HC_SR04_HandleTypeDef *h, float *out_cm)
{
    if (h == NULL || out_cm == NULL) {
        return HC_SR04_ERROR;
    }

    if (h->data_ready) {
        float d = h->distance_cm;
        h->data_ready = false;
        if (d < h->min_cm || d > h->max_cm) {
            return HC_SR04_INVALID;
        }
        *out_cm = d;
        return HC_SR04_OK;
    }

    /* Sin dato todavia: chequear timeout (no bloqueante) */
    if (h->state != HC_SR04_STATE_IDLE) {
        if ((HAL_GetTick() - h->trigger_tick) > h->timeout_ms) {
            HAL_TIM_IC_Stop_IT(h->htim, h->channel);
            h->state = HC_SR04_STATE_IDLE;
            return HC_SR04_TIMEOUT;
        }
    }
    return HC_SR04_BUSY;
}

void HC_SR04_TIM_IC_Callback(HC_SR04_HandleTypeDef *h)
{
    if (h == NULL) {
        return;
    }

    uint32_t captured = HAL_TIM_ReadCapturedValue(h->htim, h->channel);

    if (h->state == HC_SR04_STATE_WAIT_RISE) {
        /* Llego el flanco de subida: guardar t_rise y pasar a esperar bajada */
        h->t_rise = captured;
        h->state  = HC_SR04_STATE_WAIT_FALL;
        __HAL_TIM_SET_CAPTUREPOLARITY(h->htim, h->channel,
                                      TIM_INPUTCHANNELPOLARITY_FALLING);
    }
    else if (h->state == HC_SR04_STATE_WAIT_FALL) {
        /* Llego el flanco de bajada: calcular ancho y distancia */
        h->t_fall = captured;

        /* Resta unsigned: maneja el wrap-around del contador automaticamente. */
        uint32_t width_us = h->t_fall - h->t_rise;

        h->distance_cm = (float)width_us / HC_SR04_US_PER_CM;

        /* Medicion completa: detener IC, marcar dato y notificar */
        HAL_TIM_IC_Stop_IT(h->htim, h->channel);
        h->state      = HC_SR04_STATE_IDLE;
        h->data_ready = true;

        if (h->on_complete != NULL) {
            h->on_complete(h);   /* contexto ISR: usar ...FromISR() adentro */
        }
    }
    /* Si llega en IDLE, se ignora (captura espuria). */
}

void HC_SR04_HandleInterrupt(TIM_HandleTypeDef *htim)
{
    for (int i = 0; i < HC_SR04_MAX_INSTANCES; i++) {
        HC_SR04_HandleTypeDef *h = s_instances[i];
        if (h != NULL &&
            h->htim == htim &&
            htim->Channel == (HAL_TIM_ActiveChannel)h->active_ch) {
            HC_SR04_TIM_IC_Callback(h);
            return;
        }
    }
}
