/**
  ******************************************************************************
  * @file    hc_sr04.c
  * @brief   Implementacion del driver HAL del HC-SR04 (Input Capture, IT-based):
  *          mide el ancho del pulso de ECHO por flancos y lo convierte a
  *          distancia en GetDistance.
  ******************************************************************************
  */

#include "hc_sr04.h"

// Estado interno de la maquina de captura
typedef enum {
    HC_SR04_STATE_IDLE = 0,
    HC_SR04_STATE_WAIT_RISE,
    HC_SR04_STATE_WAIT_FALL
} HC_SR04_MeasureState;

// Definicion del struct opaco
struct HC_SR04_Handle {
    TIM_HandleTypeDef *htim;       // timer en modo Input Capture
    uint32_t           channel;    // TIM_CHANNEL_1..4
    uint32_t           active_ch;  // HAL_TIM_ACTIVE_CHANNEL_x
    GPIO_TypeDef      *trig_port;  // puerto del pin TRIG
    uint16_t           trig_pin;   // pin TRIG

    volatile HC_SR04_MeasureState state;
    volatile uint32_t t_rise;      // captura del flanco de subida [us]
    volatile uint32_t t_fall;      // captura del flanco de bajada [us]
    volatile uint8_t  data_ready;  // hay dato nuevo sin leer
    uint32_t          trigger_tick;// HAL_GetTick() al disparar (timeout)

    HC_SR04_CompleteCallback on_complete;
};

// Velocidad del sonido: 343 m/s -> 58 us/cm (ida y vuelta)
#define HC_SR04_ECHO_US_PER_CM 58.0f
#define HC_SR04_TRIG_PULSE_US  10u

#define HC_SR04_HW_TIMEOUT_MS  30u    // eco maximo: 25 ms + margen

// Registro interno de instancias para el dispatch del callback global del HAL
#ifndef HC_SR04_MAX_INSTANCES
#define HC_SR04_MAX_INSTANCES  1
#endif

static HC_SR04_HandleTypeDef *s_instances[HC_SR04_MAX_INSTANCES] = {0};

// Pool estatico de handles
static struct HC_SR04_Handle s_pool[HC_SR04_MAX_INSTANCES];
static unsigned              s_pool_count = 0u;

HC_SR04_HandleTypeDef *HC_SR04_Create(void)
{
    if (s_pool_count >= HC_SR04_MAX_INSTANCES) { return NULL; }
    return &s_pool[s_pool_count++];
}

// Mapea TIM_CHANNEL_x al codigo HAL_TIM_ACTIVE_CHANNEL_x
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

// Delay bloqueante en us, solo para el pulso de TRIG (10 us). Spinea sobre el
// contador del propio timer (1 us/tick).
static void hc_sr04_delay_us(TIM_HandleTypeDef *htim, uint32_t us)
{
    uint32_t start = __HAL_TIM_GET_COUNTER(htim);
    while ((__HAL_TIM_GET_COUNTER(htim) - start) < us) { }
}

HC_SR04_Status HC_SR04_Init(HC_SR04_HandleTypeDef *h,
                            TimerChannel_t echo,
                            GpioPin_t trig)
{
    if (h == NULL || echo.htim == NULL || trig.port == NULL) {
        return HC_SR04_ERROR;
    }

    // Se desarma el par en htim/channel sueltos
    h->htim       = echo.htim;
    h->channel    = echo.channel;
    h->active_ch  = hc_sr04_active_channel(echo.channel);
    h->trig_port  = trig.port;
    h->trig_pin   = trig.pin;

    h->state        = HC_SR04_STATE_IDLE;
    h->t_rise       = 0;
    h->t_fall       = 0;
    h->data_ready   = 0;
    h->trigger_tick = 0;
    h->on_complete  = NULL;

    HAL_GPIO_WritePin(h->trig_port, h->trig_pin, GPIO_PIN_RESET);   // TRIG en bajo

    // Registrar la instancia para el dispatcher
    for (int i = 0; i < HC_SR04_MAX_INSTANCES; i++) {
        if (s_instances[i] == h) {           // ya estaba
            return HC_SR04_OK;
        }
    }
    for (int i = 0; i < HC_SR04_MAX_INSTANCES; i++) {
        if (s_instances[i] == NULL) {
            s_instances[i] = h;
            return HC_SR04_OK;
        }
    }
    return HC_SR04_ERROR;  // sin lugar en el registro
}

void HC_SR04_SetCompleteCallback(HC_SR04_HandleTypeDef *h, HC_SR04_CompleteCallback cb)
{
    if (h != NULL) {
        h->on_complete = cb;
    }
}

HC_SR04_Status HC_SR04_Trigger(HC_SR04_HandleTypeDef *h)
{
    if (h == NULL) return HC_SR04_ERROR;

    if (h->state != HC_SR04_STATE_IDLE) return HC_SR04_BUSY;

    h->data_ready = 0;
    h->state      = HC_SR04_STATE_WAIT_RISE;

    __HAL_TIM_SET_CAPTUREPOLARITY(h->htim, h->channel, TIM_INPUTCHANNELPOLARITY_RISING);

    if (HAL_TIM_IC_Start_IT(h->htim, h->channel) != HAL_OK) {
        h->state = HC_SR04_STATE_IDLE;
        return HC_SR04_ERROR;
    }

    // Pulso de TRIG, 10 us en alto
    HAL_GPIO_WritePin(h->trig_port, h->trig_pin, GPIO_PIN_SET);
    hc_sr04_delay_us(h->htim, HC_SR04_TRIG_PULSE_US);
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
        uint32_t width_us = h->t_fall - h->t_rise;
        float d = (float)width_us / HC_SR04_ECHO_US_PER_CM;
        h->data_ready = 0;
        *out_cm = d;   // se escribe siempre, aun si la lectura se descarta
        if (d < HC_SR04_HW_MIN_CM || d > HC_SR04_HW_MAX_CM) {
            return HC_SR04_INVALID;
        }
        return HC_SR04_OK;
    }

    // Sin dato todavia: chequear timeout
    if (h->state != HC_SR04_STATE_IDLE) {
        if ((HAL_GetTick() - h->trigger_tick) > HC_SR04_HW_TIMEOUT_MS) {
            HAL_TIM_IC_Stop_IT(h->htim, h->channel);
            h->state = HC_SR04_STATE_IDLE;
            return HC_SR04_TIMEOUT;
        }
    }
    return HC_SR04_BUSY;
}

static void hc_sr04_tim_ic_callback(HC_SR04_HandleTypeDef *h)
{
    if (h == NULL) return;

    uint32_t captured = HAL_TIM_ReadCapturedValue(h->htim, h->channel);

    if (h->state == HC_SR04_STATE_WAIT_RISE) {
        h->t_rise = captured;
        h->state  = HC_SR04_STATE_WAIT_FALL;
        __HAL_TIM_SET_CAPTUREPOLARITY(h->htim, h->channel,
                                      TIM_INPUTCHANNELPOLARITY_FALLING);
    }
    else if (h->state == HC_SR04_STATE_WAIT_FALL) {
        h->t_fall = captured;

        HAL_TIM_IC_Stop_IT(h->htim, h->channel);
        h->state      = HC_SR04_STATE_IDLE;
        h->data_ready = 1;

        if (h->on_complete != NULL) {
            h->on_complete(h);   // contexto ISR: usar ...FromISR() adentro
        }
    }
    // Si llega en IDLE, lo ignoramos
}

// True si el handle corresponde a la interrupcion actual
static int hc_sr04_matches(const HC_SR04_HandleTypeDef *h, TIM_HandleTypeDef *htim)
{
    return h != NULL &&
           h->htim == htim &&
           htim->Channel == (HAL_TIM_ActiveChannel)h->active_ch;
}

void HC_SR04_HandleInterrupt(TIM_HandleTypeDef *htim)
{
    for (int i = 0; i < HC_SR04_MAX_INSTANCES; i++) {
        if (hc_sr04_matches(s_instances[i], htim)) {
            hc_sr04_tim_ic_callback(s_instances[i]);
            return;
        }
    }
}
