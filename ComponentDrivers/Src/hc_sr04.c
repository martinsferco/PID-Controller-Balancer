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
  *   3) Al llegar el flanco de bajada, se guarda t_fall (la ISR solo latchea los
  *      timestamps crudos). El ancho del pulso es (t_fall - t_rise) en us (el timer
  *      corre a 1 us/tick) y la distancia en cm es ancho_us / 58 (ida y vuelta del
  *      sonido a ~343 m/s); ese calculo lo hace HC_SR04_GetDistance en contexto de
  *      tarea, no la ISR.
  *
  *  El timer NO debe resetearse entre flancos: usamos captura libre y restamos.
  *  La resta unsigned maneja el wrap-around del contador automaticamente
  *  (32 bits en TIM2/TIM5; tambien funciona si el ARR es 0xFFFF en 16 bits).
  ******************************************************************************
  */

#include "hc_sr04.h"

/* Estado interno de la maquina de captura (oculto: es interno de la FSM). */
typedef enum {
    HC_SR04_STATE_IDLE = 0,
    HC_SR04_STATE_WAIT_RISE,
    HC_SR04_STATE_WAIT_FALL
} HC_SR04_MeasureState;

/* Definicion del struct opaco (oculta a los usuarios del .h). */
struct HC_SR04_Handle {
    TIM_HandleTypeDef *htim;       /* timer en modo Input Capture            */
    uint32_t           channel;    /* TIM_CHANNEL_1..4                       */
    uint32_t           active_ch;  /* HAL_TIM_ACTIVE_CHANNEL_x (uso interno) */
    GPIO_TypeDef      *trig_port;  /* puerto del pin TRIG                    */
    uint16_t           trig_pin;   /* pin TRIG                               */

    volatile HC_SR04_MeasureState state;
    volatile uint32_t t_rise;      /* captura del flanco de subida [us]      */
    volatile uint32_t t_fall;      /* captura del flanco de bajada [us]      */
    volatile uint8_t  data_ready;  /* hay dato nuevo sin leer                */
    uint32_t          trigger_tick;/* HAL_GetTick() al disparar (timeout)    */

    HC_SR04_CompleteCallback on_complete;
};

/* Velocidad del sonido: 343 m/s -> 29.15 us/cm (ida) -> 58.3 us/cm (ida+vuelta) */
#define HC_SR04_ECHO_US_PER_CM 58.0f
#define HC_SR04_TRIG_PULSE_US  10u

/* Limites FISICOS del HC-SR04 (datasheet): NO son configurables. El driver ES
 * el HC-SR04, asi que su rango es un hecho del componente. Otro sensor de
 * distancia = otro modulo. Fuera de estos limites el eco es imposible y la
 * lectura se marca INVALID. La ventana util de la barra (recorte a la zona de
 * la pelota) es politica de la aplicacion y vive en task_sensor. */
#define HC_SR04_HW_MIN_CM      2.0f
#define HC_SR04_HW_MAX_CM      450.0f
#define HC_SR04_HW_TIMEOUT_MS  30u    /* eco maximo ~25 ms + margen */

/* Registro interno de instancias para el dispatch del callback global del HAL. */
#ifndef HC_SR04_MAX_INSTANCES
#define HC_SR04_MAX_INSTANCES  1
#endif

static HC_SR04_HandleTypeDef *s_instances[HC_SR04_MAX_INSTANCES] = {0};

/* Pool estatico de handles: el driver es dueno de la memoria (sin malloc).
 * Distinto de s_instances, que es el registro de punteros para el dispatch de
 * la ISR del HAL. */
static struct HC_SR04_Handle s_pool[HC_SR04_MAX_INSTANCES];
static unsigned              s_pool_count = 0u;

HC_SR04_HandleTypeDef *HC_SR04_Create(void)
{
    if (s_pool_count >= HC_SR04_MAX_INSTANCES) { return NULL; }
    return &s_pool[s_pool_count++];
}

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
 * No se usa para medir el ECHO (eso es 100% por interrupcion).
 *
 * Spinea sobre el contador del propio timer, que corre a 1 us/tick (PSC=15 a
 * 16 MHz). Es preciso al microsegundo por construccion e inmune a cambios de
 * reloj del CPU (mientras el PSC mantenga 1 us/tick). La resta unsigned maneja
 * el wrap-around del contador. Requiere que el timer ya este corriendo (en
 * Trigger se llama despues de HAL_TIM_IC_Start_IT).
 *
 * Sigue siendo busy-wait, pero para 10 us es inofensivo y estandar (los delays
 * de RTOS solo llegan a ~1 ms). Para eliminar el spin por completo se podria
 * generar el pulso de TRIG por hardware con One-Pulse Mode (OPM), a costa de
 * configurar un canal de salida dedicado en CubeMX. */
static void hc_sr04_delay_us(TIM_HandleTypeDef *htim, uint32_t us)
{
    uint32_t start = __HAL_TIM_GET_COUNTER(htim);
    while ((__HAL_TIM_GET_COUNTER(htim) - start) < us) {
        /* espera activa */
    }
}

/* ------------------------------------------------------------------------- */
/* API publica                                                               */
/* ------------------------------------------------------------------------- */

HC_SR04_Status HC_SR04_Init(HC_SR04_HandleTypeDef *h,
                            TimerChannel_t echo,
                            GpioPin_t trig)
{
    if (h == NULL || echo.htim == NULL || trig.port == NULL) {
        return HC_SR04_ERROR;
    }

    /* Se desarma el par: internamente el driver sigue usando htim/channel
     * sueltos (y active_ch derivado). */
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

    /* Preparar la maquina de estados para esperar el flanco de subida */
    h->data_ready = 0;
    h->state      = HC_SR04_STATE_WAIT_RISE;

    /* Configurar captura por flanco de SUBIDA y arrancar la IC con interrupcion */
    __HAL_TIM_SET_CAPTUREPOLARITY(h->htim, h->channel, TIM_INPUTCHANNELPOLARITY_RISING);

    if (HAL_TIM_IC_Start_IT(h->htim, h->channel) != HAL_OK) {
        h->state = HC_SR04_STATE_IDLE;
        return HC_SR04_ERROR;
    }

    /* Pulso de TRIG: 10 us en alto */
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
        /* La distancia se calcula aca (contexto de tarea), no en la ISR.
         * Resta unsigned: maneja el wrap-around del contador automaticamente. */
        uint32_t width_us = h->t_fall - h->t_rise;
        float d = (float)width_us / HC_SR04_ECHO_US_PER_CM;
        h->data_ready = 0;
        /* Se escribe *out_cm SIEMPRE (aun si la lectura se descarta): con el
         * status INVALID el llamador puede loguear el valor crudo rechazado,
         * que es la pista mas util para diagnosticar apuntado o rango mal
         * configurado. El status sigue siendo el que manda. */
        *out_cm = d;
        if (d < HC_SR04_HW_MIN_CM || d > HC_SR04_HW_MAX_CM) {
            return HC_SR04_INVALID;
        }
        return HC_SR04_OK;
    }

    /* Sin dato todavia: chequear timeout (no bloqueante) */
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
        /* Llego el flanco de subida: guardar t_rise y pasar a esperar bajada */
        h->t_rise = captured;
        h->state  = HC_SR04_STATE_WAIT_FALL;
        __HAL_TIM_SET_CAPTUREPOLARITY(h->htim, h->channel,
                                      TIM_INPUTCHANNELPOLARITY_FALLING);
    }
    else if (h->state == HC_SR04_STATE_WAIT_FALL) {
        /* Llego el flanco de bajada: latchear el timestamp crudo. El calculo de
         * distancia NO se hace aca: se difiere a GetDistance (contexto de tarea)
         * para minimizar el trabajo dentro de la ISR. */
        h->t_fall = captured;

        /* Medicion completa: detener IC, marcar dato y notificar */
        HAL_TIM_IC_Stop_IT(h->htim, h->channel);
        h->state      = HC_SR04_STATE_IDLE;
        h->data_ready = 1;

        if (h->on_complete != NULL) {
            h->on_complete(h);   /* contexto ISR: usar ...FromISR() adentro */
        }
    }
    /* Si llega en IDLE, se ignora (captura espuria). */
}

/* True si el handle corresponde a la interrupcion actual: no es NULL, es del
 * mismo timer, y del canal que disparo la captura (htim->Channel). */
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
