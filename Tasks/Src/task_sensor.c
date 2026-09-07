/**
  ******************************************************************************
  * @file    task_sensor.c
  * @brief   Task del sensor: dispara el HC-SR04 con el tick de 100 ms,
  *          espera el echo y publica la distancia medida. HC_SR04_OK se
  *          publica tal cual; HC_SR04_INVALID se resuelve a HC_SR04_HW_MIN_CM
  *          o SENSOR_MIN_CM/2 segun la causa.
  ******************************************************************************
  */

#include "task_sensor.h"
#include "app_config.h"
#include "debug_uart.h"

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "queue.h"

static const char *SensorStatusName(HC_SR04_Status st)
{
  switch (st)
  {
    case HC_SR04_OK:      return "OK";
    case HC_SR04_BUSY:    return "BUSY";
    case HC_SR04_TIMEOUT: return "TIMEOUT";
    case HC_SR04_INVALID: return "INVALID";
    default:              return "ERROR";
  }
}

void SensorTask(void *argument)
{
  TaskSensorContext *context = (TaskSensorContext *)argument;

  for (;;)
  {
    // Tick de 100 ms (lo da la ISR de TIM4)
    if (xSemaphoreTake(context->sem_timer, portMAX_DELAY) != pdTRUE) { continue; }

    // Descartar cualquier aviso viejo, para esperar SOLO esta medicion
    (void)xSemaphoreTake(context->sem_sensor, 0);

    if (HC_SR04_Trigger(context->sensor) != HC_SR04_OK)
    {
      DebugUart_Print("[%10lu] SENSOR trigger BUSY\r\n", (unsigned long)HAL_GetTick());
      continue;
    }

    if (xSemaphoreTake(context->sem_sensor, pdMS_TO_TICKS(SENSOR_ECHO_TIMEOUT_MS)) != pdTRUE)
    {
      float descarte = 0.0f;
      (void)HC_SR04_GetDistance(context->sensor, &descarte);
      DebugUart_Print("[%10lu] SENSOR status=TIMEOUT\r\n", (unsigned long)HAL_GetTick());
      continue;
    }

    float dist = 0.0f;
    HC_SR04_Status st = HC_SR04_GetDistance(context->sensor, &dist);

    float borde;

    if (st == HC_SR04_OK)
    {
      borde = dist;
    }
    else if (st == HC_SR04_INVALID)
    {
      borde = (dist < HC_SR04_HW_MIN_CM) ? HC_SR04_HW_MIN_CM : SENSOR_MIN_CM / 2.0f;
    }
    else
    {
      DebugUart_Print("[%10lu] SENSOR status=%s\r\n", (unsigned long)HAL_GetTick(), SensorStatusName(st));
      continue;
    }

    DebugUart_Print("[%10lu] SENSOR dist=%.2fcm status=%s borde=%.2fcm\r\n",
                     (unsigned long)HAL_GetTick(), dist, SensorStatusName(st), borde);

    xQueueOverwrite(context->queue_pos, &borde);
  }
}
