/**
  ******************************************************************************
  * @file    task_sensor.c
  * @brief   Task del sensor HC-SR04. Dispara una medicion, se bloquea hasta el
  *          aviso de la ISR (sin polling ni busy-wait), lee la distancia y la
  *          reporta por UART. Comportamiento identico al baseline (Etapa 0).
  ******************************************************************************
  */

#include "task_sensor.h"
#include "app.h"
#include "app_config.h"

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "tim.h"        /* htim2 */
#include <stdio.h>

/* Task del sensor: dispara, se bloquea hasta el aviso de la ISR, lee y reporta.
 * No hace polling ni busy-wait: mientras espera el echo, la CPU queda libre. */
void SensorTask(void *argument)
{
  (void)argument;

  HC_SR04_Init(&g_sensor, &htim2, TIM_CHANNEL_1, TRIG_GPIO_Port, TRIG_Pin);
  HC_SR04_SetRange(&g_sensor, SENSOR_MIN_CM, SENSOR_MAX_CM);   /* ajustar al largo de la barra */
  HC_SR04_SetCompleteCallback(&g_sensor, App_OnSensorComplete_FromISR);

  for (;;)
  {
    /* Descartar cualquier aviso viejo para esperar SOLO esta medicion. */
    (void)xSemaphoreTake(g_sensorSem, 0);

    if (HC_SR04_Trigger(&g_sensor) == HC_SR04_OK)
    {
      /* Bloqueo hasta el aviso de la ISR (guarda > timeout interno). */
      if (xSemaphoreTake(g_sensorSem, pdMS_TO_TICKS(SENSOR_ECHO_TIMEOUT_MS)) == pdTRUE)
      {
        float dist = 0.0f;
        HC_SR04_Status st = HC_SR04_GetDistance(&g_sensor, &dist);
        if (st == HC_SR04_OK) {
          /* Sin %f (newlib-nano no lo imprime por defecto): parte entera y 1 decimal. */
          int entero   = (int)dist;
          int decima   = (int)((dist - (float)entero) * 10.0f);
          printf("Distancia: %d.%d cm\r\n", entero, decima);
          /* TODO (etapa siguiente): alimentar el lazo PID con 'dist'. */
        } else if (st == HC_SR04_INVALID) {
          printf("Fuera de rango\r\n");
        }
      }
      else
      {
        /* No llego el echo: GetDistance detecta el timeout y resetea la FSM a IDLE. */
        float dummy = 0.0f;
        (void)HC_SR04_GetDistance(&g_sensor, &dummy);
        printf("TIMEOUT\r\n");
      }
    }

    vTaskDelay(pdMS_TO_TICKS(SENSOR_PERIOD_MS));   /* ~20 Hz de muestreo (ajustable segun el PID) */
  }
}
