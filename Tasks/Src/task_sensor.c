/**
  ******************************************************************************
  * @file    task_sensor.c
  * @brief   Task del sensor (prio 5). Se despierta con SemTimer (tick de 100 ms
  *          de TIM4), dispara el HC-SR04, espera el echo (SemSensor) y publica
  *          la distancia cruda en QueuePos.
  *
  *          Politica: medir siempre que sea posible. Una lectura que cae fuera
  *          de la ventana util pero a menos de SENSOR_EDGE_GRACE_CM del borde se
  *          recorta y se publica igual (la pelota esta en la punta de la barra o
  *          en la zona muerta del sensor, y esa es informacion real). Solo se
  *          descarta el eco que volvio del ambiente, que si se usara mandaria al
  *          PID a inclinar por una pelota que no esta.
  ******************************************************************************
  */

#include "task_sensor.h"
#include "app.h"
#include "app_config.h"

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "queue.h"
#include "tim.h"        /* htim2 */

void SensorTask(void *argument)
{
  (void)argument;

  TimerChannel_t echo = { &htim2, TIM_CHANNEL_1 };
  GpioPin_t      trig = { TRIG_GPIO_Port, TRIG_Pin };
  HC_SR04_Init(&g_sensor, echo, trig);
  HC_SR04_SetRange(&g_sensor, SENSOR_MIN_CM, SENSOR_MAX_CM);
  HC_SR04_SetCompleteCallback(&g_sensor, App_OnSensorComplete_FromISR);

  for (;;)
  {
    /* Tick de 100 ms (lo da la ISR de TIM4). */
    if (xSemaphoreTake(SemTimer, portMAX_DELAY) != pdTRUE) { continue; }

    /* Descartar cualquier aviso viejo, para esperar SOLO esta medicion. */
    (void)xSemaphoreTake(SemSensor, 0);

    if (HC_SR04_Trigger(&g_sensor) != HC_SR04_OK)
    {
      continue;
    }

    if (xSemaphoreTake(SemSensor, pdMS_TO_TICKS(SENSOR_ECHO_TIMEOUT_MS)) != pdTRUE)
    {
      /* Sin echo: GetDistance detecta el timeout y resetea la FSM a IDLE. */
      float descarte = 0.0f;
      (void)HC_SR04_GetDistance(&g_sensor, &descarte);
      continue;
    }

    float dist = 0.0f;
    HC_SR04_Status st = HC_SR04_GetDistance(&g_sensor, &dist);

    if (st == HC_SR04_OK)
    {
      xQueueOverwrite(QueuePos, &dist);
    }
    else if (st == HC_SR04_INVALID)
    {
      /* La medicion se hizo bien: solo cayo fuera de la ventana util. A un pelo
       * del borde es la pelota en la punta de la barra (o pegada al sensor): se
       * recorta y se publica, porque descartarla dejaria al PID sin dato nuevo y
       * al servo congelado en la ultima inclinacion. Lejos de la ventana es un
       * eco del ambiente: esa si se descarta. */
      if ((dist > (SENSOR_MIN_CM - SENSOR_EDGE_GRACE_CM)) &&
          (dist < (SENSOR_MAX_CM + SENSOR_EDGE_GRACE_CM)))
      {
        float borde = (dist < SENSOR_MIN_CM) ? SENSOR_MIN_CM : SENSOR_MAX_CM;
        xQueueOverwrite(QueuePos, &borde);
      }
    }
  }
}
