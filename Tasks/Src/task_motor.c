/**
  ******************************************************************************
  * @file    task_motor.c
  * @brief   Task del actuador (prio 4). Recibe el angulo de QueueAngulo y lo
  *          aplica al servo, con toggle de LD2 (PA5) como heartbeat del lazo.
  *
  *          Habla SOLO en grados: los microsegundos son asunto del driver. El
  *          recorte contra los topes fisicos tampoco se hace aca, sino en el
  *          driver via Servo_SetTravel(), asi vale para cualquier llamador.
  ******************************************************************************
  */

#include "task_motor.h"
#include "app.h"
#include "app_config.h"

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "tim.h"        /* htim3 */

void MotorTask(void *argument)
{
  (void)argument;

  /* Servo_Init arranca el PWM; Servo_SetTravel declara la guarda contra los
   * topes, que es lo unico que la aplicacion decide sobre el recorrido. Si
   * alguna de las dos falla, el servo quedaria sin limite: no se sigue. */
  Servo_Status st = Servo_Init(&g_servo, &htim3, TIM_CHANNEL_1);
  if (st == SERVO_OK)
  {
    st = Servo_SetTravel(&g_servo, SERVO_MIN_DEG, SERVO_MAX_DEG);
  }
  if (st != SERVO_OK) { Error_Handler(); }

  Servo_SetAngle(&g_servo, SERVO_LEVEL_DEG);   /* arrancar con la barra nivelada */

  /* Estado del failsafe: se actua SOLO en el flanco, para nivelar una vez al
   * perder la pelota en vez de reescribir el mismo angulo en cada timeout. */
  uint8_t perdida = 0u;

  for (;;)
  {
    float angle = 0.0f;
    if (xQueueReceive(QueueAngulo, &angle, pdMS_TO_TICKS(APP_MOTOR_TIMEOUT_MS)) == pdTRUE)
    {
      perdida = 0u;
      Servo_SetAngle(&g_servo, angle);
      HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);   /* heartbeat del lazo */
    }
    else if (!perdida)
    {
      /* FAILSAFE: pasaron APP_MOTOR_TIMEOUT_MS sin angulo nuevo, o sea que el
       * sensor dejo de ver la pelota. Sin esto el servo se quedaria clavado en
       * la ultima inclinacion, que es la peor posicion posible para que la
       * pelota vuelva. Nivelar es lo unico razonable sin medicion.
       *
       * El integrador del PID no se descarga aca, y no hace falta: PidTask solo
       * integra cuando LLEGA una posicion, asi que mientras el sensor no
       * entregue queda congelado en vez de acumular. */
      perdida = 1u;
      Servo_SetAngle(&g_servo, SERVO_LEVEL_DEG);
    }
  }
}
