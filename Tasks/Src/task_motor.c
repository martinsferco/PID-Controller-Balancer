/**
  ******************************************************************************
  * @file    task_motor.c
  * @brief   Task del actuador (prio 4). Recibe el angulo de queue_angulo y lo
  *          aplica al servo, con toggle de LD2 (PA5) como heartbeat del lazo.
  *
  *          Habla SOLO en grados: los microsegundos son asunto del driver. El
  *          recorte contra los topes fisicos tampoco se hace aca, sino en el
  *          driver via Servo_SetTravel(), asi vale para cualquier llamador.
  *
  *          El servo ya viene creado, inicializado, con el recorrido declarado
  *          y nivelado desde App_Init; la task solo recibe el contexto y corre.
  ******************************************************************************
  */

#include "task_motor.h"
#include "app_config.h"

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

void MotorTask(void *argument)
{
  TaskMotorContext *context = (TaskMotorContext *)argument;

  /* Estado del failsafe: se actua SOLO en el flanco, para nivelar una vez al
   * perder la pelota en vez de reescribir el mismo angulo en cada timeout. */
  uint8_t perdida = 0u;

  for (;;)
  {
    float angle = 0.0f;
    if (xQueueReceive(context->queue_angulo, &angle, pdMS_TO_TICKS(APP_MOTOR_TIMEOUT_MS)) == pdTRUE)
    {
      perdida = 0u;
      Servo_SetAngle(context->servo, angle);
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
      Servo_SetAngle(context->servo, SERVO_LEVEL_DEG);
    }
  }
}
