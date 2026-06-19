/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "hc_sr04.h"
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
static HC_SR04_HandleTypeDef g_sensor;        /* instancia del sensor        */
static SemaphoreHandle_t     g_sensorSem;     /* aviso ISR -> task (binario) */
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MX_FREERTOS_Init(void);
/* USER CODE BEGIN PFP */
void SensorTask(void *argument);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM2_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  g_sensorSem = xSemaphoreCreateBinary();
  xTaskCreate(SensorTask, "SensorTask", 256, NULL, 1, NULL);
  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();  /* Call init function for freertos objects (in cmsis_os2.c) */
  MX_FREERTOS_Init();

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 64;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV4;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* Puente HAL -> libreria: el HAL invoca este callback en cada captura de
 * Input Capture (contexto ISR). El dispatcher la entrega a la instancia del
 * sensor que corresponde, segun (timer, canal activo). Ver punto 2.3. */
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
  HC_SR04_HandleInterrupt(htim);
}

/* Hook que el driver invoca AL COMPLETAR una medicion (contexto ISR de TIM2).
 * Despierta a SensorTask. Seguro desde ISR porque TIM2 esta en prioridad NVIC 5
 * (>= configMAX_SYSCALL_INTERRUPT_PRIORITY). */
static void sensor_on_complete(HC_SR04_HandleTypeDef *h)
{
  (void)h;
  BaseType_t higherPriorityTaskWoken = pdFALSE;
  xSemaphoreGiveFromISR(g_sensorSem, &higherPriorityTaskWoken);
  portYIELD_FROM_ISR(higherPriorityTaskWoken);
}

/* Retarget de printf hacia USART2 (COM virtual del ST-Link), solo para pruebas. */
int __io_putchar(int ch)
{
  HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
  return ch;
}

/* Task del sensor: dispara, se bloquea hasta el aviso de la ISR, lee y reporta.
 * No hace polling ni busy-wait: mientras espera el echo, la CPU queda libre. */
void SensorTask(void *argument)
{
  (void)argument;

  HC_SR04_Init(&g_sensor, &htim2, TIM_CHANNEL_1, TRIG_GPIO_Port, TRIG_Pin);
  HC_SR04_SetRange(&g_sensor, 2.0f, 50.0f);          /* ajustar al largo de la barra */
  HC_SR04_SetCompleteCallback(&g_sensor, sensor_on_complete);

  for (;;)
  {
    /* Descartar cualquier aviso viejo para esperar SOLO esta medicion. */
    (void)xSemaphoreTake(g_sensorSem, 0);

    if (HC_SR04_Trigger(&g_sensor) == HC_SR04_OK)
    {
      /* Bloqueo hasta el aviso de la ISR (guarda de 80 ms > timeout interno). */
      if (xSemaphoreTake(g_sensorSem, pdMS_TO_TICKS(80)) == pdTRUE)
      {
        float dist = 0.0f;
        HC_SR04_Status st = HC_SR04_GetDistance(&g_sensor, &dist);
        if (st == HC_SR04_OK) {
          /* Sin %f (newlib-nano no lo imprime por defecto): parte entera y 1 decimal. */
          int entero   = (int)dist;
          int decima   = (int)((dist - (float)entero) * 10.0f);
          printf("Distancia: %d.%d cm\r\n", entero, decima);
          /* TODO (paso siguiente): alimentar el lazo PID con 'dist'. */
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

    vTaskDelay(pdMS_TO_TICKS(50));   /* ~20 Hz de muestreo (ajustable segun el PID) */
  }
}

/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM5 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM5)
  {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
