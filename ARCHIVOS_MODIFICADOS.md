# Inventario de archivos — qué escribimos nosotros y qué generó CubeMX

Este documento separa, archivo por archivo, **el código propio del proyecto** de **el código
generado por STM32CubeIDE / CubeMX**, e indica exactamente qué partes se tocaron en los archivos
generados.

Base de comparación: commit `31f0f0f` *("Proyecto generado por CubeMX (configuración inicial)")*
contra la rama **`production`**. Se puede reproducir con:

```
git diff --stat 31f0f0f..production -- . ':(exclude)Drivers' ':(exclude)Middlewares' ':(exclude)Debug'
```

**Resumen:** 42 archivos cambiados, de los cuales **26 son enteramente nuestros** y solo
**2 son ediciones a mano sobre código generado**.

---

## 1. Archivos escritos desde cero (20 de código)

Ninguno de estos existe en el proyecto original de CubeMX.

### `App/` — wiring de la aplicación

| Archivo | Rol |
|---|---|
| `App/Inc/app.h` | Declaraciones del wiring, hooks de ISR, tipo `PosFil_t` |
| `App/Inc/app_config.h` | Único lugar de constantes: montaje físico, ganancias, stacks/prioridades, flags de banco |
| `App/Src/app.c` | Instancias de drivers, 4 colas + queue set + 2 semáforos, arranque de periféricos, creación de las 5 tasks (`App_Init()`) |

### `ComponentDrivers/` — drivers de hardware (HAL, multi-instancia, RTOS-agnósticos)

| Archivo | Rol |
|---|---|
| `Inc/hc_sr04.h` + `Src/hc_sr04.c` | Sensor de distancia por ultrasonido, FSM sobre Input Capture |
| `Inc/servo_mg90s.h` + `Src/servo_mg90s.c` | Servo por PWM, API **solo en grados** (`Servo_SetAngle`) |
| `Inc/potentiometer.h` + `Src/potentiometer.c` | Potenciómetro por ADC (polling) |

### `Control/` — módulos puros (solo `float`: sin HAL, sin RTOS, testeables)

| Archivo | Rol |
|---|---|
| `Inc/kalman.h` + `Src/kalman.c` | Filtro de Kalman de 2 estados (posición y velocidad) |
| `Inc/pid.h` + `Src/pid.c` | PID con derivada sobre la medición, anti-windup, banda de integración y descarga al cruzar el setpoint |

### `Tasks/` — una task de FreeRTOS por archivo

`Inc/` + `Src/` de: `task_sensor`, `task_kalman`, `task_pid`, `task_motor`, `task_pot`.

> Los drivers `hc_sr04.*` y `servo_mg90s.*` nacieron en `Core/`, se movieron a `App/` en la
> Etapa 1 y de ahí a `ComponentDrivers/` en la reorganización posterior. Siempre con `git mv`,
> así que el historial de cada archivo se conserva.

### Documentación (raíz del proyecto)

`DOCUMENTACION_TECNICA.md` · `Documentacion_Tecnica.html` · `Documentacion_Tecnica.pdf` ·
`GUIA_CONTINUACION_HARDWARE.md` · `Informe_Avances.md` · `Calibracion_Servo_Rango.md` ·
`ARCHIVOS_MODIFICADOS.md` (este archivo).

En la rama `tests` se agregan además `selftest.c/.h` y `TESTS.md`.

---

## 2. Archivos de CubeMX editados A MANO

Estos son los **únicos dos** archivos generados donde se escribió código propio.

### `Core/Src/main.c`

| Zona | Qué se agregó |
|---|---|
| `USER CODE Includes` | `#include "app.h"` |
| `USER CODE 2` | `App_Init();` — crea semáforos, colas y las 5 tasks |
| `USER CODE WHILE` | Se reemplazó el arranque CMSIS (`osKernelInitialize` / `MX_FREERTOS_Init` / `osKernelStart`) por **`vTaskStartScheduler()`** nativo |
| `SystemClock_Config` | Reloj **HSI directo a 16 MHz, sin PLL**: `PLL.PLLState = RCC_PLL_NONE`, `SYSCLKSource = HSI`, `AHBCLKDivider = DIV1` |
| `USER CODE 4` | `HAL_TIM_IC_CaptureCallback()` → despacha a `HC_SR04_HandleInterrupt()` |
| `USER CODE Callback 1` | Dentro de `HAL_TIM_PeriodElapsedCallback`: si es TIM4 → `App_OnTimerTick_FromISR()` (tick de 100 ms) |

> Las dos ediciones del medio (`USER CODE WHILE` y `SystemClock_Config`) son las únicas que caen
> **fuera** de una sección `USER CODE`, así que son las únicas que hay que revisar a mano después
> de una regeneración de CubeMX.

### `Core/Inc/FreeRTOSConfig.h`

| Cambio | Motivo |
|---|---|
| `configTOTAL_HEAP_SIZE`: `15360` → **`24576`** | Hacen falta 5 tasks + colas + semáforos |
| `#define configUSE_QUEUE_SETS 1` en `USER CODE Defines` | CubeMX no expone ese parámetro en la GUI; va en la sección `USER CODE` para que **sobreviva a las regeneraciones**. Lo necesita `xQueueSelectFromSet()` en `task_pid` |

---

## 3. Archivos que cambiaron pero NO se tocaron a mano

Son resultado de lo configurado en la GUI de CubeMX. Sus secciones `USER CODE` están vacías.
Se listan para que no confundan al mirar el diff.

| Archivo | Qué cambió y por qué |
|---|---|
| `Core/Src/tim.c` + `Core/Inc/tim.h` | Se agregaron `MX_TIM3_Init` (PWM del servo) y `MX_TIM4_Init` (PSC = 1599 / ARR = 999 → 100 ms) |
| `Core/Src/adc.c` + `Core/Inc/adc.h` | Archivos nuevos: ADC1 canal **IN4 = PA4** para el potenciómetro |
| `Core/Src/stm32f4xx_it.c` + `Core/Inc/stm32f4xx_it.h` | `TIM4_IRQHandler` y el `extern htim4` |
| `Core/Inc/stm32f4xx_hal_conf.h` | Se destrabó `HAL_ADC_MODULE_ENABLED` al habilitar el ADC |

---

## 4. Archivos de configuración del proyecto

| Archivo | Qué cambió |
|---|---|
| `PIDController.ioc` | Configuración de periféricos: TIM2 (IC del sensor), TIM3 (PWM del servo), TIM4 (tick 100 ms), ADC1/PA4, USART2, heap de FreeRTOS, reloj HSI |
| `.cproject` | Alta de `App`, `ComponentDrivers`, `Control` y `Tasks` como *source folders*, y sus `Inc/` en el *include path* (Debug y Release) |
| `.mxproject`, `.settings/language.settings.xml` | Se actualizan solos junto con los anteriores |

---

## Nota sobre las ramas

| Rama | Contenido |
|---|---|
| **`production`** | Rama actual. Solo el lazo de control, código limpio para el informe. Carpetas separadas `ComponentDrivers/` · `Control/` · `Tasks/` |
| `master` | Versión previa a la limpieza |
| `tests` | Versión con todo el andamiaje de diagnóstico: `selftest.*`, `App_LogTrace`, flags `APP_RUN_SELFTESTS` / `APP_USE_SYNTHETIC_SENSOR` / `APP_LOG_ENABLED`, y layout plano en `App/`. **Para tunear o depurar: `git checkout tests`** |
