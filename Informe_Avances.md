# Informe de Avances — Ball-and-Beam (control PID en tiempo real)

Sistema de **balanceo (ball-and-beam)** sobre **STM32F401RE (NUCLEO)** con **FreeRTOS**: un objeto
desliza sobre una barra articulada; la distancia se mide con un **HC-SR04**, se **filtra con Kalman**,
un **PID** calcula la corrección y un **servo MG90S** inclina la barra. El **setpoint** lo fija un
**potenciómetro** leído por ADC.

## Arquitectura de software

Todo el código de aplicación vive en **`App/`** (CubeMX no la toca); `main.c`/`freertos.c` solo tienen
*glue* mínimo en secciones `USER CODE`, de modo que el proyecto **sobrevive a la regeneración de CubeMX**.

### Módulos

| Capa | Archivos (`App/`) | Rol |
|---|---|---|
| Drivers | `hc_sr04.*`, `servo_mg90s.*`, `potentiometer.*` | HAL, multi-instancia, RTOS-agnósticos |
| Algoritmos | `kalman.*`, `pid.*` | Módulos **puros** (solo `float`), testeables |
| Tests | `selftest.*` | Casos dummy on-target (opt-in por flag) |
| Wiring | `app.*`, `app_config.h` | Instancias, IPC, hooks de ISR, `App_Init()` |
| Tasks | `task_sensor/kalman/pid/motor/pot.*` | Una task por archivo |

### Tareas (FreeRTOS, API nativa)

| Task | Prio | Consume → Produce |
|---|---|---|
| `SensorTask` | 5 | `SemTimer`, `SemSensor` → `QueuePos` |
| `KalmanTask` | 4 | `QueuePos` → `QueuePosFil` |
| `MotorTask` | 4 | `QueueAngulo` → PWM servo (+ heartbeat LD2) |
| `PidTask` | 3 | QueueSet{`QueuePosFil`, `QueueObjetivo`} → `QueueAngulo` |
| `PotTask` | 1 | ADC (pote) → `QueueObjetivo` |

### IPC

- **4 colas** `float` de profundidad 1 (`xQueueOverwrite`/`xQueueReceive`): `QueuePos`,
  `QueuePosFil`, `QueueObjetivo`, `QueueAngulo`.
- **Queue Set** en el PID (`xQueueSelectFromSet`) para bloquear a la vez en posición filtrada y
  setpoint (`configUSE_QUEUE_SETS=1`, en `USER CODE` de `FreeRTOSConfig.h`).
- **2 semáforos binarios**: `SemTimer` (ISR de **TIM4** cada 100 ms → tick hard real-time del sensor)
  y `SemSensor` (ISR de **Input Capture** del HC-SR04, echo completo).

### Algoritmos

- **Kalman** 1D, 2 estados `[posición, velocidad]`, modelo de velocidad constante, `dt = 0.1 s`.
  `Q` = ruido de proceso, `R` = varianza de medición del HC-SR04 (~0.09).
- **PID** discreto con **derivada sobre la medición** (sin *derivative-kick*), **anti-windup** por
  integración condicional (clamping) y **saturación** de salida (±20° alrededor del centro).

## Configuración de hardware (CubeMX)

- Reloj: SYSCLK 64 MHz (PLL/HSI), **HCLK 16 MHz** → timers a 16 MHz.
- **TIM2** Input Capture (ECHO, PA0/CH1), PSC=15 (1 µs/tick); **TRIG** PA9. NVIC prio 5.
- **TIM3** PWM (servo, PA6/D12), PSC=15, ARR=19999 (50 Hz).
- **TIM4** base 100 ms (PSC=1599, ARR=999), NVIC prio 5.
- **ADC1** IN4 (pote, PA4), 12 bits, polling, sampling 480 ciclos.
- **USART2** 115200 (COM virtual ST-Link).
- FreeRTOS: heap 24576, CMSIS_V2 + API nativa.

## Cableado

- **HC-SR04:** TRIG→D8/PA9, ECHO→A0/PA0 vía divisor 1kΩ/2.2kΩ, VCC 5 V, GND común.
- **Servo MG90S:** señal PA6/D12, alimentación externa 5 V, **masa común** con la Nucleo.
- **Potenciómetro:** extremos a **3.3 V** y GND, cursor a **PA4** (entra directo al ADC, **sin divisor**).

## Flags de compilación (`app_config.h`)

| Flag | Producción | Uso |
|---|---|---|
| `APP_RUN_SELFTESTS` | 0 | 1 = corre los self-tests de Kalman/PID al arrancar |
| `APP_USE_SYNTHETIC_SENSOR` | 0 | 1 = señal sintética para validar la cadena sin HW |
| `APP_LOG_ENABLED` | 0 | 1 = traza `z, fil, sp, u, ang` por UART (subir para tuning) |

## Estado

- **Etapas 0–4** implementadas y commiteadas (baseline, `App/`, CubeMX, Kalman/PID + tests, pipeline
  RTOS). **Etapa 5** (lazo real) con calibración enganchada y ganancias iniciales; **tuning fino
  pendiente sobre el hardware**. **Etapa 6** (cierre): stacks dimensionados por rol, logs gateados,
  chequeos de creación de IPC.
- **Pendiente [hardware/IDE]:** verificación funcional on-target (UART + servo + lazo), tuning
  iterativo (Kalman `Q/R`, PID `Kp/Ki/Kd`) y la prueba de fuego de regeneración de CubeMX.

## Cómo tunear

1. `APP_LOG_ENABLED=1`, build + flash, abrir UART (115200, COM3).
2. Kalman: `R≈0.09`, subir `Q` si el filtro va lento, bajarlo si pasa ruido.
3. PID: primero `Kp` (respuesta sin oscilar), luego `Kd` (amortigua), luego `Ki` chico (error
   estacionario). Vigilar saturación/anti-windup.
4. Al cerrar, `APP_LOG_ENABLED=0` (build de producción sin latencia de UART en el lazo).
