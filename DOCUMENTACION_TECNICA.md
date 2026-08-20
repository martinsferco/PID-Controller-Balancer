# Documentación técnica — Ball-and-Beam (PID en tiempo real sobre STM32 + FreeRTOS)

Documento de referencia para entender **todo el proyecto desde cero**: qué hace cada archivo, cada
configuración de CubeMX y cómo fluye la información en tiempo real. Complemento operativo:
`GUIA_CONTINUACION_HARDWARE.md`.

---

## 1. Qué es el proyecto

Un sistema de **balanceo (ball-and-beam)**: un objeto desliza sobre una **barra articulada** inclinada
por un servo. El objetivo es **posicionar el objeto** en un punto de la barra y mantenerlo estable.

Cadena de control:

```
       distancia            posición             error->corrección        ángulo
HC-SR04 ─────────> Kalman ───────────────> PID ───────────────────> Servo MG90S
(mide)             (filtra ruido)          (calcula acción)          (inclina la barra)
                                             ▲
                                             │ setpoint (punto deseado)
                                     Potenciómetro (ADC)
```

- **Plataforma:** STM32F401RE (NUCLEO-F401RE), HAL (CubeMX) + **FreeRTOS** (API nativa).
- **Idea arquitectónica central:** *todo* el código de aplicación vive en la carpeta **`App/`** (que
  CubeMX no toca); `main.c`/`freertos.c` quedan solo con *glue* mínimo dentro de secciones
  `USER CODE`. Así el proyecto **sobrevive a cada regeneración de CubeMX**.

---

## 2. Configuración de hardware (CubeMX / `PIDController.ioc`)

### 2.1 Reloj
- Oscilador **HSI** (16 MHz interno), **sin PLL** — mismo estilo que el proyecto de referencia BlinkyRTOS.
- SYSCLK = HSI directo = **16 MHz**. AHB divisor **/1** → **HCLK = 16 MHz**. APB1 /2, APB2 /1.
- **Consecuencia clave:** TIM2/TIM3/TIM4 (bus APB1) corren a **16 MHz** (el HAL aplica ×2 cuando el
  prescaler de APB1 ≠ 1). Por eso los prescalers se calculan sobre 16 MHz (PSC=15 → 1 µs/tick).

### 2.2 Periféricos

| Periférico | Uso | Config | Pines |
|---|---|---|---|
| **TIM2** | Input Capture ECHO (HC-SR04) | PSC=**15** (1 µs/tick), 32-bit, NVIC prio **5** | ECHO=PA0 (CH1) |
| **GPIO** | TRIG del sensor | Output push-pull | TRIG=PA9 (D8) |
| **TIM3** | PWM del servo | PSC=**15**, ARR=**19999** (20 ms=50 Hz) | Señal=PA6 (D12) |
| **TIM4** | Base de tiempo 100 ms | PSC=**1599**, ARR=**999** (100 ms), NVIC prio **5** | — (sin pines) |
| **ADC1** | Potenciómetro | 12 bits, IN4, polling, sampling 480 ciclos, sin DMA/IRQ | Pote=PA4 (A2) |
| **USART2** | Consola / traza | 115200 8N1 (COM virtual ST-Link) | TX=PA2, RX=PA3 |
| **TIM5** | Time base del HAL (`HAL_IncTick`) | (lo pone CubeMX solo) | — |
| **LD2** | Heartbeat del lazo | GPIO Output | PA5 (LED onboard) |

### 2.3 FreeRTOS
- **API nativa de FreeRTOS**: tasks con `xTaskCreate`, arranque con `vTaskStartScheduler()` (estilo
  BlinkyRTOS). CubeMX genera la capa CMSIS-RTOS v2 pero **no se usa**: en `main.c` el arranque
  `osKernelInitialize/MX_FREERTOS_Init/osKernelStart` se reemplazó por `vTaskStartScheduler()` en
  `USER CODE WHILE` (⚠️ re-aplicar tras cada regeneración de CubeMX; ver §7.3).
- `configTOTAL_HEAP_SIZE = 24576`, `configMAX_PRIORITIES = 56`, `configMINIMAL_STACK_SIZE = 128`.
- `configUSE_QUEUE_SETS = 1` → **agregado a mano** en `USER CODE` de `FreeRTOSConfig.h` (CubeMX no lo
  expone en la GUI; así sobrevive a regeneraciones).

### 2.4 ¿Por qué las prioridades NVIC en 5?
`configMAX_SYSCALL_INTERRUPT_PRIORITY` = 5. Una ISR que llama `...FromISR()` (dar un semáforo) debe
tener prioridad NVIC **numéricamente ≥ 5** (menos urgente). TIM2 (echo) y TIM4 (tick) dan semáforos,
por eso ambas van en **5**. Con prioridad < 5 → **hard fault**.

---

## 3. Estructura de carpetas

```
PIDController/
├─ App/                      ← TODO el código de aplicación (CubeMX no lo toca)
│  ├─ Inc/  *.h
│  └─ Src/  *.c
├─ Core/                     ← generado por CubeMX (solo glue en USER CODE)
│  ├─ Inc/  main.h, tim.h, adc.h, usart.h, gpio.h, FreeRTOSConfig.h, stm32f4xx_it.h ...
│  └─ Src/  main.c, tim.c, adc.c, usart.c, gpio.c, stm32f4xx_it.c, freertos.c ...
├─ Drivers/                  ← HAL + CMSIS (generado)
├─ Middlewares/              ← FreeRTOS (generado)
├─ PIDController.ioc         ← configuración CubeMX
├─ Informe_Avances.md        ← resumen para el informe del anteproyecto
├─ GUIA_CONTINUACION_HARDWARE.md
└─ DOCUMENTACION_TECNICA.md  ← este archivo
```

`App/` está dada de alta como **Source Folder** en el `.cproject` (Debug y Release) y `../App/Inc`
está en el include path de ambas configuraciones.

---

## 4. Archivos de `App/` — uno por uno

### 4.1 Drivers (HAL, multi-instancia, RTOS-agnósticos)

Los tres drivers no llaman a FreeRTOS ni dependen del reloj; solo usan HAL. Son **multi-instancia**
(varios sensores/servos/potes con distintos handles).

#### `hc_sr04.h` / `hc_sr04.c` — sensor ultrasónico
Mide distancia por **Input Capture** basado en interrupciones (sin busy-wait para el echo).
- **`HC_SR04_Init(h, htim, channel, trig_port, trig_pin)`** — configura el handle, deja TRIG en bajo
  y registra la instancia en una tabla interna (`s_instances[]`) para el dispatch del callback global.
- **`HC_SR04_SetRange(h, min, max)`** — rango válido en cm (fuera de rango ⇒ `HC_SR04_INVALID`).
- **`HC_SR04_SetCompleteCallback(h, cb)`** — hook que se llama (en ISR) al completar una medición.
- **`HC_SR04_Trigger(h)`** — emite el pulso de 10 µs en TRIG y arma la captura por flanco de subida.
  Es lo único con una micro-demora bloqueante (`hc_sr04_delay_us`, ~10 µs, inofensiva).
- **`HC_SR04_GetDistance(h, &cm)`** — no bloqueante: devuelve `OK`/`BUSY`/`TIMEOUT`/`INVALID`.
- **`HC_SR04_HandleInterrupt(htim)`** — *dispatcher*: se llama desde `HAL_TIM_IC_CaptureCallback` y
  entrega la captura a la instancia correcta según `(timer, canal activo)`.
- **FSM interna:** IDLE → WAIT_RISE (guarda `t_rise`) → WAIT_FALL (guarda `t_fall`). Distancia =
  `(t_fall - t_rise) / 58` cm. La resta *unsigned* maneja el wrap-around del contador.

#### `servo_mg90s.h` / `servo_mg90s.c` — servo por PWM
Con el timer a 1 µs/tick y ARR=19999, **el CCR en cuentas == ancho de pulso en µs**.
- **`Servo_Init(s, htim, channel)`** — arranca el PWM y centra (1500 µs). Defaults 1000–2000 µs = 0–180°.
- **`Servo_SetLimits(s, min_us, max_us, min_deg, max_deg)`** — **calibración** (el mapeo default rara
  vez coincide con la mecánica real).
- **`Servo_SetPulseUs(s, us)`** — fija el pulso directo (clampeado a `[min_us, max_us]`).
- **`Servo_SetAngle(s, deg)`** — mapea linealmente grados→µs y clampea. (Es lo que usa el lazo.)
- **`Servo_GetPulseUs(s)`** — último pulso aplicado.

#### `potentiometer.h` / `potentiometer.c` — potenciómetro por ADC
Lee por **polling** (sin DMA/IRQ), pensado para baja frecuencia.
- **`Pot_Init(p, hadc, range_cm)`** — solo inicializa el struct (full_scale=4095 por 12 bits,
  timeout 10 ms). El ADC ya lo configuró CubeMX.
- **`Pot_ReadRaw(p, &raw)`** — `HAL_ADC_Start` → `HAL_ADC_PollForConversion` → `HAL_ADC_GetValue` → Stop.
- **`Pot_ReadPosition_cm(p, &cm)`** — lee y mapea el crudo a `[0, range_cm]`.

### 4.2 Algoritmos (módulos puros — solo `float`, sin HAL ni FreeRTOS)

#### `kalman.h` / `kalman.c` — filtro de Kalman 2 estados
Estado `x = [posición, velocidad]`, modelo de **velocidad constante**, `dt` fijo.
- Matrices: `F=[[1,dt],[0,1]]`, `H=[1,0]`, `Q = q·[[dt³/3, dt²/2],[dt²/2, dt]]`, `R` escalar.
- **`Kalman_Init(kf, dt, q, r, x0)`** — parámetros + covarianza inicial `P=I`.
- **`Kalman_Update(kf, z)`** — un ciclo *predict* + *update*; devuelve la posición estimada.
- **`Kalman_Reset(kf, x0)`** — reinicia estado y covarianza (se usa para arrancar en la 1ª muestra).
- `q` = ruido de proceso (↑ = más responsivo), `r` = varianza de medición del HC-SR04 (~0.09).

#### `pid.h` / `pid.c` — PID discreto
- **Derivada sobre la medición** (no sobre el error) ⇒ sin *derivative-kick* al mover el setpoint.
- **Anti-windup** por integración condicional (*clamping*): si la salida satura y el error empuja más
  hacia la saturación, congela el integrador; si el error apunta a salir, deja integrar.
- **Saturación** de salida a `[out_min, out_max]`.
- **`PID_Init(kp, ki, kd, dt)`** (anti-windup ON por defecto), **`PID_SetLimits(min, max)`**,
  **`PID_Compute(setpoint, meas)`** (devuelve la acción `u`), **`PID_Reset()`**.
- Campo `anti_windup` (int): el `selftest` lo pone en 0 para comparar contra la variante sin AW.

#### `selftest.h` / `selftest.c` — pruebas on-target (opt-in)
Todo el `.c` está dentro de `#if APP_RUN_SELFTESTS` (con el flag en 0 **no aporta código**).
`SelfTest_Run()` imprime PASS/FAIL por UART. Casos:
- **Kalman:** convergencia con entrada constante, atenuación de outlier, seguimiento de rampa,
  R grande (suave) vs R chico (responsivo).
- **PID:** solo-P exacto, saturación, sin derivative-kick, anti-windup vs sin AW, lazo simulado
  (planta de juguete `pos += k·u·dt`) convergiendo al setpoint.
- Imprime floats con formato `%d.%03d` (newlib-nano no soporta `%f`).

### 4.3 Wiring y configuración

#### `app_config.h` — **el único lugar para tunear**
Constantes y flags de todo el proyecto:
- **Tasks:** stacks (words) y prioridades (Sensor 5, Kalman 4, Motor 4, PID 3, Pot 1).
- **Sensor:** `SENSOR_MIN_CM/MAX_CM`, `SENSOR_ECHO_TIMEOUT_MS` (80 ms, < período de 100 ms).
- **Pote:** `POT_RANGE_CM`, `POT_PERIOD_MS` (200 ms).
- **Kalman:** `KALMAN_DT/Q/R`. **PID:** `PID_DT/KP/KI/KD`, `PID_OUT_MIN/MAX`, `SERVO_CENTER_DEG`.
- **Servo:** `SERVO_MIN_US/MAX_US/MIN_DEG/MAX_DEG` (calibración).
- **Flags:** `APP_RUN_SELFTESTS`, `APP_USE_SYNTHETIC_SENSOR`, `APP_LOG_ENABLED` (todos 0 en producción).

#### `app.h` / `app.c` — capa de *wiring*
Concentra las instancias, la IPC y los hooks de ISR. Es lo único que `main.c` necesita conocer.
- **Globales:** `g_sensor`, `g_servo`, `g_pot`; semáforos `SemTimer`, `SemSensor`; colas `QueuePos`,
  `QueuePosFil`, `QueueObjetivo`, `QueueAngulo`; `QueueSetPid`; `g_dbg_raw` (última muestra, para log).
- **`App_Init()`** (llamado desde `USER CODE 2` de `main.c`): crea semáforos y colas (con chequeo
  NULL→`Error_Handler`), crea el **queue set** y le agrega `QueuePosFil`+`QueueObjetivo`, `Pot_Init`,
  arranca **`HAL_TIM_Base_Start_IT(&htim4)`** (tick de 100 ms) y crea las **5 tasks**. Si
  `APP_RUN_SELFTESTS`, corre los tests antes.
- **`App_OnSensorComplete_FromISR(h)`** — hook del sensor: da `SemSensor` (`GiveFromISR` + `YIELD`).
- **`App_OnTimerTick_FromISR()`** — llamado desde la ISR de TIM4: da `SemTimer`.
- **`__io_putchar(ch)`** — retarget de `printf` a USART2.
- **`App_LogTrace(z, fil, sp, u, ang)`** — imprime la traza del lazo (formato sin `%f`).

### 4.4 Tasks (una por archivo, API nativa FreeRTOS)

| Archivo | Task | Prio | Espera / recibe | Produce |
|---|---|---|---|---|
| `task_sensor.c` | `SensorTask` | 5 | `SemTimer` (+`SemSensor`) | `QueuePos` |
| `task_kalman.c` | `KalmanTask` | 4 | `QueuePos` | `QueuePosFil` |
| `task_pid.c` | `PidTask` | 3 | QueueSet{`QueuePosFil`,`QueueObjetivo`} | `QueueAngulo` |
| `task_motor.c` | `MotorTask` | 4 | `QueueAngulo` | PWM servo + toggle LD2 |
| `task_pot.c` | `PotTask` | 1 | (temporizada 200 ms) → ADC | `QueueObjetivo` |

- **`SensorTask`**: espera el tick de 100 ms (`SemTimer`), dispara el HC-SR04, espera el echo
  (`SemSensor`, timeout 80 ms), lee la distancia y la publica en `QueuePos`. En
  `APP_USE_SYNTHETIC_SENSOR` genera una señal triangular + ruido en vez de usar el sensor.
- **`KalmanTask`**: recibe la distancia cruda, la filtra y publica la posición estimada. Arranca el
  filtro con la 1ª muestra (`Kalman_Reset`).
- **`PidTask`**: bloquea en el **queue set**; si llegó posición filtrada, calcula `u`, mapea a
  `ang = SERVO_CENTER_DEG + u` y publica en `QueueAngulo` (y loguea si `APP_LOG_ENABLED`); si llegó
  setpoint, lo actualiza. Lee con timeout 0 y solo actúa si vino dato (robustez ante *stale tokens*
  del set con `xQueueOverwrite`).
- **`MotorTask`**: recibe el ángulo, `Servo_SetAngle`, y hace toggle de LD2 (heartbeat).
- **`PotTask`**: cada 200 ms (`vTaskDelayUntil`) lee el pote y publica el setpoint. En modo sintético
  publica un setpoint fijo (centro) sin leer el ADC.

---

## 5. El *glue* en `Core/` (lo único nuestro fuera de `App/`)

Todo dentro de secciones `USER CODE` (para sobrevivir a CubeMX):

- **`Core/Src/main.c`**
  - `USER CODE Includes`: `#include "app.h"`.
  - `USER CODE 2`: `App_Init();`.
  - `USER CODE 4`: `HAL_TIM_IC_CaptureCallback(htim){ HC_SR04_HandleInterrupt(htim); }`.
  - `HAL_TIM_PeriodElapsedCallback`, `USER CODE Callback 1`: `if (htim->Instance == TIM4)
    App_OnTimerTick_FromISR();` (la rama de `TIM5 → HAL_IncTick()` queda intacta).
  - `USER CODE WHILE`: `vTaskStartScheduler();` — **arranque nativo (estilo BlinkyRTOS)** en vez de la
    capa CMSIS. ⚠️ CubeMX regenera `osKernelInitialize/MX_FREERTOS_Init/osKernelStart` fuera de
    `USER CODE`: tras cada *Generate Code* hay que **borrar esas 3 líneas** y dejar solo
    `vTaskStartScheduler()`.
- **`Core/Inc/FreeRTOSConfig.h`**: `#define configUSE_QUEUE_SETS 1` en `USER CODE Defines`.
- **`Core/Src/freertos.c`**: **intacto** (CubeMX). `MX_FREERTOS_Init` y `defaultTask` sin tocar; ya no
  se llaman (no ponemos código de app ahí).
- Resto de `Core/` (`tim.c`, `adc.c`, `usart.c`, `gpio.c`, `stm32f4xx_it.c`): generado por CubeMX. La
  ISR `TIM4_IRQHandler` (en `stm32f4xx_it.c`) llama `HAL_TIM_IRQHandler(&htim4)` → dispara nuestro
  `HAL_TIM_PeriodElapsedCallback`.

---

## 6. Flujo en tiempo real (un ciclo de 100 ms)

```
ISR TIM4 (cada 100 ms) ── da SemTimer ──► SensorTask (prio 5)
   SensorTask dispara TRIG ─► (rebote) ─► ISR Input Capture (TIM2) ── da SemSensor ──► SensorTask
   SensorTask calcula distancia ─► QueuePos
                                     │
                                     ▼
                             KalmanTask (prio 4) ─► QueuePosFil
                                                        │
   PotTask (prio 1, cada 200 ms) ─► QueueObjetivo ──┐   │
                                                    ▼   ▼
                                            PidTask (prio 3) [Queue Set]
                                                    │  calcula u, ang
                                                    ▼
                                             QueueAngulo
                                                    │
                                                    ▼
                                            MotorTask (prio 4) ─► Servo PWM (+ LD2)
```

- **Colas:** `float`, profundidad 1, `xQueueOverwrite` (siempre el dato más reciente) / `xQueueReceive`.
- **Queue set:** deja al PID bloquear en dos fuentes (posición filtrada y setpoint) a la vez con
  `xQueueSelectFromSet`.
- **Semáforos binarios:** `SemTimer` (tick determinístico de TIM4) y `SemSensor` (fin de medición IC).

---

## 7. Flags de compilación (`app_config.h`)

| Flag | Producción | Qué hace en 1 |
|---|---|---|
| `APP_RUN_SELFTESTS` | 0 | Corre `SelfTest_Run()` al arrancar (PASS/FAIL por UART) y no crea el lazo hasta terminar |
| `APP_USE_SYNTHETIC_SENSOR` | 0 | El sensor se reemplaza por una señal sintética; el pote por un setpoint fijo (valida la cadena sin hardware) |
| `APP_LOG_ENABLED` | 0 | Imprime la traza `z, fil, sp, u, ang` por UART (subir para el tuning; bajar en producción por la latencia del UART) |

---

## 8. Decisiones de diseño (por qué está hecho así)

- **Todo en `App/`** → robustez ante regeneración de CubeMX (la premisa central del proyecto).
- **API nativa de FreeRTOS** (no CMSIS v2) y `freertos.c` intacto → estilo del proyecto de referencia.
- **Colas depth-1 + `xQueueOverwrite`** → control en tiempo real: siempre importa el dato más nuevo,
  no encolar históricos.
- **Queue set** (en vez de mutex/flags) → el PID espera *dos* eventos con una sola primitiva bloqueante.
- **Módulos puros para Kalman/PID** → testeables (y compilables en host si algún día hay gcc).
- **Tests on-target detrás de flag** → validación sin ensuciar el binario de producción.
- **`%d.%03d` en vez de `%f`** → newlib-nano no imprime floats por defecto y no se puede tocar la
  config del linker.
- **Prioridades NVIC 5 en TIM2/TIM4** → requisito de FreeRTOS para `...FromISR()`.

---

## 9. Historial de construcción (git, rama `feature/arquitectura-app` → `master`)

1. `chore: baseline + commit driver servo` — punto de partida versionado.
2. `refactor: mover app a carpeta App/ y dejar glue minimo en main.c` — arquitectura `App/`.
3. `feat: CubeMX ADC1 (pote), TIM4 100ms, heap y queue sets` — periféricos del lazo.
4. `feat: modulos puros Kalman y PID + self-tests on-target (opt-in)`.
5. `feat: pipeline FreeRTOS (5 tasks, 4 colas, queue set, semaforos) + smoke-test sintetico`.
6. `feat: lazo cerrado real ball-and-beam + tuning inicial`.
7. `chore: ajuste de stacks, gating de logs y verificacion post-regeneracion`.
8. `fix: setpoint fijo en modo sensor sintetico (smoke test sin pote)`.
9. `merge: arquitectura ball-and-beam completa (Etapas 0-6, validada por smoke test)`.

**Estado:** software completo, compila, arranca y validado por el smoke test sintético. Pendiente solo
el montaje físico + tuning (ver `GUIA_CONTINUACION_HARDWARE.md`).
