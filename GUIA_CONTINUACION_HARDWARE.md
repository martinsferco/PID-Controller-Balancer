# Guía para continuar con el hardware — Ball-and-Beam

Esta guía retoma el proyecto **cuando tengas el hardware físico** (sensor HC-SR04, servo MG90S,
potenciómetro y la barra montada). El **software ya está completo, compila y fue validado** con el
sensor sintético; lo único que falta es cablear, calibrar y tunear el lazo real.

> Para entender el proyecto desde cero (archivo por archivo, configuración por configuración) leé
> **`DOCUMENTACION_TECNICA.md`**.

---

## 0. Punto de partida

- Rama `master`, todo integrado y commiteado. Working tree limpio.
- Flags de producción en `App/Inc/app_config.h`: `APP_RUN_SELFTESTS=0`, `APP_USE_SYNTHETIC_SENSOR=0`,
  `APP_LOG_ENABLED=0`.
- Al abrir STM32CubeIDE: si estaba abierto de antes, **Refresh (F5)** el proyecto.

---

## 1. Cableado

### 1.1 HC-SR04 (sensor de distancia)

| HC-SR04 | Nucleo | Nota |
|---|---|---|
| VCC | 5V | alimentación |
| GND | GND | masa común |
| TRIG | D8 / **PA9** | salida GPIO (pulso de 10 µs) |
| ECHO | A0 / **PA0** | entra por **divisor de tensión** (ver abajo) |

**Divisor en ECHO (obligatorio):** el ECHO sale a 5 V y el pin es de 3.3 V. Divisor:
`ECHO --[1 kΩ]--+--> PA0` y `+--[2.2 kΩ]--> GND`. (PA0 es 5V-tolerant, pero el divisor es buena práctica.)

### 1.2 Servo MG90S

| Servo | Conexión | Nota |
|---|---|---|
| Señal (naranja) | D12 / **PA6** | PWM 50 Hz |
| V+ (rojo) | **5 V externos** | fuente aparte (cargador de pared), NO del USB |
| GND (marrón) | GND externo **+ GND Nucleo** | **masa común obligatoria** |

⚠️ El temblor errático del servo suele ser **eléctrico** (masa floja / alimentación pobre), no de
software. Usar cable de GND directo servo→pin GND de la Nucleo y estañar el +5 V.

### 1.3 Potenciómetro (setpoint)

| Pote | Nucleo | Nota |
|---|---|---|
| Extremo 1 | **3.3 V** | ⚠️ **3.3 V, NO 5 V** (entra directo al ADC, sin divisor) |
| Extremo 2 | GND | |
| Cursor (wiper) | A2 / **PA4** | `ADC1_IN4` |

⚠️ El pote a 5 V dañaría PA4 en modo analógico. **Siempre 3.3 V.**

---

## 2. Primer arranque del lazo real (con logs)

1. En `App/Inc/app_config.h`, poné **solo** el log en 1 para diagnosticar:
   ```c
   #define APP_USE_SYNTHETIC_SENSOR 0   /* sensor real */
   #define APP_LOG_ENABLED          1   /* traza por UART (temporal, para tunear) */
   ```
2. Build + flash. Abrí PuTTY en **COM3 @ 115200**.
3. Deberías ver la traza: `z=... fil=... sp=... u=... ang=...` cada 100 ms.
   - `z` = distancia real del sensor (cm). Movē la mano frente al sensor y verificá que cambia.
   - `sp` = setpoint del pote. Girá el pote y verificá que `sp` va de 0 a `POT_RANGE_CM`.
   - `fil` = distancia filtrada (Kalman). Debe seguir a `z` suavizada.
   - `u`, `ang` = acción de control y ángulo del servo.

**Si `z` no cambia** → revisar cableado TRIG/ECHO y el divisor. **Si `sp` no cambia** → revisar el
pote (3.3 V y cursor a PA4). **Si el servo no se mueve** → revisar alimentación externa y masa común.

---

## 3. Calibración de rangos (en `app_config.h`)

Ajustá estas constantes al montaje físico real:

```c
#define SENSOR_MIN_CM   2.0f    /* distancia mínima útil de la barra          */
#define SENSOR_MAX_CM   50.0f   /* distancia máxima útil (largo de la barra)  */
#define POT_RANGE_CM    50.0f   /* = largo útil (mapea el pote al mismo rango) */

/* Servo: medir contra los topes físicos para no forzarlo */
#define SERVO_MIN_US    1000u   /* pulso en el extremo mínimo                  */
#define SERVO_MAX_US    2000u   /* pulso en el extremo máximo                  */
#define SERVO_MIN_DEG   0.0f
#define SERVO_MAX_DEG   180.0f
#define SERVO_CENTER_DEG 90.0f  /* ángulo con la barra horizontal (neutro)    */

/* Rango de acción del PID alrededor del centro (grados) */
#define PID_OUT_MIN    (-20.0f)
#define PID_OUT_MAX    ( 20.0f)
```

**Procedimiento servo:** con `APP_LOG_ENABLED=1`, probá ángulos y observá el recorrido; ajustá
`SERVO_MIN_US/MAX_US` para que 0°–180° coincidan con los extremos mecánicos SIN golpear los topes.
Fijá `SERVO_CENTER_DEG` en el ángulo que deja la barra horizontal.

---

## 4. Tuning del control (método incremental)

Dejá `APP_LOG_ENABLED=1` y logueá `z, fil, sp, u, ang`. Podés capturar el UART a archivo y graficar
`fil` vs `sp` en la PC.

### 4.1 Kalman (filtro) — primero
- Empezá con `KALMAN_R = 0.09` (ruido del HC-SR04) y `KALMAN_Q` chico (ej. 0.01).
- Si `fil` va **muy lento** (mucho lag respecto de `z`): **subí Q**.
- Si pasa **mucho ruido** a `fil`: **bajá Q** (o subí R).
- Objetivo: `fil` suave pero que siga los cambios reales sin retardo grande.

### 4.2 PID (control) — después
Método clásico, una ganancia a la vez:
1. `KI=0`, `KD=0`. Subí **`KP`** hasta que el objeto responda al setpoint sin oscilar feo.
2. Agregá **`KD`** para amortiguar la oscilación / overshoot.
3. Agregá **`KI`** chico para eliminar el error estacionario (que llegue exacto al setpoint).
4. Vigilá la saturación: si `u` se queda pegado en ±20 mucho tiempo, el anti-windup ya lo maneja,
   pero puede indicar `KP` muy alto o rango de acción chico.

Reglas rápidas: oscilación permanente = `KP` alto o falta `KD`; deriva lenta al setpoint = falta `KI`.

---

## 5. Cierre del tuning (build de producción)

Cuando el objeto siga el pote y se mantenga estable:
```c
#define APP_LOG_ENABLED  0   /* apagar la latencia del UART en el lazo */
```
Build + flash. Ese es el binario final.

---

## 6. Verificación final (checklist del anteproyecto)

- [ ] Girando el pote, el objeto **se mueve al nuevo setpoint y se estabiliza** sin caerse.
- [ ] `fil` converge a `sp` con error chico y sin oscilación sostenida.
- [ ] Muestreo del sensor cada **100 ms** (verificable con toggle GPIO / osciloscopio en un pin).
- [ ] Pote leído cada **200 ms**.
- [ ] Las 5 tasks corriendo (Sensor 5, Kalman 4, Motor 4, PID 3, Pot 1).
- [ ] (Opcional) medir stacks con `uxTaskGetStackHighWaterMark` y ajustar `*_TASK_STACK`.

---

## 7. Prueba de fuego: robustez ante CubeMX (Etapa 6, paso 4)

Para demostrar que la arquitectura sobrevive a la regeneración:
1. Abrí `PIDController.ioc`, hacé **Generate Code** (sin cambiar nada).
2. Verificá que:
   - `App/` sigue en el build y compila.
   - El glue de `main.c` quedó intacto: `App_Init()` en USER CODE 2; ramas de callback
     (`HAL_TIM_IC_CaptureCallback` y `TIM4` en `HAL_TIM_PeriodElapsedCallback`) en su lugar.
   - `configUSE_QUEUE_SETS 1` sigue en `FreeRTOSConfig.h` (sección USER CODE).
   - **Arranque nativo:** CubeMX vuelve a generar `osKernelInitialize(); MX_FREERTOS_Init();
     osKernelStart();` (fuera de USER CODE). **Borrá esas 3 líneas** — el arranque correcto es
     `vTaskStartScheduler();` que quedó en `USER CODE BEGIN WHILE`. (Si no las borrás no rompe, pero
     arranca por la capa CMSIS y crea un `defaultTask` de más.)
   - El comportamiento no cambió.
3. Si `App/` se cayó del build tras regenerar → **New → Source Folder → App** (el include `../App/Inc`
   normalmente persiste).

---

## 8. Self-tests de los módulos (opcional, cuando quieras revalidar Kalman/PID)

```c
#define APP_RUN_SELFTESTS 1
```
Build + flash + UART: imprime PASS/FAIL de convergencia, outlier, rampa, anti-windup, lazo simulado,
etc. Volvé a `0` al terminar.

---

## Troubleshooting rápido

| Síntoma | Causa probable | Acción |
|---|---|---|
| `z` no cambia / siempre TIMEOUT | TRIG/ECHO mal, divisor mal | revisar cableado 1.1 |
| `z` con saltos raros | ruido, alimentación 5 V pobre | fuente estable, masa común |
| `sp` errático | pote mal cableado / a 5 V | pote a **3.3 V**, cursor a PA4 |
| servo tiembla | masa floja / alim. pobre | GND directo, +5 V firme |
| hard fault al arrancar | prioridad NVIC < 5 en una ISR que da semáforo | TIM2 y TIM4 en prio **5** |
| no linkea `xQueueCreateSet` | `configUSE_QUEUE_SETS` no está | verificar FreeRTOSConfig.h |
| lag/overshoot del filtro | tuning Kalman | subir `KALMAN_Q` |
