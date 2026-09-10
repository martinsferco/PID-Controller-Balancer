# Sistema de Control PID en Tiempo Real para Balanceo

Control PID en tiempo real sobre **STM32F401RE (NUCLEO-F401RE)** con **FreeRTOS**. Un objeto
se desliza libremente sobre una barra articulada en uno de sus extremos; un servo eleva o baja
el extremo libre, y el sistema mantiene al objeto estable en una posición elegida por el
usuario mediante un potenciómetro.

La distancia del objeto se mide con un sensor ultrasónico, se filtra con un filtro de Kalman,
y un controlador PID calcula la inclinación necesaria para llevar el error a cero, todo de
forma periódica y concurrente bajo el planificador de FreeRTOS.

> Proyecto final de la materia **Sistemas de Tiempo Real** — FAMAF, Universidad Nacional de
> Córdoba.

## Cómo funciona

El lazo de control cerrado encadena cinco etapas, ejecutándose cada una en su propia tarea:

1. **Medición.** Cada 100 ms el sensor **HC-SR04** dispara un pulso y mide, por Input Capture,
   el ancho del eco para obtener la distancia al objeto.
2. **Filtrado.** El **filtro de Kalman** (2 estados: posición y velocidad) reduce el ruido del
   sensor y estima además la velocidad del objeto.
3. **Control.** El **PID** compara la posición filtrada contra el *setpoint* y calcula una
   señal de corrección. Usa la velocidad estimada por el Kalman como término derivativo
   (derivada sobre la medición, sin *derivative kick* ante cambios de setpoint).
4. **Actuación.** La corrección se convierte en un ángulo y se aplica al **servo MG90S** vía
   una señal PWM de 50 Hz, que inclina la barra.
5. **Referencia.** En paralelo, el **potenciómetro** se lee por ADC y define el *setpoint*
   (posición deseada, en cm) que el usuario puede ajustar en cualquier momento.

### Entradas y salidas

| Señal | Tipo | Rol |
|-------|------|-----|
| Potenciómetro | Entrada analógica (ADC) | Posición deseada (*setpoint*), mapeada a cm |
| HC-SR04 | Entrada por Input Capture | Posición actual del objeto (realimentación) |
| Servo MG90S | Salida PWM | Inclinación de la barra (actuador) |

## Arquitectura de software

El código propio está separado en capas con responsabilidades bien delimitadas, de modo que el
proyecto pueda regenerarse desde CubeMX sin pisar el trabajo propio, y que quede claro qué es
específico del microcontrolador y qué es reutilizable:

- **`App/`** — punto de composición (*composition root*). `app_config.h` centraliza **todas**
  las constantes del sistema (parámetros físicos, ganancias del PID y del Kalman, prioridades
  y tamaños de pila). `app.c` instancia los drivers, crea colas, semáforos y tareas, y arma el
  cableado de las interrupciones.
- **`ComponentDrivers/`** — drivers de hardware sobre la capa HAL, **sin dependencia de
  FreeRTOS**: HC-SR04 (no bloqueante, por Input Capture), servo MG90S (PWM) y potenciómetro
  (ADC por *polling*). Cada uno expone una interfaz simple (ángulo en grados, valor
  normalizado, distancia en cm).
- **`Control/`** — algoritmos puros, **sin HAL ni FreeRTOS**: filtro de Kalman, controlador
  PID y una utilidad de mapeo lineal. Al no depender del hardware, se pueden probar de forma
  aislada.
- **`Tasks/`** — una tarea de FreeRTOS por etapa del lazo, que conecta los drivers de
  `ComponentDrivers/` con los algoritmos de `Control/` a través de la IPC definida en `App/`.

### Tareas de FreeRTOS

| Tarea | Prioridad | Disparo | Responsabilidad |
|-------|:---------:|---------|-----------------|
| `SensorTask` | 5 | Semáforo de timer (100 ms) | Dispara el HC-SR04, espera el eco y publica la distancia cruda |
| `KalmanTask` | 4 | `QueuePos` | Filtra la posición y estima la velocidad |
| `MotorTask`  | 4 | `QueueAngulo` | Aplica el ángulo al servo; nivela la barra si no llega dato (*failsafe*) |
| `PidTask`    | 3 | `QueuePosFil` | Calcula la señal de control contra el *setpoint* vigente |
| `PotTask`    | 1 | Periódica, 200 ms (`vTaskDelayUntil`) | Lee el potenciómetro y actualiza el *setpoint* |

Las prioridades siguen el impacto de cada tarea sobre la estabilidad del lazo: `SensorTask` es
el punto de entrada de toda la cadena, por lo que un retraso en ella se propaga al resto;
`PotTask` es interacción de usuario y puede esperar sin afectar el control.

### Comunicación entre tareas (IPC)

- **Colas de profundidad 1** (con *overwrite*, para trabajar siempre con el dato más reciente):
  - `QueuePos` — posición cruda: `SensorTask` → `KalmanTask`
  - `QueuePosFil` — posición **y** velocidad filtradas: `KalmanTask` → `PidTask`
  - `QueueAngulo` — ángulo de corrección: `PidTask` → `MotorTask`
- **Setpoint sin cola** — `PotTask` no necesita despertar al PID por sí solo, así que el
  *setpoint* viaja como una variable `float` compartida a la que `PidTask` accede por puntero
  (acceso atómico de 32 bits en Cortex-M4).
- **Semáforos binarios** — `SemTimer` (lo libera la ISR del timer de 100 ms para despertar a
  `SensorTask`) y `SemSensor` (lo libera la ISR de Input Capture cuando la medición está lista).

Toda la memoria (tareas, colas y semáforos) se reserva de forma **estática** con las variantes
`...Static` de FreeRTOS: el sistema no reserva memoria dinámica en tiempo de ejecución.

### Interrupciones

- **TIM4 (update, 100 ms)** — genera el tick que inicia cada ciclo de medición.
- **TIM2 (Input Capture)** — captura los flancos de subida y bajada del pin Echo del HC-SR04
  para medir el ancho del pulso; al completar la medición, notifica a `SensorTask`.

### Comportamiento seguro (*failsafe*)

Si el sensor deja de entregar mediciones válidas (eco fuera de rango, zona ciega o *timeout*),
la cadena se degrada de forma controlada: el Kalman se reinicia ante una interrupción
prolongada del flujo de datos y `MotorTask` lleva la barra a la posición horizontal en lugar de
sostener una corrección vieja.

## Parámetros principales

Todos configurables desde `App/Inc/app_config.h`:

| Parámetro | Valor | Descripción |
|-----------|:-----:|-------------|
| Período de muestreo | 100 ms | Tick del sensor y paso del Kalman/PID |
| PID | Kp = 8.0, Ki = 1.0, Kd = 3.6 | Ganancias del controlador |
| PID (salida) | ±80° | Saturación de la corrección, con anti-windup |
| Kalman | Q = 20.0, R = 0.04 | Ruido de proceso y varianza de medición |
| Servo | 10°–170°, 90° nivelado | Recorrido permitido y posición horizontal |
| Rango útil | ~4–20 cm | Posiciones alcanzables sobre la barra |

Una guarda verificada en tiempo de compilación (`_Static_assert`) garantiza que el rango de
salida del PID nunca comande al servo fuera de su recorrido permitido.

## Hardware

- **STM32F401RE (NUCLEO-F401RE)** — reloj interno HSI, con los timers configurados a 1 µs/tick.
- **Sensor ultrasónico HC-SR04** — TIM2 en Input Capture (Echo) + GPIO (Trig).
- **Servo MG90S** — TIM3 en PWM a 50 Hz.
- **Potenciómetro lineal** — ADC1, 12 bits, por *polling*.

## Estructura del proyecto

```
App/               Configuración y composition root (app_config.h, app.c)
ComponentDrivers/  Drivers de HC-SR04, servo y potenciómetro (HAL, sin RTOS)
Control/           Kalman, PID y mapeo lineal (módulos puros)
Tasks/             Tareas de FreeRTOS que conectan todo
Core/              Arranque e inicialización generados por CubeMX
Drivers/           Capa HAL de STM32 y CMSIS (fabricante)
Middlewares/       Kernel de FreeRTOS (terceros)
```

## Compilación

El proyecto se desarrolla sobre **STM32CubeIDE**. Importar la carpeta como proyecto existente,
compilar y flashear a la placa NUCLEO-F401RE por el ST-Link integrado.

## Ramas

- **`main`** — código de producción.
- **`debug`** — igual a `main` más una traza completa del lazo por **USART2** (115200 8N1, sobre
  el puerto COM virtual del ST-Link), útil para observar y ajustar los valores del lazo en vivo.

## Maqueta

El diseño y la construcción de la maqueta estuvieron a cargo de **Valentín Sabino** (Licenciatura
en Diseño Industrial, Universidad Nacional de Rosario). Se modeló en Autodesk Inventor y se
imprimió en 3D (FDM, PETG); usa un riel único con un carro de dos ruedas sobre rodamientos y un
contrapeso que facilita el arranque del movimiento.

## Autor

**Martín Sferco**
