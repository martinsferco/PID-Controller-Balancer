# Rama `tests` — instrumentacion, self-tests y traza por UART

Esta rama conserva **todo el andamiaje de diagnostico** del proyecto ball-and-beam.
La rama `master` es la version final "limpia": tiene el lazo de control y la
configuracion de USART2 de CubeMX, pero **sin** self-tests, sin traza por UART y
sin sensor sintetico.

Usar esta rama cuando haya que depurar, tunear o validar sin hardware; volver a
`master` para la version de entrega.

## Que hay aca y no en master

| Elemento | Archivo | Para que sirve |
|---|---|---|
| `APP_RUN_SELFTESTS` | `App/Inc/app_config.h` | Corre `SelfTest_Run()` antes de crear las tasks |
| `APP_USE_SYNTHETIC_SENSOR` | `App/Inc/app_config.h` | Reemplaza el HC-SR04 por una señal triangular con ruido (valida la cadena sin hardware) |
| `APP_LOG_ENABLED` | `App/Inc/app_config.h` | Traza del lazo por UART desde `PidTask` |
| Self-tests on-target | `App/Src/selftest.c`, `App/Inc/selftest.h` | Casos de Kalman y PID con resultado `PASS`/`FAIL` por UART |
| `App_LogTrace()` + `print_f()` | `App/Src/app.c` | Impresion de floats sin `%f` (newlib-nano) |
| `__io_putchar()` | `App/Src/app.c` | Retarget de `printf` a USART2 (COM virtual del ST-Link) |
| `g_dbg_raw` | `App/Src/app.c` / `app.h` | Ultima muestra cruda, para que la traza del PID la pueda mostrar |
| Rama sintetica de `PotTask` | `App/Src/task_pot.c` | Setpoint fijo al centro cuando no hay pote cableado |

## Como usarla

1. Editar los flags en `App/Inc/app_config.h` (todos en `0` por defecto):
   - **Tuning del lazo con hardware real:** `APP_LOG_ENABLED 1`.
   - **Validar la cadena sin hardware externo (solo Nucleo por USB):**
     `APP_USE_SYNTHETIC_SENSOR 1` + `APP_LOG_ENABLED 1`.
   - **Verificar Kalman y PID:** `APP_RUN_SELFTESTS 1`.
2. Compilar y flashear.
3. Abrir el puerto serie (PuTTY, COM3) a **115200 8N1**.

Formato de la traza: `z=<cruda> fil=<Kalman> sp=<setpoint> u=<PID> ang=<servo>`,
una linea cada 100 ms.

> Con `APP_LOG_ENABLED 1` el `printf` corre dentro de `PidTask`; por eso su stack
> (`PID_TASK_STACK`) esta dimensionado mas grande que el de las otras tasks.
