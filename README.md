# PID Controller — Ball and Beam

Sistema de control PID en tiempo real sobre STM32F401RE (NUCLEO-F401RE) + FreeRTOS. Un
objeto se desliza sobre una barra inclinada por un servo; el sistema lo mantiene estable en
un punto elegido con un potenciómetro.

## Cadena de control

```
HC-SR04 (mide distancia) -> Kalman (filtra) -> PID (calcula correccion) -> Servo MG90S (inclina la barra)
                                                    ^
                                                    | setpoint
                                            Potenciometro (ADC)
```

## Hardware

- STM32F401RE (NUCLEO-F401RE)
- Sensor ultrasónico HC-SR04
- Servo MG90S
- Potenciómetro (setpoint)

## Estructura

- `App/` — código de la aplicación (config, composition root)
- `ComponentDrivers/` — drivers de sensor, servo y potenciómetro (HAL puro, sin RTOS)
- `Control/` — Kalman y PID (módulos puros, sin hardware)
- `Tasks/` — tasks de FreeRTOS que conectan todo
- `Core/`, `Drivers/`, `Middlewares/` — generado por CubeMX/HAL/FreeRTOS

## Documentación

- [`DOCUMENTACION_TECNICA.md`](DOCUMENTACION_TECNICA.md) — cómo funciona cada parte, configuración de CubeMX
- [`Notas_Diseno.md`](Notas_Diseno.md) — por qué se eligió cada constante de `app_config.h`
- [`GUIA_CONTINUACION_HARDWARE.md`](GUIA_CONTINUACION_HARDWARE.md) — guía para retomar el trabajo en el hardware
