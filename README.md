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

## Ramas

- `main` — código de producción
- `debug` — igual a `main`, con traza de todo el lazo por USART2 (115200 8N1, COM del ST-Link)
