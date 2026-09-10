# Sistema de Control PID en Tiempo Real para Balanceo

Control PID en tiempo real sobre **STM32F401RE (NUCLEO-F401RE)** con **FreeRTOS**. Un objeto
se desliza libremente sobre una barra articulada en uno de sus extremos; un servo eleva o baja
el extremo libre, y el sistema mantiene al objeto estable en la posición que el usuario elige
con un potenciómetro.

> Proyecto final de la materia **Sistemas de Tiempo Real** — FAMAF, Universidad Nacional de
> Córdoba.

## Cómo funciona

El sistema mide de forma periódica la posición del objeto con un sensor ultrasónico, filtra esa
lectura para quitarle ruido, y un controlador PID calcula cuánto inclinar la barra para llevar
al objeto hacia el punto deseado. Esa corrección se aplica al servo, que ajusta la inclinación,
cerrando el lazo de control.

En paralelo, el usuario define la posición objetivo girando un potenciómetro, que el sistema
lee y traduce a una posición sobre la barra. De esta manera el punto de equilibrio puede
cambiarse en cualquier momento sin detener el control.

Cuando el sensor deja de entregar mediciones confiables, el sistema se degrada de forma segura:
en lugar de sostener una corrección vieja, lleva la barra a su posición horizontal.

## Arquitectura

El proyecto corre sobre FreeRTOS, con una tarea dedicada a cada etapa del lazo (medición,
filtrado, control, actuación y lectura del setpoint), coordinadas entre sí mediante los
mecanismos de sincronización del sistema operativo. Este modelado concurrente permite cumplir
los tiempos de cada etapa y mantener el lazo estable.

El código propio está organizado en capas con responsabilidades bien separadas, lo que facilita
probar los algoritmos de forma aislada, reutilizarlos en otras plataformas y regenerar el
proyecto desde CubeMX sin pisar el trabajo hecho a mano:

- **`App/`** — configuración del sistema y punto de composición: crea e interconecta todas las
  piezas.
- **`ComponentDrivers/`** — drivers de los periféricos (sensor, servo y potenciómetro), cada uno
  con una interfaz simple y sin dependencia del sistema operativo.
- **`Control/`** — los algoritmos de control (filtro de Kalman y PID), escritos como módulos
  puros, sin depender del hardware.
- **`Tasks/`** — las tareas de FreeRTOS que conectan los drivers con los algoritmos de control.
- **`Core/`, `Drivers/`, `Middlewares/`** — arranque, capa HAL y kernel de FreeRTOS, generados
  por CubeMX y provistos por el fabricante y terceros.

## Hardware

- Placa de desarrollo **NUCLEO-F401RE** (STM32F401RE)
- Sensor ultrasónico **HC-SR04** (posición del objeto)
- Servomotor **MG90S** (inclinación de la barra)
- **Potenciómetro** lineal (posición deseada)

## Compilación

El proyecto se desarrolla sobre **STM32CubeIDE**: se importa como proyecto existente, se compila
y se flashea a la placa NUCLEO-F401RE por el ST-Link integrado.

## Ramas

- **`main`** — código de producción.
- **`debug`** — igual a `main`, con una traza completa del lazo por USART2 (puerto COM virtual
  del ST-Link) para observar y ajustar el comportamiento en vivo.

## Maqueta

El diseño y la construcción de la maqueta estuvieron a cargo de **Valentín Sabino** (Licenciatura
en Diseño Industrial, Universidad Nacional de Rosario): modelada en Autodesk Inventor e impresa
en 3D.

## Autor

**Martín Sferco**
