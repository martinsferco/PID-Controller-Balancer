# Notas de diseño — razonamiento de las constantes de `app_config.h`

Este documento conserva las derivaciones y el razonamiento detrás de los valores de
`App/Inc/app_config.h`. El header queda con una descripción breve por `#define`; el porqué
de cada número vive acá.

---

## Montaje físico

### Barra (`BEAM_LENGTH_CM = 30.0`)
Largo útil de la barra, medido **desde la cara del sensor**.

### Sensor HC-SR04 (`SENSOR_MIN_CM`, `SENSOR_MAX_CM`, `SENSOR_ECHO_TIMEOUT_MS`)
- `SENSOR_MIN_CM = 3.0`: zona muerta real del sensor.
- `SENSOR_MAX_CM = 32.0`: barra de 30 cm + margen.
- `SENSOR_ECHO_TIMEOUT_MS = 80`: guarda del `xSemaphoreTake` del echo en la task. Tiene que
  ser **menor que el período** del tick del sensor (100 ms) y **mayor que el timeout interno**
  del driver. Es un timeout de *scheduling*, distinto del timeout de hardware del driver.

### Banda de gracia en los bordes (`SENSOR_EDGE_GRACE_CM = 3.0`)
Una medición que cae fuera de `[SENSOR_MIN_CM, SENSOR_MAX_CM]` pero a menos de esto del borde
se recorta y se usa igual, porque significa que la pelota está en la punta de la barra o pegada
al sensor. Más lejos se descarta: eso ya es un eco del ambiente, y usarlo haría inclinar la
barra por una pelota que no está ahí. Por eso la banda tiene que ser chica.

### Potenciómetro / setpoint (`POTENTIOMETER_MIN_CM = 5.0`, `POTENTIOMETER_MAX_CM = 25.0`)
Rango de setpoint que barre el pote de tope a tope. **No** es `0..BEAM_LENGTH_CM` porque ninguno
de esos dos extremos es alcanzable: por abajo el sensor no ve nada antes de `SENSOR_MIN_CM`, y
por arriba la pelota se cae de la punta. Pedir un setpoint inalcanzable deja al proporcional
inclinando para siempre contra un tope.

`SETPOINT_DEFAULT_CM = BEAM_LENGTH_CM * 0.5`: setpoint por defecto hasta que `PotTask` publique
su primera lectura.

### Servo MG90S (`SERVO_MIN_DEG`, `SERVO_MAX_DEG`, `SERVO_LEVEL_DEG`)
La recta grados ↔ µs vive en el driver (`servo_mg90s.h`): un MG90S recorre 0..180° entre 500 y
2500 µs, y eso es un hecho del componente. En `app_config.h` va solo lo que decide la aplicación,
todo en grados:
- `SERVO_MIN_DEG = 10.0` / `SERVO_MAX_DEG = 170.0`: recorrido permitido (guarda contra los topes).
- `SERVO_LEVEL_DEG = 90.0`: barra **horizontal**, referencia del lazo.
- `SERVO_CENTER_DEG = SERVO_LEVEL_DEG`: centro del movimiento = la horizontal. Es el 0 del PID;
  el lazo comanda `SERVO_CENTER_DEG + SERVO_DIR * u`.

### Sentido del actuador (`SERVO_DIR = -1.0`)
Absorbe de una sola vez el sentido de giro del horn y la geometría del acople, que es el único
lugar del proyecto donde entra el sentido físico; **se mide, no se deduce** de la escala del
servo. Criterio: con un ángulo por encima de `SERVO_LEVEL_DEG` la punta **del sensor** tiene que
**subir** (ese ángulo se pide solo cuando la pelota está más cerca que el setpoint, o sea cuando
tiene que alejarse). En este montaje pasa lo contrario, de ahí el `-1`.

---

## Lazo de control

### Filtro de Kalman (`KALMAN_DT`, `KALMAN_Q`, `KALMAN_R`)
Modelo de 2 estados (posición y velocidad). Q y R se dimensionan por el índice de seguimiento

    lambda = sqrt(Q) * dt^2 / sqrt(R)

Con `lambda << 1` el filtro suaviza mucho pero retrasa; con `lambda ~ 1` sigue la medición con
una o dos muestras de atraso. Los valores elegidos dan **lambda = 0.22**, que alcanza para seguir
los saltos reales de la pelota sin dejar pasar la cuantización del sensor.
- `KALMAN_DT = 0.1` (100 ms: el tick del sensor).
- `KALMAN_Q = 20.0` (densidad de ruido de proceso).
- `KALMAN_R = 0.04` (varianza de medición, sigma 0.2 cm).

### Controlador PID (`PID_KP`, `PID_KI`, `PID_KD`)
La planta es un **doble integrador**: la inclinación de la barra manda la *aceleración* de la
pelota, no su velocidad. Con la ganancia medida en hardware (K = 1.2 cm/s² por grado de horn) el
lazo cerrado queda descrito por

    wn   = sqrt(K * KP)
    zeta = KD * sqrt(K) / (2 * sqrt(KP))

- `PID_KP = 8.0` → wn = 3.1 rad/s, o sea `wn*dt = 0.31`: el techo razonable para un muestreo de
  10 Hz.
- `PID_KD = 3.6` → zeta = 0.70, amortiguamiento cerca del crítico. **KD no es opcional**: en un
  doble integrador, con KD = 0 el amortiguamiento es cero por construcción y no existe KP que no
  oscile.
- `PID_KI = 1.0` → solo para limpiar el error estacionario que deja la fricción. El margen de
  estabilidad lineal es `KI < KD*KP*K = 34`, así que el valor está muy por debajo.

Si el servo tiembla con la pelota quieta, el ajuste va en `KALMAN_Q` (suaviza la velocidad que
alimenta el término D), **no** en `PID_KD` (fija el amortiguamiento).

### Banda de integración (`PID_I_BAND = 4.0`)
En cm de error: el integrador acumula solo con `|error| <= PID_I_BAND`, y se descarga al cruzar
el setpoint. Lejos del objetivo el proporcional ya pide todo lo que el servo puede dar, así que
ahí el integrador no agrega autoridad: lo único que hace es cargarse durante el transitorio para
sobrepasar cuando la pelota por fin llega. Se elige apenas por encima del error estacionario que
se quiere limpiar.

### Saturación de la acción de control (`PID_OUT_MIN = -80.0`, `PID_OUT_MAX = 80.0`)
En grados de horn medidos desde la horizontal.

---

## Tasks y tiempos de FreeRTOS

- Stacks en **words** (no bytes). Prioridad mayor = más urgente.
- `POT_PERIOD_MS = 200`: período de lectura del pote. Es una mano girando, no hace falta más
  rápido.
- Timeout de "sin dato nuevo" de `MotorTask`, `KalmanTask` y `PidTask`: un define por task
  (`MOTOR_TASK_TIMEOUT_MS`, `KALMAN_TASK_TIMEOUT_MS`, `PID_TASK_TIMEOUT_MS`), hoy los tres en 500 ms
  para poder ajustarlos por separado el día que dejen de coincidir. El lazo publica cada 100 ms, así
  que esto son 5 muestras perdidas. Cada task reacciona distinto al vencerlo:
  - `MotorTask`: nivela la barra. Si el sensor deja de ver la pelota, dejar el servo clavado en la
    última inclinación es la peor posición posible para que vuelva.
  - `KalmanTask`: se marca como no inicializada, para forzar un `Kalman_Reset` con la próxima
    muestra real en vez de filtrarla asumiendo que pasó un `KALMAN_DT` desde la anterior.
  - `PidTask`: no hace nada especial — el integrador queda congelado a propósito (ver punto
    anterior de `MotorTask`), no se resetea.

---

## Coherencia entre el lazo y el servo (`_Static_assert`)

El lazo comanda `SERVO_CENTER_DEG + SERVO_DIR * u` con u en `[PID_OUT_MIN, PID_OUT_MAX]`, y
`Servo_SetAngle()` satura al recorrido permitido **en silencio**. Si el rango de acción se saliera
de la guarda nadie se enteraría: el PID quedaría integrando contra un actuador saturado que no
reporta nada. Mejor que no compile. Se chequean los dos extremos contra los dos límites, porque
con `SERVO_DIR` negativo el techo de u produce el piso del ángulo.

Es `_Static_assert` y no `#if` porque el preprocesador solo hace aritmética entera y estos
valores son float.
