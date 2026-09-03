# Notas de diseño — razonamiento de las constantes de `app_config.h`

Este documento conserva las derivaciones y el razonamiento detrás de los valores de
`App/Inc/app_config.h`. El header queda con una descripción breve por `#define`; el porqué
de cada número vive acá.

---

## Montaje físico

### Barra (`BEAM_LENGTH_CM = 30.0`)
Largo útil de la barra, medido **desde la cara del sensor**.

### Sensor HC-SR04 (`SENSOR_MIN_CM`, `SENSOR_MAX_CM`, `SENSOR_ECHO_TIMEOUT_MS`)
Todo el lazo (sensor, Kalman, PID, setpoint del pote) trabaja en la misma referencia: el
**borde** del carro que encara al sensor, que es justo lo que mide el HC-SR04 — así
`task_sensor.c` publica en `queue_pos` la distancia medida (real, no inventada salvo en el caso
`HC_SR04_INVALID` de abajo).

- `HC_SR04_HW_MIN_CM = 2.0` (vive en `hc_sr04.h`, no acá): zona muerta real del componente
  (datasheet) — por debajo de esto el eco no es físicamente posible, y el driver ya clasifica
  la lectura como `HC_SR04_INVALID`. Es del driver y no de la app porque otro módulo HC-SR04
  podría tener otro límite; `app_config.h` la referencia en vez de duplicarla.

  Se probó subirla a 3.0 (medida en banco donde por debajo de 3 cm el eco parecía poco
  confiable), pero eso mueve el límite de "físicamente posible" a "confiable" al lugar
  equivocado: hace que el *driver* rechace como inválidas lecturas de 2-3 cm que son reales,
  en vez de que sea la *app* la que decida no confiar en ellas. Revertido: la confiabilidad es
  política de `SENSOR_SAFETY_MARGIN_CM`, no de `HC_SR04_HW_MIN_CM`.
- `SENSOR_SAFETY_MARGIN_CM = 2.0`: headroom de la app sobre esa zona muerta, usado para calcular
  `SENSOR_MIN_CM` de abajo. **No es un piso que se le fuerce a toda lectura** — ver la política
  de `task_sensor.c` a continuación, es más sutil que eso.
- `SENSOR_MIN_CM = HC_SR04_HW_MIN_CM + SENSOR_SAFETY_MARGIN_CM = 4.0`: base del rango de
  setpoint del pote (ver más abajo) y, dividido a la mitad, de la proximidad asumida para el
  caso `HC_SR04_INVALID` de distancia gigantesca (ver la política de `task_sensor.c` a
  continuación) — no es un piso que se le fuerce directamente a una lectura.
- `SENSOR_MAX_CM = 21.0`: borde máximo, medido directamente con el carro apoyado contra su tope
  mecánico (choca contra el motor antes de llegar a `BEAM_LENGTH_CM`). Es una medida física
  directa, no una derivación. A diferencia de `SENSOR_MIN_CM`, este sí es un techo real: más
  allá no es físicamente alcanzable, así que **toda** lectura (`OK` o `INVALID`) se recorta acá.
- `SENSOR_ECHO_TIMEOUT_MS = 80`: guarda del `xSemaphoreTake` del echo en la task. Tiene que
  ser **menor que el período** del tick del sensor (100 ms) y **mayor que el timeout interno**
  del driver. Es un timeout de *scheduling*, distinto del timeout de hardware del driver.

#### Política de `task_sensor.c`: por qué `HC_SR04_OK` e `HC_SR04_INVALID` se tratan distinto

Antes esto era una sola regla de recorte (cualquier lectura por debajo de `SENSOR_MIN_CM` se
publicaba como `SENSOR_MIN_CM`, sin importar si el driver la había clasificado `OK` o
`INVALID`). Se descartó: con el carro genuinamente cerca del sensor pero sin llegar a la zona
muerta (el driver confirma un eco real, `HC_SR04_OK`, con `dist` entre 2 y 4 cm), clampear a un
valor **constante** le esconde al lazo de control el movimiento real del carro — Kalman ve la
misma medición en cada muestra, la velocidad estimada colapsa a cero, y el PID deja de ver un
error que refleje la posición real. Se vio en banco: con el carro pegado al sensor, el setpoint
del pote también en su mínimo (que antes coincidía exactamente con `SENSOR_MIN_CM`), el PID no
producía ninguna corrección — el error daba ≈0 aunque no hubiera forma de saber si el carro
estaba realmente ahí o mucho más cerca.

La regla ahora distingue las dos razones por las que puede no haber un número confiable:

- **`HC_SR04_OK`**: el driver confirma un eco real. Se publica la medición **tal cual**, sin
  importar si cae por debajo de `SENSOR_MIN_CM` — es información real, no ambigua. Solo se
  recorta el extremo lejano (a `SENSOR_MAX_CM`): un eco real más allá de eso no es físicamente
  alcanzable, el carro choca contra su tope mecánico antes.
- **`HC_SR04_INVALID`**: el driver NO confirma un eco real, y la razón puede ser una de dos, que
  `task_sensor.c` distingue mirando `dist` (el único chequeo que puede hacer, ya que
  `HC_SR04_INVALID` en sí no dice cuál de las dos fue):
  - `dist < HC_SR04_HW_MIN_CM`: zona ciega, no hay rebote — el carro está prácticamente tocando
    el sensor. Se publica `HC_SR04_HW_MIN_CM` directamente: es el piso físico real, no hace
    falta inventar otro número.
  - `dist > HC_SR04_HW_MAX_CM`: eco fantasma — con el carro pegado al sensor, el ECHO se queda
    en alto hasta el timeout interno del driver, que lo convierte en una distancia enorme. Esta
    lectura no tiene **ninguna** relación con la distancia real (a diferencia del caso
    anterior, donde el número al menos venía de un intento de medición cerca del límite), así
    que en vez del piso físico se asume una proximidad más conservadora: `SENSOR_MIN_CM / 2.0`
    (hoy 2.0, directo en `task_sensor.c` en vez de una constante propia en `app_config.h` —
    coincide numéricamente con `HC_SR04_HW_MIN_CM`, pero son cosas conceptualmente distintas).

  Como la barra está encerrada, ninguna de las dos puede ser un eco real del ambiente: la única
  explicación física en ambos casos es carro pegado al sensor. `HC_SR04_HW_MAX_CM` se bajó de
  450 a 400 cm (más cerca del rango real de datasheet del módulo) para que este caso se
  detecte más rápido.

Con esto, `SENSOR_EDGE_GRACE_CM` (la banda de gracia que existía para decidir, dentro del caso
`INVALID`, si una lectura estaba "cerca" o "lejos" del borde superior) quedó sin uso: dado que
`HC_SR04_HW_MAX_CM` es enorme comparado con la barra (30 cm), `HC_SR04_INVALID` en la práctica
solo ocurre por estar demasiado cerca o por el eco fantasma (nunca por estar "demasiado lejos"
en este montaje), así que esa rama de la banda de gracia era código muerto — nunca se
ejecutaba. Se eliminó la constante junto con el código.

Nunca se descarta una lectura (`OK` o `INVALID`, siempre se publica algo): si `SensorTask`
dejara de publicar, la cascada de timeouts (Kalman → PID → Motor) termina en el failsafe de
`MotorTask`, que nivela la barra y no reintenta solo.

### Potenciómetro / setpoint (`POTENTIOMETER_MIN_CM`, `POTENTIOMETER_MAX_CM`)
Rango de setpoint que barre el pote de tope a tope, en la misma referencia que publica
`task_sensor.c` (borde del carro que encara al sensor) — el mismo rango que ve el sensor,
directamente `SENSOR_MIN_CM`/`SENSOR_MAX_CM`, sin ningún offset:

- `POTENTIOMETER_MIN_CM = SENSOR_MIN_CM = 4.0`
- `POTENTIOMETER_MAX_CM = SENSOR_MAX_CM = 21.0`

Se probó un margen propio (`POTENTIOMETER_MARGIN_CM`, separando este rango de la ventana de
medición) para que el pote nunca pudiera pedir justo el valor donde `HC_SR04_INVALID` clampeaba
— visto en banco: con `POTENTIOMETER_MIN_CM` igual a `SENSOR_MIN_CM` (mismo valor de esta
sección) y el diseño anterior de `task_sensor.c` (donde **toda** lectura por debajo de
`SENSOR_MIN_CM`, `OK` o `INVALID`, se clampeaba ahí), el carro pegado al sensor y el pote en su
mínimo hacían coincidir exactamente la posición clampeada con el setpoint: el PID veía error ≈ 0
y no corregía nada, aunque no hubiera forma de saber si el carro estaba realmente ahí o más
cerca todavía. Con la política actual de `task_sensor.c` (`HC_SR04_INVALID` clampea a
`HC_SR04_HW_MIN_CM` o a `SENSOR_MIN_CM / 2`, ambos bien por debajo de `SENSOR_MIN_CM`) esa
coincidencia ya no se da, así que el margen aparte no aportaba nada más. Se sacó.

**No** es `0..BEAM_LENGTH_CM` porque ninguno de esos dos extremos es alcanzable: por abajo el
sensor no ve nada antes de `SENSOR_MIN_CM`, y por arriba el carro choca contra el motor. Pedir
un setpoint inalcanzable deja al proporcional inclinando para siempre contra un tope.

`task_pid.c` compara este setpoint directamente contra `est.pos` (la posición que entrega
Kalman filtrando `queue_pos`): como ambos están en la misma referencia de borde, no hace falta
convertir nada en el lazo de control.

`SETPOINT_DEFAULT_CM = (POTENTIOMETER_MIN_CM + POTENTIOMETER_MAX_CM) * 0.5 = 12.5`: setpoint
por defecto hasta que `PotTask` publique su primera lectura, punto medio del rango de borde
realmente alcanzable.

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
