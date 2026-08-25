# Servo: calibracion, limites y quien los aplica

Estado: **cerrado y verificado en hardware el 2026-08-23.**

## Convencion adoptada

Escala **absoluta** del eje, en grados:

| Angulo | Horn | Pulso |
|-------:|------|------:|
|   0    | un extremo del recorrido | 500 us |
|  10    | **piso permitido**  | 611 us |
|  45    | -                   | 1000 us |
|  90    | **HORIZONTAL** (barra nivelada) | 1500 us |
| 135    | -                   | 2000 us |
| 170    | **techo permitido** | 2389 us |
| 180    | el otro extremo     | 2500 us |

**Que extremo es "arriba" NO lo define esta tabla** (y una version anterior de este
documento lo afirmaba al reves, lo que costo una sesion de confusion). Verificado en
hardware el 2026-08-23 en este montaje: **al aumentar el angulo el horn va hacia ARRIBA**,
o sea 0 = abajo de todo y 180 = arriba de todo. Pero eso no es un hecho del MG90S: depende
de en que diente quedo calzado el horn y de que lado se lo mira, y cambia si algun dia se
lo desmonta.

Por eso **el codigo no usa esa etiqueta en ningun lado**, y no hay que "corregir" nada
cuando no coincide con lo que uno esperaba:

- La recta `deg -> us` es monotona: angulo mayor = pulso mas ancho, siempre. Es lo unico
  que promete el driver, y se verifica leyendo `Servo_GetPulseUs()`.
- La guarda `10..170` es **simetrica** alrededor de 90: da igual cual punta es cual.
- `SERVO_LEVEL_DEG = 90` se midio **empiricamente** con la barra puesta (modo `HOLD`), no
  se dedujo de la convencion.
- El unico lugar donde entra el sentido fisico es **`SERVO_DIR`** en `app_config.h`, y
  tambien se determina empiricamente: absorbe de una sola vez el sentido del horn y la
  geometria del acople. Si el lazo corrige para el lado que no debe, se cambia ese signo y
  no se toca nada mas.

Recta nominal del servo de hobby: `T[us] = 500 + (2000/180) * grados`, o sea 11.1111 us por
grado. Los 500 y 2500 us son los **extremos fisicos** del MG90S; el recorrido permitido se
recorta a 10..170 grados, dejando 10 grados de guarda a cada punta.

## Verificado en hardware (2026-08-23)

1. **La horizontal cae en 90 grados (1500 us).** Medido en modo `HOLD`: la recta nominal
   vale tal cual y el horn quedo montado en el diente justo, no hubo que corregir nada.
   (Una medicion previa a ojo sugeria ~82 grados; era error de lectura del goniometro.)
2. **Los dos extremos 10 y 170 se alcanzan sin zumbido.** Buscados saliendo del centro de a
   50 us, sin lanzar el horn contra un extremo desconocido.
3. **Aguanta extremo a extremo, infinito** (modo `ENDPOINTS`, 10 <-> 170 cada 1 s) sin
   resets ni ruidos: no hay brownout en la inversion ni golpe contra los topes.

Si algun dia hay que desarmar y remontar el horn, la referencia se re-mide en modo `HOLD`:
se ajusta **solo** `SERVO_LEVEL_DEG` mirando la barra, y `Servo_GetUsPerDeg()` dice cuanto
vale un grado.

## La interfaz habla en GRADOS

El driver expone un solo comando, y es en grados:

```c
Servo_SetAngle(&g_servo, 90.0f);     /* lo unico que mueve el servo */
```

Los microsegundos **no aparecen en ninguna llamada**. La recta us <-> grados es parte del
driver, igual que el nombre del archivo: un MG90S recorre sus 180 grados entre 500 y 2500
us, y eso es un hecho del componente, no una decision de la aplicacion. Vive en
`servo_mg90s.h` como `SERVO_MIN_US` / `SERVO_MAX_US` / `SERVO_US_PER_DEG`, y se toca solo si
se cambia de servo.

Lo unico que declara la aplicacion es el recorrido permitido, en grados:

```c
Servo_SetTravel(&g_servo, 10.0f, 170.0f);    /* la guarda, dentro de 0..180 */
```

Ninguna task puede mandar un pulso crudo, asi que no hay forma de saltearse el recorrido.
Para la traza hay lectura, que no es comando: `Servo_GetPulseUs()`.

## Quien aplica cada limite

```
app_config.h        DECLARA la politica     SERVO_MIN_DEG 10 / SERVO_MAX_DEG 170
      |
      v  (una sola vez, al arrancar)
task_motor.c        TRANSPORTA              Servo_SetTravel(10, 170)
      |
      v  (en cada llamada, para cualquier llamador)
servo_mg90s.c       APLICA 10..170          servo_limit_deg()        <- guarda de la app
servo_mg90s.c       APLICA 500..2500 us     SERVO_MIN_US/MAX_US      <- recta del componente
```

- **10..170 grados**: politica de la aplicacion. Vive en `app_config.h` como dato, la
  aplica el driver en cada `Servo_SetAngle()`. Esta en el driver y no en la task para que
  valga para *cualquier* llamador (el lazo, un modo de banco, un PID mal tuneado): si
  estuviera en `MotorTask`, cada llamador nuevo tendria que recordar reimplementarlo.
- **500..2500 us (0..180 grados)**: hecho del hardware. Constantes del driver, no
  configurables ni visibles en la API. `Servo_SetTravel()` **rechaza** una guarda que caiga
  fuera de 0..180, y `Servo_SetAngle()` recorta el pulso final como red ultima.
- **Tercera capa, mas blanda**: `PID_OUT_MIN/MAX` (+-8 grados) satura la salida del PID, asi
  que el lazo pide 82..98 grados y *nunca se acerca* a la guarda. Esa no es una pared: es
  que el lazo no camina hacia la pared.

Si `Servo_SetTravel()` rechazara la guarda, quedaria la de `Servo_Init()` -- que es 0..180,
o sea SIN guarda -- y el servo podria llegar a sus topes. Por eso `MotorTask` imprime su
status al arrancar: `0.0` es el acuse de que entro.

## Nota sobre el rango util del lazo

`PID_OUT_MIN/MAX` (+-8 grados) es cuanto se le permite inclinar a la barra, y 8 grados ya
son bastante pendiente en 30 cm. Los 10..170 son la envolvente mecanica del servo, no lo
que usa el control. Ampliar el rango del lazo se hace subiendo `PID_OUT_*`, no tocando la
guarda.
