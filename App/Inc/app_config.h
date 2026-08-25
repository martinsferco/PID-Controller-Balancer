/**
  ******************************************************************************
  * @file    app_config.h
  * @brief   Constantes y flags de configuracion de la aplicacion ball-and-beam.
  *          Un unico lugar para tunear stacks, prioridades, rango del sensor y
  *          periodos.
  ******************************************************************************
  */

#ifndef APP_CONFIG_H
#define APP_CONFIG_H

/* ========================= DEBUG / BANCO ================================== *
 * APP_LOG_ENABLED: traza por USART2 (COM3 @115200 8N1). Ortogonal al modo.    *
 *                                                                            *
 * APP_MOTOR_MODE: QUE hace MotorTask. Un unico selector, asi nunca hay dos    *
 * modos compitiendo. El modo activo se imprime al arrancar.                  *
 *                                                                            *
 *   LOOP      lazo de control normal (entrega)                                *
 *   SWEEP     +-APP_SWEEP_DEG alrededor del centro, APP_SWEEP_CYCLES veces    *
 *   ENDPOINTS LO <-> HI infinito (el estimulo mas grande)                     *
 *   STEP      escalera LO -> HI -> LO en saltos: CALIBRAR                     *
 *   HOLD      clavado en APP_HOLD_DEG: para montar el horn                    *
 *                                                                            *
 * En todo modo != LOOP el lazo de control NO arranca (sensor y PID corren,    *
 * pero sus angulos se descartan). Desacoplar el horn de la barra antes.       */
#define APP_LOG_ENABLED         1

#define APP_MOTOR_MODE_LOOP      0
#define APP_MOTOR_MODE_SWEEP     1
#define APP_MOTOR_MODE_ENDPOINTS 2
#define APP_MOTOR_MODE_STEP      3
#define APP_MOTOR_MODE_HOLD      4

#define APP_MOTOR_MODE          APP_MOTOR_MODE_LOOP   /* BANCO DEL SENSOR: ve la pelota? */

/* APP_LOG_LOOP: traza periodica del LAZO (linea z/fil/sp/u/ang de PidTask y las
 * quejas por medicion de SensorTask). Se apaga sola en los modos de banco: ahi
 * SensorTask y PidTask siguen corriendo, pero sus angulos se descartan, asi que
 * su traza a 10 Hz solo tapa la del modo (y parece "salida vieja"). Los mensajes
 * de arranque y los del propio modo siguen saliendo con APP_LOG_ENABLED. */
#if (APP_LOG_ENABLED == 1) && (APP_MOTOR_MODE == APP_MOTOR_MODE_LOOP)
  #define APP_LOG_LOOP          1
#else
  #define APP_LOG_LOOP          0
#endif

/* --- SWEEP --- */
#define APP_SWEEP_DEG           8.0f    /* amplitud (+-, grados reales)       */
#define APP_SWEEP_CYCLES        3u      /* idas y vueltas; 0 = infinito       */
#define APP_SWEEP_STEP_MS       600u

/* --- ENDPOINTS --- */
#define APP_ENDPOINT_LO_DEG     SERVO_MIN_DEG
#define APP_ENDPOINT_HI_DEG     SERVO_MAX_DEG
#define APP_ENDPOINT_STEP_MS    1000u

/* --- STEP (calibracion) --------------------------------------------------- *
 * Recorre LO -> HI -> LO en saltos de APP_STEP_DEG, esperando APP_STEP_MS en  *
 * cada uno e imprimiendo el angulo y el pulso real. Mirando la barra anotas:  *
 *   1) en que angulo queda HORIZONTAL  -> va a SERVO_LEVEL_DEG               *
 *   2) en que angulos toca los topes   -> acotan SERVO_MIN_DEG/MAX_DEG       *
 * La bajada sirve para ver juego mecanico: si la barra no repite las mismas   *
 * posiciones subiendo y bajando, hay backlash en el acople.                  */
/* Banda acotada y ARRANCANDO EN LA HORIZONTAL a proposito: la barra esta puesta
 * y nunca se inclino mas de 8 grados, asi que no se la manda de golpe a un
 * extremo. Sube de a 5 grados desde 90, y si llega arriba sin que la pelota
 * ruede, se sube APP_STEP_HI_DEG y se repite. */
/* Banda CENTRADA en la zona donde la traza del lazo mostro que la pelota queda
 * en reposo (ang 55..68), mas margen para los dos lados. 3 s por escalon para
 * que la pelota llegue a DETENERSE antes de que se imprima su posicion. */
#define APP_STEP_LO_DEG         55.0f
#define APP_STEP_HI_DEG         120.0f
#define APP_STEP_DEG            5.0f    /* grados por salto                   */
#define APP_STEP_MS             3000u   /* que la pelota se detenga           */

/* --- HOLD --- */
#define APP_HOLD_DEG            SERVO_LEVEL_DEG  /* barra nivelada            */

/* --- Tasks (prioridades del anteproyecto) --------------------------------- *
 * Stack en WORDS (no bytes). Prioridad mayor = mas urgente.                  *
 * Sensor 5, Kalman 4, Motor 4, PID 3, Pot 1.                                 *
 * Con APP_LOG_ENABLED las tasks que hacen printf necesitan mas stack         *
 * (newlib-nano pide ~300 bytes); en produccion vuelven a los valores chicos.  */
#if (APP_LOG_ENABLED == 1) || (APP_MOTOR_MODE != APP_MOTOR_MODE_LOOP)
  #define SENSOR_TASK_STACK     384u
  #define MOTOR_TASK_STACK      384u
#else
  #define SENSOR_TASK_STACK     256u
  #define MOTOR_TASK_STACK      192u
#endif
#define KALMAN_TASK_STACK       256u
#define PID_TASK_STACK          384u
#define POT_TASK_STACK          192u

#define SENSOR_TASK_PRIO        5u
#define KALMAN_TASK_PRIO        4u
#define MOTOR_TASK_PRIO         4u
#define PID_TASK_PRIO           3u
#define POT_TASK_PRIO           1u

/* --- Barra (montaje fisico) ----------------------------------------------- *
 * Largo util de la barra medido DESDE LA CARA DEL SENSOR. Si el sensor esta   *
 * montado unos cm antes del inicio de la barra, sumarlos aca.                 */
#define BEAM_LENGTH_CM          30.0f

/* --- Sensor HC-SR04 ------------------------------------------------------- */
#define SENSOR_MIN_CM           3.0f    /* zona muerta real (2 cm es optimista) */
#define SENSOR_MAX_CM           32.0f   /* barra 30 cm + margen; mas lejos se descarta */
#define SENSOR_ECHO_TIMEOUT_MS  80u     /* guarda < periodo (100 ms), > timeout interno */

/* Banda de gracia en los bordes de la ventana util. Una medicion que cae fuera
 * de [SENSOR_MIN_CM, SENSOR_MAX_CM] pero DENTRO de esta banda se recorta al
 * borde y se USA en vez de descartarse: significa que la pelota esta en la
 * punta de la barra, o pegada al sensor (su zona muerta), y eso es informacion
 * real que el lazo necesita. Perder esa muestra dejaria al PID sin dato nuevo
 * -- y al servo congelado en la ultima inclinacion -- justo cuando mas
 * autoridad hace falta.
 *
 * Tiene que ser CHICA a proposito. Si fuera generosa, un eco que volvio de la
 * pared (150, 280 cm) entraria en la banda, se recortaria al borde lejano y el
 * PID volcaria la barra por una pelota que no esta ahi. Fuera de la banda la
 * medicion se descarta, que es lo correcto: no hubo eco de la pelota.
 * Lo aplica SensorTask. */
#define SENSOR_EDGE_GRACE_CM    3.0f

/* --- Potenciometro (setpoint) --------------------------------------------- *
 * Rango de setpoint que barre el pote de tope a tope. NO es 0..BEAM_LENGTH_CM
 * a proposito: ninguno de esos dos extremos es un setpoint ALCANZABLE.
 *   - por abajo, el HC-SR04 no ve nada mas cerca de SENSOR_MIN_CM (3 cm), asi
 *     que un setpoint de 0 pide una posicion que el lazo no puede ni medir;
 *   - por arriba, 30 cm es la punta misma de la barra: la pelota se cae antes
 *     de llegar.
 * Pedir un setpoint inalcanzable no es inofensivo: el error nunca baja, el
 * proporcional queda pidiendo inclinacion para siempre y la barra se queda
 * volcada contra un tope. Se deja un margen de 2 cm sobre la zona muerta del
 * sensor y de 5 cm antes de la punta, asi TODA la vuelta del pote pide algo
 * que la pelota puede efectivamente hacer. */
#define POTENTIOMETER_MIN_CM     5.0f   /* cm con el pote a fondo hacia un lado */
#define POTENTIOMETER_MAX_CM    25.0f   /* cm con el pote a fondo hacia el otro */
#define POT_PERIOD_MS           200u    /* lectura del setpoint cada 200 ms    */

/* --- Failsafe del actuador ------------------------------------------------- *
 * Si PidTask deja de publicar angulos, el sensor dejo de ver la pelota (se fue
 * de la barra, o el eco vuelve del ambiente). Sin failsafe MotorTask se queda
 * bloqueado para siempre y el servo clavado en la ULTIMA inclinacion -- que con
 * +-40 grados de rango puede ser una barra bien volcada, la peor posicion
 * posible para que la pelota vuelva. Pasado este tiempo sin angulo nuevo, se
 * nivela la barra: es la unica accion razonable a ciegas.
 * El lazo publica cada 100 ms, asi que esto son 5 muestras perdidas. */
#define APP_MOTOR_TIMEOUT_MS    500u

/* --- Setpoint fijo (banco, sin pote) -------------------------------------- *
 * Con el pote DESCONECTADO, PA4 queda flotando y el ADC devuelve basura: el   *
 * setpoint bailaria por todo el rango. Con el flag en 1, PotTask NO lee el    *
 * ADC y publica SETPOINT_FIXED_CM (las 5 tasks siguen existiendo igual).      *
 * En 0 (normal, con el pote cableado a 3.3 V / PA4) el setpoint sale del ADC. *
 * Sirve tambien para aislar: si el lazo se porta raro, poner esto en 1 saca   *
 * al pote de la ecuacion sin desarmar nada.                                   */
#define APP_USE_FIXED_SETPOINT  0
#define SETPOINT_FIXED_CM       (BEAM_LENGTH_CM * 0.5f)  /* mitad de la barra  */

/* --- Filtro de Kalman ----------------------------------------------------- *
 * Q y R se dimensionan por el INDICE DE SEGUIMIENTO lambda = sqrt(Q)*dt^2/sqrt(R),
 * que es lo que decide cuanto retrasa el filtro:
 *   lambda << 1  -> el filtro no le cree a la medicion: suave pero LENTO
 *   lambda ~ 1   -> sigue la medicion con ~1-2 muestras de retraso
 *
 * Los valores viejos (Q 0.05, R 0.09) daban lambda = 0.0075, dos ordenes de
 * magnitud abajo. Medido en la traza del 2026-08-23: `fil` tardaba SIETE
 * muestras (0.7 s) en alcanzar un escalon de `z`. En un lazo de periodo 0.1 s
 * eso es fatal: el PID controlaba sobre una posicion de hace 0.7 s, la derivada
 * llegaba tarde y la pelota cruzaba media barra antes de que el filtro se
 * enterara (sobrepasos hasta z=24 con u saturado en -40).
 *
 * R: el HC-SR04 de este montaje es MUCHO mas limpio que los 0.3 cm asumidos.
 * Con la pelota quieta, z alterna 10.276 / 10.293 = 0.017 cm, que es 1 us de
 * eco (la cuantizacion del timer, 58 us/cm). R = 0.01 -> sigma = 0.1 cm, que
 * sigue siendo conservador contra eso.
 * Q: sqrt(Q)*dt = 1 cm/s de cambio de velocidad por muestra, del orden de lo
 * que la pelota hace de verdad con la barra inclinada. Da lambda = 1.0. */
#define KALMAN_DT               0.1f    /* 100 ms (tick del sensor)            */
#define KALMAN_Q                20.0f   /* lambda = 0.22: 1.0 pasaba los saltos de 1 cm */
#define KALMAN_R                0.04f   /* sigma 0.2 cm: hay saltos de 1 cm reales */

/* --- Controlador PID ------------------------------------------------------ *
 * Dimensionado con el UMBRAL DE RODADURA medido en hardware (2026-08-23): la
 * pelota rompe a rodar con 100-105 grados de horn, o sea ~12 grados desde el
 * centro. Como una esfera rueda con 1-2 grados de inclinacion de barra, el
 * vinculo atenua ~10x -> ganancia de planta K ~ 1.2 cm/s2 por grado de HORN
 * (no por grado de barra: el PID comanda horn).
 *
 * De ahi: wn = sqrt(K*KP) y zeta = KD*sqrt(K)/(2*sqrt(KP)).
 *   KP 3.0 -> wn = 1.9 rad/s, y wn*dt = 0.19 (sano para muestreo a 10 Hz).
 *   KD 2.0 -> zeta = 0.64, algo subamortiguado a proposito: un poco de
 *             sobrepaso en la primera corrida dice mas que una respuesta muerta.
 * KD es grande comparado con KP justamente porque K es chica.
 *
 * KD NO ES OPCIONAL: el ball-and-beam es un doble integrador (la inclinacion
 * manda la ACELERACION), asi que con KD = 0 el amortiguamiento es cero por
 * construccion y no existe KP que no oscile.
 *
 * KI queda en 0 en la primera corrida. La zona muerta de ~12 grados deja un
 * error estacionario de ~4 cm (por debajo de eso la pelota no se despega), y
 * KI parece la cura obvia -- pero con roce seco lo tipico es que genere ciclo
 * limite: integra hasta despegarla, se pasa, y repite. Se prueba despues, y
 * chico (0.05-0.1), mirando si el error baja sin que aparezca oscilacion nueva. */
/* SEGUNDA ITERACION (traza del 2026-08-23). La pelota se clavaba en z=10.28 con
 * u=14.18: error 4.72 cm, o sea exactamente donde el proporcional iguala el
 * umbral de arranque de 14 grados. De ahi la relacion que manda:
 *
 *     zona muerta [cm] = umbral de arranque [grados] / KP
 *
 * Con KP 3.0 son 4.7 cm de error permanente. KP 6.0 la baja a 2.3 cm, y es
 * mucho mas efectivo que KI para esto. Techo de KP: wn = sqrt(K*KP) = 2.7 rad/s
 * -> wn*dt = 0.27, todavia debajo de 0.3 (con KP 9 se pasa y el muestreo a
 * 10 Hz deja de alcanzar).
 * KD 3.5 mantiene zeta ~ 0.8 con el KP nuevo (zeta = 0.224*KD).
 * KI 0.5 chico, solo para limpiar el resto: con roce seco el integrador tiende
 * a generar ciclo limite (integra hasta despegarla, se pasa, repite). Si
 * aparece cazeria alrededor del setpoint, bajarlo a 0.2 o a 0. */
/* TERCERA ITERACION (2026-08-24): KP 8, KI 0, KD 0. El objetivo de esta corrida
 * NO es que ande bien: es MEDIR si la relacion zona_muerta = umbral/KP vale.
 * Prediccion falsable: con umbral 14 grados y KP 8 la pelota tiene que quedarse
 * clavada a ~1.75 cm del setpoint (con KP 4 eran 3.5 cm). Por eso KI vuelve a 0:
 * con el integrador acumulando el error clavado no se queda quieto en ningun
 * valor y no hay nada que comparar. KP 8 pasa apenas el techo de wn*dt (0.31 vs
 * 0.30), asi que es tambien el ultimo KP que se puede pedir SIN amortiguamiento.
 *   - se clava a ~1.75 cm sin oscilar -> la ley vale: seguir subiendo KP o
 *     agregar feedforward de rozamiento.
 *   - se clava mucho mas lejos -> el umbral de 14 grados ya no vale, el problema
 *     es mecanico (relacion de palanca / roce), no de ganancias.
 *   - empieza a oscilar o sobrepasar -> llegamos al limite sin KD, y hay que
 *     arreglar la derivada (velocidad del Kalman) antes de seguir con KP.
 * Arrancar la corrida con la pelota CERCA del setpoint: con KP 8 y la pelota en
 * una punta, u satura en 80 y se mide un transitorio, no la zona muerta. */
/* RESULTADO DE LA TERCERA ITERACION (traza del 2026-08-24) -----------------
 * Salio el desenlace 3, y de paso se valido el modelo:
 *
 *  - K MEDIDO = 1.2 cm/s2 por grado de horn, de la fase de frenado de la traza
 *    (la pelota pasa de 24 a 6.4 cm/s en 0.6 s con u promedio ~ -25 grados).
 *    Justo el valor que este bloque venia asumiendo.
 *  - OSCILACION de +-9 cm alrededor del setpoint, con u saturado en 80 en cada
 *    extremo y periodo ~3.3 s (mas largo que los 2.0 s de wn=sqrt(K*KP) porque
 *    media excursion pasa en saturacion). Decae, pero por la friccion seca, no
 *    por el controlador: cuatro cruces de media barra antes de asentarse.
 *  - LA ZONA MUERTA NO ES UNIFORME. Dos reposos distintos en la misma corrida:
 *    en z=13.397 la sostiene u=12.8 (que SI cumple umbral/KP = 14/8 = 1.75 cm),
 *    y en z=10.638 aguanta u=34.9 sin moverse. O sea que el umbral depende de
 *    DONDE esta la pelota: hay hondonada local (o el sensor engancho un eco
 *    fijo). Eso no lo arregla ninguna ganancia -- se verifica con el modo HOLD.
 *
 * CUARTA ITERACION: KP 8, KI 0, KD 3.6. Con K=1.2 y zeta = 0.194*KD, el 3.6 da
 * zeta ~ 0.70. Es casi el mismo KD 3.5 que se habia descartado antes: el valor
 * estaba bien, el ESTIMADOR estaba mal. La diferencia finita sobre la posicion
 * convertia la cuantizacion real del sensor (escalones de 0.345 cm, no de
 * 0.017) en 3.5 cm/s de ruido = 13 grados de temblor; la velocidad del Kalman
 * atenua esa misma innovacion unas 6 veces (beta/dt = 1.6 con lambda = 0.22),
 * o sea ~2 grados. Ver PID_ComputeRate() en pid.h.
 *
 * Si el servo tiembla con la pelota quieta, el ajuste va en KALMAN_Q (bajarlo
 * suaviza la velocidad), NO en PID_KD: KD fija el amortiguamiento del lazo, Q
 * fija cuanto ruido pasa. Mirar la columna `vel` de la traza. */
#define PID_DT                  0.1f
#define PID_KP                 	8.0f
#define PID_KI                 	1.0f    /* con banda + descarga al cruzar    */
#define PID_KD                 	3.6f    /* ahora come la velocidad del Kalman */
/* Banda de integracion, en cm de error. El integrador acumula SOLO con
 * |error| <= PID_I_BAND; afuera queda congelado (lo aplica pid.c). Y se
 * descarga de golpe cuando el error cambia de signo.
 *
 * Por que hace falta la banda: lejos del setpoint el proporcional ya pide todo
 * lo que el servo puede dar, asi que el integrador no agrega autoridad -- lo
 * unico que hace es cargarse durante el transitorio para sobrepasar cuando la
 * pelota por fin llega. El anti-windup no cubre esto solo: frena la carga
 * cuando la salida YA esta saturada, pero entre 4 y 10 cm de error la salida
 * todavia no satura (con KP 8) y ahi el integrador cargaba libremente.
 *
 * Como se eligio el 4.0: apenas por encima del error estacionario medido
 * (2.5-3 cm), que es exactamente lo que se quiere limpiar y nada mas.
 *
 * Por que la descarga al cruzar: con friccion seca el integrador sube hasta
 * despegar la pelota, pero la friccion dinamica es MENOR que la estatica, asi
 * que arranca acelerando y se pasa -- y del otro lado hay que descargar la
 * misma carga al mismo ritmo lento. Ese ida y vuelta es el ciclo limite que se
 * vio con KI 0.5. Tirar la carga en el cruce lo corta: el integrador no puede
 * ser nunca la causa de un sobrepaso. */
#define PID_I_BAND              4.0f

/* Tiempo sin POSICION nueva tras el cual PidTask descarga el integrador. Es el
 * mismo valor que el failsafe de MotorTask a proposito: si el motor nivela la
 * barra por falta de datos, el integrador tiene que soltar la accion vieja en
 * el mismo momento, o el lazo vuelve con una correccion calculada contra una
 * posicion que ya no existe. */
#define PID_LOST_TIMEOUT_MS     APP_MOTOR_TIMEOUT_MS
/* +-40 de los +-80 disponibles. Con KP 3.0 el proporcional satura con 13 cm de
 * error, o sea casi toda la media barra queda en zona lineal y la saturacion
 * actua solo con la pelota cerca de una punta. */
#define PID_OUT_MIN             (-80.0f)
#define PID_OUT_MAX             (80.0f)
/* SERVO_CENTER_DEG (= la horizontal) esta en el bloque de calibracion del servo:
 * el lazo manda SERVO_CENTER_DEG + SERVO_DIR*u, con u acotado por PID_OUT_*.
 * Con 90 +- 8 se pide 82..98 grados: bien adentro de la guarda de 10..170.     */

/* Sentido del actuador. Absorbe de una sola vez el sentido de giro del horn Y la
 * geometria del acople, que es el unico lugar del proyecto donde entra el
 * sentido fisico. NO se deduce de la escala del servo: hay que medirlo.
 *
 * Criterio (una linea): con un angulo POR ENCIMA de 90, la punta DEL SENSOR
 * tiene que SUBIR. Porque ang > 90 ocurre solo cuando la pelota esta mas cerca
 * que el setpoint, y entonces tiene que alejarse del sensor: cuesta abajo hacia
 * la punta lejana.
 *
 * MEDIDO EN HARDWARE 2026-08-23: con ang = 170 la punta del sensor BAJA, o sea
 * corrige para el lado contrario -> -1. (Si algun dia se remonta el horn girado
 * o se cambia el vinculo de lado del pivote, se re-mide y se cambia este signo;
 * no hay que tocar nada mas.) */
#define SERVO_DIR               (-1.0f)

/* --- Recorrido del servo --------------------------------------------------- *
 * La recta us <-> grados NO esta aca: vive en el driver (servo_mg90s.h), que es
 * su dueno -- un MG90S recorre 0..180 grados entre 500 y 2500 us, y eso es un
 * hecho del componente. Aca solo se declara lo que SI es decision de la
 * aplicacion, y todo en GRADOS:
 *
 *  - SERVO_MIN_DEG / SERVO_MAX_DEG: la GUARDA contra los topes fisicos. El
 *    driver la aplica a todo llamador via Servo_SetTravel(). VERIFICADO EN
 *    HARDWARE 2026-08-23: 10 y 170 se alcanzan sin zumbido, y el test ENDPOINTS
 *    aguanta extremo a extremo sin resets.
 *  - SERVO_LEVEL_DEG: el angulo con la barra HORIZONTAL, o sea la referencia del
 *    lazo. VERIFICADO EN HARDWARE 2026-08-23 en modo HOLD: con 90 grados
 *    (1500 us) la barra queda horizontal, el horn quedo montado en el diente
 *    justo. Si algun dia hay que remontarlo, se re-mide en HOLD; un grado vale
 *    SERVO_US_PER_DEG us (lo define el driver).                                */
#define SERVO_MIN_DEG           10.0f   /* recorrido permitido: piso           */
#define SERVO_MAX_DEG           170.0f  /* recorrido permitido: techo          */
#define SERVO_LEVEL_DEG         90.0f   /* barra HORIZONTAL (referencia)       */

/* Centro del movimiento de la barra = la horizontal. Es el 0 del PID: el lazo
 * manda SERVO_CENTER_DEG + SERVO_DIR*u, con u en [PID_OUT_MIN, PID_OUT_MAX]. */
#define SERVO_CENTER_DEG        SERVO_LEVEL_DEG

/* --- Coherencia entre el rango de accion y la guarda del servo ------------- *
 * El lazo manda SERVO_CENTER_DEG + SERVO_DIR*u, con u en [PID_OUT_MIN,
 * PID_OUT_MAX]. Servo_SetAngle() satura a la guarda [SERVO_MIN_DEG,
 * SERVO_MAX_DEG], pero devuelve SERVO_OK igual: el recorte es SILENCIOSO. Si el
 * rango de accion se saliera de la guarda, nadie se enteraria y el PID quedaria
 * integrando contra un actuador saturado que no reporta nada. Mejor que no
 * compile que descubrirlo mirando la barra.
 *
 * Es _Static_assert y no #if porque el preprocesador solo hace aritmetica
 * entera y estos valores son float. GCC lo acepta (solo objeta con -Wpedantic,
 * que este proyecto no usa). */
/* Se chequean los DOS extremos contra los DOS limites, porque el angulo que se
 * comanda es SERVO_CENTER_DEG + SERVO_DIR*u: con SERVO_DIR negativo el techo de
 * u produce el piso del angulo. Asi el chequeo vale para cualquier signo de
 * SERVO_DIR y tambien si algun dia PID_OUT_MIN/MAX dejan de ser simetricos. */
_Static_assert((SERVO_CENTER_DEG + SERVO_DIR * PID_OUT_MAX) <= SERVO_MAX_DEG,
               "el angulo del lazo con PID_OUT_MAX pasa SERVO_MAX_DEG");
_Static_assert((SERVO_CENTER_DEG + SERVO_DIR * PID_OUT_MAX) >= SERVO_MIN_DEG,
               "el angulo del lazo con PID_OUT_MAX pasa SERVO_MIN_DEG");
_Static_assert((SERVO_CENTER_DEG + SERVO_DIR * PID_OUT_MIN) <= SERVO_MAX_DEG,
               "el angulo del lazo con PID_OUT_MIN pasa SERVO_MAX_DEG");
_Static_assert((SERVO_CENTER_DEG + SERVO_DIR * PID_OUT_MIN) >= SERVO_MIN_DEG,
               "el angulo del lazo con PID_OUT_MIN pasa SERVO_MIN_DEG");

#endif /* APP_CONFIG_H */
