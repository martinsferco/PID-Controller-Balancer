/**
  ******************************************************************************
  * @file    app_config.h
  * @brief   Configuracion de la aplicacion ball-and-beam. Un unico lugar para
  *          las constantes del montaje fisico, las ganancias del lazo de control
  *          y los parametros de las tasks de FreeRTOS.
  ******************************************************************************
  */

#ifndef APP_CONFIG_H
#define APP_CONFIG_H

/* ============================= Traza por UART =============================
 * Con 1 la aplicacion imprime por USART2 (COM virtual del ST-Link, 115200 8N1)
 * el banner de arranque y una linea por ciclo con el estado del lazo. Con 0 no
 * se compila nada de eso y el lazo corre igual. */
#define APP_LOG_ENABLED         1

/* ============================= Montaje fisico ============================= */

/* Largo util de la barra, medido DESDE LA CARA DEL SENSOR. */
#define BEAM_LENGTH_CM          30.0f

/* --- Sensor HC-SR04 (posicion de la pelota) ------------------------------- */
#define SENSOR_MIN_CM           3.0f    /* zona muerta real del sensor          */
#define SENSOR_MAX_CM           32.0f   /* barra de 30 cm + margen              */
#define SENSOR_ECHO_TIMEOUT_MS  80u     /* guarda: < periodo, > timeout interno */

/* Banda de gracia en los bordes de la ventana util: una medicion que cae fuera
 * de [SENSOR_MIN_CM, SENSOR_MAX_CM] pero a menos de esto del borde se recorta y
 * se usa igual, porque significa que la pelota esta en la punta de la barra o
 * pegada al sensor. Mas lejos se descarta: eso ya es un eco del ambiente, y
 * usarlo haria inclinar la barra por una pelota que no esta ahi. Por eso la
 * banda tiene que ser chica. */
#define SENSOR_EDGE_GRACE_CM    3.0f

/* --- Potenciometro (setpoint) --------------------------------------------- *
 * Rango de setpoint que barre el pote de tope a tope. No es 0..BEAM_LENGTH_CM
 * porque ninguno de esos dos extremos es alcanzable: por abajo el sensor no ve
 * nada antes de SENSOR_MIN_CM y por arriba la pelota se cae de la punta. Pedir
 * un setpoint inalcanzable deja al proporcional inclinando para siempre contra
 * un tope. */
#define POTENTIOMETER_MIN_CM    5.0f
#define POTENTIOMETER_MAX_CM    25.0f

/* Setpoint por defecto, hasta que PotTask publique su primera lectura. */
#define SETPOINT_DEFAULT_CM     (BEAM_LENGTH_CM * 0.5f)

/* --- Servo MG90S (actuador) ----------------------------------------------- *
 * La recta grados <-> us vive en el driver (servo_mg90s.h): un MG90S recorre
 * 0..180 grados entre 500 y 2500 us, y eso es un hecho del componente. Aca va
 * solo lo que decide la aplicacion, y todo en grados. */
#define SERVO_MIN_DEG           10.0f   /* recorrido permitido: piso            */
#define SERVO_MAX_DEG           170.0f  /* recorrido permitido: techo           */
#define SERVO_LEVEL_DEG         90.0f   /* barra HORIZONTAL: referencia del lazo */

/* Centro del movimiento = la horizontal. Es el 0 del PID: el lazo comanda
 * SERVO_CENTER_DEG + SERVO_DIR * u. */
#define SERVO_CENTER_DEG        SERVO_LEVEL_DEG

/* Sentido del actuador. Absorbe de una sola vez el sentido de giro del horn y la
 * geometria del acople, que es el unico lugar del proyecto donde entra el
 * sentido fisico; se mide, no se deduce de la escala del servo. Criterio: con un
 * angulo por encima de SERVO_LEVEL_DEG la punta DEL SENSOR tiene que SUBIR (ese
 * angulo se pide solo cuando la pelota esta mas cerca que el setpoint, o sea
 * cuando tiene que alejarse). En este montaje pasa lo contrario, de ahi el -1. */
#define SERVO_DIR               (-1.0f)

/* ============================ Lazo de control ============================= */

/* --- Filtro de Kalman (2 estados: posicion y velocidad) -------------------- *
 * Q y R se dimensionan por el indice de seguimiento
 * lambda = sqrt(Q) * dt^2 / sqrt(R): con lambda << 1 el filtro suaviza mucho
 * pero retrasa, y con lambda ~ 1 sigue la medicion con una o dos muestras de
 * atraso. Estos valores dan lambda = 0.22, que alcanza para seguir los saltos
 * reales de la pelota sin dejar pasar la cuantizacion del sensor. */
#define KALMAN_DT               0.1f    /* 100 ms: el tick del sensor           */
#define KALMAN_Q                20.0f   /* densidad de ruido de proceso         */
#define KALMAN_R                0.04f   /* varianza de medicion (sigma 0.2 cm)  */

/* --- Controlador PID ------------------------------------------------------ *
 * La planta es un DOBLE INTEGRADOR: la inclinacion de la barra manda la
 * aceleracion de la pelota, no su velocidad. Con la ganancia medida en hardware
 * (K = 1.2 cm/s2 por grado de horn) el lazo cerrado queda descrito por
 * wn = sqrt(K*KP) y zeta = KD*sqrt(K) / (2*sqrt(KP)):
 *
 *   KP 8.0 -> wn = 3.1 rad/s, o sea wn*dt = 0.31: el techo razonable para un
 *             muestreo de 10 Hz.
 *   KD 3.6 -> zeta = 0.70, amortiguamiento cerca del critico. KD NO es opcional:
 *             en un doble integrador, con KD = 0 el amortiguamiento es cero por
 *             construccion y no existe KP que no oscile.
 *   KI 1.0 -> solo para limpiar el error estacionario que deja la friccion. El
 *             margen de estabilidad lineal es KI < KD*KP*K = 34, asi que el
 *             valor esta muy por debajo.
 *
 * Si el servo tiembla con la pelota quieta, el ajuste va en KALMAN_Q (suaviza la
 * velocidad que alimenta el termino D), no en PID_KD (fija el amortiguamiento). */
#define PID_DT                  0.1f
#define PID_KP                  8.0f
#define PID_KI                  1.0f
#define PID_KD                  3.6f

/* Banda de integracion, en cm de error: el integrador acumula solo con
 * |error| <= PID_I_BAND, y se descarga al cruzar el setpoint. Lejos del objetivo
 * el proporcional ya pide todo lo que el servo puede dar, asi que ahi el
 * integrador no agrega autoridad: lo unico que hace es cargarse durante el
 * transitorio para sobrepasar cuando la pelota por fin llega. Se elige apenas
 * por encima del error estacionario que se quiere limpiar. */
#define PID_I_BAND              4.0f

/* Saturacion de la accion de control, en grados de horn medidos desde la
 * horizontal. */
#define PID_OUT_MIN             (-80.0f)
#define PID_OUT_MAX             (80.0f)

/* ======================== Tasks y tiempos de FreeRTOS ===================== */

/* Stack en WORDS (no bytes). Prioridad mayor = mas urgente. */
#define SENSOR_TASK_STACK       384u
#define KALMAN_TASK_STACK       256u
#define PID_TASK_STACK          384u
#define MOTOR_TASK_STACK        384u
#define POT_TASK_STACK          192u

#define SENSOR_TASK_PRIO        5u
#define KALMAN_TASK_PRIO        4u
#define MOTOR_TASK_PRIO         4u
#define PID_TASK_PRIO           3u
#define POT_TASK_PRIO           1u

/* Periodo de lectura del pote: es una mano girando, no hace falta mas rapido. */
#define POT_PERIOD_MS           200u

/* Failsafe del actuador: tiempo sin angulo nuevo tras el cual MotorTask nivela
 * la barra. Si el sensor deja de ver la pelota, dejar el servo clavado en la
 * ultima inclinacion es la peor posicion posible para que vuelva. El lazo
 * publica cada 100 ms, asi que esto son 5 muestras perdidas. */
#define APP_MOTOR_TIMEOUT_MS    500u

/* ==================== Coherencia entre el lazo y el servo =================
 * El lazo comanda SERVO_CENTER_DEG + SERVO_DIR * u con u en [PID_OUT_MIN,
 * PID_OUT_MAX], y Servo_SetAngle() satura al recorrido permitido en SILENCIO.
 * Si el rango de accion se saliera de la guarda nadie se enteraria: el PID
 * quedaria integrando contra un actuador saturado que no reporta nada. Mejor que
 * no compile. Se chequean los dos extremos contra los dos limites, porque con
 * SERVO_DIR negativo el techo de u produce el piso del angulo.
 *
 * Es _Static_assert y no #if porque el preprocesador solo hace aritmetica entera
 * y estos valores son float. */
_Static_assert((SERVO_CENTER_DEG + SERVO_DIR * PID_OUT_MAX) <= SERVO_MAX_DEG,
               "el angulo del lazo con PID_OUT_MAX pasa SERVO_MAX_DEG");
_Static_assert((SERVO_CENTER_DEG + SERVO_DIR * PID_OUT_MAX) >= SERVO_MIN_DEG,
               "el angulo del lazo con PID_OUT_MAX pasa SERVO_MIN_DEG");
_Static_assert((SERVO_CENTER_DEG + SERVO_DIR * PID_OUT_MIN) <= SERVO_MAX_DEG,
               "el angulo del lazo con PID_OUT_MIN pasa SERVO_MAX_DEG");
_Static_assert((SERVO_CENTER_DEG + SERVO_DIR * PID_OUT_MIN) >= SERVO_MIN_DEG,
               "el angulo del lazo con PID_OUT_MIN pasa SERVO_MIN_DEG");

#endif /* APP_CONFIG_H */
