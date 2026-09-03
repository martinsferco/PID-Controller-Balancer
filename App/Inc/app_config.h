/**
  ******************************************************************************
  * @file    app_config.h
  * @brief   Configuracion de la aplicacion sistema de control PID para balanceo. Un unico lugar para
  *          las constantes del montaje fisico, las ganancias del lazo de control
  *          y los parametros de las tasks de FreeRTOS.
  *
  *          El razonamiento y las derivaciones detras de cada valor estan en
  *          Notas_Diseno.md (raiz del repo).
  ******************************************************************************
  */

#ifndef APP_CONFIG_H
#define APP_CONFIG_H

#include "hc_sr04.h"    /* HC_SR04_HW_MIN_CM: limite fisico del sensor */

/* ============================= Montaje fisico ============================= */

/* Largo util de la barra, medido desde la cara del sensor. */
#define BEAM_LENGTH_CM          30.0f

/* --- Sensor HC-SR04 (posicion del borde del carro que encara al sensor) --- */
/* HC_SR04_HW_MIN_CM (hc_sr04.h) es la zona muerta real del componente: por
 * debajo de eso directamente no hay eco, y el driver ya lo clasifica como
 * HC_SR04_INVALID -- subir esta constante (en vez del margen de abajo) achica
 * la ventana de lecturas OK sin necesidad. SENSOR_SAFETY_MARGIN_CM es el
 * margen de seguridad de la app por encima de ese piso: pegado al limite
 * exacto la lectura deja de ser confiable (eco debil/ausente por campo
 * cercano), y el sensor deja de responder si se le pide operar justo ahi.
 * Se subio de 1.0 a 2.0 tras verificar en banco que, con HC_SR04_HW_MIN_CM en
 * su valor real de datasheet, hacia falta mas margen.
 * SENSOR_MIN_CM NO es un piso que task_sensor.c le fuerce a toda lectura: una
 * medicion HC_SR04_OK mas cerca que esto se publica tal cual (es real, no
 * ambigua). Se usa como base del rango de setpoint de mas abajo, y tambien
 * (dividido a la mitad, directo en task_sensor.c) como la proximidad asumida
 * para el caso HC_SR04_INVALID de distancia gigantesca (eco fantasma). */
#define SENSOR_SAFETY_MARGIN_CM 2.0f
#define SENSOR_MIN_CM           (HC_SR04_HW_MIN_CM + SENSOR_SAFETY_MARGIN_CM)

/* Borde maximo medido con el carro apoyado contra su tope mecanico (choca
 * contra el motor antes de llegar a BEAM_LENGTH_CM). Se mide directamente
 * con el carro en ese limite, no se deduce de la geometria del carro. A
 * diferencia de SENSOR_MIN_CM, este si es un techo real: mas alla no es
 * fisicamente alcanzable, asi que toda lectura (OK o INVALID) se recorta acá. */
#define SENSOR_MAX_CM           21.0f
#define SENSOR_ECHO_TIMEOUT_MS  80u     /* guarda: < periodo, > timeout interno */

/* --- Potenciometro (setpoint, en borde del carro) --------------------------- */
/* Rango de setpoint que barre el pote de tope a tope, en la misma referencia
 * que publica task_sensor.c (borde del carro que encara al sensor): el mismo
 * rango que ve el sensor, SENSOR_MIN_CM..SENSOR_MAX_CM. Se probo separarlo con
 * un margen propio (POTENTIOMETER_MARGIN_CM) para que el pote nunca pudiera
 * pedir justo el valor donde HC_SR04_INVALID clampeaba -- eso importaba
 * cuando ese clamp era SENSOR_MIN_CM, pero ya no: ver la politica de
 * task_sensor.c, ahora clampea a HC_SR04_HW_MIN_CM o a SENSOR_MIN_CM / 2,
 * bien por debajo. Sin esa coincidencia, el margen aparte no aportaba nada;
 * se saco. */
#define POTENTIOMETER_MIN_CM    SENSOR_MIN_CM
#define POTENTIOMETER_MAX_CM    SENSOR_MAX_CM

/* Setpoint por defecto, hasta que PotTask publique su primera lectura: punto
 * medio del rango de borde realmente alcanzable. */
#define SETPOINT_DEFAULT_CM     ((POTENTIOMETER_MIN_CM + POTENTIOMETER_MAX_CM) * 0.5f)

/* --- Servo MG90S (actuador) ----------------------------------------------- */
/* La recta grados <-> us vive en el driver; aca solo lo que decide la app, en
 * grados. */
#define SERVO_MIN_DEG           10.0f   /* recorrido permitido: piso            */
#define SERVO_MAX_DEG           170.0f  /* recorrido permitido: techo           */
#define SERVO_LEVEL_DEG         90.0f   /* barra HORIZONTAL: referencia del lazo */

/* Centro del movimiento = la horizontal. Es el 0 del PID: el lazo comanda
 * SERVO_CENTER_DEG + SERVO_DIR * u. */
#define SERVO_CENTER_DEG        SERVO_LEVEL_DEG

/* Sentido del actuador (giro del horn + geometria del acople). Se mide, no se
 * deduce; ver Notas_Diseno.md. */
#define SERVO_DIR               (-1.0f)

/* ============================ Lazo de control ============================= */

/* --- Filtro de Kalman (2 estados: posicion y velocidad) ------------------- */
/* Dimensionados por lambda = sqrt(Q)*dt^2/sqrt(R) = 0.22 (ver Notas_Diseno.md). */
#define KALMAN_DT               0.1f    /* 100 ms: el tick del sensor           */
#define KALMAN_Q                20.0f   /* densidad de ruido de proceso         */
#define KALMAN_R                0.04f   /* varianza de medicion (sigma 0.2 cm)  */

/* --- Controlador PID ------------------------------------------------------ */
/* Planta = doble integrador. Ganancias derivadas de wn/zeta con K medido en
 * hardware (ver Notas_Diseno.md). KD no es opcional: fija el amortiguamiento. */
#define PID_DT                  0.1f
#define PID_KP                  8.0f
#define PID_KI                  1.0f
#define PID_KD                  3.6f

/* Banda de integracion [cm de error]: integra solo con |error| <= esto y se
 * descarga al cruzar el setpoint. */
#define PID_I_BAND              4.0f

/* Saturacion de la accion de control [grados de horn desde la horizontal]. */
#define PID_OUT_MIN             (-80.0f)
#define PID_OUT_MAX             (80.0f)

/* ======================== Tasks y tiempos de FreeRTOS ===================== */

/* Stack en WORDS (no bytes). Prioridad mayor = mas urgente. */
#define SENSOR_TASK_STACK       400u
#define KALMAN_TASK_STACK       300u
#define PID_TASK_STACK          400u
#define MOTOR_TASK_STACK        400u
#define POT_TASK_STACK          300u

#define SENSOR_TASK_PRIO        5u
#define KALMAN_TASK_PRIO        4u
#define MOTOR_TASK_PRIO         4u
#define PID_TASK_PRIO           3u
#define POT_TASK_PRIO           1u

/* Periodo de lectura del pote: es una mano girando, no hace falta mas rapido. */
#define POT_PERIOD_MS           200u

/* Timeout de "sin dato nuevo" de cada task del lazo: 5 muestras perdidas del
 * lazo de 100 ms. Un define por task (aunque hoy coincidan en el valor) para
 * poder ajustarlos por separado el dia que dejen de coincidir. */
#define MOTOR_TASK_TIMEOUT_MS   500u    /* MotorTask: nivela la barra              */
#define KALMAN_TASK_TIMEOUT_MS  500u    /* KalmanTask: se re-arma para Kalman_Reset */
#define PID_TASK_TIMEOUT_MS     500u    /* PidTask: sigue esperando, sin resetear   */

/* ==================== Coherencia entre el lazo y el servo =================
 * El lazo comanda SERVO_CENTER_DEG + SERVO_DIR * u y Servo_SetAngle() satura en
 * silencio: si el rango de accion se saliera de la guarda, el PID integraria
 * contra un actuador saturado sin que nadie se entere. Mejor que no compile.
 * Es _Static_assert (no #if) porque los valores son float. Ver Notas_Diseno.md. */
_Static_assert((SERVO_CENTER_DEG + SERVO_DIR * PID_OUT_MAX) <= SERVO_MAX_DEG,
               "el angulo del lazo con PID_OUT_MAX pasa SERVO_MAX_DEG");
_Static_assert((SERVO_CENTER_DEG + SERVO_DIR * PID_OUT_MAX) >= SERVO_MIN_DEG,
               "el angulo del lazo con PID_OUT_MAX pasa SERVO_MIN_DEG");
_Static_assert((SERVO_CENTER_DEG + SERVO_DIR * PID_OUT_MIN) <= SERVO_MAX_DEG,
               "el angulo del lazo con PID_OUT_MIN pasa SERVO_MAX_DEG");
_Static_assert((SERVO_CENTER_DEG + SERVO_DIR * PID_OUT_MIN) >= SERVO_MIN_DEG,
               "el angulo del lazo con PID_OUT_MIN pasa SERVO_MIN_DEG");

#endif /* APP_CONFIG_H */
