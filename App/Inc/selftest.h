/**
  ******************************************************************************
  * @file    selftest.h
  * @brief   Self-tests on-target de los modulos puros (Kalman, PID). Corren en
  *          la placa e imprimen PASS/FAIL por UART. Se activan con el flag
  *          APP_RUN_SELFTESTS (app_config.h): con el flag en 0, selftest.c no
  *          aporta codigo al binario.
  ******************************************************************************
  */

#ifndef SELFTEST_H
#define SELFTEST_H

#ifdef __cplusplus
extern "C" {
#endif

/**
  * @brief  Corre todos los casos de prueba e imprime el resultado por UART.
  *         Llamar (bajo #if APP_RUN_SELFTESTS) antes de crear las tasks.
  */
void SelfTest_Run(void);

#ifdef __cplusplus
}
#endif

#endif /* SELFTEST_H */
