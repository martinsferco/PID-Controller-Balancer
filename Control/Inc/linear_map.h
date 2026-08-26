/**
  ******************************************************************************
  * @file    linear_map.h
  * @brief   Mapeo afin de un valor de un rango de entrada a uno de salida.
  *          Modulo PURO: solo <float>, sin HAL ni FreeRTOS. Reusable para
  *          convertir cualquier lectura normalizada a una magnitud fisica.
  ******************************************************************************
  */

#ifndef LINEAR_MAP_H
#define LINEAR_MAP_H

#ifdef __cplusplus
extern "C" {
#endif

/**
  * @brief  Mapea x del rango [in_min, in_max] al rango [out_min, out_max] de
  *         forma lineal. No recorta: si x cae fuera del rango de entrada, el
  *         resultado extrapola. Con in_min == in_max devuelve out_min (evita
  *         dividir por cero).
  */
float linear_map(float x, float in_min, float in_max, float out_min, float out_max);

#ifdef __cplusplus
}
#endif

#endif /* LINEAR_MAP_H */
