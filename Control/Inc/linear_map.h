/**
  ******************************************************************************
  * @file    linear_map.h
  * @brief   Mapeo afin de un valor de un rango de entrada a uno de salida.
  ******************************************************************************
  */

#ifndef LINEAR_MAP_H
#define LINEAR_MAP_H

#ifdef __cplusplus
extern "C" {
#endif

/**
  * @brief  Mapea x del rango [in_min, in_max] al rango [out_min, out_max] de
  *         forma lineal.
  */
float linear_map(float x, float in_min, float in_max, float out_min, float out_max);

#ifdef __cplusplus
}
#endif

#endif // LINEAR_MAP_H
