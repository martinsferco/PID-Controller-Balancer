/**
  ******************************************************************************
  * @file    linear_map.c
  * @brief   Implementacion del mapeo afin.
  ******************************************************************************
  */

#include "linear_map.h"

float linear_map(float x, float in_min, float in_max, float out_min, float out_max)
{
    const float span = in_max - in_min;
    if (span == 0.0f) { return out_min; }   /* guarda: rango de entrada nulo */
    return out_min + (x - in_min) * (out_max - out_min) / span;
}
