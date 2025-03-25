#ifndef __VECTOR_UTILS_H__
#define __VECTOR_UTILS_H__

#include "Utils.h"

bool Saturate_Vector_2d(float *x, float *y, float max);
bool Saturate_Vector_2d_fixed(float *x, float *y, float max, uint8_t fixed_index);

#endif
