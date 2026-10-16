/**
 * @file vector_utils.h
 * @author jinchl (jcl709905683@gmail.com)
 * @brief 
 * @version 0.1
 * @date 2026-03-11
 * 
 * @copyright Copyright (c) 2026
 * 
 */


#pragma once
#include "utils.h"

static inline bool Saturate_Vector_2d(float *x, float *y, float max)
{
    bool retval = false;
    float mag   = NORM2_f(*x, *y);
    max         = fabsf(max);

    if (mag < 1e-10f)
    {
        mag = 1e-10f;
    }

    if (mag > max)
    {
        const float f  = max / mag;
        *x            *= f;
        *y            *= f;
        retval         = true;
    }

    return retval;
}

static inline void Saturate_Vector_2d_Fast(float *x, float *y, float max)
{
    float mag_sq = SQ(*x) + SQ(*y);
    float max_sq = SQ(max);

    if (mag_sq > max_sq)
    {
        float mag = sqrtf(mag_sq);

        float inv_mag = max / mag;

        *x *= inv_mag;
        *y *= inv_mag;
    }
}

static inline bool Saturate_Vector_2d_Fixed(float *x, float *y, float max, uint8_t fixed_index)
{
    bool retval   = false;
    float max_abs = 0;
    float temp    = 0;

    switch (fixed_index)
    {
    case 0:
        max_abs = sqrtf(SQ(max) - SQ(*x));
        temp    = *y;
        temp    = CONSTRAIN(temp, -max_abs, max_abs);
        retval  = temp != *y;
        *y      = temp;
        break;

    case 1:
        max_abs = sqrtf(SQ(max) - SQ(*y));
        temp    = *x;
        temp    = CONSTRAIN(temp, -max_abs, max_abs);
        retval  = temp != *x;
        *x      = temp;
        break;

    default:
        retval = Saturate_Vector_2d(x, y, max);
        break;
    }

    return retval;
}
