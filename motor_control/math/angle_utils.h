/**
 * @file angle_utils.h
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

#ifndef NORMALIZE_USE_C_MATH
#define NORMALIZE_USE_C_MATH 0
#endif

// limited rad to 0~2pi rads
static inline float Normalize_Rad(float rad)
{
#if NORMALIZE_USE_C_MATH
    float a = fmodf(rad, _2PI);
    return a >= 0 ? a : (a + _2PI);
#else
    while (rad >= _2PI)
    {
        rad -= _2PI;
    }
    while (rad < 0.0f)
    {
        rad += _2PI;
    }
    return rad;
#endif
}

// limited rad to 0~pi rads
static inline float Normalize_Rad_PI(float rad)
{
#if NORMALIZE_USE_C_MATH
    float a = fmodf(rad, _PI);
    return a >= 0 ? a : (a + _PI);
#else
    while (rad >= _PI)
    {
        rad -= _PI;
    }
    while (rad < 0.0f)
    {
        rad += _PI;
    }
    return rad;
#endif
}

// limited deg to 0~360 degree
static inline float Normalize_Deg(float deg)
{
#if NORMALIZE_USE_C_MATH
    float a = fmodf(deg, 360.0f);
    return a >= 0 ? a : (a + 360.0f);
#else
    while (deg >= 360.0f)
    {
        deg -= 360.0f;
    }
    while (deg < 0.0f)
    {
        deg += 360.0f;
    }
    return deg;
#endif
}

// limited deg to 0~180 degree
static inline float Normalize_Deg_180(float deg)
{
#if NORMALIZE_USE_C_MATH
    float a = fmodf(deg, 180.0f);
    return a >= 0 ? a : (a + 180.0f);
#else
    while (deg >= 180.0f)
    {
        deg -= 180.0f;
    }
    while (deg < 0.0f)
    {
        deg += 180.0f;
    }
    return deg;
#endif
}

// Calculate the delta between the two angles
static inline float Rad_Delta(float rad0, float rad1)
{
    float rads_delta = 0;

    rad0       = Normalize_Rad(rad0);
    rad1       = Normalize_Rad(rad1);
    rads_delta = rad0 - rad1;

    if (rads_delta < -_PI)
    {
        rads_delta += _2PI;
    }
    else if (rads_delta > _PI)
    {
        rads_delta -= _2PI;
    }

    return rads_delta;
}

static inline float Deg_Delta(float deg0, float deg1)
{
    float degs_delta = 0;

    deg0       = Normalize_Deg(deg0);
    deg1       = Normalize_Deg(deg1);
    degs_delta = deg0 - deg1;

    if (degs_delta < -180.0f)
    {
        degs_delta += 360.0f;
    }
    else if (degs_delta > 180.0f)
    {
        degs_delta -= 360.0f;
    }

    return degs_delta;
}

// Calculate the delta between the two angles
static inline float ABS_Rad_Delta(float rad0, float rad1)
{
    float rads      = 0;
    float abs_delta = 0;

    rad0      = Normalize_Rad(rad0);
    rad1      = Normalize_Rad(rad1);
    abs_delta = fabsf(rad0 - rad1);

    if (abs_delta < _PI)
    {
        rads = abs_delta;
    }
    else
    {
        rads = _2PI - abs_delta;
    }

    return rads;
}

static inline float ABS_Deg_Delta(float deg0, float deg1)
{
    float degs      = 0;
    float abs_delta = 0;

    deg0      = Normalize_Deg(deg0);
    deg1      = Normalize_Deg(deg1);
    abs_delta = fabsf(deg0 - deg1);

    if (abs_delta < 180)
    {
        degs = abs_delta;
    }
    else
    {
        degs = 360 - abs_delta;
    }

    return degs;
}

static inline int8_t Rad_Move_Dir(float rad0, float rad1)
{
    float rads_delta = 0;
    int8_t dir       = 0;

    rad0       = Normalize_Rad(rad0);
    rad1       = Normalize_Rad(rad1);
    rads_delta = rad0 - rad1;

    if (rads_delta < -_PI)
    {
        rads_delta += _2PI;
    }
    else if (rads_delta > _PI)
    {
        rads_delta -= _2PI;
    }

    dir = -SIGN_F(rads_delta);

    return dir;
}

static inline int8_t Deg_Move_Dir(float deg0, float deg1)
{
    float degs_delta = 0;
    int8_t dir       = 0;

    deg0       = Normalize_Deg(deg0);
    deg1       = Normalize_Deg(deg1);
    degs_delta = deg0 - deg1;

    if (degs_delta < -180.0f)
    {
        degs_delta += 360.0f;
    }
    else if (degs_delta > 180.0f)
    {
        degs_delta -= 360.0f;
    }

    dir = -SIGN_F(degs_delta);

    return dir;
}

float Rad_Average(float rad0, float rad1);
float Rad_Weighted_Average(float rad0, float rad1, float weight);
float Deg_Average(float deg0, float deg1);
float Deg_Weighted_Average(float deg0, float deg1, float weight);
float Rad_Average_Mult(float *rads, uint32_t nums);
float Rad_Weighted_Average_Mult(float *rads, float *weight, uint32_t nums);
float Deg_Average_Mult(float *degs, uint32_t nums);
float Deg_Weighted_Average_Mult(float *degs, float *weight, uint32_t nums);
