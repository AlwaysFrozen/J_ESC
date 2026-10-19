/**
 * @file angle_utils.c
 * @author jinchl (jcl709905683@gmail.com)
 * @brief 
 * @version 0.1
 * @date 2026-03-11
 * 
 * @copyright Copyright (c) 2026
 * 
 */


#include "angle_utils.h"

float Rad_Average(float rad0, float rad1)
{
    float sin_sum = sinf(rad0) + sinf(rad1);
    float cos_sum = cosf(rad0) + cosf(rad1);

    return Normalize_Rad(atan2f(sin_sum, cos_sum));
}

float Rad_Weighted_Average(float rad0, float rad1, float weight)
{
    weight = CONSTRAIN(weight, 0.0f, 1.0f);

    float sin_sum = sinf(rad0) * weight + sinf(rad1) * (1.0f - weight);
    float cos_sum = cosf(rad0) * weight + cosf(rad1) * (1.0f - weight);

    return Normalize_Rad(atan2f(sin_sum, cos_sum));
}

float Deg_Average(float deg0, float deg1)
{
    deg0          = DEG_TO_RAD(deg0);
    deg1          = DEG_TO_RAD(deg1);
    float sin_sum = sinf(deg0) + sinf(deg1);
    float cos_sum = cosf(deg0) + cosf(deg1);

    return Normalize_Deg(RAD_TO_DEG(atan2f(sin_sum, cos_sum)));
}

float Deg_Weighted_Average(float deg0, float deg1, float weight)
{
    weight = CONSTRAIN(weight, 0.0f, 1.0f);

    deg0          = DEG_TO_RAD(deg0);
    deg1          = DEG_TO_RAD(deg1);
    float sin_sum = sinf(deg0) * weight + sinf(deg1) * (1.0f - weight);
    float cos_sum = cosf(deg0) * weight + cosf(deg1) * (1.0f - weight);

    return Normalize_Deg(RAD_TO_DEG(atan2f(sin_sum, cos_sum)));
}

float Rad_Average_Mult(float *rads, uint32_t nums)
{
    float sin_sum = 0.0;
    float cos_sum = 0.0;

    for (uint32_t i = 0; i < nums; i++)
    {
        sin_sum += sinf(rads[i]);
        cos_sum += cosf(rads[i]);
    }

    return Normalize_Rad(atan2f(sin_sum, cos_sum));
}

float Rad_Weighted_Average_Mult(float *rads, float *weight, uint32_t nums)
{
    float sin_sum = 0.0;
    float cos_sum = 0.0;

    for (uint32_t i = 0; i < nums; i++)
    {
        sin_sum += sinf(rads[i]) * weight[i];
        cos_sum += cosf(rads[i]) * weight[i];
    }

    return Normalize_Rad(atan2f(sin_sum, cos_sum));
}

float Deg_Average_Mult(float *degs, uint32_t nums)
{
    float sin_sum = 0.0;
    float cos_sum = 0.0;

    for (uint32_t i = 0; i < nums; i++)
    {
        sin_sum += sinf(DEG_TO_RAD(degs[i]));
        cos_sum += cosf(DEG_TO_RAD(degs[i]));
    }

    return Normalize_Deg(RAD_TO_DEG(atan2f(sin_sum, cos_sum)));
}

float Deg_Weighted_Average_Mult(float *degs, float *weight, uint32_t nums)
{
    float sin_sum = 0.0;
    float cos_sum = 0.0;

    for (uint32_t i = 0; i < nums; i++)
    {
        sin_sum += sinf(DEG_TO_RAD(degs[i])) * weight[i];
        cos_sum += cosf(DEG_TO_RAD(degs[i])) * weight[i];
    }

    return Normalize_Deg(RAD_TO_DEG(atan2f(sin_sum, cos_sum)));
}
