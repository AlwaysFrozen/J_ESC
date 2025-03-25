#include "ANGLE_Utils.h"

// limited rad to 0~2pi rads
float Normalize_Rad(float rad)
{
    float a = fmodf(rad, _2PI);
    return a >= 0 ? a : (a + _2PI);
}

// limited rad to 0~pi rads
float Normalize_Rad_PI(float rad)
{
    float a = fmodf(rad, _PI);
    return a >= 0 ? a : (a + _PI);
}

// limited deg to 0~360 degree
float Normalize_Angle(float deg)
{
    float a = fmodf(deg, 360.0f);
    return a >= 0 ? a : (a + 360.0f);
}

// limited deg to 0~180 degree
float Normalize_Angle_180(float deg)
{
    float a = fmodf(deg, 180.0f);
    return a >= 0 ? a : (a + 180.0f);
}

// Calculate the difference between the two angles
float ABS_Rad_Diff(float rad0,float rad1)
{
    float rad = 0;
    float abs_delta = 0;

    rad0 = Normalize_Rad(rad0);
    rad1 = Normalize_Rad(rad1);
    abs_delta = fabsf(rad0 - rad1);

    if (abs_delta < _PI)
    {
        rad = abs_delta;
    }
    else
    {
        rad = _2PI - abs_delta;
    }

    return rad;
}

float ABS_Angle_Diff(float deg0, float deg1)
{
    float angle = 0;
    float abs_delta = 0;

    deg0 = Normalize_Angle(deg0);
    deg1 = Normalize_Angle(deg1);
    abs_delta = fabsf(deg0 - deg1);

    if (abs_delta < 180)
    {
        angle = abs_delta;
    }
    else
    {
        angle = 360 - abs_delta;
    }

    return angle;
}

int8_t Rad_Diff_Dir(float rad0, float rad1)
{
    float angle_delta = 0;
    int8_t dir = 0;

    rad0 = Normalize_Rad(rad0);
    rad1 = Normalize_Rad(rad1);
    angle_delta = rad0 - rad1;

    if (angle_delta < -_PI)
    {
        angle_delta += _2PI;
    }
    else if (angle_delta > _PI)
    {
        angle_delta -= _2PI;
    }

    dir = -SIGN_F(angle_delta);

    return dir;
}

int8_t Angle_Diff_Dir(float deg0, float deg1)
{
    float angle_delta = 0;
    int8_t dir = 0;

    deg0 = Normalize_Angle(deg0);
    deg1 = Normalize_Angle(deg1);
    angle_delta = deg0 - deg1;

    if (angle_delta < -180.0f)
    {
        angle_delta += 360.0f;
    }
    else if (angle_delta > 180.0f)
    {
        angle_delta -= 360.0f;
    }

    dir = -SIGN_F(angle_delta);

    return dir;
}

float Rad_Average(float rad0, float rad1)
{
    float sin_sum = sinf(rad0) + sinf(rad1);
    float cos_sum = cosf(rad0) + cosf(rad1);

    return Normalize_Rad(atan2f(sin_sum, cos_sum));
}

float Rad_Weighted_Average(float rad0, float rad1, float weight)
{
    float sin_sum = sinf(rad0) * weight + sinf(rad1) * (1.0f - weight);
    float cos_sum = cosf(rad0) * weight + cosf(rad1) * (1.0f - weight);

    return Normalize_Rad(atan2f(sin_sum, cos_sum));
}

float Angle_Average(float deg0, float deg1)
{
    deg0 = DEG_TO_RAD(deg0);
    deg1 = DEG_TO_RAD(deg1);
    float sin_sum = sinf(deg0) + sinf(deg1);
    float cos_sum = cosf(deg0) + cosf(deg1);

    return Normalize_Angle(RAD_TO_DEG(atan2f(sin_sum, cos_sum)));
}

float Angle_Weighted_Average(float deg0, float deg1, float weight)
{
    deg0 = DEG_TO_RAD(deg0);
    deg1 = DEG_TO_RAD(deg1);
    float sin_sum = sinf(deg0) * weight + sinf(deg1) * (1.0f - weight);
    float cos_sum = cosf(deg0) * weight + cosf(deg1) * (1.0f - weight);

    return Normalize_Angle(RAD_TO_DEG(atan2f(sin_sum, cos_sum)));
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

float Angle_Average_Mult(float *degs, uint32_t nums)
{
    float sin_sum = 0.0;
    float cos_sum = 0.0;

    for (uint32_t i = 0; i < nums; i++)
    {
        sin_sum += sinf(DEG_TO_RAD(degs[i]));
        cos_sum += cosf(DEG_TO_RAD(degs[i]));
    }

    return Normalize_Angle(RAD_TO_DEG(atan2f(sin_sum, cos_sum)));
}

float Angle_Weighted_Average_Mult(float *degs, float *weight, uint32_t nums)
{
    float sin_sum = 0.0;
    float cos_sum = 0.0;

    for (uint32_t i = 0; i < nums; i++)
    {
        sin_sum += sinf(DEG_TO_RAD(degs[i])) * weight[i];
        cos_sum += cosf(DEG_TO_RAD(degs[i])) * weight[i];
    }

    return Normalize_Angle(RAD_TO_DEG(atan2f(sin_sum, cos_sum)));
}
