#include "Utils.h"

float Max_Abs(float va, float vb)
{
    if (fabsf(va) > fabsf(vb))
    {
        return va;
    }
    else
    {
        return vb;
    }

}

float Min_Abs(float va, float vb)
{
    if (fabsf(va) < fabsf(vb))
    {
        return va;
    }
    else
    {
        return vb;
    }
}

void Truncate_Number(float *number, float min, float max)
{
    if (*number > max)
    {
        *number = max;
    }
    else if (*number < min)
    {
        *number = min;
    }
}

void Truncate_Number_Abs(float *number, float max)
{
    if (*number > max)
    {
        *number = max;
    }
    else if (*number < -max)
    {
        *number = -max;
    }
}

uint8_t Number_In_Absolute_Range_f32(float num,float base,float range)
{
    if(fabsf(num - base) < fabsf(range))
    {
        return 1;
    }
    else
    {
        return 0;
    }
}
