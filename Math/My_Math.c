#include "My_Math.h"
#include "math.h"
#include "FOC.h"

void Ramp_Init(Ramp_t *ramp,float in,float out,float output_max,float output_min,float rise_time_ms,float fall_time_ms,float dt)
{
    ramp->input = in;
    ramp->output = out;
    ramp->output_max = output_max;
    ramp->output_min = output_min;
    ramp->rise_time_ms = rise_time_ms;
    ramp->fall_time_ms = fall_time_ms;
    ramp->dt = dt;

    if(ramp->rise_time_ms)
    {
        ramp->delta_max = ramp->output_max / ramp->rise_time_ms * ramp->dt;
    }
    else
    {
        ramp->delta_max = ramp->output_max - ramp->output_min;
    }
    if(ramp->fall_time_ms)
    {
        ramp->delta_min = ramp->output_min / ramp->fall_time_ms * ramp->dt;
    }
    else
    {
        ramp->delta_min = ramp->output_min - ramp->output_max;
    }
}

float Ramp_Update(Ramp_t *ramp,float in)
{
    ramp->input = in;
    ramp->delta = ramp->input - ramp->output;
    ramp->delta = _constrain(ramp->delta,ramp->delta_min,ramp->delta_max);
    ramp->output += ramp->delta;
    ramp->output = _constrain(ramp->output,ramp->output_min,ramp->output_max);

    return ramp->output;
}

float LPF_Alpha_Cal(float fc,float dt)
{
    float rc = 1.0f / (_2PI * fc);
    float alpha = _constrain((dt / (dt + rc)),0.0f,1.0f);
    return alpha;
}

void LPF_Mult_F32(float *value,float *sample,uint32_t num,float fc,float dt)
{
    float rc = 1.0f / (_2PI * fc);
    float alpha = _constrain((dt / (dt + rc)),0.0f,1.0f);
    for(uint32_t i = 0;i < num;i++)
    {
        value[i] += (sample[i] - value[i]) * alpha;
    }
}

void LPF_F32(float *value,float sample,float fc,float dt)
{
    float rc = 1.0f / (_2PI * fc);
    float alpha = _constrain((dt / (dt + rc)),0.0f,1.0f);
    *value += (sample - *value) * alpha;
}

void LPF_UVW_F32(UVW_Axis_t *value,UVW_Axis_t *sample,float fc,float dt)
{
    float rc = 1.0f / (_2PI * fc);
    float alpha = _constrain((dt / (dt + rc)),0.0f,1.0f);

    value->U += (sample->U - value->U) * alpha;
    value->V += (sample->V - value->V) * alpha;
    value->W += (sample->W - value->W) * alpha;
}

void Inverse_LPF_Mult_F32(float *value,float *sample,float *sample_last,uint32_t num,float fc,float dt)
{
    float rc = _constrain((1.0f / (_2PI * fc)),0.0f,1.0f);
    for(uint32_t i = 0;i < num;i++)
    {
        value[i] = ((rc + dt) * sample[i] - rc * sample_last[i]) / dt;
    }
}

void Inverse_LPF_F32(float *value,float sample,float sample_last,float fc,float dt)
{
    if (fc <= 0.0f || dt <= 0.0f) 
    {
        *value = sample;
    }
    float rc = _constrain((1.0f / (_2PI * fc)),0.0f,1.0f);
    *value = ((rc + dt) * sample - rc * sample_last) / dt;
}

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

void Quick_Sort(int32_t *number, uint32_t first, uint32_t last)
{
    uint32_t i, j, pivot;
    int32_t temp;

    if (first < last)
    {
        pivot = first;
        i = first;
        j = last;
        while (i < j)
        {
            while (number[i] <= number[pivot] && i < last)
                i++;
            while (number[j] > number[pivot] && j > first)
                j--;
            if (i < j)
            {
                temp = number[i];
                number[i] = number[j];
                number[j] = temp;
            }
        }
        temp = number[pivot];
        number[pivot] = number[j];
        number[j] = temp;
        if (j >= 1)
        {
            Quick_Sort(number, first, j - 1);
        }
        Quick_Sort(number, j + 1, last);
    }
}

void Bubble_Sort_U8(uint8_t *arr,uint16_t len)
{
    uint8_t temp;

    for(uint8_t i = 0;i < len - 1;i++)
    {
        for(uint8_t j = 0;j < len - 1- i;j++) 
        {
            if(arr[j] > arr[j + 1]) 
            {
                temp = arr[j];
                arr[j] = arr[j + 1];
                arr[j + 1] = temp;
            }
        }
    }
}

void Bubble_Sort_U16(uint16_t *arr,uint16_t len)
{
    uint16_t temp;

    for(uint16_t i = 0;i < len - 1;i++)
    {
        for(uint16_t j = 0;j < len - 1- i;j++) 
        {
            if(arr[j] > arr[j + 1]) 
            {
                temp = arr[j];
                arr[j] = arr[j + 1];
                arr[j + 1] = temp;
            }
        }
    }
}

void Bubble_Sort_I32(int32_t *arr,uint16_t len)
{
    uint16_t temp;

    for(uint16_t i = 0;i < len - 1;i++)
    {
        for(uint16_t j = 0;j < len - 1- i;j++) 
        {
            if(arr[j] > arr[j + 1]) 
            {
                temp = arr[j];
                arr[j] = arr[j + 1];
                arr[j + 1] = temp;
            }
        }
    }
}

void Bubble_Sort_F32(float *arr,uint16_t len)
{
    float temp;

    for(uint16_t i = 0;i < len - 1;i++)
    {
        for(uint16_t j = 0;j < len - 1- i;j++) 
        {
            if(arr[j] > arr[j + 1]) 
            {
                temp = arr[j];
                arr[j] = arr[j + 1];
                arr[j + 1] = temp;
            }
        }
    }
}

void Index_Sort(float *arr,uint16_t len,uint16_t *index_arr)
{
    float temp,temp_index;

    for(uint16_t i = 0;i < len - 1;i++)
    {
        for(uint16_t j = 0;j < len - 1- i;j++) 
        {
            if(arr[j] > arr[j + 1]) 
            {
                temp = arr[j];
                arr[j] = arr[j + 1];
                arr[j + 1] = temp;

                temp_index = index_arr[j];
                index_arr[j] = index_arr[j + 1];
                index_arr[j + 1] = temp_index;
            }
        }
    }
}

int32_t Find_Most_Repeated_Element(int32_t arr[], uint32_t len,int32_t *res)
{
    uint32_t slow_p = 0, fast_p = 1;
    uint32_t max_cnt = 0, temp_cnt = 0;

    // Bubble_Sort_I32(arr,len);
    Quick_Sort(arr,0,len - 1);

    while (fast_p < len)
    {
        if (arr[slow_p] == arr[fast_p])
        {
            if (++fast_p == len)
            {
                temp_cnt = fast_p - slow_p;
                if (temp_cnt > max_cnt)
                {
                    max_cnt = temp_cnt;
                    *res = arr[slow_p];
                }
            }
        }
        else
        {
            temp_cnt = fast_p - slow_p;
            if (temp_cnt > max_cnt)
            {
                max_cnt = temp_cnt;
                *res = arr[slow_p];
            }
            slow_p = fast_p++;
        }
    }

    return max_cnt;
}

bool Saturate_Vector_2d(float *x, float *y, float max)
{
    bool retval = false;
    float mag = sqrtf(SQ(*x) + SQ(*y));
    max = fabsf(max);

    if (mag < 1e-10f)
    {
        mag = 1e-10f;
    }

    if (mag > max)
    {
        const float f = max / mag;
        *x *= f;
        *y *= f;
        retval = true;
    }

    return retval;
}

bool Saturate_Vector_2d_fixed(float *x, float *y, float max, uint8_t fixed_index)
{
    bool retval = false;
    float max_abs = 0;
    float temp = 0;

    switch(fixed_index)
    {
        case 1:
            max_abs = sqrtf(SQ(max) - SQ(*x));
            temp = *y;
            temp = _constrain(temp,-max_abs,max_abs);
            retval = temp != *y;
            *y = temp;
            break;

        case 2:
            max_abs = sqrtf(SQ(max) - SQ(*y));
            temp = *x;
            temp = _constrain(temp,-max_abs,max_abs);
            retval = temp != *x;
            *x = temp;
            break;

        default:
            retval = Saturate_Vector_2d(x, y, max);
            break;
    }

    return retval;
}


// limited rad to 0~2pi rads
float Normalize_Angle(float rad)
{
    float a = fmodf(rad, _2PI);
    return a >= 0 ? a : (a + _2PI);
}

// limited rad to 0~pi rads
float Normalize_Angle_PI(float rad)
{
    float a = fmodf(rad, _PI);
    return a >= 0 ? a : (a + _PI);
}

// limited deg to 0~360 degree
float Normalize_Angle_Degree(float deg)
{
    float a = fmodf(deg, 360.0f);
    return a >= 0 ? a : (a + 360.0f);
}

// limited deg to 0~180 degree
float Normalize_Angle_180Degree(float deg)
{
    float a = fmodf(deg, 180.0f);
    return a >= 0 ? a : (a + 180.0f);
}

// Calculate the difference between the two angles
float ABS_Angle_Delta(float rad0,float rad1)
{
    float rad = 0;
    float abs_delta = 0;

    rad0 = Normalize_Angle(rad0);
    rad1 = Normalize_Angle(rad1);
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

float ABS_Angle_Delta_Degree(float deg0, float deg1)
{
    float angle = 0;
    float abs_delta = 0;

    deg0 = Normalize_Angle_Degree(deg0);
    deg1 = Normalize_Angle_Degree(deg1);
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

int8_t Angle_Delta_Dir(float rad0, float rad1)
{
    float angle_delta = 0;
    int8_t dir = 0;

    rad0 = Normalize_Angle(rad0);
    rad1 = Normalize_Angle(rad1);
    angle_delta = rad0 - rad1;

    if (angle_delta < -_PI)
    {
        angle_delta += _2PI;
    }
    else if (angle_delta > _PI)
    {
        angle_delta -= _2PI;
    }

    dir = -SIGN(angle_delta);

    return dir;
}

int8_t Angle_Delta_Dir_Degree(float deg0, float deg1)
{
    float angle_delta = 0;
    int8_t dir = 0;

    deg0 = Normalize_Angle_Degree(deg0);
    deg1 = Normalize_Angle_Degree(deg1);
    angle_delta = deg0 - deg1;

    if (angle_delta < -180.0f)
    {
        angle_delta += 360.0f;
    }
    else if (angle_delta > 180.0f)
    {
        angle_delta -= 360.0f;
    }

    dir = -SIGN(angle_delta);

    return dir;
}

float Angle_Average(float rad0, float rad1)
{
    float sin_sum = sinf(rad0) + sinf(rad1);
    float cos_sum = cosf(rad0) + cosf(rad1);

    return Normalize_Angle(atan2f(sin_sum, cos_sum));
}

float Angle_Weighted_Average(float rad0, float rad1, float weight)
{
    float sin_sum = sinf(rad0) * weight + sinf(rad1) * (1.0f - weight);
    float cos_sum = cosf(rad0) * weight + cosf(rad1) * (1.0f - weight);

    return Normalize_Angle(atan2f(sin_sum, cos_sum));
}

float Angle_Average_Degree(float deg0, float deg1)
{
    deg0 = DEG_TO_RAD(deg0);
    deg1 = DEG_TO_RAD(deg1);
    float sin_sum = sinf(deg0) + sinf(deg1);
    float cos_sum = cosf(deg0) + cosf(deg1);

    return Normalize_Angle_Degree(RAD_TO_DEG(atan2f(sin_sum, cos_sum)));
}

float Angle_Weighted_Average_Degree(float deg0, float deg1, float weight)
{
    deg0 = DEG_TO_RAD(deg0);
    deg1 = DEG_TO_RAD(deg1);
    float sin_sum = sinf(deg0) * weight + sinf(deg1) * (1.0f - weight);
    float cos_sum = cosf(deg0) * weight + cosf(deg1) * (1.0f - weight);

    return Normalize_Angle_Degree(RAD_TO_DEG(atan2f(sin_sum, cos_sum)));
}

float Angle_Average_Mult(float *rads, uint32_t nums)
{
    float sin_sum = 0.0;
    float cos_sum = 0.0;

    for (uint32_t i = 0; i < nums; i++)
    {
        sin_sum += sinf(rads[i]);
        cos_sum += cosf(rads[i]);
    }

    return Normalize_Angle(atan2f(sin_sum, cos_sum));
}

float Angle_Weighted_Average_Mult(float *rads, float *weight, uint32_t nums)
{
    float sin_sum = 0.0;
    float cos_sum = 0.0;

    for (uint32_t i = 0; i < nums; i++)
    {
        sin_sum += sinf(rads[i]) * weight[i];
        cos_sum += cosf(rads[i]) * weight[i];
    }

    return Normalize_Angle(atan2f(sin_sum, cos_sum));
}

float Angle_Average_Degree_Mult(float *degs, uint32_t nums)
{
    float sin_sum = 0.0;
    float cos_sum = 0.0;

    for (uint32_t i = 0; i < nums; i++)
    {
        sin_sum += sinf(DEG_TO_RAD(degs[i]));
        cos_sum += cosf(DEG_TO_RAD(degs[i]));
    }

    return Normalize_Angle_Degree(RAD_TO_DEG(atan2f(sin_sum, cos_sum)));
}

float Angle_Weighted_Average_Degree_Mult(float *degs, float *weight, uint32_t nums)
{
    float sin_sum = 0.0;
    float cos_sum = 0.0;

    for (uint32_t i = 0; i < nums; i++)
    {
        sin_sum += sinf(DEG_TO_RAD(degs[i])) * weight[i];
        cos_sum += cosf(DEG_TO_RAD(degs[i])) * weight[i];
    }

    return Normalize_Angle_Degree(RAD_TO_DEG(atan2f(sin_sum, cos_sum)));
}

