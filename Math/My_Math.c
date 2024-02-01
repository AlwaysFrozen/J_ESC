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

void LPF_Mult_F32(float *value,float *sample,uint32_t num,float fc,float ts)
{
    float t = 1.0f / (_2PI * fc);
    float alpha = _constrain((ts / (ts + t)),0.0f,1.0f);
    for(uint32_t i = 0;i < num;i++)
    {
        value[i] += (sample[i] - value[i]) * alpha;
    }
}

void LPF_F32(float *value,float sample,float fc,float ts)
{
    float t = 1.0f / (_2PI * fc);
    float alpha = _constrain((ts / (ts + t)),0.0f,1.0f);
    *value += (sample - *value) * alpha;
}

void Inverse_LPF_Mult_F32(float *value,float *sample,float *sample_last,uint32_t num,float fc,float ts)
{
    float t = _constrain((1.0f / (_2PI * fc)),0.0f,1.0f);
    for(uint32_t i = 0;i < num;i++)
    {
        value[i] = ((t + ts) * sample[i] - t * sample_last[i]) / ts;
    }
}

void Inverse_LPF_F32(float *value,float sample,float sample_last,float fc,float ts)
{
    if (fc <= 0.0f || ts <= 0.0f) 
    {
        *value = sample;
    }
    float t = _constrain((1.0f / (_2PI * fc)),0.0f,1.0f);
    *value = ((t + ts) * sample - t * sample_last) / ts;
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

void quickSort(int32_t *number, uint32_t first, uint32_t last)
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
            quickSort(number, first, j - 1);
        }
        quickSort(number, j + 1, last);
    }
}

void bubbleSort_u8(uint8_t *arr,uint16_t len)
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

void bubbleSort_u16(uint16_t *arr,uint16_t len)
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

void bubbleSort_i32(int32_t *arr,uint16_t len)
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

void bubbleSort_f32(float *arr,uint16_t len)
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

    // bubbleSort_i32(arr,len);
    quickSort(arr,0,len - 1);

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

bool saturate_vector_2d(float *x, float *y, float max)
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

// limited angle to 0~2pi rads
float Normalize_Angle(float angle)
{
    float a = fmodf(angle, _2PI);
    return a >= 0 ? a : (a + _2PI);
}

// limited angle to 0~pi rads
float Normalize_Angle_PI(float angle)
{
    float a = fmodf(angle, _PI);
    return a >= 0 ? a : (a + _PI);
}

// limited angle to 0~360 degree
float Normalize_Angle_Degree(float angle)
{
    float a = fmodf(angle, 360.0f);
    return a >= 0 ? a : (a + 360.0f);
}

// limited angle to 0~180 degree
float Normalize_Angle_180Degree(float angle)
{
    float a = fmodf(angle, 180.0f);
    return a >= 0 ? a : (a + 180.0f);
}

// Calculate the difference between the two angles
float ABS_Angle_Delta(float angle0,float angle1)
{
    float angle = 0;
    float abs_delta = 0;

    angle0 = Normalize_Angle(angle0);
    angle1 = Normalize_Angle(angle1);
    abs_delta = fabsf(angle0 - angle1);

    if (abs_delta < _PI)
    {
        angle = abs_delta;
    }
    else
    {
        angle = _2PI - abs_delta;
    }

    return angle;
}

float ABS_Angle_Delta_Degree(float angle0, float angle1)
{
    float angle = 0;
    float abs_delta = 0;

    angle0 = Normalize_Angle_Degree(angle0);
    angle1 = Normalize_Angle_Degree(angle1);
    abs_delta = fabsf(angle0 - angle1);

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
