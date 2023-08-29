#include "My_Math.h"
#include "math.h"
#include "FOC.h"

void Ramp_Init(Ramp_t *ramp,float in,float out,float slope_up,float slope_down,float ts)
{
    ramp->input = in;
    ramp->output = out;
    ramp->slope_up = slope_up;
    ramp->slope_down = slope_down;
    ramp->ts = ts;
    ramp->up_lim = ramp->slope_up * ramp->ts;
    ramp->down_lim = ramp->slope_down * ramp->ts;
}

void Ramp(Ramp_t *ramp)
{
    ramp->delta = ramp->input - ramp->output;

    if(ramp->delta > ramp->up_lim)
    {
        ramp->delta = ramp->up_lim;
    }
    else if(ramp->delta < ramp->down_lim)
    {
        ramp->delta = ramp->down_lim;
    }

    ramp->output += ramp->delta;
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
    *value += sample - *value * alpha;
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

void quickSort(uint32_t *number,uint32_t first,uint32_t last)
{
    uint32_t i,j,pivot,temp;

    if(first < last)
    {
        pivot = first;
        i = first;
        j = last;
        while(i < j) 
        {
            while(number[i] <= number[pivot] && i < last)
                i++;
            while(number[j] > number[pivot])
                j--;
            if(i < j)
            {
                temp = number[i];
                number[i] = number[j];
                number[j] = temp;
            }
        }
        temp = number[pivot];
        number[pivot] = number[j];
        number[j] = temp;
        quickSort(number, first, j - 1);
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
