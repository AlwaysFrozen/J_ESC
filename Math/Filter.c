#include "Filter.h"

float LPF_Alpha_Cal(float fc,float dt)
{
    float rc = 1.0f / (_2PI * fc);
    float alpha = CONSTRAIN((dt / (dt + rc)),0.0f,1.0f);
    return alpha;
}

void LPF_Mult_F32(float *value,float *sample,uint32_t num,float fc,float dt)
{
    float rc = 1.0f / (_2PI * fc);
    float alpha = CONSTRAIN((dt / (dt + rc)),0.0f,1.0f);
    for(uint32_t i = 0;i < num;i++)
    {
        value[i] += (sample[i] - value[i]) * alpha;
    }
}

void LPF_F32(float *value,float sample,float fc,float dt)
{
    float rc = 1.0f / (_2PI * fc);
    float alpha = CONSTRAIN((dt / (dt + rc)),0.0f,1.0f);
    *value += (sample - *value) * alpha;
}

void LPF_UVW_F32(UVW_Axis_t *value,UVW_Axis_t *sample,float fc,float dt)
{
    float rc = 1.0f / (_2PI * fc);
    float alpha = CONSTRAIN((dt / (dt + rc)),0.0f,1.0f);

    value->U += (sample->U - value->U) * alpha;
    value->V += (sample->V - value->V) * alpha;
    value->W += (sample->W - value->W) * alpha;
}

void Inverse_LPF_Mult_F32(float *value,float *sample,float *sample_last,uint32_t num,float fc,float dt)
{
    float rc = CONSTRAIN((1.0f / (_2PI * fc)),0.0f,1.0f);
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
    float rc = CONSTRAIN((1.0f / (_2PI * fc)),0.0f,1.0f);
    *value = ((rc + dt) * sample - rc * sample_last) / dt;
}

uint8_t Init_IIR_Filter(IIR_Filter_t *filter,uint8_t order,float dt,float fc,float *a,float *b)
{
    if(order > IIR_FILTER_MAX_ORDER)
    {
        return 0;
    }

    filter->order = order;
    filter->dt = dt;
    filter->fc = fc;
    memset(filter->a,0,sizeof(filter->a));
    memset(filter->b,0,sizeof(filter->b));
    memset(filter->x,0,sizeof(filter->x));
    memset(filter->y,0,sizeof(filter->y));

    for (uint8_t i = 0; i <= filter->order; i++) 
    {
        filter->a[i] = a[i];
        filter->b[i] = b[i];
        filter->x[i] = 0.0;
        filter->y[i] = 0.0;
    }

    return 1;
}

float IIR_Filter_Update(IIR_Filter_t *filter,float x)
{
    for (uint8_t i = filter->order; i > 0; i--) 
    {
        filter->x[i] = filter->x[i - 1];
        filter->y[i] = filter->y[i - 1];
    }
    filter->x[0] = x;

    float output = 0.0;
    for (uint8_t i = 0; i <= filter->order; i++) 
    {
        output += filter->b[i] * filter->x[i];
    }
    for (uint8_t i = 1; i <= filter->order; i++) 
    {
        output -= filter->a[i] * filter->y[i];
    }

    filter->y[0] = output;
    return output;
}

uint8_t Init_FIR_Filter(FIR_Filter_t *filter,uint8_t order,float dt,float fc,float *b)
{
    if(order > FIR_FILTER_MAX_ORDER)
    {
        return 0;
    }

    filter->order = order;
    filter->dt = dt;
    filter->fc = fc;
    memset(filter->b,0,sizeof(filter->b));
    memset(filter->x,0,sizeof(filter->x));

    for (int i = 0; i <= filter->order; i++) 
    {
        filter->b[i] = b[i];
        filter->x[i] = 0.0;
    }

    return 1;
}

float FIR_Filter_Update(FIR_Filter_t *filter,float x)
{
    for (uint8_t i = filter->order; i > 0; i--) 
    {
        filter->x[i] = filter->x[i - 1];
    }
    filter->x[0] = x;

    float output = 0.0;
    for (uint8_t i = 0; i <= filter->order; i++) 
    {
        output += filter->b[i] * filter->x[i];
    }

    return output;
}
