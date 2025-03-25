#ifndef __FILTER_H__
#define __FILTER_H__

#include "Utils.h"
#include "Motor.h"

/*
sample:input
value:output
filter_constant:filter constant
fc = a / (2 * pi * dt)
a = fc * 2 * pi * dt

a = (fc * 2 * pi * dt) / ((fc * 2 * pi * dt) + 1)
*/

#define FirstOrder_LPF_Cal(sample, value, filter_constant)         ((value) = (1.0f-filter_constant) * (value) + (filter_constant) * (sample))

float LPF_Alpha_Cal(float fc,float dt);
void LPF_Mult_F32(float *value,float *sample,uint32_t num,float fc,float dt);
void LPF_F32(float *value,float sample,float fc,float dt);
void LPF_UVW_F32(UVW_Axis_t *value,UVW_Axis_t *sample,float fc,float dt);
void Inverse_LPF_F32(float *value,float sample,float sample_last,float fc,float dt);
void Inverse_LPF_Mult_F32(float *value,float *sample,float *sample_last,uint32_t num,float fc,float dt);

#define IIR_FILTER_MAX_ORDER        (4)
#define FIR_FILTER_MAX_ORDER        (8)

typedef struct
{
    uint8_t order;
    float dt;
    float fc;
    float a[IIR_FILTER_MAX_ORDER + 1];
    float b[IIR_FILTER_MAX_ORDER + 1];
    float x[IIR_FILTER_MAX_ORDER + 1];
    float y[IIR_FILTER_MAX_ORDER + 1];
}IIR_Filter_t;

typedef struct
{
    uint8_t order;
    float dt;
    float fc;
    float b[FIR_FILTER_MAX_ORDER + 1];
    float x[FIR_FILTER_MAX_ORDER + 1];
}FIR_Filter_t;

uint8_t Init_IIR_Filter(IIR_Filter_t *filter,uint8_t order,float dt,float fc,float *a,float *b);
float IIR_Filter_Update(IIR_Filter_t *filter,float x);
uint8_t Init_FIR_Filter(FIR_Filter_t *filter,uint8_t order,float dt,float fc,float *b);
float FIR_Filter_Update(FIR_Filter_t *filter,float x);

#endif
