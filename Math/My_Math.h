#ifndef __MY_MATH_H__
#define __MY_MATH_H__

#include "stdbool.h"
#include "stdint.h"

/*
sample:input
value:output
filter_constant:filter constant
fc = a / (2 * pi * t)
*/
#define FirstOrder_LPF_Cacl(sample, value, filter_constant)         ((value) = (1.0f-filter_constant) * (value) + (filter_constant) * (sample))
#define FirstOrder_LPF_Cacl1(sample, value, filter_constant)	    ((value) -= (filter_constant) * ((value) - (sample)))

#define _sign(a)                                (((a) < 0) ? (-1) : ((a) > 0))
#define _round(x)                               ((x) >= 0 ? (long)((x) + 0.5f) : (long)((x)-0.5f))
#define _constrain(amt, low, high)              ((amt) < (low) ? (low) : ((amt) > (high) ? (high) : (amt)))
#define _isset(a)                               ((a) != (NOT_SET))
#define SQ(x)                                   ((x) * (x))
#define NORM2_f(x, y)                           (sqrtf(SQ(x) + SQ(y)))
#define IS_INF(x)                               ((x) == (1.0f / 0.0f) || (x) == (-1.0f / 0.0f))
#define IS_NAN(x)                               ((x) != (x))
#define NAN_ZERO(x)                             (x = IS_NAN(x) ? 0.0f : x)
#define SIGN(x)                                 (((x) < 0.0f) ? -1.0f : 1.0f)
#define MIN(a, b)                               (((a) < (b)) ? (a) : (b))
#define MAX(a, b)                               (((a) > (b)) ? (a) : (b))

#define DEG_TO_RAD(x)   (x / 57.295779f)
#define RAD_TO_DEG(x)   (x * 57.295779f)

#define _1_3            0.33333333333f
#define _2_3            0.66666666666f

#define _SQRT3          1.73205080757f
#define _2_SQRT3        1.15470053838f
#define _1_SQRT3        0.57735026919f
#define _SQRT3_2        0.86602540378f
#define _SQRT2          1.41421356237f

#define _PI_6           0.52359877559f
#define _PI_3           1.04719755119f
#define _PI_2           1.57079632679f
#define _2PI_3          2.09439510239f
#define _5PI_6          2.61799387799f
#define _PI             3.14159265359f
#define _7PI_6          3.66519142918f
#define _3PI_2          4.71238898038f
#define _11PI_6         5.75958653158f
#define _2PI            6.28318530718f

#define _e              2.71828182845f


typedef struct My_Math
{
    float input;
    float output;
    float output_max;
    float output_min;
    float rise_time_ms;
    float fall_time_ms;
    float dt;
    float delta;
    float delta_max;
    float delta_min;
}Ramp_t;

void Ramp_Init(Ramp_t *ramp,float in,float out,float output_max,float output_min,float rise_time_ms,float fall_time_ms,float dt);
float Ramp_Update(Ramp_t *ramp,float in);
void LPF_Mult_F32(float *value,float *sample,uint32_t num,float fc,float ts);
void LPF_F32(float *value,float sample,float fc,float ts);
void Inverse_LPF_F32(float *value,float sample,float sample_last,float fc,float ts);
void Inverse_LPF_Mult_F32(float *value,float *sample,float *sample_last,uint32_t num,float fc,float ts);
float Max_Abs(float va, float vb);
float Min_Abs(float va, float vb);
void Truncate_Number(float *number, float min, float max);
void Truncate_Number_Abs(float *number, float max);
uint8_t Number_In_Absolute_Range_f32(float num,float base,float range);
void quickSort(int32_t *number,uint32_t first,uint32_t last);
void bubbleSort_u8(uint8_t *arr,uint16_t len);
void bubbleSort_u16(uint16_t *arr,uint16_t len);
void bubbleSort_i32(int32_t *arr,uint16_t len);
void bubbleSort_f32(float *arr,uint16_t len);
void Index_Sort(float *arr,uint16_t len,uint16_t *index_arr);
int32_t Find_Most_Repeated_Element(int32_t arr[], uint32_t len,int32_t *res);

bool saturate_vector_2d(float *x, float *y, float max);
float Normalize_Angle(float angle);
float Normalize_Angle_PI(float angle);
float Normalize_Angle_Degree(float angle);
float Normalize_Angle_180Degree(float angle);
float ABS_Angle_Delta(float angle0,float angle1);
float ABS_Angle_Delta_Degree(float angle0, float angle1);

#endif
