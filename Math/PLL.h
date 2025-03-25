#ifndef __PLL_H__
#define __PLL_H__

#include "math.h"
#include "stdint.h"
#include "stdbool.h"
#include "string.h"
#include "Filter.h"


typedef struct
{
    float Kp;
    float Ki;
    float err;
    float integral;
    float fc;
    float dt;
    float wc;// rads/s
    float wc_f;// rads/s
    float hz;
    float hz_f;
    float theta;
    float theta_unlimit;

    float alpha;
    IIR_Filter_t *p_iir;

} PLL_t;

void PLL_Init(PLL_t *pPLL,float Kp,float Ki,float fc,float dt,IIR_Filter_t *p_iir);
void PLL_Run(PLL_t *pPLL,float sin_value,float cos_value);
void Normalize_PLL_Run(PLL_t *pPLL,float sin_value,float cos_value);

#endif
