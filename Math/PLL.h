#ifndef __PLL_H__
#define __PLL_H__

#include "math.h"
#include "stdint.h"
#include "stdbool.h"
#include "string.h"

typedef struct
{
    float Kp;
    float Ki;
    float err;
    float integral;
    float out;
    float fc;
    float dt;
    float alpha;
    float hz;
    float hz_f;
    float theta;
    float theta_unlimit;
} PLL_t;

void PLL_Init(PLL_t *pPLL,float Kp,float Ki,float fc,float dt);
void PLL_Run(PLL_t *pPLL,float sin_value,float cos_value);


#endif
