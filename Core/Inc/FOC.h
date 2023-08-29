#ifndef __FOC_H__
#define __FOC_H__

#include "stdint.h"
#include "stdbool.h"
#include "math.h"
#include "My_Math.h"
#include "FOC_Config.h"
#include "FOC_Motor.h"

bool saturate_vector_2d(float *x, float *y, float max);
float Normalize_Angle(float angle);
float Normalize_Angle_PI(float angle);
float Normalize_Angle_Degree(float angle);
float Normalize_Angle_180Degree(float angle);
float ABS_Angle_Delta(float angle0,float angle1);
float ABS_Angle_Delta_Degree(float angle0, float angle1);
void Clarke_Transmission(float a,float b,float c,float *alpha,float *beta);
void Inverse_Clarke_Transmission(float alpha,float beta,float *a,float *b,float *c);
void Park_Transmission(float alpha,float beta,float *d,float *q,float theta);
void Inverse_Park_Transmission(float d,float q,float *alpha,float *beta,float theta);
void SVPWM_AB(float v_alpha_rate,float v_beta_rate,uint8_t *sector,uint16_t period,int32_t *CMP1,int32_t *CMP2,int32_t *CMP3);
void SVPWM_AB_Voltage(float v_alpha, float v_beta, uint32_t v_dc,uint8_t *sector, uint16_t period, int32_t *CMP1, int32_t *CMP2, int32_t *CMP3);
void SVPWM_DQ(float Ud_rate,float Uq_rate,float angle,uint8_t *sector,uint16_t period,int32_t *CMP1,int32_t *CMP2,int32_t *CMP3);

void MTPA_Cal(FOC_Para_t * foc_para,float flux_wb,float Ld,float Lq);
void FOC_Pll_Run(float ang, float *ang_last,float *speed,float dt,float kp,float ki);


#endif
