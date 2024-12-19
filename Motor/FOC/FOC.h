#ifndef __FOC_H__
#define __FOC_H__

#include "stdint.h"
#include "stdbool.h"
#include "math.h"
#include "My_Math.h"
#include "FOC_Config.h"
#include "FOC_Motor.h"

void Clarke_Transmission(UVW_Axis_t *uvw, AB_Axis_t *ab);
void Clarke_Transmission_2(UVW_Axis_t *uvw, AB_Axis_t *ab);
void Inverse_Clarke_Transmission(AB_Axis_t *ab, UVW_Axis_t *uvw);
void Park_Transmission(AB_Axis_t *ab, DQ_Axis_t *dq, float theta);
void Inverse_Park_Transmission(DQ_Axis_t *dq, AB_Axis_t *ab, float theta);
uint8_t SVPWM_AB(AB_Axis_t *v_ab_rate, TIM_t *tim);
uint8_t SVPWM_AB_Voltage(AB_Axis_t *v_ab, float us_max, TIM_t *tim);
uint8_t SVPWM_DQ(DQ_Axis_t *v_dq_rate, float angle, TIM_t *tim);
uint8_t SVPWM_DQ_Voltage(DQ_Axis_t *v_dq, float us_max, float angle, TIM_t *tim);
void SPWM_AB(AB_Axis_t *v_ab_rate, TIM_t *tim);
void SPWM_AB_Voltage(AB_Axis_t *v_ab, uint32_t v_dc, TIM_t *tim);
void MTPA_Cal(FOC_CONTROL_t *ctrl,FOC_Para_t * para);


#endif
