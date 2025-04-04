#ifndef __FLUX_OBSERVER_H__
#define __FLUX_OBSERVER_H__

#include "stdbool.h"
#include "stdint.h"
#include "math.h"
#include "FOC_Config.h"
#include "FOC_Motor.h"
#include "PLL.h"

typedef struct _OBSERVER_Obj_
{
    FOC_Motor_Type_t motor_type;
    
    float rs_ohm;
    float ls_H;
    float l_diff_half;
    float flux_wb;
    float dt;
    float gain;

    AB_Axis_t R_I;
    AB_Axis_t Flux_Sum;
    AB_Axis_t Flux_Stator;
    AB_Axis_t Flux_Rotor;
    float flux_r_err;
    float Theta;
    PLL_t pll;

    float E_ang;
    float E_rps;
    float E_rpm;
} NFO_t;

void NFO_Init(FOC_CONTROL_t *ctrl,NFO_t *obs,float speed_fc,IIR_Filter_t *p_iir);
void NFO_Run(NFO_t *obs,FOC_Para_t *para);
#endif
