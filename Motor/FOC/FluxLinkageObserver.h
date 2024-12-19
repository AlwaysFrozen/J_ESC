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
    float l_diff;
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
} FLO_t;

extern FLO_t flo_observer;


void FLO_Init(FOC_CONTROL_t *ctrl,FLO_t *obs,float speed_fc);
void FLO_Run(FLO_t *obs,FOC_Para_t *para);
#endif
