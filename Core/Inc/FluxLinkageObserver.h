#ifndef __FLUX_OBSERVER_H__
#define __FLUX_OBSERVER_H__

#include "stdbool.h"
#include "stdint.h"
#include "math.h"
#include "FOC_Config.h"
#include "FOC_Motor.h"

typedef struct
{
    float value[2];
} MATH_vec2;


typedef struct _PLL_Obj_
{
    float Kp;               //!< the proportional gain for the PI controller
    float Ki;               //!< the integral gain for the PI controller
    float Interg;           //!< the integrator start value for the PI
    float Ui;               //!< the integrator start value for the PI controller
    float err;              //!< the err input value
    float speed_hz;
    float speed_hz_f;
    float theta;
} PLL_Obj;

typedef struct _OBSERVER_Obj_
{
    float rs_ohm;
    float ls_H;
    float flux_wb;
    float dt;
    float gain;

    float R_I_a;
    float R_I_b;
    float flux_a;
    float flux_b;
    float flux_s_a;
    float flux_s_b;
    float flux_r_a;
    float flux_r_b;
    float flux_r_err;
    float Theta;
    PLL_Obj pll;

    float E_ang;
    float E_rps;
    float E_rpm;
} FLO_t;

extern FLO_t flo_observer;


void FLO_Init(FLO_t *obs);
void FLO_Run(FLO_t *obs,FOC_Para_t *foc_para);
#endif
