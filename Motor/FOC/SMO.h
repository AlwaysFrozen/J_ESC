#ifndef __SMO_H__
#define __SMO_H__

#include "stdbool.h"
#include "stdint.h"
#include "math.h"
#include "My_Math.h"
#include "FOC_Config.h"
#include "FOC_Motor.h"
#include "PLL.h"

#define SMO_USE_ARCTAN          1
#define SMO_USE_PLL             0

typedef struct SMO
{
    FOC_Motor_Type_t motor_type;

    float rs_ohm;
    float ls_H;
    float Ld;
    float Lq;
    float l_diff;
    float flux_wb;
    float dt;

    float Gsmopos;       // Parameter: Motor dependent control gain
    float Fsmopos;       // Parameter: Motor dependent plant matrix
    float Gsmopos_DAxis; // Parameter: Motor dependent control gain
    float Fsmopos_DAxis; // Parameter: Motor dependent plant matrix
    float Kslide;        // Parameter: Sliding control gain
    float MaxErr;        // Max current observer error
    float KslfMin;
    float ThetaOffset; // Output: Offset used to compensate rotor angle
    float SpeedFilter;

    AB_Axis_t V_alpha_beta;
    AB_Axis_t I_alpha_beta;
    AB_Axis_t I_alpha_beta_estimated;
    AB_Axis_t I_alpha_beta_error;
    AB_Axis_t Z_alpha_beta;
    AB_Axis_t E_alpha_beta;
    AB_Axis_t E_alpha_beta_final;

    float Kslf;      // Parameter: Sliding control filter gain
    float Theta;     // Output: Compensated rotor angle
    float PrevTheta;
    float AccumTheta;
    float DeltaTheta;
    uint16_t AccumThetaCnt;
    float Omega;       // Output: Rotor speed
    float OmegaFltred; // Output: Filtered Rotor speed for speed PI
    float erps;

    PLL_t pll;

    float E_ang;
    float E_rps;
    float E_rpm;
} SMO_t;

void SMO_Init(FOC_CONTROL_t *ctrl,SMO_t *s,float speed_fc,IIR_Filter_t *p_iir);
void SMO_Run(SMO_t *s, FOC_Para_t *para);

#endif
