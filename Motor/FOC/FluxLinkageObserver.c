#include "FluxLinkageObserver.h"
#include "My_Math.h"
#include "FOC.h"
#include "PID.h"

FLO_t flo_observer;

void FLO_Init(FOC_CONTROL_t *ctrl,FLO_t *obs,float speed_fc)
{
    memset(obs, 0, sizeof(FLO_t));
    obs->motor_type = ctrl->motor_type;
    obs->rs_ohm = ctrl->Rs;
    obs->ls_H = ctrl->Ls;
    obs->l_diff = ctrl->Ldiff / 2;
    obs->flux_wb = ctrl->Flux;
    obs->dt = ctrl->current_loop_dt;

    #ifdef MOTOR_2PP_SERVO
    obs->gain = 50000;
    PLL_Init(&obs->pll,0.1f,0.1f,speed_fc,ctrl->current_loop_dt);
    #endif

    #ifdef MOTOR_14PP_BLDC
    obs->gain = 50000000;
    PLL_Init(&obs->pll,0.3f,0.1f,speed_fc,ctrl->current_loop_dt);
    #endif

    #ifdef MOTOR_1PP_BLDC_HALL
    obs->gain = 500000000;
    PLL_Init(&obs->pll,0.3f,0.1f,speed_fc,ctrl->current_loop_dt);
    #endif

    #ifdef MOTOR_1PP_BLDC_FAN_SLOW
    obs->gain = 1e9;
    PLL_Init(&obs->pll,0.3f,0.1f,speed_fc,ctrl->current_loop_dt);
    #endif
}

void FLO_Run(FLO_t *obs,FOC_Para_t *para)
{
    /*
        U = R * I + φ
        φ = φs + φr                     ->  s = stator  r = rotor
        φs = L * I
        so
        φr_est = φ - φs                 ->  φr_est is the estimate value
        φr_err = φr_real - φr_est       ->  φr_real is known as obs->flux_wb    
        φ += (U - R * I + φr_est * φr_err) * dt
    */

    // cal resistance voltage drop
    obs->R_I.Alpha = obs->rs_ohm * para->I_alpha_beta.Alpha;
    obs->R_I.Beta = obs->rs_ohm * para->I_alpha_beta.Beta;
    // cal flux
    obs->Flux_Sum.Alpha += (para->V_alpha_beta.Alpha - obs->R_I.Alpha + obs->Flux_Rotor.Alpha * obs->flux_r_err * obs->gain) * obs->dt;
    obs->Flux_Sum.Beta += (para->V_alpha_beta.Beta - obs->R_I.Beta + obs->Flux_Rotor.Beta * obs->flux_r_err * obs->gain) * obs->dt;
    // cal stator flux
    if(obs->motor_type == IPM)
    {
        // https://zhuanlan.zhihu.com/p/259856568 for IPM
        float l_diff = obs->l_diff * cosf(obs->pll.theta * 2);
        AB_Axis_t l;
        l.Alpha = obs->ls_H + l_diff;
        l.Beta = obs->ls_H - l_diff;
        float l_ab = obs->l_diff * sinf(obs->pll.theta * 2);
        obs->Flux_Stator.Alpha = l.Alpha * para->I_alpha_beta.Alpha + l_ab * para->I_alpha_beta.Beta;
        obs->Flux_Stator.Beta = l.Beta * para->I_alpha_beta.Beta + l_ab * para->I_alpha_beta.Alpha;
    }
    else
    {
        obs->Flux_Stator.Alpha = obs->ls_H * para->I_alpha_beta.Alpha;
        obs->Flux_Stator.Beta = obs->ls_H * para->I_alpha_beta.Beta;
    }
    // cal rotor flux
    obs->Flux_Rotor.Alpha = obs->Flux_Sum.Alpha - obs->Flux_Stator.Alpha;
    obs->Flux_Rotor.Beta = obs->Flux_Sum.Beta - obs->Flux_Stator.Beta;
    // cal rotor flux error
    obs->flux_r_err = SQ(obs->flux_wb) - (SQ(obs->Flux_Rotor.Alpha) + SQ(obs->Flux_Rotor.Beta));
    
    // // cal arctan theta
    // obs->Theta = Normalize_Angle(atan2f(obs->Flux_Rotor.Beta,obs->Flux_Rotor.Alpha));

    // see https://zhuanlan.zhihu.com/p/652503676
    // PLL_Run(&obs->pll,obs->Flux_Rotor.Beta,obs->Flux_Rotor.Alpha);
    Normalize_PLL_Run(&obs->pll,obs->Flux_Rotor.Beta,obs->Flux_Rotor.Alpha);

    // observer optput
    obs->E_ang = obs->pll.theta;
    obs->E_rps = obs->pll.hz_f;
    obs->E_rpm = obs->E_rps * 60;
}
