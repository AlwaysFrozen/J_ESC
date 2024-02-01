#include "FluxLinkageObserver.h"
#include "My_Math.h"
#include "FOC.h"
#include "PID.h"

FLO_t flo_observer;

void FLO_Init(FLO_t *obs,float speed_fc,float dt)
{
    memset(obs, 0, sizeof(FLO_t));
    obs->rs_ohm = Rs_R;
    obs->ls_H = Ls_H;
    obs->flux_wb = FLUX_Wb;
    obs->dt = dt;

    #ifdef MOTOR_2PP_SERVO
    obs->gain = 50000;
    PLL_Init(&obs->pll,50,100,speed_fc,dt);
    #endif

    #ifdef MOTOR_14PP_BLDC
    obs->gain = 50000000;
    PLL_Init(&obs->pll,100,100,speed_fc,dt);
    #endif
}

void FLO_Run(FLO_t *obs,FOC_Para_t *foc_para)
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
    obs->R_I_a = obs->rs_ohm * foc_para->Ia;
    obs->R_I_b = obs->rs_ohm * foc_para->Ib;
    // cal flux
    obs->flux_a += (foc_para->Ua - obs->R_I_a + obs->flux_r_a * obs->flux_r_err * obs->gain) * obs->dt;
    obs->flux_b += (foc_para->Ub - obs->R_I_b + obs->flux_r_b * obs->flux_r_err * obs->gain) * obs->dt;
    // cal stator flux
    obs->flux_s_a = obs->ls_H * foc_para->Ia;
    obs->flux_s_b = obs->ls_H * foc_para->Ib;
    // cal rotor flux
    obs->flux_r_a = obs->flux_a - obs->flux_s_a;
    obs->flux_r_b = obs->flux_b - obs->flux_s_b;
    // cal rotor flux error
    obs->flux_r_err = SQ(obs->flux_wb) - (SQ(obs->flux_r_a) + SQ(obs->flux_r_b));
    
    // // cal arctan theta
    // obs->Theta = Normalize_Angle(atan2f(obs->flux_r_b,obs->flux_r_a));

    // see https://zhuanlan.zhihu.com/p/652503676
    PLL_Run(&obs->pll,obs->flux_r_b,obs->flux_r_a);

    // observer optput
    obs->E_ang = obs->pll.theta;
    obs->E_rps = obs->pll.hz_f;
    obs->E_rpm = obs->E_rps * 60;
}
