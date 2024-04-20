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
    #endif

    #ifdef MOTOR_14PP_BLDC
    obs->gain = 50000000;
    #endif

    PLL_Init(&obs->pll,0.1f,0.1f,speed_fc,dt);
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
    #if IS_IPM
    // https://zhuanlan.zhihu.com/p/259856568 for IPM
    float l_delta = (Ld_H - Lq_H) / 2;
    float l_diff = l_delta * cosf(obs->pll.theta * 2);
    float l_a = obs->ls_H + l_diff;
    float l_b = obs->ls_H - l_diff;
    float l_ab = l_delta * sinf(obs->pll.theta * 2);
    obs->flux_s_a = l_a * foc_para->Ia + l_ab * foc_para->Ib;
    obs->flux_s_b = l_b * foc_para->Ib + l_ab * foc_para->Ia;
    #else
    obs->flux_s_a = obs->ls_H * foc_para->Ia;
    obs->flux_s_b = obs->ls_H * foc_para->Ib;
    #endif
    // cal rotor flux
    obs->flux_r_a = obs->flux_a - obs->flux_s_a;
    obs->flux_r_b = obs->flux_b - obs->flux_s_b;
    // cal rotor flux error
    obs->flux_r_err = SQ(obs->flux_wb) - (SQ(obs->flux_r_a) + SQ(obs->flux_r_b));
    
    // // cal arctan theta
    // obs->Theta = Normalize_Angle(atan2f(obs->flux_r_b,obs->flux_r_a));

    // see https://zhuanlan.zhihu.com/p/652503676
    // PLL_Run(&obs->pll,obs->flux_r_b,obs->flux_r_a);
    Normalize_PLL_Run(&obs->pll,obs->flux_r_b,obs->flux_r_a);

    // observer optput
    obs->E_ang = obs->pll.theta;
    obs->E_rps = obs->pll.hz_f;
    obs->E_rpm = obs->E_rps * 60;
}
