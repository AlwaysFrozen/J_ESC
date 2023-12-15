#include "FluxLinkageObserver.h"
#include "My_Math.h"
#include "FOC.h"
#include "PID.h"

FLO_t flo_observer;

void FLO_Init(FLO_t *obs)
{
    memset(obs, 0, sizeof(FLO_t));
    obs->rs_ohm = PhaseRes_R;
    obs->ls_H = PhaseInd_H;
    obs->flux_wb = FLUX_Wb;
    obs->dt = 1.0f / FOC_CC_LOOP_FREQ;

    #ifdef MOTOR_2PP_SERVO
    obs->gain = 500000;
    obs->pll.Kp = 0.05f;
    obs->pll.Ki = 100.0f;
    #endif

    #ifdef MOTOR_14PP_BLDC
    obs->gain = 50000000;
    obs->pll.Kp = 100.0f;
    obs->pll.Ki = 100.0f;
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
    // cal arctan theta
    obs->Theta = Normalize_Angle(atan2f(obs->flux_r_b,obs->flux_r_a));
    // see https://zhuanlan.zhihu.com/p/652503676
    // cal pll speed
    obs->pll.err = obs->flux_r_b * cosf(obs->pll.theta) - obs->flux_r_a * sinf(obs->pll.theta);
    obs->pll.Interg += obs->pll.err * obs->pll.Ki * obs->dt;
    obs->pll.Ui = obs->pll.err * obs->pll.Kp + obs->pll.Interg;
    obs->pll.speed_hz = obs->pll.Ui / (obs->dt * _2PI);
    FirstOrder_LPF_Cacl(obs->pll.speed_hz, obs->pll.speed_hz_f, 0.003f);
    // cal pll theta
    obs->pll.theta += obs->pll.Ui;
    obs->pll.theta = Normalize_Angle(obs->pll.theta);

    // observer optput
    obs->E_ang = obs->pll.theta;
    obs->E_rps = obs->pll.speed_hz_f;
    obs->E_rpm = obs->E_rps * 60;
}
