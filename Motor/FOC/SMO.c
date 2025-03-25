#include "SMO.h"
#include "My_Math.h"
#include "FOC.h"

void SMO_Init(FOC_CONTROL_t *ctrl,SMO_t *s,float speed_fc,IIR_Filter_t *p_iir)
{
    //                R * Ts
    // Fsmopos = 1 - --------
    //                  L
    //            Ts
    // Gsmopos = ----
    //            L
    // Ts = Sampling Period. If sampling at PWM, Ts = 50 us
    // R = Phase Resistance. If not provided by motor datasheet,
    //     measure phase to phase resistance with multimeter, and
    //     divide over two to get phase resistance. If 4 Ohms are
    //     measured from phase to phase, then R = 2 Ohms
    // L = Phase inductance. If not provided by motor datasheet,
    //     measure phase to phase inductance with multimeter, and
    //     divide over two to get phase inductance. If 2 mH are
    //     measured from phase to phase, then L = 1 mH

    s->motor_type = ctrl->motor_type;
    s->rs_ohm = ctrl->Rs;
    s->ls_H = ctrl->Ls;
    s->Ld = ctrl->Ld;
    s->Lq = ctrl->Lq;
    s->l_diff = ctrl->Ldiff;
    s->flux_wb = ctrl->Flux;
    s->dt = ctrl->current_loop_dt;

    float TemporalFloat;
    TemporalFloat = 1 - s->rs_ohm * s->dt / s->ls_H;
    if (TemporalFloat < 0.0f)
        s->Fsmopos = 0.0f;
    else
        s->Fsmopos = TemporalFloat;

    TemporalFloat = s->dt / s->ls_H;
    if (TemporalFloat > 1.0f)
        s->Gsmopos = 0.99999f;
    else
        s->Gsmopos = TemporalFloat;

    if(s->motor_type == IPM)
    {
        TemporalFloat = 1 - s->rs_ohm * s->dt / s->Ld;
        if (TemporalFloat < 0.0f)
            s->Fsmopos_DAxis = 0.0f;
        else
            s->Fsmopos_DAxis = TemporalFloat;

        TemporalFloat = s->dt / s->Ld;
        if (TemporalFloat > 1.0f)
            s->Gsmopos_DAxis = 0.99999f;
        else
            s->Gsmopos_DAxis = TemporalFloat;
    }
    /*
    * https://zhuanlan.zhihu.com/p/416224632
    * Kslide should greater than the max value of them
    * fabsf(-1 * foc_ctrl.Rs * SMO_observer.I_alpha_beta_estimated.Alpha) + SMO_observer.Z_alpha_beta.Alpha * SIGN_F(SMO_observer.I_alpha_beta_estimated.Alpha) - SMO_observer.E_rps * _2PI * (s->Ld - s->Lq) * SMO_observer.I_alpha_beta_estimated.Beta * SIGN_F(SMO_observer.I_alpha_beta_estimated.Alpha);
    * fabsf(-1 * foc_ctrl.Rs * SMO_observer.I_alpha_beta_estimated.Beta) + SMO_observer.Z_alpha_beta.Beta * SIGN_F(SMO_observer.I_alpha_beta_estimated.Beta) + SMO_observer.E_rps * _2PI * (s->Ld - s->Lq) * SMO_observer.I_alpha_beta_estimated.Alpha * SIGN_F(SMO_observer.I_alpha_beta_estimated.Beta);
    */

    #ifdef MOTOR_2PP_SERVO
    s->Kslide = 10;
    s->MaxErr = 1;
    #endif

    #ifdef MOTOR_14PP_BLDC
    s->Kslide = 1;
    s->MaxErr = 1;
    #endif

    #ifdef MOTOR_7PP_BLDC
    s->Kslide = 1;
    s->MaxErr = 1;
    #endif

    #ifdef MOTOR_1PP_BLDC_HALL
    s->Kslide = 1;
    s->MaxErr = 1;
    #endif

    #ifdef MOTOR_1PP_BLDC_FAN_SLOW
    s->Kslide = 1;
    s->MaxErr = 1;
    #endif

#if SMO_USE_ARCTAN
    // /* filter coef = 2*pi*Erps/fpwm */
    // s->Kslf = 5 * _2PI * MinErpm / 60 / FOC_CC_LOOP_FREQ;
    // s->KslfMin = 5 * _2PI * MinErpm / 60 / FOC_CC_LOOP_FREQ;

    s->Kslf = 0.01f;
    s->KslfMin = 0.01f;

    s->SpeedFilter = 0.003f;
    s->ThetaOffset = 0;

    s->PrevTheta = 0;
    s->AccumTheta = 0;
    s->DeltaTheta = 0;
    s->AccumThetaCnt = 0;
#endif

#if SMO_USE_PLL
    s->ThetaOffset = 0;
    PLL_Init(&s->pll,0.01f,1,speed_fc,s->dt,p_iir);
#endif
}

void SMO_Run(SMO_t *s, FOC_Para_t *para)
{
    memcpy(&s->V_alpha_beta,&para->V_alpha_beta,sizeof(AB_Axis_t));
    memcpy(&s->I_alpha_beta,&para->I_alpha_beta,sizeof(AB_Axis_t));

    // Sliding mode current observer
    if(s->motor_type == IPM)
    {
        // https://zhuanlan.zhihu.com/p/421676488 for IPM
        s->I_alpha_beta_estimated.Alpha = s->Fsmopos_DAxis * s->I_alpha_beta_estimated.Alpha + s->Gsmopos_DAxis * (s->V_alpha_beta.Alpha - s->E_alpha_beta.Alpha - s->Z_alpha_beta.Alpha) - fabsf(s->E_rps) * _2PI * s->dt * (s->Ld - s->Lq) * s->I_alpha_beta_estimated.Beta / s->Ld;
        s->I_alpha_beta_estimated.Beta = s->Fsmopos_DAxis * s->I_alpha_beta_estimated.Beta + s->Gsmopos_DAxis * (s->V_alpha_beta.Beta - s->E_alpha_beta.Beta - s->Z_alpha_beta.Beta) + fabsf(s->E_rps) * _2PI * s->dt * (s->Ld - s->Lq) * s->I_alpha_beta_estimated.Alpha / s->Ld;
    }
    else
    {
        // AN1078 for SPM
        s->I_alpha_beta_estimated.Alpha = s->Fsmopos * s->I_alpha_beta_estimated.Alpha + s->Gsmopos * (s->V_alpha_beta.Alpha - s->E_alpha_beta.Alpha - s->Z_alpha_beta.Alpha);
        s->I_alpha_beta_estimated.Beta = s->Fsmopos * s->I_alpha_beta_estimated.Beta + s->Gsmopos * (s->V_alpha_beta.Beta - s->E_alpha_beta.Beta - s->Z_alpha_beta.Beta);
    }

    // Current errors
    s->I_alpha_beta_error.Alpha = s->I_alpha_beta_estimated.Alpha - s->I_alpha_beta.Alpha;
    s->I_alpha_beta_error.Beta = s->I_alpha_beta_estimated.Beta - s->I_alpha_beta.Beta;

    // Sliding control calculator
    if (fabsf(s->I_alpha_beta_error.Alpha) < s->MaxErr)
    {
        s->Z_alpha_beta.Alpha = s->Kslide * s->I_alpha_beta_error.Alpha / s->MaxErr;
    }
    else
    {
        if (s->I_alpha_beta_error.Alpha > 0)
            s->Z_alpha_beta.Alpha = s->Kslide;
        // else if (s->I_alpha_beta_error.Alpha == 0)
        //     s->Z_alpha_beta.Alpha = 0;
        else
            s->Z_alpha_beta.Alpha = -s->Kslide;
    }

    if (fabsf(s->I_alpha_beta_error.Beta) < s->MaxErr)
    {
        s->Z_alpha_beta.Beta = s->Kslide * s->I_alpha_beta_error.Beta / s->MaxErr;
    }
    else
    {
        if (s->I_alpha_beta_error.Beta > 0)
            s->Z_alpha_beta.Beta = s->Kslide;
        // else if (s->I_alpha_beta_error.Beta == 0)
        //     s->Z_alpha_beta.Beta = 0;
        else
            s->Z_alpha_beta.Beta = -s->Kslide;
    }

#if SMO_USE_PLL
    // see https://zhuanlan.zhihu.com/p/652503676
    // PLL_Run(&s->pll,-s->Z_alpha_beta.Alpha,s->Z_alpha_beta.Beta);
    // PLL_Run(&s->pll,SIGN_F(s->pll.hz) * -s->Z_alpha_beta.Alpha,SIGN_F(s->pll.hz) * s->Z_alpha_beta.Beta);

    /* 
        正反转时PLL结构不同,需加符号函数进行修正
        但由于零速时观测转速震荡导致增加符号函数后速度和角度无法收敛
    */
    Normalize_PLL_Run(&s->pll,-s->Z_alpha_beta.Alpha,s->Z_alpha_beta.Beta);
    // Normalize_PLL_Run(&s->pll,SIGN_F(s->pll.hz) * -s->Z_alpha_beta.Alpha,SIGN_F(s->pll.hz) * s->Z_alpha_beta.Beta);

    s->E_ang = s->pll.theta;
    s->E_rps = s->pll.hz_f;
#endif

#if SMO_USE_ARCTAN
    // Sliding control filter -> BEMF calculator
    s->E_alpha_beta.Alpha = s->E_alpha_beta.Alpha + s->Kslf * (s->Z_alpha_beta.Alpha - s->E_alpha_beta.Alpha);
    s->E_alpha_beta.Beta = s->E_alpha_beta.Beta + s->Kslf * (s->Z_alpha_beta.Beta - s->E_alpha_beta.Beta);

    // New filter used to calculate Position
    s->E_alpha_beta_final.Alpha = s->E_alpha_beta_final.Alpha + s->Kslf * (s->E_alpha_beta.Alpha - s->E_alpha_beta_final.Alpha);
    s->E_alpha_beta_final.Beta = s->E_alpha_beta_final.Beta + s->Kslf * (s->E_alpha_beta.Beta - s->E_alpha_beta_final.Beta);

    // Rotor angle calculator -> Theta = atan(E_alpha_beta.Beta,E_alpha_beta.Alpha)
    // electrical angle in radians
    // atan2(double y,double x)  return value :-PI~PI but we need 0~2PI

    /* 按道理来说此时估算角度应该有90°的相位滞后,实际上并没有. But why? */
    /*
        这里认为Zalpha Z_alpha_beta.Beta 既是 BEMF 在ab axis分量

        Z_alpha_beta.Alpha  =   -ωe*ψf*sinθe
        Z_alpha_beta.Beta   =   ωe*ψf*cosθe

        E_alpha_beta.Alpha  =   -ωe*ψf*sin(θe + pi/4) * sqrt(2)/2
        E_alpha_beta.Beta   =   ωe*ψf*cos(θe + pi/4) * sqrt(2)/2

        E_alpha_beta_final.Alpha = -ωe*ψf*sin(θe + pi/2) * SQ(sqrt(2)/2) = -ωe*ψf*sin(θe + pi/2) / 2
        E_alpha_beta_final.Beta  = ωe*ψf*cos(θe + pi/2) * SQ(sqrt(2)/2)  = ωe*ψf*cos(θe + pi/2) / 2

        

        经过第一次低通滤波后 E_alpha_beta.Alpha E_alpha_beta.Beta 滞后 Z_alpha_beta.Alpha Z_alpha_beta.Beta 45° 幅值衰减为 sqrt(2)/2 倍
        经过第二次低通滤波后 E_alpha_beta_final.Alpha E_alpha_beta_final.Beta 滞后 E_alpha_beta.Alpha E_alpha_beta.Beta 45° 幅值衰减为 sqrt(2)/2 倍
        所以 E_alpha_beta_final.Alpha E_alpha_beta_final.Beta 滞后 Z_alpha_beta.Alpha Z_alpha_beta.Beta 90° 幅值衰减为 1/2 倍
        又因为 Z_alpha_beta.Beta 滞后 Z_alpha_beta.Alpha 90° , E_alpha_beta_final.Beta 滞后 E_alpha_beta_final.Alpha 90°
        所以 E_alpha_beta_final.Alpha 与 Z_alpha_beta.Beta 相位相同 E_alpha_beta_final.Beta 与 Z_alpha_beta.Alpha 反相

        因为 atan(Z_alpha_beta.Beta,Z_alpha_beta.Alpha) 即 atan(BEMFbeta,BEMFalpha) 超前 θe 90° 正好与两次低通滤波造成的 90° 相位滞后抵消
        所以 atan(E_alpha_beta.Beta,E_alpha_beta.Alpha) 与 θe 相位相同 不再需要补偿滞后的90°
        所以 ThetaOffset = 0 即可
    */
    s->Theta = Normalize_Rad(atan2f(s->E_alpha_beta_final.Beta,s->E_alpha_beta_final.Alpha) + s->ThetaOffset);
    /*
        可用此方法估算转速.
        实际使用时确实可根据转速变化,但单位是什么?
        sqrtf((SQ(s->E_alpha_beta_final.Alpha) + SQ(s->E_alpha_beta_final.Beta)) / SQ(foc_ctrl.Flux));
    */
    // 电气转速估算 执行频率为 FOC_SC_LOOP_FREQ
    // 输出结果为 SPEEDLOOPTIME 内转过的电角度值（弧度制）
    s->AccumThetaCnt++;
    if (s->AccumThetaCnt == FOC_SPEED_LOOP_DIV) // speed loop div
    {
        s->DeltaTheta = s->Theta - s->PrevTheta;
        if (s->DeltaTheta < -_PI)
        {
            s->DeltaTheta += _2PI;
        }
        else if (s->DeltaTheta > _PI)
        {
            s->DeltaTheta -= _2PI;
        }
        s->AccumTheta = s->DeltaTheta;
        s->PrevTheta = s->Theta;
        s->Omega = s->AccumTheta;
        s->AccumThetaCnt = 0;
        s->AccumTheta = 0;
    }

    // 电气转速滤波
    s->OmegaFltred = s->OmegaFltred + s->SpeedFilter * (s->Omega - s->OmegaFltred);
    s->erps = s->OmegaFltred * FOC_CC_LOOP_FREQ / FOC_SPEED_LOOP_DIV / _2PI;

    // 反电势滤波系数更新 截止频率等于电频率 此时幅值衰减为0.70710678倍 相位滞后45°
    // filter coef = 2*pi*Erps/fpwm
    // Omega:rads per SPEEDLOOPTIME
    // Erps = s->OmegaFltred * FOC_SC_LOOP_FREQ / _2PI

    // s->Kslf = _2PI * fabsf(s->OmegaFltred) * FOC_SC_LOOP_FREQ / _2PI  / FOC_CC_LOOP_FREQ;
    s->Kslf = fabsf(s->OmegaFltred) / FOC_SPEED_LOOP_DIV;
    if (s->Kslf < s->KslfMin)
    {
        s->Kslf = s->KslfMin;
    }
    else if (s->Kslf > 1.0f)
    {
        s->Kslf = 1.0f;
    }
    s->E_ang = s->Theta;
    s->E_rps = s->erps;
#endif


    s->E_rpm = s->E_rps * 60;
}
