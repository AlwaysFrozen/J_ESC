#include "SMO.h"
#include "My_Math.h"
#include "FOC.h"


SMO_t smo_observer;

void SMO_Init(SMO_t *s,float speed_fc,float dt)
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

    float TemporalFloat;
    TemporalFloat = 1 - Rs_R * dt / Ls_H;
    if (TemporalFloat < 0.0f)
        s->Fsmopos = 0.0f;
    else
        s->Fsmopos = TemporalFloat;

    TemporalFloat = dt / Ls_H;
    if (TemporalFloat > 1.0f)
        s->Gsmopos = 0.99999f;
    else
        s->Gsmopos = TemporalFloat;

#if IS_IPM
    TemporalFloat = 1 - Rs_R * dt / Ld_H;
    if (TemporalFloat < 0.0f)
        s->Fsmopos_DAxis = 0.0f;
    else
        s->Fsmopos_DAxis = TemporalFloat;

    TemporalFloat = dt / Ld_H;
    if (TemporalFloat > 1.0f)
        s->Gsmopos_DAxis = 0.99999f;
    else
        s->Gsmopos_DAxis = TemporalFloat;
#endif
    /*
    * https://zhuanlan.zhihu.com/p/416224632
    * Kslide should greater than the max value of them
    * fabsf(-1 * Rs_R * smo_observer.EstIalpha) + smo_observer.Zalpha * SIGN(smo_observer.EstIalpha) - smo_observer.E_rps * _2PI * (Ld_H - Lq_H) * smo_observer.EstIbeta * SIGN(smo_observer.EstIalpha);
    * fabsf(-1 * Rs_R * smo_observer.EstIbeta) + smo_observer.Zbeta * SIGN(smo_observer.EstIbeta) + smo_observer.E_rps * _2PI * (Ld_H - Lq_H) * smo_observer.EstIalpha * SIGN(smo_observer.EstIbeta);
    */

    #ifdef MOTOR_2PP_SERVO
    s->Kslide = 10;
    s->MaxErr = 1;
    #endif

    #ifdef MOTOR_14PP_BLDC
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
    PLL_Init(&s->pll,0.0001,0.2,speed_fc,dt);
#endif
}

void SMO_Run(SMO_t *s, FOC_Para_t *foc_para)
{
#if SMO_USE_MEASURE_VOL
    s->Valpha = foc_para->Ua;
    s->Vbeta = foc_para->Ub;
    s->Ialpha = foc_para->Ia;
    s->Ibeta = foc_para->Ib;
#else
    s->Valpha = foc_para->Ua_rate * Virtual_Moto.V_bus_v_f * _2_3;
    s->Vbeta = foc_para->Ub_rate * Virtual_Moto.V_bus_v_f * _2_3;
    s->Ialpha = foc_para->Ia;
    s->Ibeta = foc_para->Ib;
#endif

    // Sliding mode current observer
#if IS_IPM
    // https://zhuanlan.zhihu.com/p/421676488 for IPM
    s->EstIalpha = s->Fsmopos_DAxis * s->EstIalpha + s->Gsmopos_DAxis * (s->Valpha - s->Ealpha - s->Zalpha) - fabsf(s->E_rps) * _2PI * dt * (Ld_H - Lq_H) * s->EstIbeta / Ld_H;
    s->EstIbeta = s->Fsmopos_DAxis * s->EstIbeta + s->Gsmopos_DAxis * (s->Vbeta - s->Ebeta - s->Zbeta) + fabsf(s->E_rps) * _2PI * dt * (Ld_H - Lq_H) * s->EstIalpha / Ld_H;
#else
    // AN1078 for SPM
    s->EstIalpha = s->Fsmopos * s->EstIalpha + s->Gsmopos * (s->Valpha - s->Ealpha - s->Zalpha);
    s->EstIbeta = s->Fsmopos * s->EstIbeta + s->Gsmopos * (s->Vbeta - s->Ebeta - s->Zbeta);
#endif

    // Current errors
    s->IalphaError = s->EstIalpha - s->Ialpha;
    s->IbetaError = s->EstIbeta - s->Ibeta;

    // Sliding control calculator
    if (fabsf(s->IalphaError) < s->MaxErr)
    {
        s->Zalpha = s->Kslide * s->IalphaError / s->MaxErr;
    }
    else
    {
        if (s->IalphaError > 0)
            s->Zalpha = s->Kslide;
        // else if (s->IalphaError == 0)
        //     s->Zalpha = 0;
        else
            s->Zalpha = -s->Kslide;
    }

    if (fabsf(s->IbetaError) < s->MaxErr)
    {
        s->Zbeta = s->Kslide * s->IbetaError / s->MaxErr;
    }
    else
    {
        if (s->IbetaError > 0)
            s->Zbeta = s->Kslide;
        // else if (s->IbetaError == 0)
        //     s->Zbeta = 0;
        else
            s->Zbeta = -s->Kslide;
    }

#if SMO_USE_PLL
    // see https://zhuanlan.zhihu.com/p/652503676
    PLL_Run(&s->pll,-s->Zalpha,s->Zbeta);
#if !SMO_USE_PLL_SPEED
    s->pll.theta = Normalize_Angle(s->pll.theta + s->ThetaOffset);
#endif
#endif

#if SMO_USE_ARCTAN
    // Sliding control filter -> BEMF calculator
    s->Ealpha = s->Ealpha + s->Kslf * (s->Zalpha - s->Ealpha);
    s->Ebeta = s->Ebeta + s->Kslf * (s->Zbeta - s->Ebeta);

    // New filter used to calculate Position
    s->EalphaFinal = s->EalphaFinal + s->Kslf * (s->Ealpha - s->EalphaFinal);
    s->EbetaFinal = s->EbetaFinal + s->Kslf * (s->Ebeta - s->EbetaFinal);

    // Rotor angle calculator -> Theta = atan(Ebeta,Ealpha)
    // electrical angle in radians
    // atan2(double y,double x)  return value :-PI~PI but we need 0~2PI

    /* 按道理来说此时估算角度应该有90°的相位滞后,实际上并没有. But why? */
    /*
        这里认为Zalpha Zbeta 既是 BEMF 在ab axis分量

        Zalpha  =   -ωe*ψf*sinθe
        Zbeta   =   ωe*ψf*cosθe

        Ealpha  =   -ωe*ψf*sin(θe + pi/4) * sqrt(2)/2
        Ebeta   =   ωe*ψf*cos(θe + pi/4) * sqrt(2)/2

        EalphaFinal = -ωe*ψf*sin(θe + pi/2) * SQ(sqrt(2)/2) = -ωe*ψf*sin(θe + pi/2) / 2
        EbetaFinal  = ωe*ψf*cos(θe + pi/2) * SQ(sqrt(2)/2)  = ωe*ψf*cos(θe + pi/2) / 2

        

        经过第一次低通滤波后 Ealpha Ebeta 滞后 Zalpha Zbeta 45° 幅值衰减为 sqrt(2)/2 倍
        经过第二次低通滤波后 EalphaFinal EbetaFinal 滞后 Ealpha Ebeta 45° 幅值衰减为 sqrt(2)/2 倍
        所以 EalphaFinal EbetaFinal 滞后 Zalpha Zbeta 90° 幅值衰减为 1/2 倍
        又因为 Zbeta 滞后 Zalpha 90° , EbetaFinal 滞后 EalphaFinal 90°
        所以 EalphaFinal 与 Zbeta 相位相同 EbetaFinal 与 Zalpha 反相

        因为 atan(Zbeta,Zalpha) 即 atan(BEMFbeta,BEMFalpha) 超前 θe 90° 正好与两次低通滤波造成的 90° 相位滞后抵消
        所以 atan(Ebeta,Ealpha) 与 θe 相位相同 不再需要补偿滞后的90°
        所以 ThetaOffset = 0 即可
    */
    s->Theta = Normalize_Angle(atan2f(s->EbetaFinal,s->EalphaFinal) + s->ThetaOffset);
    /*
        可用此方法估算转速.
        实际使用时确实可根据转速变化,但单位是什么?
        sqrtf((SQ(s->EalphaFinal) + SQ(s->EbetaFinal)) / SQ(FLUX_Wb));
    */
#if !SMO_USE_PLL_SPEED
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
    // Omega:rads/SPEEDLOOPTIME
    // Erps = s->OmegaFltred * FOC_SC_LOOP_FREQ / _2PI

    // s->Kslf = _2PI * fabsf(s->OmegaFltred) * FOC_SC_LOOP_FREQ / _2PI  / FOC_CC_LOOP_FREQ;
    s->Kslf = fabsf(s->OmegaFltred) / FOC_SPEED_LOOP_DIV;
#else
    // use pll speed
    s->erps = s->pll.speed_hz_f;
    s->Kslf = _2PI * fabsf(s->pll.speed_hz_f) / FOC_CC_LOOP_FREQ;
#endif
    if (s->Kslf < s->KslfMin)
    {
        s->Kslf = s->KslfMin;
    }
    else if (s->Kslf > 1.0f)
    {
        s->Kslf = 1.0f;
    }
#endif

#if SMO_USE_ARCTAN 
    s->E_ang = s->Theta;
    s->E_rps = s->erps;
#elif SMO_USE_PLL
    s->E_ang = s->pll.theta;
    s->E_rps = s->pll.speed_hz_f;
#endif

    s->E_rpm = s->E_rps * 60;
}
