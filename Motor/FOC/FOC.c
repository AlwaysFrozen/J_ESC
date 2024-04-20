#include "Board_Config.h"
#include "FOC.h"

/*
    ωe:electrical angle speed rads/s
    ψf:rotor flux Wb

    uα = R*iα + La*diα/dt + Lαβ*diβ/dt - ωe*ψf*sinθe
    uβ = R*iβ + Lβ*diβ/dt + Lαβ*diα/dt + ωe*ψf*cosθe

    ud = R*id + Ld*did/dt - ωe*Lq*iq
    uq = R*iq + Lq*diq/dt + ωe*(Ld*id + ψf)
*/

// SVPWM    (A,B,C) ==> (X,X,X)
// sector 1 (1,0,0) and (1,1,0)
// __------------__
// ____--------____
// ______----______
// sector 2 (1,1,0) and (0,1,0)
// ____--------____
// __------------__
// ______----______
// sector 3 (0,1,0) and (0,1,1)
// ______----______
// __------------__
// ____--------____
// sector 4 (0,1,1) and (0,0,1)
// ______----______
// ____--------____
// __------------__
// sector 5 (0,0,1) and (1,0,1)
// ____--------____
// ______----______
// __------------__
// sector 6 (1,0,1) and (1,0,0)
// __------------__
// ______----______
// ____--------____

// sector 1 ==> 6
// __------------______--------__________----____________----__________--------______------------__
// ____--------______------------____------------______--------__________----____________----______
// ______----____________----__________--------______------------____------------______--------____


void Clarke_Transmission(float a, float b, float c, float *alpha, float *beta)
{
    #if HAVE_3_PHASE_CURRENT_SENSOR
    *alpha = (a - 0.5f * (b + c)) * _2_3;
    *beta = (_SQRT3_2 * (b - c)) * _2_3;
    #endif

    #if HAVE_2_PHASE_CURRENT_SENSOR
    *alpha = a;
    *beta = _1_SQRT3 * (a + 2 * b);
    #endif
}

void Inverse_Clarke_Transmission(float alpha, float beta, float *a, float *b, float *c)
{
    *a = alpha;
    *b = (-alpha + _SQRT3 * beta) / 2;
    *c = (-alpha - _SQRT3 * beta) / 2;
}

void Park_Transmission(float alpha, float beta, float *d, float *q, float theta)
{
    float sin_theta = sinf(theta);
    float cos_theta = cosf(theta);

    *d = alpha * cos_theta + beta * sin_theta;
    *q = -alpha * sin_theta + beta * cos_theta;
}

void Inverse_Park_Transmission(float d, float q, float *alpha, float *beta, float theta)
{
    float sin_theta = sinf(theta);
    float cos_theta = cosf(theta);

    *alpha = d * cos_theta - q * sin_theta;
    *beta = d * sin_theta + q * cos_theta;
}

// from VESC Project
void SVPWM_AB(float v_alpha_rate, float v_beta_rate, uint8_t *sector, uint16_t period, int32_t *CMP1, int32_t *CMP2, int32_t *CMP3)
{
    if (v_beta_rate >= 0.0f)
    {
        if (v_alpha_rate > 0.0f)
        {
            if (v_beta_rate / v_alpha_rate >= _SQRT3)
            {
                *sector = 2;
            }
            else
            {
                *sector = 1;
            }
        }
        else if (v_alpha_rate < 0.0f)
        {
            if (v_beta_rate / v_alpha_rate >= -_SQRT3)
            {
                *sector = 3;
            }
            else
            {
                *sector = 2;
            }
        }
        else
        {
            *sector = 2;
        }
    }
    else
    {
        if (v_alpha_rate > 0.0f)
        {
            if (v_beta_rate / v_alpha_rate >= -_SQRT3)
            {
                *sector = 6;
            }
            else
            {
                *sector = 5;
            }
        }
        else if (v_alpha_rate < 0.0f)
        {
            if (v_beta_rate / v_alpha_rate >= _SQRT3)
            {
                *sector = 5;
            }
            else
            {
                *sector = 4;
            }
        }
        else
        {
            *sector = 5;
        }
    }

    int32_t tA, tB, tC;

    switch (*sector)
    {
        // sector 1-2
        case 1:
        {
            // Vector on-times
            int32_t t1 = (v_alpha_rate - _1_SQRT3 * v_beta_rate) * period;
            int32_t t2 = (_2_SQRT3 * v_beta_rate) * period;

            tC = (period - t1 - t2) / 2;
            tB = tC + t2;
            tA = tB + t1;
            break;
        }

        // sector 2-3
        case 2:
        {
            // Vector on-times
            int32_t t2 = (v_alpha_rate + _1_SQRT3 * v_beta_rate) * period;
            int32_t t3 = (-v_alpha_rate + _1_SQRT3 * v_beta_rate) * period;

            tC = (period - t2 - t3) / 2;
            tA = tC + t2;
            tB = tA + t3;
            break;
        }

        // sector 3-4
        case 3:
        {
            // Vector on-times
            int32_t t3 = (_2_SQRT3 * v_beta_rate) * period;
            int32_t t4 = (-v_alpha_rate - _1_SQRT3 * v_beta_rate) * period;

            tA = (period - t3 - t4) / 2;
            tC = tA + t4;
            tB = tC + t3;
            break;
        }

        // sector 4-5
        case 4:
        {
            // Vector on-times
            int32_t t4 = (-v_alpha_rate + _1_SQRT3 * v_beta_rate) * period;
            int32_t t5 = (-_2_SQRT3 * v_beta_rate) * period;

            tA = (period - t4 - t5) / 2;
            tB = tA + t4;
            tC = tB + t5;
            break;
        }

        // sector 5-6
        case 5:
        {
            // Vector on-times
            int32_t t5 = (-v_alpha_rate - _1_SQRT3 * v_beta_rate) * period;
            int32_t t6 = (v_alpha_rate - _1_SQRT3 * v_beta_rate) * period;

            tB = (period - t5 - t6) / 2;
            tA = tB + t6;
            tC = tA + t5;
            break;
        }

        // sector 6-1
        case 6:
        {
            // Vector on-times
            int32_t t6 = (-_2_SQRT3 * v_beta_rate) * period;
            int32_t t1 = (v_alpha_rate + _1_SQRT3 * v_beta_rate) * period;

            tB = (period - t6 - t1) / 2;
            tC = tB + t6;
            tA = tC + t1;
            break;
        }
    }

    tA = _constrain(tA, 0.0f, period);
    tB = _constrain(tB, 0.0f, period);
    tC = _constrain(tC, 0.0f, period);

    *CMP1 = tA;
    *CMP2 = tB;
    *CMP3 = tC;
}

void SVPWM_AB_Voltage(float v_alpha, float v_beta, uint32_t v_dc,uint8_t *sector, uint16_t period, int32_t *CMP1, int32_t *CMP2, int32_t *CMP3)
{
    float v_max = v_dc * _2_3;
    float v_alpha_rate = v_alpha / v_max;
    float v_beta_rate = v_beta / v_max;

    if (v_beta >= 0.0f)
    {
        if (v_alpha > 0.0f)
        {
            if (v_beta / v_alpha >= _SQRT3)
            {
                *sector = 2;
            }
            else
            {
                *sector = 1;
            }
        }
        else if (v_alpha < 0.0f)
        {
            if (v_beta / v_alpha >= -_SQRT3)
            {
                *sector = 3;
            }
            else
            {
                *sector = 2;
            }
        }
        else
        {
            *sector = 2;
        }
    }
    else
    {
        if (v_alpha > 0.0f)
        {
            if (v_beta / v_alpha >= -_SQRT3)
            {
                *sector = 6;
            }
            else
            {
                *sector = 5;
            }
        }
        else if (v_alpha < 0.0f)
        {
            if (v_beta / v_alpha >= _SQRT3)
            {
                *sector = 5;
            }
            else
            {
                *sector = 4;
            }
        }
        else
        {
            *sector = 5;
        }
    }

    int32_t tA, tB, tC;

    switch (*sector)
    {
        // sector 1-2
        case 1:
        {
            // Vector on-times
            int32_t t1 = (v_alpha_rate - _1_SQRT3 * v_beta_rate) * period;
            int32_t t2 = (_2_SQRT3 * v_beta_rate) * period;

            tC = (period - t1 - t2) / 2;
            tB = tC + t2;
            tA = tB + t1;
            break;
        }

        // sector 2-3
        case 2:
        {
            // Vector on-times
            int32_t t2 = (v_alpha_rate + _1_SQRT3 * v_beta_rate) * period;
            int32_t t3 = (-v_alpha_rate + _1_SQRT3 * v_beta_rate) * period;

            tC = (period - t2 - t3) / 2;
            tA = tC + t2;
            tB = tA + t3;
            break;
        }

        // sector 3-4
        case 3:
        {
            // Vector on-times
            int32_t t3 = (_2_SQRT3 * v_beta_rate) * period;
            int32_t t4 = (-v_alpha_rate - _1_SQRT3 * v_beta_rate) * period;

            tA = (period - t3 - t4) / 2;
            tC = tA + t4;
            tB = tC + t3;
            break;
        }

        // sector 4-5
        case 4:
        {
            // Vector on-times
            int32_t t4 = (-v_alpha_rate + _1_SQRT3 * v_beta_rate) * period;
            int32_t t5 = (-_2_SQRT3 * v_beta_rate) * period;

            tA = (period - t4 - t5) / 2;
            tB = tA + t4;
            tC = tB + t5;
            break;
        }

        // sector 5-6
        case 5:
        {
            // Vector on-times
            int32_t t5 = (-v_alpha_rate - _1_SQRT3 * v_beta_rate) * period;
            int32_t t6 = (v_alpha_rate - _1_SQRT3 * v_beta_rate) * period;

            tB = (period - t5 - t6) / 2;
            tA = tB + t6;
            tC = tA + t5;
            break;
        }

        // sector 6-1
        case 6:
        {
            // Vector on-times
            int32_t t6 = (-_2_SQRT3 * v_beta_rate) * period;
            int32_t t1 = (v_alpha_rate + _1_SQRT3 * v_beta_rate) * period;

            tB = (period - t6 - t1) / 2;
            tC = tB + t6;
            tA = tC + t1;
            break;
        }
    }

    tA = _constrain(tA, 0.0f, period);
    tB = _constrain(tB, 0.0f, period);
    tC = _constrain(tC, 0.0f, period);

    *CMP1 = tA;
    *CMP2 = tB;
    *CMP3 = tC;
}

void SVPWM_DQ(float Ud_rate, float Uq_rate, float angle, uint8_t *sector, uint16_t period, int32_t *CMP1, int32_t *CMP2, int32_t *CMP3)
{
    float angle_out;
    float angle_offset;
    float Uout;    

    if (Ud_rate)
    {
        Uout = sqrtf(Ud_rate * Ud_rate + Uq_rate * Uq_rate);
        angle_out = Normalize_Angle(angle + atan2f(Uq_rate, Ud_rate));
    }
    else
    {
        Uout = Uq_rate;
        angle_out = Normalize_Angle(angle + _PI_2);
    }

    *sector = floorf(angle_out / _PI_3);
    angle_offset = angle_out - *sector * _PI_3;
    *sector += 1;

    float st, ct;
    st = sinf(angle_offset);
    ct = cosf(angle_offset);

    /*
    t0~t7 is the action time of the 8 vectors corresponding to the three-phase full bridge
    T1 is the operation time of the No. 1 effective vector of the current sector
    T2 is the operation time of the No. 2 effective vector of the current sector
    T1 and T2 correspond to the operation time of different sectors t1 to t6
    T0 is the sum of the action time of the 0 vector in the current sector, and because the action time of t0 and t7 vector is equal, 
    T0 divided by two, that is, the action time of t0 vector, is used as the duty cycle calculation for convenience
    */
    float T1 = Uout * (ct - _1_SQRT3 * st);
    float T2 = _2_SQRT3 * Uout * st;
    float T0 = (1.0f - T1 - T2) * 0.5f; // actually this T0 is (t0 + t7) / 2,because t0 == t7

    float Ta, Tb, Tc;
    switch (*sector)
    {
        case 1:
            Ta = T1 + T2 + T0;
            Tb = T2 + T0;
            Tc = T0;
            break;

        case 2:
            Ta = T1 + T0;
            Tb = T1 + T2 + T0;
            Tc = T0;
            break;

        case 3:
            Ta = T0;
            Tb = T1 + T2 + T0;
            Tc = T2 + T0;
            break;

        case 4:
            Ta = T0;
            Tb = T1 + T0;
            Tc = T1 + T2 + T0;
            break;

        case 5:
            Ta = T2 + T0;
            Tb = T0;
            Tc = T1 + T2 + T0;
            break;

        case 6:
            Ta = T1 + T2 + T0;
            Tb = T0;
            Tc = T1 + T0;
            break;

        default:
            // possible error state
            Ta = 0;
            Tb = 0;
            Tc = 0;
            break;
    }

    Ta = _constrain(Ta, 0.0f, 1.0f);
    Tb = _constrain(Tb, 0.0f, 1.0f);
    Tc = _constrain(Tc, 0.0f, 1.0f);

    *CMP1 = period * Ta;
    *CMP2 = period * Tb;
    *CMP3 = period * Tc;
}
void SVPWM_DQ_Voltage(float Ud, float Uq, float Udc, float angle, uint8_t *sector, uint16_t period, int32_t *CMP1, int32_t *CMP2, int32_t *CMP3)
{
    float angle_out;
    float angle_offset;
    float Uout;    

    if (Ud)
    {
        Uout = sqrtf(Ud * Ud + Uq * Uq) / (Udc * _2_3);
        angle_out = Normalize_Angle(angle + atan2f(Uq, Ud));
    }
    else
    {
        Uout = Uq / (Udc * _2_3);
        angle_out = Normalize_Angle(angle + _PI_2);
    }

    *sector = floorf(angle_out / _PI_3);
    angle_offset = angle_out - *sector * _PI_3;
    *sector += 1;

    float st, ct;
    st = sinf(angle_offset);
    ct = cosf(angle_offset);
    float T1 = Uout * (ct - _1_SQRT3 * st);
    float T2 = _2_SQRT3 * Uout * st;
    float T0 = (1.0f - T1 - T2) * 0.5f; // actually this T0 is (t0 + t7) / 2,because t0 == t7

    float Ta, Tb, Tc;
    switch (*sector)
    {
        case 1:
            Ta = T1 + T2 + T0;
            Tb = T2 + T0;
            Tc = T0;
            break;

        case 2:
            Ta = T1 + T0;
            Tb = T1 + T2 + T0;
            Tc = T0;
            break;

        case 3:
            Ta = T0;
            Tb = T1 + T2 + T0;
            Tc = T2 + T0;
            break;

        case 4:
            Ta = T0;
            Tb = T1 + T0;
            Tc = T1 + T2 + T0;
            break;

        case 5:
            Ta = T2 + T0;
            Tb = T0;
            Tc = T1 + T2 + T0;
            break;

        case 6:
            Ta = T1 + T2 + T0;
            Tb = T0;
            Tc = T1 + T0;
            break;

        default:
            // possible error state
            Ta = 0;
            Tb = 0;
            Tc = 0;
            break;
    }

    Ta = _constrain(Ta, 0.0f, 1.0f);
    Tb = _constrain(Tb, 0.0f, 1.0f);
    Tc = _constrain(Tc, 0.0f, 1.0f);

    *CMP1 = period * Ta;
    *CMP2 = period * Tb;
    *CMP3 = period * Tc;
}

void MTPA_Cal(FOC_Para_t * foc_para,float flux_wb,float Ld,float Lq)
{
    float ld_lq_diff = Ld - Lq;

    // // https://zhuanlan.zhihu.com/p/558759346
    // // foc_para->Id_target = SQ(foc_para->Iq_target) * ld_lq_diff / flux_wb;
    // foc_para->Id_target = SQ(foc_para->Iq) * ld_lq_diff / flux_wb;

    // https://zhuanlan.zhihu.com/p/624474437
    // foc_para->Id_target = (sqrtf(SQ(flux_wb) + 4 * SQ(ld_lq_diff) * SQ(foc_para->Iq_target)) - flux_wb) / (2 * ld_lq_diff);
    foc_para->Id_target = (sqrtf(SQ(flux_wb) + 4 * SQ(ld_lq_diff) * SQ(foc_para->Iq)) - flux_wb) / (2 * ld_lq_diff);

    // // form VESC
    // float iq_ref = Min_Abs(foc_para->Iq_target,foc_para->Iq);
    // foc_para->Id_target = (flux_wb - sqrtf(SQ(flux_wb) + 8.0f * SQ(ld_lq_diff * iq_ref))) / (4.0f * ld_lq_diff);
    // foc_para->Iq_target = SIGN(foc_para->Iq_target) * sqrtf(SQ(foc_para->Iq_target) - SQ(foc_para->Id_target));
}

