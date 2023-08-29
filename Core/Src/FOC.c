#include "Board_Config.h"
#include "FOC.h"

// limited composite vector
bool saturate_vector_2d(float *x, float *y, float max)
{
    bool retval = false;
    float mag = sqrtf(SQ(*x) + SQ(*y));
    max = fabsf(max);

    if (mag < 1e-10f)
    {
        mag = 1e-10f;
    }

    if (mag > max)
    {
        const float f = max / mag;
        *x *= f;
        *y *= f;
        retval = true;
    }

    return retval;
}

// limited angle to 0~2pi rads
float Normalize_Angle(float angle)
{
    float a = fmodf(angle, _2PI);
    return a >= 0 ? a : (a + _2PI);
}

// limited angle to 0~pi rads
float Normalize_Angle_PI(float angle)
{
    float a = fmodf(angle, _PI);
    return a >= 0 ? a : (a + _PI);
}

// limited angle to 0~360 degree
float Normalize_Angle_Degree(float angle)
{
    float a = fmodf(angle, 360.0f);
    return a >= 0 ? a : (a + 360.0f);
}

// limited angle to 0~180 degree
float Normalize_Angle_180Degree(float angle)
{
    float a = fmodf(angle, 180.0f);
    return a >= 0 ? a : (a + 180.0f);
}

float ABS_Angle_Delta(float angle0,float angle1)
{
    float angle = 0;
    float abs_delta = fabsf(angle0 - angle1);

    angle0 = Normalize_Angle(angle0);
    angle1 = Normalize_Angle(angle1);

    if (abs_delta < _PI)
    {
        angle = abs_delta;
    }
    else
    {
        angle = _2PI - abs_delta;
    }

    return angle;
}

float ABS_Angle_Delta_Degree(float angle0, float angle1)
{
    float angle = 0;
    float abs_delta = fabsf(angle0 - angle1);

    angle0 = Normalize_Angle_Degree(angle0);
    angle1 = Normalize_Angle_Degree(angle1);

    if (abs_delta < 180)
    {
        angle = abs_delta;
    }
    else
    {
        angle = 360 - abs_delta;
    }

    return angle;
}

void Clarke_Transmission(float a, float b, float c, float *alpha, float *beta)
{
    #if HAVE_3_PHASE_CURRENT_SENSOR
    *alpha = (a - 0.5f * (b + c)) * 0.6666666f;
    *beta = (_SQRT3_2 * (b - c)) * 0.6666666f;
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

    uint32_t tA, tB, tC;

    switch (*sector)
    {
        // sector 1-2
        case 1:
        {
            // Vector on-times
            uint32_t t1 = (v_alpha_rate - _1_SQRT3 * v_beta_rate) * period;
            uint32_t t2 = (_2_SQRT3 * v_beta_rate) * period;

            tC = (period - t1 - t2) / 2;
            tB = tC + t2;
            tA = tB + t1;
            break;
        }

        // sector 2-3
        case 2:
        {
            // Vector on-times
            uint32_t t2 = (v_alpha_rate + _1_SQRT3 * v_beta_rate) * period;
            uint32_t t3 = (-v_alpha_rate + _1_SQRT3 * v_beta_rate) * period;

            tC = (period - t2 - t3) / 2;
            tA = tC + t2;
            tB = tA + t3;
            break;
        }

        // sector 3-4
        case 3:
        {
            // Vector on-times
            uint32_t t3 = (_2_SQRT3 * v_beta_rate) * period;
            uint32_t t4 = (-v_alpha_rate - _1_SQRT3 * v_beta_rate) * period;

            tA = (period - t3 - t4) / 2;
            tC = tA + t4;
            tB = tC + t3;
            break;
        }

        // sector 4-5
        case 4:
        {
            // Vector on-times
            uint32_t t4 = (-v_alpha_rate + _1_SQRT3 * v_beta_rate) * period;
            uint32_t t5 = (-_2_SQRT3 * v_beta_rate) * period;

            tA = (period - t4 - t5) / 2;
            tB = tA + t4;
            tC = tB + t5;
            break;
        }

        // sector 5-6
        case 5:
        {
            // Vector on-times
            uint32_t t5 = (-v_alpha_rate - _1_SQRT3 * v_beta_rate) * period;
            uint32_t t6 = (v_alpha_rate - _1_SQRT3 * v_beta_rate) * period;

            tB = (period - t5 - t6) / 2;
            tA = tB + t6;
            tC = tA + t5;
            break;
        }

        // sector 6-1
        case 6:
        {
            // Vector on-times
            uint32_t t6 = (-_2_SQRT3 * v_beta_rate) * period;
            uint32_t t1 = (v_alpha_rate + _1_SQRT3 * v_beta_rate) * period;

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
    // U1 > 0 -> A = 1
    // U2 > 0 -> B = 1
    // U3 > 0 -> C = 1
    // N = 4C + 2B + A
    float U1 = v_beta;
    float U2 = _SQRT3 * v_alpha - v_beta;
    float U3 = -_SQRT3 * v_alpha - v_beta;
    uint8_t N = 0;

    if (U1 > 0)
    {
        N += 1;
    }
    if (U2 > 0)
    {
        N += 2;
    }
    if (U3 > 0)
    {
        N += 4;
    }

    float Tcmp1,Tcmp2,Tcmp3,Tx,Ty,f_temp,Ta,Tb,Tc;
    Tcmp1 = 0.0F;
    Tcmp2 = 0.0F;
    Tcmp3 = 0.0F;
    switch (N)
    {
        case 1:
            *sector = 2;
            Tx = (-1.5F * v_alpha + 0.866025388F * v_beta) * (period / v_dc);
            Ty = (1.5F * v_alpha + 0.866025388F * v_beta) * (period / v_dc);
            break;

        case 2:
            *sector = 6;
            Tx = (1.5F * v_alpha + 0.866025388F * v_beta) * (period / v_dc);
            Ty = -(1.73205078F * v_beta * period / v_dc);
            break;

        case 3:
            *sector = 1;
            Tx = -((-1.5F * v_alpha + 0.866025388F * v_beta) * (period / v_dc));
            Ty = 1.73205078F * v_beta * period / v_dc;
            break;

        case 4:
            *sector = 4;
            Tx = -(1.73205078F * v_beta * period / v_dc);
            Ty = (-1.5F * v_alpha + 0.866025388F * v_beta) * (period / v_dc);
            break;

        case 5:
            *sector = 3;
            Tx = 1.73205078F * v_beta * period / v_dc;
            Ty = -((1.5F * v_alpha + 0.866025388F * v_beta) * (period / v_dc));
            break;

        case 6:
            *sector = 5;
            Tx = -((1.5F * v_alpha + 0.866025388F * v_beta) * (period / v_dc));
            Ty = -((-1.5F * v_alpha + 0.866025388F * v_beta) * (period / v_dc));
            break;

    }

    f_temp = Tx + Ty;
    if (f_temp > period)
    {
        Tx /= f_temp;
        Ty /= (Tx + Ty);
    }

    Ta = (period - (Tx + Ty)) / 4.0F;
    Tb = Tx / 2.0F + Ta;
    Tc = Ty / 2.0F + Tb;

    switch (N)
    {
        case 1:
            Tcmp1 = Tb;
            Tcmp2 = Ta;
            Tcmp3 = Tc;
            break;

        case 2:
            Tcmp1 = Ta;
            Tcmp2 = Tc;
            Tcmp3 = Tb;
            break;

        case 3:
            Tcmp1 = Ta;
            Tcmp2 = Tb;
            Tcmp3 = Tc;
            break;

        case 4:
            Tcmp1 = Tc;
            Tcmp2 = Tb;
            Tcmp3 = Ta;
            break;

        case 5:
            Tcmp1 = Tc;
            Tcmp2 = Ta;
            Tcmp3 = Tb;
            break;

        case 6:
            Tcmp1 = Tb;
            Tcmp2 = Tc;
            Tcmp3 = Ta;
            break;
    }

    *CMP1 = Tcmp1;
    *CMP2 = Tcmp2;
    *CMP3 = Tcmp3;
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

void MTPA_Cal(FOC_Para_t * foc_para,float flux_wb,float Ld,float Lq)
{
    float ld_lq_diff = Ld - Lq;

    // https://zhuanlan.zhihu.com/p/558759346
    // foc_para->Id_target = SQ(foc_para->Iq_ref) * ld_lq_diff / flux_wb;
    foc_para->Id_target = SQ(foc_para->Iq) * ld_lq_diff / flux_wb;

    // // https://zhuanlan.zhihu.com/p/624474437
    // foc_para->Id_target = (sqrtf(SQ(flux_wb) + 4 * SQ(ld_lq_diff) * SQ(foc_para->Iq)) - flux_wb) / (2 * ld_lq_diff);

    // // form VESC
    // float iq_ref = Min_Abs(foc_para->Iq_ref,foc_para->Iq);
    // foc_para->Id_target = (flux_wb - sqrtf(SQ(flux_wb) + 8.0f * SQ(ld_lq_diff * iq_ref))) / (4.0f * ld_lq_diff);
    // foc_para->Iq_ref = SIGN(foc_para->Iq_ref) * sqrtf(SQ(foc_para->Iq_ref) - SQ(foc_para->Id_target));
}

void FOC_Pll_Run(float ang, float *ang_last,float *speed,float dt,float kp,float ki)
{
    UTILS_NAN_ZERO(*ang_last);
    float delta_theta = ang - *ang_last;
    delta_theta = Normalize_Angle(delta_theta);
    UTILS_NAN_ZERO(*speed);
    *ang_last += (*speed + kp * delta_theta) * dt;
    *ang_last = Normalize_Angle(*ang_last);
    *speed += ki * delta_theta * dt;
}
