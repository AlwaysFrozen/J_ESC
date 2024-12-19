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


void Clarke_Transmission(UVW_Axis_t *uvw, AB_Axis_t *ab)
{
    ab->Alpha = (uvw->U - 0.5f * (uvw->V + uvw->W)) * _2_3;
    ab->Beta = (_SQRT3_2 * (uvw->V - uvw->W)) * _2_3;
}

void Clarke_Transmission_2(UVW_Axis_t *uvw, AB_Axis_t *ab)
{
    ab->Alpha = uvw->U;
    ab->Beta = _1_SQRT3 * (uvw->U + 2 * uvw->V);
}

void Inverse_Clarke_Transmission(AB_Axis_t *ab, UVW_Axis_t *uvw)
{
    uvw->U = ab->Alpha;
    uvw->V = (-ab->Alpha + _SQRT3 * ab->Beta) / 2;
    uvw->W = (-ab->Alpha - _SQRT3 * ab->Beta) / 2;
}

void Park_Transmission(AB_Axis_t *ab, DQ_Axis_t *dq, float theta)
{
    float sin_theta = sinf(theta);
    float cos_theta = cosf(theta);

    dq->D = ab->Alpha * cos_theta + ab->Beta * sin_theta;
    dq->Q = -ab->Alpha * sin_theta + ab->Beta * cos_theta;
}

void Inverse_Park_Transmission(DQ_Axis_t *dq, AB_Axis_t *ab, float theta)
{
    float sin_theta = sinf(theta);
    float cos_theta = cosf(theta);

    ab->Alpha = dq->D * cos_theta - dq->Q * sin_theta;
    ab->Beta = dq->D * sin_theta + dq->Q * cos_theta;
}

uint8_t SVPWM_AB(AB_Axis_t *v_ab_rate, TIM_t *tim)
{
    uint8_t sector;

    if (v_ab_rate->Beta >= 0.0f)
    {
        if (v_ab_rate->Alpha > 0.0f)
        {
            if (v_ab_rate->Beta / v_ab_rate->Alpha >= _SQRT3)
            {
                sector = 2;
            }
            else
            {
                sector = 1;
            }
        }
        else if (v_ab_rate->Alpha < 0.0f)
        {
            if (v_ab_rate->Beta / v_ab_rate->Alpha >= -_SQRT3)
            {
                sector = 3;
            }
            else
            {
                sector = 2;
            }
        }
        else
        {
            sector = 2;
        }
    }
    else
    {
        if (v_ab_rate->Alpha > 0.0f)
        {
            if (v_ab_rate->Beta / v_ab_rate->Alpha >= -_SQRT3)
            {
                sector = 6;
            }
            else
            {
                sector = 5;
            }
        }
        else if (v_ab_rate->Alpha < 0.0f)
        {
            if (v_ab_rate->Beta / v_ab_rate->Alpha >= _SQRT3)
            {
                sector = 5;
            }
            else
            {
                sector = 4;
            }
        }
        else
        {
            sector = 5;
        }
    }

    int32_t tA, tB, tC;

    switch (sector)
    {
        // sector 1-2
        case 1:
        {
            // Vector on-times
            int32_t t1 = (v_ab_rate->Alpha - _1_SQRT3 * v_ab_rate->Beta) * tim->ARR;
            int32_t t2 = (_2_SQRT3 * v_ab_rate->Beta) * tim->ARR;

            tC = (tim->ARR - t1 - t2) / 2;
            tB = tC + t2;
            tA = tB + t1;
            break;
        }

        // sector 2-3
        case 2:
        {
            // Vector on-times
            int32_t t2 = (v_ab_rate->Alpha + _1_SQRT3 * v_ab_rate->Beta) * tim->ARR;
            int32_t t3 = (-v_ab_rate->Alpha + _1_SQRT3 * v_ab_rate->Beta) * tim->ARR;

            tC = (tim->ARR - t2 - t3) / 2;
            tA = tC + t2;
            tB = tA + t3;
            break;
        }

        // sector 3-4
        case 3:
        {
            // Vector on-times
            int32_t t3 = (_2_SQRT3 * v_ab_rate->Beta) * tim->ARR;
            int32_t t4 = (-v_ab_rate->Alpha - _1_SQRT3 * v_ab_rate->Beta) * tim->ARR;

            tA = (tim->ARR - t3 - t4) / 2;
            tC = tA + t4;
            tB = tC + t3;
            break;
        }

        // sector 4-5
        case 4:
        {
            // Vector on-times
            int32_t t4 = (-v_ab_rate->Alpha + _1_SQRT3 * v_ab_rate->Beta) * tim->ARR;
            int32_t t5 = (-_2_SQRT3 * v_ab_rate->Beta) * tim->ARR;

            tA = (tim->ARR - t4 - t5) / 2;
            tB = tA + t4;
            tC = tB + t5;
            break;
        }

        // sector 5-6
        case 5:
        {
            // Vector on-times
            int32_t t5 = (-v_ab_rate->Alpha - _1_SQRT3 * v_ab_rate->Beta) * tim->ARR;
            int32_t t6 = (v_ab_rate->Alpha - _1_SQRT3 * v_ab_rate->Beta) * tim->ARR;

            tB = (tim->ARR - t5 - t6) / 2;
            tA = tB + t6;
            tC = tA + t5;
            break;
        }

        // sector 6-1
        case 6:
        {
            // Vector on-times
            int32_t t6 = (-_2_SQRT3 * v_ab_rate->Beta) * tim->ARR;
            int32_t t1 = (v_ab_rate->Alpha + _1_SQRT3 * v_ab_rate->Beta) * tim->ARR;

            tB = (tim->ARR - t6 - t1) / 2;
            tC = tB + t6;
            tA = tC + t1;
            break;
        }
    }

    tA = _constrain(tA, 0, tim->ARR);
    tB = _constrain(tB, 0, tim->ARR);
    tC = _constrain(tC, 0, tim->ARR);

    tim->CCR.CCR1 = tA;
    tim->CCR.CCR2 = tB;
    tim->CCR.CCR3 = tC;

    return sector;
}

uint8_t SVPWM_AB_Voltage(AB_Axis_t *v_ab, float us_max, TIM_t *tim)
{
    uint8_t sector;
    AB_Axis_t v_ab_rate;

    v_ab_rate.Alpha = v_ab->Alpha / us_max;
    v_ab_rate.Beta = v_ab->Beta / us_max;

    if (v_ab_rate.Beta >= 0.0f)
    {
        if (v_ab_rate.Alpha > 0.0f)
        {
            if (v_ab_rate.Beta / v_ab_rate.Alpha >= _SQRT3)
            {
                sector = 2;
            }
            else
            {
                sector = 1;
            }
        }
        else if (v_ab_rate.Alpha < 0.0f)
        {
            if (v_ab_rate.Beta / v_ab_rate.Alpha >= -_SQRT3)
            {
                sector = 3;
            }
            else
            {
                sector = 2;
            }
        }
        else
        {
            sector = 2;
        }
    }
    else
    {
        if (v_ab_rate.Alpha > 0.0f)
        {
            if (v_ab_rate.Beta / v_ab_rate.Alpha >= -_SQRT3)
            {
                sector = 6;
            }
            else
            {
                sector = 5;
            }
        }
        else if (v_ab_rate.Alpha < 0.0f)
        {
            if (v_ab_rate.Beta / v_ab_rate.Alpha >= _SQRT3)
            {
                sector = 5;
            }
            else
            {
                sector = 4;
            }
        }
        else
        {
            sector = 5;
        }
    }

    int32_t tA, tB, tC;

    switch (sector)
    {
        // sector 1-2
        case 1:
        {
            // Vector on-times
            int32_t t1 = (v_ab_rate.Alpha - _1_SQRT3 * v_ab_rate.Beta) * tim->ARR;
            int32_t t2 = (_2_SQRT3 * v_ab_rate.Beta) * tim->ARR;

            tC = (tim->ARR - t1 - t2) / 2;
            tB = tC + t2;
            tA = tB + t1;
            break;
        }

        // sector 2-3
        case 2:
        {
            // Vector on-times
            int32_t t2 = (v_ab_rate.Alpha + _1_SQRT3 * v_ab_rate.Beta) * tim->ARR;
            int32_t t3 = (-v_ab_rate.Alpha + _1_SQRT3 * v_ab_rate.Beta) * tim->ARR;

            tC = (tim->ARR - t2 - t3) / 2;
            tA = tC + t2;
            tB = tA + t3;
            break;
        }

        // sector 3-4
        case 3:
        {
            // Vector on-times
            int32_t t3 = (_2_SQRT3 * v_ab_rate.Beta) * tim->ARR;
            int32_t t4 = (-v_ab_rate.Alpha - _1_SQRT3 * v_ab_rate.Beta) * tim->ARR;

            tA = (tim->ARR - t3 - t4) / 2;
            tC = tA + t4;
            tB = tC + t3;
            break;
        }

        // sector 4-5
        case 4:
        {
            // Vector on-times
            int32_t t4 = (-v_ab_rate.Alpha + _1_SQRT3 * v_ab_rate.Beta) * tim->ARR;
            int32_t t5 = (-_2_SQRT3 * v_ab_rate.Beta) * tim->ARR;

            tA = (tim->ARR - t4 - t5) / 2;
            tB = tA + t4;
            tC = tB + t5;
            break;
        }

        // sector 5-6
        case 5:
        {
            // Vector on-times
            int32_t t5 = (-v_ab_rate.Alpha - _1_SQRT3 * v_ab_rate.Beta) * tim->ARR;
            int32_t t6 = (v_ab_rate.Alpha - _1_SQRT3 * v_ab_rate.Beta) * tim->ARR;

            tB = (tim->ARR - t5 - t6) / 2;
            tA = tB + t6;
            tC = tA + t5;
            break;
        }

        // sector 6-1
        case 6:
        {
            // Vector on-times
            int32_t t6 = (-_2_SQRT3 * v_ab_rate.Beta) * tim->ARR;
            int32_t t1 = (v_ab_rate.Alpha + _1_SQRT3 * v_ab_rate.Beta) * tim->ARR;

            tB = (tim->ARR - t6 - t1) / 2;
            tC = tB + t6;
            tA = tC + t1;
            break;
        }
    }

    tA = _constrain(tA, 0, tim->ARR);
    tB = _constrain(tB, 0, tim->ARR);
    tC = _constrain(tC, 0, tim->ARR);

    tim->CCR.CCR1 = tA;
    tim->CCR.CCR2 = tB;
    tim->CCR.CCR3 = tC;

    return sector;
}

uint8_t SVPWM_DQ(DQ_Axis_t *v_dq_rate, float angle, TIM_t *tim)
{
    uint8_t sector;
    float angle_out;
    float angle_offset;
    float Uout;    

    if (v_dq_rate->D)
    {
        Uout = sqrtf(v_dq_rate->D * v_dq_rate->D + v_dq_rate->Q * v_dq_rate->Q);
        angle_out = Normalize_Angle(angle + atan2f(v_dq_rate->Q, v_dq_rate->D));
    }
    else
    {
        Uout = v_dq_rate->Q;
        angle_out = Normalize_Angle(angle + _PI_2);
    }

    sector = floorf(angle_out / _PI_3);
    angle_offset = angle_out - sector * _PI_3;
    sector += 1;

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
    switch (sector)
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

    tim->CCR.CCR1 = tim->ARR * Ta;
    tim->CCR.CCR2 = tim->ARR * Tb;
    tim->CCR.CCR3 = tim->ARR * Tc;

    return sector;
}

uint8_t SVPWM_DQ_Voltage(DQ_Axis_t *v_dq, float us_max, float angle, TIM_t *tim)
{
    uint8_t sector;
    float angle_out;
    float angle_offset;
    float Uout;    

    if (v_dq->D)
    {
        Uout = sqrtf(v_dq->D * v_dq->D + v_dq->Q * v_dq->Q) / us_max;
        angle_out = Normalize_Angle(angle + atan2f(v_dq->Q, v_dq->D));
    }
    else
    {
        Uout = v_dq->Q / us_max;
        angle_out = Normalize_Angle(angle + _PI_2);
    }

    sector = floorf(angle_out / _PI_3);
    angle_offset = angle_out - sector * _PI_3;
    sector += 1;

    float st, ct;
    st = sinf(angle_offset);
    ct = cosf(angle_offset);
    float T1 = Uout * (ct - _1_SQRT3 * st);
    float T2 = _2_SQRT3 * Uout * st;
    float T0 = (1.0f - T1 - T2) * 0.5f; // actually this T0 is (t0 + t7) / 2,because t0 == t7

    float Ta, Tb, Tc;
    switch (sector)
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

    tim->CCR.CCR1 = tim->ARR * Ta;
    tim->CCR.CCR2 = tim->ARR * Tb;
    tim->CCR.CCR3 = tim->ARR * Tc;

    return sector;
}

void SPWM_AB(AB_Axis_t *v_ab_rate, TIM_t *tim)
{
    UVW_Axis_t uvw;
    float max = 0,min = 0,ei;

    Inverse_Clarke_Transmission(v_ab_rate,&uvw);
    max = uvw.U > uvw.V ? uvw.U : uvw.V;
    max = uvw.W > max ? uvw.W : max;
    min = uvw.U < uvw.V ? uvw.U : uvw.V;
    min = uvw.W < min ? uvw.W : min;
    ei = -0.5f * (max + min);
    uvw.U = uvw.U + ei;
    uvw.V = uvw.V + ei;
    uvw.W = uvw.W + ei;
    uvw.U = uvw.U * _2_3;
    uvw.V = uvw.V * _2_3;
    uvw.W = uvw.W * _2_3;
    uvw.U += 0.5f;
    uvw.V += 0.5f;
    uvw.W += 0.5f;
    uvw.U = _constrain(uvw.U, 0.0f, 1.0f);
    uvw.V = _constrain(uvw.V, 0.0f, 1.0f);
    uvw.W = _constrain(uvw.W, 0.0f, 1.0f);
    tim->CCR.CCR1 = tim->ARR * uvw.U;
    tim->CCR.CCR2 = tim->ARR * uvw.V;
    tim->CCR.CCR3 = tim->ARR * uvw.W;
}

void SPWM_AB_Voltage(AB_Axis_t *v_ab, uint32_t v_dc, TIM_t *tim)
{
    UVW_Axis_t uvw;
    float max = 0,min = 0,ei;

    Inverse_Clarke_Transmission(v_ab,&uvw);
    max = uvw.U > uvw.V ? uvw.U : uvw.V;
    max = uvw.W > max ? uvw.W : max;
    min = uvw.U < uvw.V ? uvw.U : uvw.V;
    min = uvw.W < min ? uvw.W : min;
    ei = -0.5f * (max + min);
    uvw.U = uvw.U + ei;
    uvw.V = uvw.V + ei;
    uvw.W = uvw.W + ei;
    uvw.U = uvw.U / v_dc;
    uvw.V = uvw.V / v_dc;
    uvw.W = uvw.W / v_dc;
    uvw.U += 0.5f;
    uvw.V += 0.5f;
    uvw.W += 0.5f;
    uvw.U = _constrain(uvw.U, 0.0f, 1.0f);
    uvw.V = _constrain(uvw.V, 0.0f, 1.0f);
    uvw.W = _constrain(uvw.W, 0.0f, 1.0f);
    tim->CCR.CCR1 = tim->ARR * uvw.U;
    tim->CCR.CCR2 = tim->ARR * uvw.V;
    tim->CCR.CCR3 = tim->ARR * uvw.W;
}

void MTPA_Cal(FOC_CONTROL_t *ctrl,FOC_Para_t * para)
{
    // // https://zhuanlan.zhihu.com/p/558759346
    // para->I_dq_target.D = SQ(para->I_dq.Q) * ctrl->Ldiff / ctrl->Flux;

    // https://zhuanlan.zhihu.com/p/624474437
    para->I_dq_target.D = (sqrtf(SQ(ctrl->Flux) + 4 * SQ(ctrl->Ldiff) * SQ(para->I_dq.Q)) - ctrl->Flux) / (2 * ctrl->Ldiff);

    // // form VESC
    // float iq_ref = Min_Abs(para->I_dq_target.Q,para->I_dq.Q);
    // para->I_dq_target.D = (sqrtf(SQ(ctrl->Flux) + 8 * SQ(ctrl->Ldiff * iq_ref)) - ctrl->Flux) / (4 * ctrl->Ldiff);
}

