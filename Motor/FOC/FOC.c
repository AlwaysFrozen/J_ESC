#include "Board_Config.h"
#include "FOC.h"

/*
   B
    \
      \
        \
          \
            \_ _ _ _ _ _ _ A
            /
          /
        /
      /
    /  
   C
*/

/*
    ωe:electrical angle speed rads/s
    ψf:rotor flux Wb
    θ:electrical angle

    b相反电势超前a相120°,c相反电势滞后a相120°(左加右减,上加下减)
    ψa = ψf * cos(θ)
    ψb = ψf * cos(θ - pi *2 /3)
    ψb = ψf * cos(θ + pi *2 /3)

    E = dψf / dt
    Ea = - ψf * sin(θ)
    Ea = - ψf * sin(θ - pi *2 /3)
    Ea = - ψf * sin(θ + pi *2 /3)
    
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


/*
    电流极限圆
    SQ(Iq) + SQ(Id) <= SQ(Is)
    圆心 = (0,0)
    半径 = Is
    电压极限圆
    SQ(v_dq->Q) + SQ(v_dq->D) <= SQ(Us)
    SQ(Rs * Id - We * Lq * Iq) + SQ(Rs * Iq + We * Ld * Id + We * Flux) <= SQ(Us)
    高速时电阻压降较小,忽略 令 Rs = 0
    SQ(- We * Lq * Iq) + SQ(We * Ld * Id + We * Flux) <= SQ(Us)
    SQ(Id + Flux / LD) / SQ(Us / (We * Ld)) + SQ(Iq) / SQ(Us / (We * Lq)) <= 1
    圆心 = -Flux / Ld
    长轴 = (2 * Us) / (We / Ld)
    短轴 = (2 * Us) / (We / Lq)
*/


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
            int32_t t1 = (v_ab_rate->Alpha - _1_SQRT3 * v_ab_rate->Beta) * tim->Reload;
            int32_t t2 = (_2_SQRT3 * v_ab_rate->Beta) * tim->Reload;

            tC = (tim->Reload - t1 - t2) / 2;
            tB = tC + t2;
            tA = tB + t1;
            break;
        }

        // sector 2-3
        case 2:
        {
            // Vector on-times
            int32_t t2 = (v_ab_rate->Alpha + _1_SQRT3 * v_ab_rate->Beta) * tim->Reload;
            int32_t t3 = (-v_ab_rate->Alpha + _1_SQRT3 * v_ab_rate->Beta) * tim->Reload;

            tC = (tim->Reload - t2 - t3) / 2;
            tA = tC + t2;
            tB = tA + t3;
            break;
        }

        // sector 3-4
        case 3:
        {
            // Vector on-times
            int32_t t3 = (_2_SQRT3 * v_ab_rate->Beta) * tim->Reload;
            int32_t t4 = (-v_ab_rate->Alpha - _1_SQRT3 * v_ab_rate->Beta) * tim->Reload;

            tA = (tim->Reload - t3 - t4) / 2;
            tC = tA + t4;
            tB = tC + t3;
            break;
        }

        // sector 4-5
        case 4:
        {
            // Vector on-times
            int32_t t4 = (-v_ab_rate->Alpha + _1_SQRT3 * v_ab_rate->Beta) * tim->Reload;
            int32_t t5 = (-_2_SQRT3 * v_ab_rate->Beta) * tim->Reload;

            tA = (tim->Reload - t4 - t5) / 2;
            tB = tA + t4;
            tC = tB + t5;
            break;
        }

        // sector 5-6
        case 5:
        {
            // Vector on-times
            int32_t t5 = (-v_ab_rate->Alpha - _1_SQRT3 * v_ab_rate->Beta) * tim->Reload;
            int32_t t6 = (v_ab_rate->Alpha - _1_SQRT3 * v_ab_rate->Beta) * tim->Reload;

            tB = (tim->Reload - t5 - t6) / 2;
            tA = tB + t6;
            tC = tA + t5;
            break;
        }

        // sector 6-1
        case 6:
        {
            // Vector on-times
            int32_t t6 = (-_2_SQRT3 * v_ab_rate->Beta) * tim->Reload;
            int32_t t1 = (v_ab_rate->Alpha + _1_SQRT3 * v_ab_rate->Beta) * tim->Reload;

            tB = (tim->Reload - t6 - t1) / 2;
            tC = tB + t6;
            tA = tC + t1;
            break;
        }
    }

    tA = CONSTRAIN(tA, 0, tim->Reload);
    tB = CONSTRAIN(tB, 0, tim->Reload);
    tC = CONSTRAIN(tC, 0, tim->Reload);

    tim->CMP1 = tA;
    tim->CMP2 = tB;
    tim->CMP3 = tC;

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
            int32_t t1 = (v_ab_rate.Alpha - _1_SQRT3 * v_ab_rate.Beta) * tim->Reload;
            int32_t t2 = (_2_SQRT3 * v_ab_rate.Beta) * tim->Reload;

            tC = (tim->Reload - t1 - t2) / 2;
            tB = tC + t2;
            tA = tB + t1;
            break;
        }

        // sector 2-3
        case 2:
        {
            // Vector on-times
            int32_t t2 = (v_ab_rate.Alpha + _1_SQRT3 * v_ab_rate.Beta) * tim->Reload;
            int32_t t3 = (-v_ab_rate.Alpha + _1_SQRT3 * v_ab_rate.Beta) * tim->Reload;

            tC = (tim->Reload - t2 - t3) / 2;
            tA = tC + t2;
            tB = tA + t3;
            break;
        }

        // sector 3-4
        case 3:
        {
            // Vector on-times
            int32_t t3 = (_2_SQRT3 * v_ab_rate.Beta) * tim->Reload;
            int32_t t4 = (-v_ab_rate.Alpha - _1_SQRT3 * v_ab_rate.Beta) * tim->Reload;

            tA = (tim->Reload - t3 - t4) / 2;
            tC = tA + t4;
            tB = tC + t3;
            break;
        }

        // sector 4-5
        case 4:
        {
            // Vector on-times
            int32_t t4 = (-v_ab_rate.Alpha + _1_SQRT3 * v_ab_rate.Beta) * tim->Reload;
            int32_t t5 = (-_2_SQRT3 * v_ab_rate.Beta) * tim->Reload;

            tA = (tim->Reload - t4 - t5) / 2;
            tB = tA + t4;
            tC = tB + t5;
            break;
        }

        // sector 5-6
        case 5:
        {
            // Vector on-times
            int32_t t5 = (-v_ab_rate.Alpha - _1_SQRT3 * v_ab_rate.Beta) * tim->Reload;
            int32_t t6 = (v_ab_rate.Alpha - _1_SQRT3 * v_ab_rate.Beta) * tim->Reload;

            tB = (tim->Reload - t5 - t6) / 2;
            tA = tB + t6;
            tC = tA + t5;
            break;
        }

        // sector 6-1
        case 6:
        {
            // Vector on-times
            int32_t t6 = (-_2_SQRT3 * v_ab_rate.Beta) * tim->Reload;
            int32_t t1 = (v_ab_rate.Alpha + _1_SQRT3 * v_ab_rate.Beta) * tim->Reload;

            tB = (tim->Reload - t6 - t1) / 2;
            tC = tB + t6;
            tA = tC + t1;
            break;
        }
    }

    tA = CONSTRAIN(tA, 0, tim->Reload);
    tB = CONSTRAIN(tB, 0, tim->Reload);
    tC = CONSTRAIN(tC, 0, tim->Reload);

    tim->CMP1 = tA;
    tim->CMP2 = tB;
    tim->CMP3 = tC;

    return sector;
}

// void SVPWM_AB_Voltage(float v_ab_rate.Alpha, float v_ab_rate.Beta, uint32_t v_dc,uint8_t sector, uint16_t tim->Reload, int32_t tim->CMP1, int32_t tim->CMP2, int32_t tim->CMP3)
// {
//     // U1 > 0 -> A = 1
//     // U2 > 0 -> B = 1
//     // U3 > 0 -> C = 1
//     // N = 4C + 2B + A
//     float U1 = v_ab_rate.Beta;
//     float U2 = _SQRT3 * v_ab_rate.Alpha - v_ab_rate.Beta;
//     float U3 = -_SQRT3 * v_ab_rate.Alpha - v_ab_rate.Beta;
//     uint8_t N = 0;

//     if (U1 > 0)
//     {
//         N += 1;
//     }
//     if (U2 > 0)
//     {
//         N += 2;
//     }
//     if (U3 > 0)
//     {
//         N += 4;
//     }

//     float Tcmp1,Tcmp2,Tcmp3,Tx,Ty,f_temp,Ta,Tb,Tc;
//     Tcmp1 = 0.0F;
//     Tcmp2 = 0.0F;
//     Tcmp3 = 0.0F;
//     switch (N)
//     {
//         case 1:
//             sector = 2;
//             Tx = (-1.5F * v_ab_rate.Alpha + 0.866025388F * v_ab_rate.Beta) * (tim->Reload / v_dc);
//             Ty = (1.5F * v_ab_rate.Alpha + 0.866025388F * v_ab_rate.Beta) * (tim->Reload / v_dc);
//             break;

//         case 2:
//             sector = 6;
//             Tx = (1.5F * v_ab_rate.Alpha + 0.866025388F * v_ab_rate.Beta) * (tim->Reload / v_dc);
//             Ty = -(1.73205078F * v_ab_rate.Beta * tim->Reload / v_dc);
//             break;

//         case 3:
//             sector = 1;
//             Tx = -((-1.5F * v_ab_rate.Alpha + 0.866025388F * v_ab_rate.Beta) * (tim->Reload / v_dc));
//             Ty = 1.73205078F * v_ab_rate.Beta * tim->Reload / v_dc;
//             break;

//         case 4:
//             sector = 4;
//             Tx = -(1.73205078F * v_ab_rate.Beta * tim->Reload / v_dc);
//             Ty = (-1.5F * v_ab_rate.Alpha + 0.866025388F * v_ab_rate.Beta) * (tim->Reload / v_dc);
//             break;

//         case 5:
//             sector = 3;
//             Tx = 1.73205078F * v_ab_rate.Beta * tim->Reload / v_dc;
//             Ty = -((1.5F * v_ab_rate.Alpha + 0.866025388F * v_ab_rate.Beta) * (tim->Reload / v_dc));
//             break;

//         case 6:
//             sector = 5;
//             Tx = -((1.5F * v_ab_rate.Alpha + 0.866025388F * v_ab_rate.Beta) * (tim->Reload / v_dc));
//             Ty = -((-1.5F * v_ab_rate.Alpha + 0.866025388F * v_ab_rate.Beta) * (tim->Reload / v_dc));
//             break;

//     }

//     f_temp = Tx + Ty;
//     if (f_temp > tim->Reload)
//     {
//         Tx /= f_temp;
//         Ty /= (Tx + Ty);
//     }

//     Ta = (tim->Reload - (Tx + Ty)) / 4.0F;
//     Tb = Tx / 2.0F + Ta;
//     Tc = Ty / 2.0F + Tb;

//     switch (N)
//     {
//         case 1:
//             Tcmp1 = Tb;
//             Tcmp2 = Ta;
//             Tcmp3 = Tc;
//             break;

//         case 2:
//             Tcmp1 = Ta;
//             Tcmp2 = Tc;
//             Tcmp3 = Tb;
//             break;

//         case 3:
//             Tcmp1 = Ta;
//             Tcmp2 = Tb;
//             Tcmp3 = Tc;
//             break;

//         case 4:
//             Tcmp1 = Tc;
//             Tcmp2 = Tb;
//             Tcmp3 = Ta;
//             break;

//         case 5:
//             Tcmp1 = Tc;
//             Tcmp2 = Ta;
//             Tcmp3 = Tb;
//             break;

//         case 6:
//             Tcmp1 = Tb;
//             Tcmp2 = Tc;
//             Tcmp3 = Ta;
//             break;
//     }

//     tim->CMP1 = Tcmp1;
//     tim->CMP2 = Tcmp2;
//     tim->CMP3 = Tcmp3;
// }

uint8_t SVPWM_DQ(DQ_Axis_t *v_dq_rate, float angle, TIM_t *tim)
{
    uint8_t sector;
    float angle_out;
    float angle_offset;
    float Uout;    

    if (v_dq_rate->D)
    {
        Uout = NORM2_f(v_dq_rate->D, v_dq_rate->Q);
        angle_out = Normalize_Rad(angle + atan2f(v_dq_rate->Q, v_dq_rate->D));
    }
    else
    {
        Uout = v_dq_rate->Q;
        angle_out = Normalize_Rad(angle + _PI_2);
    }

    sector = floorf(angle_out / _PI_3);
    angle_offset = angle_out - sector * _PI_3;
    sector += 1;

    float st, ct;
    st = sinf(angle_offset);
    ct = cosf(angle_offset);

    /*
    t0~t7为三相全桥对应的8个矢量的作用时间
    T1为当前扇区的1号有效矢量的作用时间
    T2为当前扇区的2号有效矢量的作用时间
    T1、T2对应不同扇区t1~t6的作用时间
    T0为当前扇区的0矢量作用时间之和，又因为t0与t7矢量作用时长相等，所以为方便计算将T0除二即t0矢量的作用时长用作占空比计算
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

    Ta = CONSTRAIN(Ta, 0.0f, 1.0f);
    Tb = CONSTRAIN(Tb, 0.0f, 1.0f);
    Tc = CONSTRAIN(Tc, 0.0f, 1.0f);

    tim->CMP1 = tim->Reload * Ta;
    tim->CMP2 = tim->Reload * Tb;
    tim->CMP3 = tim->Reload * Tc;

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
        Uout = NORM2_f(v_dq->D, v_dq->Q) / us_max;
        angle_out = Normalize_Rad(angle + atan2f(v_dq->Q, v_dq->D));
    }
    else
    {
        Uout = v_dq->Q / us_max;
        angle_out = Normalize_Rad(angle + _PI_2);
    }

    sector = floorf(angle_out / _PI_3);
    angle_offset = angle_out - sector * _PI_3;
    sector += 1;

    float st, ct;
    st = sinf(angle_offset);
    ct = cosf(angle_offset);

    /*
    t0~t7为三相全桥对应的8个矢量的作用时间
    T1为当前扇区的1号有效矢量的作用时间
    T2为当前扇区的2号有效矢量的作用时间
    T1、T2对应不同扇区t1~t6的作用时间
    T0为当前扇区的0矢量作用时间之和，又因为t0与t7矢量作用时长相等，所以为方便计算将T0除二即t0矢量的作用时长用作占空比计算
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

    Ta = CONSTRAIN(Ta, 0.0f, 1.0f);
    Tb = CONSTRAIN(Tb, 0.0f, 1.0f);
    Tc = CONSTRAIN(Tc, 0.0f, 1.0f);

    tim->CMP1 = tim->Reload * Ta;
    tim->CMP2 = tim->Reload * Tb;
    tim->CMP3 = tim->Reload * Tc;

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
    uvw.U = CONSTRAIN(uvw.U, 0.0f, 1.0f);
    uvw.V = CONSTRAIN(uvw.V, 0.0f, 1.0f);
    uvw.W = CONSTRAIN(uvw.W, 0.0f, 1.0f);
    tim->CMP1 = tim->Reload * uvw.U;
    tim->CMP2 = tim->Reload * uvw.V;
    tim->CMP3 = tim->Reload * uvw.W;
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
    uvw.U = CONSTRAIN(uvw.U, 0.0f, 1.0f);
    uvw.V = CONSTRAIN(uvw.V, 0.0f, 1.0f);
    uvw.W = CONSTRAIN(uvw.W, 0.0f, 1.0f);
    tim->CMP1 = tim->Reload * uvw.U;
    tim->CMP2 = tim->Reload * uvw.V;
    tim->CMP3 = tim->Reload * uvw.W;
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

