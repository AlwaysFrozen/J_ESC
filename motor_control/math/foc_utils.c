/**
 * @file foc_utils.c
 * @author jinchl (jcl709905683@gmail.com)
 * @brief 
 * @version 0.1
 * @date 2026-03-30
 * 
 * @copyright Copyright (c) 2026
 * 
 */

#include "foc_utils.h"

#include "angle_utils.h"

/**
 * @brief 基于αβ轴输出比例的空间矢量PWM调制
 * 
 * @param v_ab_rate αβ轴电压比例
 * @param duty 占空比
 * @return uint8_t 当前扇区
 */
uint8_t SVPWM_AB(AB_Axis_t *v_ab_rate, UVW_Axis_t *duty)
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

    float tA, tB, tC;

    switch (sector)
    {
    // sector 1-2
    case 1: {
        // Vector on-times
        float t1 = (v_ab_rate->Alpha - _1_SQRT3 * v_ab_rate->Beta);
        float t2 = (_2_SQRT3 * v_ab_rate->Beta);

        tC = (1.0f - t1 - t2) / 2;
        tB = tC + t2;
        tA = tB + t1;
        break;
    }

    // sector 2-3
    case 2: {
        // Vector on-times
        float t2 = (v_ab_rate->Alpha + _1_SQRT3 * v_ab_rate->Beta);
        float t3 = (-v_ab_rate->Alpha + _1_SQRT3 * v_ab_rate->Beta);

        tC = (1.0f - t2 - t3) / 2;
        tA = tC + t2;
        tB = tA + t3;
        break;
    }

    // sector 3-4
    case 3: {
        // Vector on-times
        float t3 = (_2_SQRT3 * v_ab_rate->Beta);
        float t4 = (-v_ab_rate->Alpha - _1_SQRT3 * v_ab_rate->Beta);

        tA = (1.0f - t3 - t4) / 2;
        tC = tA + t4;
        tB = tC + t3;
        break;
    }

    // sector 4-5
    case 4: {
        // Vector on-times
        float t4 = (-v_ab_rate->Alpha + _1_SQRT3 * v_ab_rate->Beta);
        float t5 = (-_2_SQRT3 * v_ab_rate->Beta);

        tA = (1.0f - t4 - t5) / 2;
        tB = tA + t4;
        tC = tB + t5;
        break;
    }

    // sector 5-6
    case 5: {
        // Vector on-times
        float t5 = (-v_ab_rate->Alpha - _1_SQRT3 * v_ab_rate->Beta);
        float t6 = (v_ab_rate->Alpha - _1_SQRT3 * v_ab_rate->Beta);

        tB = (1.0f - t5 - t6) / 2;
        tA = tB + t6;
        tC = tA + t5;
        break;
    }

    // sector 6-1
    case 6: {
        // Vector on-times
        float t6 = (-_2_SQRT3 * v_ab_rate->Beta);
        float t1 = (v_ab_rate->Alpha + _1_SQRT3 * v_ab_rate->Beta);

        tB = (1.0f - t6 - t1) / 2;
        tC = tB + t6;
        tA = tC + t1;
        break;
    }
    }

    tA = CONSTRAIN(tA, 0, 1.0f);
    tB = CONSTRAIN(tB, 0, 1.0f);
    tC = CONSTRAIN(tC, 0, 1.0f);

    duty->U = tA;
    duty->V = tB;
    duty->W = tC;

    return sector;
}

/**
 * @brief 基于αβ轴输出电压的空间矢量PWM调制
 * 
 * @param v_ab αβ轴电压
 * @param us_max 最大电压矢量幅值
 * @param duty 占空比
 * @return uint8_t 
 */
uint8_t SVPWM_AB_Voltage(AB_Axis_t *v_ab, float us_max, UVW_Axis_t *duty)
{
    AB_Axis_t v_ab_rate;

    v_ab_rate.Alpha = v_ab->Alpha / us_max;
    v_ab_rate.Beta  = v_ab->Beta / us_max;

    return SVPWM_AB(&v_ab_rate, duty);
}

/**
 * @brief 基于dq轴输出比例的空间矢量PWM调制
 * 
 * @param u_dq_rate dq轴电压比例
 * @param angle 电气角度
 * @param duty 占空比
 * @return uint8_t 当前扇区
 */
uint8_t SVPWM_DQ(DQ_Axis_t *u_dq_rate, float angle, UVW_Axis_t *duty)
{
    uint8_t sector;
    float angle_out;
    float angle_offset;
    float Uout;

    if (u_dq_rate->D)
    {
        Uout      = NORM2_f(u_dq_rate->D, u_dq_rate->Q);
        angle_out = Normalize_Rad(angle + atan2f(u_dq_rate->Q, u_dq_rate->D));
    }
    else
    {
        Uout      = u_dq_rate->Q;
        angle_out = Normalize_Rad(angle + _PI_2);
    }

    sector        = floorf(angle_out / _PI_3);
    angle_offset  = angle_out - sector * _PI_3;
    sector       += 1;

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

    duty->U = Ta;
    duty->V = Tb;
    duty->W = Tc;

    return sector;
}

/**
 * @brief 基于dq轴输出电压的空间矢量PWM调制
 * 
 * @param u_dq dq轴电压比例
 * @param us_max 最大电压矢量幅值
 * @param angle 电气角度
 * @param duty 占空比
 * @return uint8_t 当前扇区
 */
uint8_t SVPWM_DQ_Voltage(DQ_Axis_t *u_dq, float us_max, float angle, UVW_Axis_t *duty)
{
    DQ_Axis_t u_dq_rate;

    u_dq_rate.D = u_dq->D / us_max;
    u_dq_rate.Q = u_dq->Q / us_max;

    return SVPWM_DQ(&u_dq_rate, angle, duty);
}

