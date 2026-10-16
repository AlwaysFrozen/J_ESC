/**
 * @file foc_utils.h
 * @author jinchl (jcl709905683@gmail.com)
 * @brief 
 * @version 0.1
 * @date 2026-03-30
 * 
 * @copyright Copyright (c) 2026
 * 
 */


#pragma once
#include "../math/my_math.h"
#include "motor.h"
#include "motor_api.h"
#include "foc.h"

/**
 * @brief 计算弧度对应的正余弦值
 * 
 * @param theta 弧度
 * @param p_sin_cos 正余弦弦值指针
 */
static inline void Sin_Cos_Cal(float theta, Sin_Cos_t *p_sin_cos)
{
    p_sin_cos->sin = sinf(theta);
    p_sin_cos->cos = cosf(theta);
}

/**
 * @brief 等幅值3采样克拉克变换
 * 
 * @param uvw 三相坐标系变量
 * @param ab αβ坐标系变量
 */
static inline void Clarke_Transmission(UVW_Axis_t *uvw, AB_Axis_t *ab)
{
    ab->Alpha = (uvw->U - 0.5f * (uvw->V + uvw->W)) * _2_3;
    ab->Beta  = (_SQRT3_2 * (uvw->V - uvw->W)) * _2_3;
}

/**
 * @brief 等幅值2采样克拉克变换
 * 
 * @param uvw 三相坐标系变量
 * @param ab αβ坐标系变量
 */
static inline void Clarke_Transmission_2(UVW_Axis_t *uvw, AB_Axis_t *ab)
{
    ab->Alpha = uvw->U;
    ab->Beta  = _1_SQRT3 * (uvw->U + 2.0f * uvw->V);
}

/**
 * @brief 等幅值逆克拉克变换
 * 
 * @param ab αβ坐标系变量
 * @param uvw 三相坐标系变量
 */
static inline void Inverse_Clarke_Transmission(AB_Axis_t *ab, UVW_Axis_t *uvw)
{
    uvw->U = ab->Alpha;
    uvw->V = (-ab->Alpha + _SQRT3 * ab->Beta) * 0.5f;
    uvw->W = (-ab->Alpha - _SQRT3 * ab->Beta) * 0.5f;
}

/**
 * @brief 帕克变换
 * 
 * @param ab αβ坐标系变量
 * @param dq dq坐标系变量
 * @param theta 角度
 */
static inline void Park_Transmission(AB_Axis_t *ab, DQ_Axis_t *dq, float theta)
{
    float sin_theta = sinf(theta);
    float cos_theta = cosf(theta);

    dq->D = ab->Alpha * cos_theta + ab->Beta * sin_theta;
    dq->Q = -ab->Alpha * sin_theta + ab->Beta * cos_theta;
}

/**
 * @brief 逆帕克变换
 * 
 * @param dq dq坐标系变量
 * @param ab αβ坐标系变量
 * @param theta 角度
 */
static inline void Inverse_Park_Transmission(DQ_Axis_t *dq, AB_Axis_t *ab, float theta)
{
    float sin_theta = sinf(theta);
    float cos_theta = cosf(theta);

    ab->Alpha = dq->D * cos_theta - dq->Q * sin_theta;
    ab->Beta  = dq->D * sin_theta + dq->Q * cos_theta;
}

/**
 * @brief 快速帕克变换
 * 
 * @param ab αβ坐标系变量
 * @param dq dq坐标系变量
 * @param sin_cos 正余弦值
 */
static inline void Park_Transmission_Fast(AB_Axis_t *ab, DQ_Axis_t *dq, Sin_Cos_t sin_cos)
{
    dq->D = ab->Alpha * sin_cos.cos + ab->Beta * sin_cos.sin;
    dq->Q = -ab->Alpha * sin_cos.sin + ab->Beta * sin_cos.cos;
}

/**
 * @brief 快速逆帕克变换
 * 
 * @param dq dq坐标系变量
 * @param ab αβ坐标系变量
 * @param sin_cos 正余弦值
 */
static inline void Inverse_Park_Transmission_Fast(DQ_Axis_t *dq, AB_Axis_t *ab, Sin_Cos_t sin_cos)
{
    ab->Alpha = dq->D * sin_cos.cos - dq->Q * sin_cos.sin;
    ab->Beta  = dq->D * sin_cos.sin + dq->Q * sin_cos.cos;
}

/**
 * @brief 端电压到相电压转换
 * 
 * @param terminal 三相端电压
 * @param phase 三相相电压
 */
static inline void TerminalVol_To_PhaseVol(UVW_Axis_t *terminal, UVW_Axis_t *phase)
{
    phase->U = terminal->U * _2_3 - terminal->V * _1_3 - terminal->W * _1_3;
    phase->V = -terminal->U * _1_3 + terminal->V * _2_3 - terminal->W * _1_3;
    phase->W = -terminal->U * _1_3 - terminal->V * _1_3 + terminal->W * _2_3;
}

/**
 * @brief 基于αβ轴输出比例的空间矢量PWM调制
 * 
 * @param v_ab_rate αβ轴电压比例
 * @param duty 占空比
 * @return uint8_t 当前扇区
 */
uint8_t SVPWM_AB(AB_Axis_t *v_ab_rate, UVW_Axis_t *duty);

/**
 * @brief 基于αβ轴输出电压的空间矢量PWM调制
 * 
 * @param v_ab αβ轴电压
 * @param us_max 最大电压矢量幅值
 * @param duty 占空比
 * @return uint8_t 
 */
uint8_t SVPWM_AB_Voltage(AB_Axis_t *v_ab, float us_max, UVW_Axis_t *duty);

/**
 * @brief 基于dq轴输出比例的空间矢量PWM调制
 * 
 * @param u_dq_rate dq轴电压比例
 * @param angle 电气角度
 * @param duty 占空比
 * @return uint8_t 当前扇区
 */
uint8_t SVPWM_DQ(DQ_Axis_t *u_dq_rate, float angle, UVW_Axis_t *duty);

/**
 * @brief 基于dq轴输出电压的空间矢量PWM调制
 * 
 * @param u_dq dq轴电压比例
 * @param us_max 最大电压矢量幅值
 * @param angle 电气角度
 * @param duty 占空比
 * @return uint8_t 当前扇区
 */
uint8_t SVPWM_DQ_Voltage(DQ_Axis_t *u_dq, float us_max, float angle, UVW_Axis_t *duty);

/**
 * @brief 基于αβ轴输出比例的三次谐波注入正弦PWM调制
 * 
 * @param v_ab_rate αβ轴电压比例
 * @param duty 占空比
 */
static inline void SPWM_AB(AB_Axis_t *v_ab_rate, UVW_Axis_t *duty)
{
    UVW_Axis_t uvw;
    float max = 0, min = 0, ei;

    uvw.U   = v_ab_rate->Alpha;
    uvw.V   = (-v_ab_rate->Alpha + _SQRT3 * v_ab_rate->Beta) * 0.5f;
    uvw.W   = (-v_ab_rate->Alpha - _SQRT3 * v_ab_rate->Beta) * 0.5f;
    max     = fmaxf(uvw.U, fmaxf(uvw.V, uvw.W));
    min     = fminf(uvw.U, fminf(uvw.V, uvw.W));
    ei      = -0.5f * (max + min);
    uvw.U   = (uvw.U + ei) * _2_3 + 0.5f;
    uvw.V   = (uvw.V + ei) * _2_3 + 0.5f;
    uvw.W   = (uvw.W + ei) * _2_3 + 0.5f;
    duty->U = CONSTRAIN(uvw.U, 0.0f, 1.0f);
    duty->V = CONSTRAIN(uvw.V, 0.0f, 1.0f);
    duty->W = CONSTRAIN(uvw.W, 0.0f, 1.0f);
}

/**
 * @brief 基于αβ轴输出电压的三次谐波注入正弦PWM调制
 * 
 * @param v_ab αβ轴电压
 * @param inv_v_bus 母线电压的倒数
 * @param duty 占空比
 */
static inline void SPWM_AB_Voltage(AB_Axis_t *v_ab, float inv_v_bus, UVW_Axis_t *duty)
{
    UVW_Axis_t uvw;
    float max = 0, min = 0, ei;

    uvw.U   = v_ab->Alpha;
    uvw.V   = (-v_ab->Alpha + _SQRT3 * v_ab->Beta) * 0.5f;
    uvw.W   = (-v_ab->Alpha - _SQRT3 * v_ab->Beta) * 0.5f;
    max     = fmaxf(uvw.U, fmaxf(uvw.V, uvw.W));
    min     = fminf(uvw.U, fminf(uvw.V, uvw.W));
    ei      = -0.5f * (max + min);
    uvw.U   = (uvw.U + ei) * inv_v_bus + 0.5f;
    uvw.V   = (uvw.V + ei) * inv_v_bus + 0.5f;
    uvw.W   = (uvw.W + ei) * inv_v_bus + 0.5f;
    duty->U = CONSTRAIN(uvw.U, 0.0f, 1.0f);
    duty->V = CONSTRAIN(uvw.V, 0.0f, 1.0f);
    duty->W = CONSTRAIN(uvw.W, 0.0f, 1.0f);
}

/**
 * @brief 根据各相实测电流补偿互补PWM死区，过零区内线性过渡
 *
 * @param phase_current 与占空比相序一致的三相电流，单位A
 * @param compensation_value 死区补偿占空比
 * @param transition_current 达到完整补偿的电流幅值，单位A；非正值禁用补偿
 * @param duty_raw 原始占空比
 * @param duty_comp 各相补偿占空比
 * @param duty_out 补偿并限幅后的占空比
 */
static inline void PWM_Dead_Zone_Compensation(const UVW_Axis_t *phase_current, float compensation_value, float transition_current, const UVW_Axis_t *duty_raw, UVW_Axis_t *duty_comp, UVW_Axis_t *duty_out)
{
    /* 直接使用各相电流的符号和幅值，不依赖扇区或三相电流和为零的假设。 */
    float inv_transition_current = transition_current > 0.0f ? 1.0f / transition_current : 0.0f;
    duty_comp->U                 = compensation_value * CONSTRAIN(phase_current->U * inv_transition_current, -1.0f, 1.0f);
    duty_comp->V                 = compensation_value * CONSTRAIN(phase_current->V * inv_transition_current, -1.0f, 1.0f);
    duty_comp->W                 = compensation_value * CONSTRAIN(phase_current->W * inv_transition_current, -1.0f, 1.0f);

    duty_out->U = CONSTRAIN(duty_raw->U + duty_comp->U, 0.0f, 1.0f);
    duty_out->V = CONSTRAIN(duty_raw->V + duty_comp->V, 0.0f, 1.0f);
    duty_out->W = CONSTRAIN(duty_raw->W + duty_comp->W, 0.0f, 1.0f);
}

/**
 * @brief MTPA计算
 * 
 * @param p_motor_config 电机本体配置结构体指针
 * @param p_foc_status foc运行反馈结构体指针
 */
void MTPA_Cal(Motor_Config_t *p_motor_config, FOC_Status_t *p_foc_status);

/**
 * @brief 根据目标扭矩计算MTPA总电流参考值
 *
 * @param motor 电机本体配置结构体指针
 * @param torque_ref 扭矩参考
 * @return 带扭矩方向的定子电流矢量幅值参考
 */
float IPM_MTPA_From_Torque(const Motor_Config_t *motor, float torque_ref);

/**
 * @brief MTPV计算
 * 
 * @param p_motor_config 电机本体配置结构体指针
 * @param p_foc_status foc运行反馈结构体指针
 */
void MTPV_Cal(Motor_Config_t *p_motor_config, FOC_Status_t *p_foc_status);
