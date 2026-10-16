/**
 * @file pid.h
 * @author jinchl (jcl709905683@gmail.com)
 * @brief PID 控制器的数据结构、在线计算接口及各控制环自动整定接口
 * @version 0.1
 * @date 2026-03-11
 * 
 * @copyright Copyright (c) 2026
 * 
 */


#pragma once
#include "utils.h"
#include "filter.h"

typedef struct
{
    float kp;
    float ki;
    float kd;
    float kb; ///< 反算抗饱和增益，单位 1/s
    float alpha;
    float beta;
    float gamma;
} PID_Config_t;

typedef struct
{
    float limit_max;
    float limit_min;
} PID_Limit_t;

typedef struct
{
    float proportional;
    float integral;
    float derivative;
    float derivative_f;

    float out_temp;
    float out;

    float err_k0;
    float err_k1;
    float err_k2;

    bool saturation;
} PID_Status_t;

typedef struct
{
    float dt;
    PID_Config_t config;
    PID_Limit_t limit;
    PID_Status_t status;
} PID_t;

/**
 * @brief 设置 PID 控制器的输出上下限
 *
 * @param[in,out] pPID PID 控制器对象
 * @param[in] limit_max 输出上限
 * @param[in] limit_min 输出下限
 */
static inline void PID_Set_Limit(PID_t *pPID, float limit_max, float limit_min)
{
    pPID->limit.limit_max = limit_max;
    pPID->limit.limit_min = limit_min;
}

/**
 * @brief 设置 PID 控制器关于零点对称的输出限幅
 *
 * @param[in,out] pPID PID 控制器对象
 * @param[in] limit 输出绝对值上限，输出范围为 [-limit, limit]
 */
static inline void PID_Set_Abs_Limit(PID_t *pPID, float limit)
{
    pPID->limit.limit_max = limit;
    pPID->limit.limit_min = -limit;
}

/**
 * @brief 执行一次并联式 PI 控制计算
 *
 * 控制律为 @f$u=u_{ff}+K_p e+\int K_i e\,dt@f$，输出饱和时使用反算
 * 增益 Kb 修正积分状态。
 *
 * @param[in,out] pPID PID 控制器对象及其运行状态
 * @param[in] feedforward 前馈量
 * @param[in] error 误差值
 * @return 经过输出限幅后的控制量
 */
static inline float Parallel_PI_Update(PID_t *pPID, float feedforward, float error)
{
    pPID->status.err_k0        = error;
    pPID->status.proportional  = pPID->config.kp * pPID->status.err_k0;
    pPID->status.integral     += pPID->config.ki * pPID->status.err_k0 * pPID->dt;
    pPID->status.out_temp      = feedforward + pPID->status.proportional + pPID->status.integral;
    pPID->status.out           = CONSTRAIN(pPID->status.out_temp, pPID->limit.limit_min, pPID->limit.limit_max);
    pPID->status.saturation    = (pPID->status.out != pPID->status.out_temp);
    pPID->status.integral     += (pPID->status.out - pPID->status.out_temp) * pPID->config.kb * pPID->dt;

    return pPID->status.out;
}

/**
 * @brief 执行一次串联式 PI 控制计算
 *
 * 控制律为 @f$u=u_{ff}+K_p e+\int K_pK_i e\,dt@f$，其中 Ki 的量纲为 1/s。
 * 输出饱和时使用反算增益 Kb 修正积分状态。
 *
 * @param[in,out] pPID PID 控制器对象及其运行状态
 * @param[in] feedforward 前馈量
 * @param[in] error 误差值
 * @return 经过输出限幅后的控制量
 */
static inline float Serial_PI_Update(PID_t *pPID, float feedforward, float error)
{
    pPID->status.err_k0        = error;
    pPID->status.proportional  = pPID->config.kp * pPID->status.err_k0;
    pPID->status.integral     += pPID->config.ki * pPID->status.proportional * pPID->dt;
    pPID->status.out_temp      = feedforward + pPID->status.proportional + pPID->status.integral;
    pPID->status.out           = CONSTRAIN(pPID->status.out_temp, pPID->limit.limit_min, pPID->limit.limit_max);
    pPID->status.saturation    = (pPID->status.out != pPID->status.out_temp);
    pPID->status.integral     += (pPID->status.out - pPID->status.out_temp) * pPID->config.kb * pPID->dt;

    return pPID->status.out;
}

/**
 * @brief 整定电流环并联式 PI 参数
 * @param[in,out] pPID PID 控制器对象
 * @param[in] Hz 目标带宽，单位 Hz
 * @param[in] L 电机相电感，单位 H
 * @param[in] R 电机相电阻，单位 Ohm
 * @param[in] delay 从电流采样到控制电压生效的等效延时，单位 s
 */
void Current_Loop_Parallel_PID_Tune(PID_t *pPID, float Hz, float L, float R, float delay);

/**
 * @brief 整定电流环串联式 PI 参数
 * @param[in,out] pPID PID 控制器对象
 * @param[in] Hz 目标带宽，单位 Hz
 * @param[in] L 电机相电感，单位 H
 * @param[in] R 电机相电阻，单位 Ohm
 * @param[in] delay 从电流采样到控制电压生效的等效延时，单位 s
 */
void Current_Loop_Serial_PID_Tune(PID_t *pPID, float Hz, float L, float R, float delay);

/**
 * @brief 整定弱磁环并联式 PI 参数
 * @param[in,out] pPID PID 控制器对象
 * @param[in] Hz 目标开环穿越频率，单位 Hz
 * @param[in] Ti 积分时间常数，单位 s
 * @param[in] plant_gain 弱磁点电压对 d 轴电流的小信号增益，单位 V/A
 */
void Flux_Weakening_Loop_Parallel_PID_Tune(PID_t *pPID, float Hz, float Ti, float plant_gain);

/**
 * @brief 整定弱磁环串联式 PI 参数
 * @param[in,out] pPID PID 控制器对象
 * @param[in] Hz 目标开环穿越频率，单位 Hz
 * @param[in] Ti 积分时间常数，单位 s
 * @param[in] plant_gain 弱磁点电压对 d 轴电流的小信号增益，单位 V/A
 */
void Flux_Weakening_Loop_Serial_PID_Tune(PID_t *pPID, float Hz, float Ti, float plant_gain);

/**
 * @brief 整定速度环并联式 PI 参数
 * @param[in,out] pPID PID 控制器对象
 * @param[in] Hz 目标自然频率，单位 Hz
 * @param[in] Ti 积分时间常数，单位 s
 * @param[in] j 电机与负载的总转动惯量，单位 kg*m^2
 */
void Speed_Loop_Parallel_PID_Tune(PID_t *pPID, float Hz, float Ti, float j);

/**
 * @brief 整定速度环串联式 PI 参数
 * @param[in,out] pPID PID 控制器对象
 * @param[in] Hz 目标自然频率，单位 Hz
 * @param[in] Ti 积分时间常数，单位 s
 * @param[in] j 电机与负载的总转动惯量，单位 kg*m^2
 */
void Speed_Loop_Serial_PID_Tune(PID_t *pPID, float Hz, float Ti, float j);

/**
 * @brief 整定位置环比例增益
 * @param[in,out] pPID PID 控制器对象
 * @param[in] Hz 目标位置环带宽，单位 Hz
 */
void Pos_Loop_PID_Tune(PID_t *pPID, float Hz);

/**
 * @brief 整定力控环并联式 PI 参数
 * @param[in,out] pPID PID 控制器对象
 * @param[in] Hz 目标自然频率，单位 Hz
 * @param[in] Ti 积分时间常数，单位 s
 * @param[in] kc 等效接触刚度，单位 N/m
 */
void Force_Loop_Parallel_PID_Tune(PID_t *pPID, float Hz, float Ti, float kc);

/**
 * @brief 整定力控环串联式 PI 参数
 * @param[in,out] pPID PID 控制器对象
 * @param[in] Hz 目标自然频率，单位 Hz
 * @param[in] Ti 积分时间常数，单位 s
 * @param[in] kc 等效接触刚度，单位 N/m
 */
void Force_Loop_Serial_PID_Tune(PID_t *pPID, float Hz, float Ti, float kc);

/**
 * @brief 初始化 PID 控制器
 * @param[out] pPID 待初始化的 PID 控制器对象
 * @param[in] dt 控制器采样周期，单位 s
 * @param[in] pCfg PID 参数配置
 * @param[in] pLimit PID 输出限幅配置
 */
void PID_Init(PID_t *pPID, float dt, PID_Config_t *pCfg, PID_Limit_t *pLimit);

/**
 * @brief 清零 PID 控制器的运行状态
 * @param[in,out] pPID PID 控制器对象，控制参数和限幅保持不变
 */
void PID_Reset(PID_t *pPID);

/**
 * @brief 执行一次并联式位置型 PID 控制计算
 * @param[in,out] pPID PID 控制器对象及其运行状态
 * @param[in] feedforward 前馈量
 * @param[in] target 目标值
 * @param[in] feedback 反馈值
 * @return 经过输出限幅后的控制量
 */
float Parallel_PID_Position_Update(PID_t *pPID, float feedforward, float target, float feedback);

/**
 * @brief 执行一次串联式位置型 PID 控制计算
 * @param[in,out] pPID PID 控制器对象及其运行状态
 * @param[in] feedforward 前馈量
 * @param[in] target 目标值
 * @param[in] feedback 反馈值
 * @return 经过输出限幅后的控制量
 */
float Serial_PID_Position_Update(PID_t *pPID, float feedforward, float target, float feedback);

/**
 * @brief 执行一次增量式 PID 控制计算
 * @param[in,out] pPID PID 控制器对象及其运行状态
 * @param[in] feedforward 前馈量
 * @param[in] target 目标值
 * @param[in] feedback 反馈值
 * @return 经过输出限幅后的控制量
 */
float PID_Delta_Update(PID_t *pPID, float feedforward, float target, float feedback);

/**
 * @brief 执行一次积分-比例（IP）控制计算
 * @param[in,out] pPID PID 控制器对象及其运行状态
 * @param[in] target 目标值
 * @param[in] feedback 反馈值
 * @param[in] feedforward 前馈量
 * @return IP 控制器输出
 */
float IP_Update(PID_t *pPID, float target, float feedback, float feedforward);
