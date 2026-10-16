/**
 * @file PID.c
 * @author jinchl (jcl709905683@gmail.com)
 * @brief PID 控制器整定与在线计算实现
 * @version 0.1
 * @date 2026-03-11
 * 
 * @copyright Copyright (c) 2026
 * 
 */


#include "pid.h"
#include "utils.h"

// #define Kd_Enable
// #define Kb_Enable

// #define ZETA (1.0f)
#define ZETA (0.707f)

/**
 * @brief 计算考虑控制延时后的电流环等效角带宽
 *
 * delay 为 0 时补偿系数为 1。延时补偿只缩放环路增益，不改变用于
 * 抵消 RL 对象极点的 PI 零点位置。
 */
static inline float Current_Loop_Compensated_Wc(float Hz, float delay)
{
    float Wc           = Hz * _2PI;
    float delay_sin    = sinf(Wc * delay);
    float compensation = sqrtf(1.0f + SQ(delay_sin)) - delay_sin;

    return Wc * compensation;
}

/**
 * @brief 根据 RL 电气模型整定电流环并联式 PI 参数
 *
 * 使用目标带宽计算比例、积分及反算抗饱和增益，并根据实际控制延迟
 * 修正环路增益。delay 为 0 时退化为连续域整定结果。
 *
 * @param[in,out] pPID PID 控制器对象
 * @param[in] Hz 目标带宽，单位 Hz
 * @param[in] L 电机相电感，单位 H
 * @param[in] R 电机相电阻，单位 Ohm
 * @param[in] delay 从电流采样到控制电压生效的等效延时，单位 s
 *
 * @see https://zhuanlan.zhihu.com/p/146373628
 * @see https://blog.csdn.net/wanrenqi/article/details/131523107
 */
void Current_Loop_Parallel_PID_Tune(PID_t *pPID, float Hz, float L, float R, float delay)
{
    if (pPID == NULL)
    {
        return;
    }

    if (Hz <= 0.0f || L <= FLOAT_MIN || R < 0.0f || delay < 0.0f)
    {
        pPID->config.kp = 0.0f;
        pPID->config.ki = 0.0f;
        pPID->config.kb = 0.0f;
        return;
    }

    float compensated_wc = Current_Loop_Compensated_Wc(Hz, delay);

    pPID->config.kp = L * compensated_wc;
    pPID->config.ki = R * compensated_wc;
    pPID->config.kb = 3.0f * R / L;
}

/**
 * @brief 根据 RL 电气模型整定电流环串联式 PI 参数
 *
 * @param[in,out] pPID PID 控制器对象
 * @param[in] Hz 目标带宽，单位 Hz
 * @param[in] L 电机相电感，单位 H
 * @param[in] R 电机相电阻，单位 Ohm
 * @param[in] delay 从电流采样到控制电压生效的等效延时，单位 s
 *
 * @see https://zhuanlan.zhihu.com/p/454914546
 */
void Current_Loop_Serial_PID_Tune(PID_t *pPID, float Hz, float L, float R, float delay)
{
    if (pPID == NULL)
    {
        return;
    }

    if (Hz <= 0.0f || L <= FLOAT_MIN || R < 0.0f || delay < 0.0f)
    {
        pPID->config.kp = 0.0f;
        pPID->config.ki = 0.0f;
        pPID->config.kb = 0.0f;
        return;
    }

    float compensated_wc = Current_Loop_Compensated_Wc(Hz, delay);

    pPID->config.kp = L * compensated_wc;
    pPID->config.ki = R / L;
    pPID->config.kb = 3.0f * pPID->config.ki;
}

/**
 * @brief 根据弱磁对象的小信号增益整定并联式 PI 参数
 *
 * 在弱磁起始点附近，将内电流环近似为理想环节，弱磁对象近似为静态增益
 * @f$K_{fw}=\partial |U_s|/\partial I_d@f$。根据指定穿越频率和 PI 零点
 * 约束 @f$|C(j\omega_c)K_{fw}|=1@f$，计算并联式 PI 参数。
 *
 * @param[in,out] pPID PID 控制器对象
 * @param[in] Hz 目标开环穿越频率，单位 Hz
 * @param[in] Ti 积分时间常数，单位 s
 * @param[in] plant_gain 弱磁点电压对 d 轴电流的小信号增益，单位 V/A
 */
void Flux_Weakening_Loop_Parallel_PID_Tune(PID_t *pPID, float Hz, float Ti, float plant_gain)
{
    float Wc        = Hz * _2PI;
    float zero_rate = 1.0f / (Wc * Ti);

    pPID->config.kp = 1.0f / (plant_gain * sqrtf(1.0f + SQ(zero_rate)));
    pPID->config.ki = pPID->config.kp / Ti;
}

/**
 * @brief 根据弱磁对象的小信号增益整定串联式 PI 参数
 *
 * 串联式 PI 的控制律为 @f$K_p(1+K_i/s)@f$。该函数与并联式整定
 * 具有相同的比例增益和等效积分增益。
 *
 * @param[in,out] pPID PID 控制器对象
 * @param[in] Hz 目标开环穿越频率，单位 Hz
 * @param[in] Ti 积分时间常数，单位 s
 * @param[in] plant_gain 弱磁点电压对 d 轴电流的小信号增益，单位 V/A
 */
void Flux_Weakening_Loop_Serial_PID_Tune(PID_t *pPID, float Hz, float Ti, float plant_gain)
{
    float Wc        = Hz * _2PI;
    float zero_rate = 1.0f / (Wc * Ti);

    pPID->config.kp = 1.0f / (plant_gain * sqrtf(1.0f + SQ(zero_rate)));
    pPID->config.ki = 1.0f / Ti;
}

/**
 * @brief 根据二阶系统模型整定速度环并联式 PI 参数
 *
 * @param[in,out] pPID PID 控制器对象
 * @param[in] Hz 目标自然频率，单位 Hz
 * @param[in] Ti 积分时间常数，单位 s
 * @param[in] j 电机与负载的总转动惯量，单位 kg*m^2
 */
void Speed_Loop_Parallel_PID_Tune(PID_t *pPID, float Hz, float Ti, float j)
{
    float Wn   = Hz * _2PI;
    float zete = ZETA;

    pPID->config.kp = 2 * zete * Wn * j;
    // pPID->config.ki = SQ(Wn) * j; // 理论值,使用此方法得到的ki过大会导致震动
    pPID->config.ki = pPID->config.kp / Ti; // 参考固高驱动器
}

/**
 * @brief 根据二阶系统模型整定速度环串联式 PI 参数
 *
 * @param[in,out] pPID PID 控制器对象
 * @param[in] Hz 目标自然频率，单位 Hz
 * @param[in] Ti 积分时间常数，单位 s
 * @param[in] j 电机与负载的总转动惯量，单位 kg*m^2
 */
void Speed_Loop_Serial_PID_Tune(PID_t *pPID, float Hz, float Ti, float j)
{
    float Wn   = Hz * _2PI;
    float zete = ZETA;

    pPID->config.kp = 2 * zete * Wn * j;
    // pPID->config.ki = Wn / (2 * zete); // 理论值,使用此方法得到的ki过大会导致震动
    pPID->config.ki = 1.0f / Ti; // 参考固高驱动器
}

/**
 * @brief 根据目标带宽整定位置环比例增益
 *
 * @param[in,out] pPID PID 控制器对象
 * @param[in] Hz 目标位置环带宽，单位 Hz
 */
void Pos_Loop_PID_Tune(PID_t *pPID, float Hz)
{
    float Wn = Hz * _2PI;

    pPID->config.kp = Wn;
}

/**
 * @brief 根据接触刚度模型整定力控环并联式 PI 参数
 *
 * @param[in,out] pPID PID 控制器对象
 * @param[in] Hz 目标自然频率，单位 Hz
 * @param[in] Ti 积分时间常数，单位 s
 * @param[in] kc 等效接触刚度，单位 N/m
 */
void Force_Loop_Parallel_PID_Tune(PID_t *pPID, float Hz, float Ti, float kc)
{
    float Wn   = Hz * _2PI;
    float zete = ZETA;

    pPID->config.kp = 2 * zete * Wn / kc;
    // pPID->config.ki = SQ(Wn) / kc;
    if (Ti > FLOAT_EPSILON)
    {
        pPID->config.ki = pPID->config.kp / Ti;
    }
    else
    {
        pPID->config.ki = 0;
    }
}

/**
 * @brief 根据接触刚度模型整定力控环串联式 PI 参数
 *
 * @param[in,out] pPID PID 控制器对象
 * @param[in] Hz 目标自然频率，单位 Hz
 * @param[in] Ti 积分时间常数，单位 s
 * @param[in] kc 等效接触刚度，单位 N/m
 */
void Force_Loop_Serial_PID_Tune(PID_t *pPID, float Hz, float Ti, float kc)
{
    float Wn   = Hz * _2PI;
    float zete = ZETA;

    pPID->config.kp = 2 * zete * Wn / kc;
    // pPID->config.ki = Wn / (2 * zete);
    if (Ti > FLOAT_EPSILON)
    {
        pPID->config.ki = 1.0f / Ti;
    }
    else
    {
        pPID->config.ki = 0;
    }
}

/**
 * @brief 初始化 PID 控制器的参数、限幅及运行状态
 *
 * @param[out] pPID 待初始化的 PID 控制器对象
 * @param[in] dt 控制器采样周期，单位 s
 * @param[in] pCfg PID 参数配置
 * @param[in] pLimit PID 输出限幅配置
 */
void PID_Init(PID_t *pPID, float dt, PID_Config_t *pCfg, PID_Limit_t *pLimit)
{
    if (pPID == NULL || pCfg == NULL || pLimit == NULL)
    {
        return;
    }

    memset(pPID, 0, sizeof(PID_t));
    pPID->dt = dt;
    memcpy(&pPID->config, pCfg, sizeof(PID_Config_t));
    memcpy(&pPID->limit, pLimit, sizeof(PID_Limit_t));
}

/**
 * @brief 清零 PID 控制器的动态运行状态
 *
 * @param[in,out] pPID PID 控制器对象，控制参数和限幅保持不变
 */
void PID_Reset(PID_t *pPID)
{
    if (pPID == NULL)
    {
        return;
    }

    memset(&pPID->status, 0, sizeof(PID_Status_t));
}

/**
 * @brief 执行一次并联式位置型 PID 控制计算
 *
 * @param[in,out] pPID PID 控制器对象及其运行状态
 * @param[in] feedforward 前馈量
 * @param[in] target 目标值
 * @param[in] feedback 反馈值
 * @return 经过输出限幅后的控制量
 */
float Parallel_PID_Position_Update(PID_t *pPID, float feedforward, float target, float feedback)
{
#ifdef Kd_Enable
    pPID->status.err_k1 = pPID->status.err_k0;
#endif
    pPID->status.err_k0        = target - feedback;
    pPID->status.proportional  = pPID->config.kp * pPID->status.err_k0;
    pPID->status.integral     += pPID->config.ki * pPID->status.err_k0 * pPID->dt;
#ifndef Kb_Enable
    pPID->status.integral = CONSTRAIN(pPID->status.integral, pPID->limit.limit_min, pPID->limit.limit_max);
#endif
#ifdef Kd_Enable
    pPID->status.derivative = pPID->config.kd * (pPID->status.err_k0 - pPID->status.err_k1) / pPID->dt;
    FirstOrder_LPF_Cal(pPID->status.derivative, pPID->status.derivative_f, pPID->config.alpha);
    pPID->status.out_temp = feedforward + pPID->status.proportional + pPID->status.integral + pPID->status.derivative_f;
#else
    pPID->status.out_temp = feedforward + pPID->status.proportional + pPID->status.integral;
#endif
    pPID->status.out        = CONSTRAIN(pPID->status.out_temp, pPID->limit.limit_min, pPID->limit.limit_max);
    pPID->status.saturation = (pPID->status.out != pPID->status.out_temp);
#ifdef Kb_Enable
    if (pPID->status.saturation)
    {
        pPID->status.integral += (pPID->status.out - pPID->status.out_temp) * pPID->config.kb * pPID->dt;
    }
#endif

    return pPID->status.out;
}

/**
 * @brief 执行一次串联式位置型 PID 控制计算
 *
 * @param[in,out] pPID PID 控制器对象及其运行状态
 * @param[in] feedforward 前馈量
 * @param[in] target 目标值
 * @param[in] feedback 反馈值
 * @return 经过输出限幅后的控制量
 */
float Serial_PID_Position_Update(PID_t *pPID, float feedforward, float target, float feedback)
{
#ifdef Kd_Enable
    pPID->status.err_k1 = pPID->status.err_k0;
#endif
    pPID->status.err_k0        = target - feedback;
    pPID->status.proportional  = pPID->config.kp * pPID->status.err_k0;
    pPID->status.integral     += pPID->config.ki * pPID->status.proportional * pPID->dt;
#ifdef Kd_Enable
    pPID->status.derivative = pPID->config.kd * (pPID->status.err_k0 - pPID->status.err_k1) / pPID->dt;
    FirstOrder_LPF_Cal(pPID->status.derivative, pPID->status.derivative_f, pPID->config.alpha);
    pPID->status.out_temp = feedforward + pPID->status.proportional + pPID->status.integral + pPID->status.derivative_f;
#else
    pPID->status.out_temp = feedforward + pPID->status.proportional + pPID->status.integral;
#endif
    pPID->status.out        = CONSTRAIN(pPID->status.out_temp, pPID->limit.limit_min, pPID->limit.limit_max);
    pPID->status.saturation = (pPID->status.out != pPID->status.out_temp);

    return pPID->status.out;
}

/**
 * @brief 执行一次增量式 PID 控制计算
 *
 * @param[in,out] pPID PID 控制器对象及其运行状态
 * @param[in] feedforward 前馈量
 * @param[in] target 目标值
 * @param[in] feedback 反馈值
 * @return 经过输出限幅后的控制量
 */
float PID_Delta_Update(PID_t *pPID, float feedforward, float target, float feedback)
{
#ifdef Kd_Enable
    pPID->status.err_k2 = pPID->status.err_k1;
#endif
    pPID->status.err_k1       = pPID->status.err_k0;
    pPID->status.err_k0       = target - feedback;
    pPID->status.proportional = pPID->config.kp * (pPID->status.err_k0 - pPID->status.err_k1);
    pPID->status.integral     = pPID->config.ki * pPID->status.err_k0 * pPID->dt;
    pPID->status.integral     = CONSTRAIN(pPID->status.integral, pPID->limit.limit_min, pPID->limit.limit_max);
#ifdef Kd_Enable
    pPID->status.derivative = pPID->config.kd * (pPID->status.err_k0 - 2 * pPID->status.err_k1 + pPID->status.err_k2) / pPID->dt;
    FirstOrder_LPF_Cal(pPID->status.derivative, pPID->status.derivative_f, pPID->config.alpha);
    pPID->status.out_temp = feedforward + pPID->status.proportional + pPID->status.integral + pPID->status.derivative_f;
#else
    pPID->status.out_temp = feedforward + pPID->status.proportional + pPID->status.integral;
#endif
    pPID->status.out        = CONSTRAIN(pPID->status.out_temp, pPID->limit.limit_min, pPID->limit.limit_max);
    pPID->status.saturation = (pPID->status.out != pPID->status.out_temp);

    return pPID->status.out;
}

/**
 * @brief 执行一次积分-比例（IP）控制计算
 *
 * 积分环节作用于跟踪误差，比例环节作用于反馈量，可减小目标阶跃引起的超调。
 *
 * @param[in,out] pPID PID 控制器对象及其运行状态
 * @param[in] target 目标值
 * @param[in] feedback 反馈值
 * @param[in] feedforward 前馈量
 * @return IP 控制器输出
 */
float IP_Update(PID_t *pPID, float target, float feedback, float feedforward)
{
    pPID->status.err_k0        = target - feedback;
    pPID->status.proportional  = pPID->config.kp * feedback;
    pPID->status.integral     += pPID->config.ki * pPID->status.err_k0 * pPID->dt;
    pPID->status.integral      = CONSTRAIN(pPID->status.integral, pPID->status.proportional + pPID->limit.limit_min - feedforward, pPID->status.proportional + pPID->limit.limit_max - feedforward);
    // pPID->status.out_temp += _pPID->status.integral;
    // pPID->status.out = CONSTRAIN(pPID->status.out_temp,pPID->limit.limit_min,pPID->limit.limit_max);
    // pPID->status.saturation = (pPID->status.out != pPID->status.out_temp);
    pPID->status.out = pPID->status.integral - pPID->status.proportional + feedforward;

    return pPID->status.out;
}

// float PID_2DOF_Update(PID_t *pPID, float feedforward, float target, float feedback)
// {
//     pPID->status.err_k1        = pPID->status.err_k0;
//     pPID->status.err_k0        = target - feedback;
//     pPID->status.proportional  = pPID->config.kp * (pPID->config.beta * target - feedback);
//     pPID->status.integral     += pPID->config.ki * pPID->status.err_k0 * pPID->dt;
//     pPID->status.integral      = CONSTRAIN(pPID->status.integral, pPID->limit.limit_min, pPID->limit.limit_max);

//     // 微分项（对反馈和设定值分别加权）
//     float d_target   = (target - pPID->status.err_k1) / pPID->dt;   // r[k]-r[k-1]
//     float d_feedback = (feedback - pPID->status.err_k1) / pPID->dt; // y[k]-y[k-1]

//     pPID->status.derivative = pPID->config.kd * (pPID->config.gamma * d_target - d_feedback);
//     pPID->status.out_temp   = feedforward + pPID->status.proportional + pPID->status.integral + pPID->status.derivative;
//     pPID->status.out        = CONSTRAIN(pPID->status.out_temp, pPID->limit.limit_min, pPID->limit.limit_max);
//     pPID->status.saturation = (pPID->status.out != pPID->status.out_temp);

//     return pPID->status.out;
// }

// /**
//  * @brief 二自由度 PID 计算函数
//  * @param pid 指向 PID 结构体的指针
//  * @param setpoint 设定目标值 (r)
//  * @param measure  当前测量值 (y)
//  */
// void PID_Compute(PID_t *pid, float setpoint, float measure) {
//     // 1. 更新误差序列 (k0:当前, k1:上一次, k2:上上次)
//     pid->status.err_k2 = pid->status.err_k1;
//     pid->status.err_k1 = pid->status.err_k0;
//     pid->status.err_k0 = setpoint - measure;

//     // 2. 比例环节 (proportional)
//     // 引入 beta 权重：P = kp * (beta * r - y)
//     pid->status.proportional = pid->config.kp * (pid->config.beta * setpoint - measure);

//     // 3. 积分环节 (integral)
//     // 考虑抗饱和：如果上一时刻发生饱和，可通过 kb 进行削弱（Back-calculation）
//     float error_i = pid->status.err_k0;
//     if (pid->status.saturation) {
//         // 抗饱和调整：此处简化处理，通常积分项累加需结合 dt
//         // integral = integral + ki * err * dt + kb * (out_limited - out_pre)
//     }
//     pid->status.integral += pid->config.ki * pid->status.err_k0 * pid->config.kd;

//     // 4. 微分环节 (derivative)
//     // 引入 gamma 权重：D = kd * (gamma * dr/dt - dy/dt)
//     // 此处使用一阶差分近似，并考虑对微分项进行低通滤波 (derivative_f)
//     float diff_val = (pid->config.gamma * (setpoint - pid->status.err_k1 + measure) - measure + pid->status.err_k1) / pid->config.kd;

//     // 简单的低通滤波处理（如果需要更复杂的滤波，可利用 derivative_f 存储状态）
//     pid->status.derivative = pid->config.kd * (pid->config.gamma * (setpoint - (setpoint - pid->status.err_k1)) - (measure - (measure - (pid->status.err_k0 - pid->status.err_k1))));
//     // 修正的标准二自由度微分：
//     float d_term = pid->config.kd * (pid->config.gamma * (setpoint - (setpoint - pid->status.err_k1)) - (measure - (measure - pid->status.err_k0)));
//     pid->status.derivative = d_term / pid->config.kd;

//     // 5. 计算原始输出
//     pid->status.out_temp = pid->status.proportional + pid->status.integral + pid->status.derivative;

//     // 6. 输出限幅与饱和检测
//     if (pid->status.out_temp > pid->config.limit_max) {
//         pid->status.out = pid->config.limit_max;
//         pid->status.saturation = true;
//     } else if (pid->status.out_temp < pid->config.limit_min) {
//         pid->status.out = pid->config.limit_min;
//         pid->status.saturation = true;
//     } else {
//         pid->status.out = pid->status.out_temp;
//         pid->status.saturation = false;
//     }
// }
