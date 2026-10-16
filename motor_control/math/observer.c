/**
 * @file observer.c
 * @author jinchl (jcl709905683@gmail.com)
 * @brief 
 * @version 0.1
 * @date 2026-03-19
 * 
 * @copyright Copyright (c) 2026
 * 
 */


#include "observer.h"
#include "global.h"
#include "motor.h"
#include "motor_api.h"

static void NESO_auto_tune(NESO_Config_t *p_config, float observer_band_width)
{
    float omega = fmaxf(observer_band_width, 0.0f) * _2PI;

    // codex: 本质上是把“每秒多少个周期”的带宽，转换成控制系统里常用的“每秒多少弧度”的带宽。
    p_config->neso_gain = omega;
}

static void NESO2_update(NESO_Config_t *p_config, NESO_Status_t *p_status, float vel_fb, float acc_ref)
{
    p_status->err     = vel_fb - p_status->z2;
    p_status->err_abs = fabsf(p_status->err);

    // codex: 这对应二阶观测器常见的极点配置：(s + ω)^2 = s² + 2ωs + ω²
    // 如果把观测器误差动态设计成类似二阶稳定系统，那么 2ω 和 ω² 就自然成为误差反馈增益。ω 越大，观测器误差收敛越快；但也更容易放大噪声
    p_status->l2 = p_config->neso_gain * 2.0f;
    p_status->l3 = SQ(p_config->neso_gain);

    // 动态增益调度：防止低速时观测器响应过度导致的抖动
    float abs_z2        = fabsf(p_status->z2);
    float gain_scale_z3 = (abs_z2 < p_config->neso_z3_speed_threshold) ? (abs_z2 / p_config->neso_z3_speed_threshold) : 1.0f;               // z3 增益恢复较慢
    float gain_scale_z2 = (abs_z2 < p_config->neso_z2_speed_threshold) ? (0.5f * abs_z2 / p_config->neso_z2_speed_threshold + 0.5f) : 1.0f; // z2 增益最低保持在 0.5
    float k_smooth      = fmaxf(p_config->neso_k_smooth, FLOAT_EPSILON);
    float soft_sign     = p_status->err / (p_status->err_abs + k_smooth);

    // 计算非线性观测器微分项
    p_status->dz3 = p_status->l3 * soft_sign * sqrtf(p_status->err_abs) * gain_scale_z3;
    p_status->dz2 = p_status->l2 * soft_sign * powf(p_status->err_abs, 0.75) * gain_scale_z2 + p_status->z3 + acc_ref;

    p_status->z3 += p_status->dz3 * BOARD_RT_DT;
    p_status->z2 += p_status->dz2 * BOARD_RT_DT;
}

/*
 * Full third-order nonlinear ESO.
 *
 * The plant presented to this observer is
 *
 *     pos_dot = vel
 *     vel_dot = acc_ref + disturbance
 *
 * z1, z2 and z3 estimate position, velocity and the lumped disturbance
 * acceleration respectively.  This is deliberately kept separate from the
 * existing velocity-only NESO above.
 */
static float NESO3_fal(float error, float alpha, float smooth)
{
    float abs_error = fabsf(error);
    float delta     = fmaxf(smooth, FLOAT_EPSILON);

    /* The linear inner segment prevents an infinite local gain. */
    if (abs_error <= delta)
    {
        return error / powf(delta, 1.0f - alpha);
    }

    return copysignf(powf(abs_error, alpha), error);
}

static void NESO3_update(NESO_Config_t *p_config, NESO_Status_t *p_status, float pos_fb, float acc_ref)
{
    float omega = fmaxf(p_config->neso_gain, FLOAT_EPSILON);
    float e     = pos_fb - p_status->z1;
    float k     = fmaxf(p_config->neso_k_smooth, FLOAT_EPSILON);

    p_status->err     = e;
    p_status->err_abs = fabsf(e);

    /* (s + omega)^3 = s^3 + 3*omega*s^2 + 3*omega^2*s + omega^3. */
    p_status->l1 = 3.0f * omega;
    p_status->l2 = 3.0f * SQ(omega);
    p_status->l3 = SQ(omega) * omega;

    /* alpha = 1, 0.75, 0.5 gives a nonlinear position/velocity/disturbance injection. */
    float inj1 = NESO3_fal(e, 1.0f, k);
    float inj2 = NESO3_fal(e, 0.75f, k);
    float inj3 = NESO3_fal(e, 0.50f, k);

    p_status->dz1 = p_status->z2 + p_status->l1 * inj1;
    p_status->dz2 = p_status->z3 + acc_ref + p_status->l2 * inj2;
    p_status->dz3 = p_status->l3 * inj3;

    p_status->z1 += p_status->dz1 * BOARD_RT_DT;
    p_status->z2 += p_status->dz2 * BOARD_RT_DT;
    p_status->z3 += p_status->dz3 * BOARD_RT_DT;
}

/**
 * @brief 观测器初始化
 * 
 * @param index 电机编号
 */
void observer_init(int index)
{
    Motor_Observer_Config_t *p_motor_observer_config = &global_config_in_core1.motor_observer_config[index];
    Motor_Observer_t *p_motor_observer               = &global_status.motor_observer[index];
    Motor_Status_t *p_motor_status                   = &global_status.motor_status[index];

    switch (p_motor_observer_config->observser_config.observer_type)
    {
    case Observer_Type_None:
        break;

    case Observer_Type_NESO2:
    case Observer_Type_NESO3:
        if (p_motor_observer_config->observser_config.auto_tune)
        {
            NESO_auto_tune(&p_motor_observer->neso_config, p_motor_observer_config->observser_config.observer_band_width);
        }
        memset(&p_motor_observer->neso_status, 0, sizeof(NESO_Status_t));
        p_motor_observer->neso_status.z1 = p_motor_status->pos_fb;
        p_motor_observer->neso_status.z2 = p_motor_status->vel_fb;
        break;

    default:
        motor_set_ext_err(index, EXT_FAULT_CODE_CFG_CONTROL_OBSERVER_TYPE);
        break;
    }
}

/**
 * @brief 观测器复位
 * 
 * @param index 电机编号
 */
void observer_reset(int index)
{
    Motor_Observer_Config_t *p_motor_observer_config = &global_config_in_core1.motor_observer_config[index];
    Motor_Observer_t *p_motor_observer               = &global_status.motor_observer[index];
    Motor_Status_t *p_motor_status                   = &global_status.motor_status[index];

    switch (p_motor_observer_config->observser_config.observer_type)
    {
    case Observer_Type_None:
        break;

    case Observer_Type_NESO2:
    case Observer_Type_NESO3:
        if (p_motor_observer_config->observser_config.auto_tune)
        {
            NESO_auto_tune(&p_motor_observer->neso_config, p_motor_observer_config->observser_config.observer_band_width);
        }
        memset(&p_motor_observer->neso_status, 0, sizeof(NESO_Status_t));
        p_motor_observer->neso_status.z1 = p_motor_status->pos_fb;
        p_motor_observer->neso_status.z2 = p_motor_status->vel_fb;
        break;

    default:
        motor_set_ext_err(index, EXT_FAULT_CODE_CFG_CONTROL_OBSERVER_TYPE);
        break;
    }

    memset(&p_motor_observer->observer_outputs, 0, sizeof(Observer_Output_t));
}

/**
 * @brief 观测器实时更新
 * 
 * @param index 电机编号
 */
void observer_update_rt(int index)
{
    Motor_Config_t *p_motor_config                   = &global_config_in_core1.motor_config[index];
    Motor_Observer_Config_t *p_motor_observer_config = &global_config_in_core1.motor_observer_config[index];
    Motor_Status_t *p_motor_status                   = &global_status.motor_status[index];
    Motor_Observer_t *p_motor_observer               = &global_status.motor_observer[index];

    switch (p_motor_observer_config->observser_config.observer_type)
    {
    case Observer_Type_NESO2: {
        float vel_fb        = p_motor_status->vel_fb;
        float total_inertia = p_motor_config->inertia_rotor + p_motor_config->inertia_load;

        if (fabsf(total_inertia) < FLOAT_MIN)
        {
            motor_set_ext_err(index, EXT_FAULT_CODE_CFG_MOTOR_INERTIA);
            break;
        }

        // 已知弹簧保持力从模型输入中扣除，ESO 只估计剩余扰动，避免重复补偿。
        float acc_ref = (p_motor_status->torque_ref - motor_get_spring_compensation(index)) / total_inertia;

        NESO2_update(&p_motor_observer->neso_config, &p_motor_observer->neso_status, vel_fb, acc_ref);

        p_motor_observer->observer_outputs.pos_estimate         = p_motor_observer->neso_status.z1;
        p_motor_observer->observer_outputs.vel_estimate         = p_motor_observer->neso_status.z2;
        p_motor_observer->observer_outputs.acc_estimate         = acc_ref + p_motor_observer->neso_status.z3;
        p_motor_observer->observer_outputs.tor_out_estimate     = p_motor_observer->observer_outputs.acc_estimate * total_inertia;
        p_motor_observer->observer_outputs.tor_disturb_estimate = p_motor_observer->neso_status.z3 * total_inertia;
    }
    break;

    case Observer_Type_NESO3: {
        float pos_fb        = p_motor_status->pos_fb;
        float total_inertia = p_motor_config->inertia_rotor + p_motor_config->inertia_load;

        if (fabsf(total_inertia) < FLOAT_MIN)
        {
            motor_set_ext_err(index, EXT_FAULT_CODE_CFG_MOTOR_INERTIA);
            break;
        }

        /* Keep the same torque-reference input convention as the existing ESO. */
        float acc_ref = (p_motor_status->torque_ref - motor_get_spring_compensation(index)) / total_inertia;

        NESO3_update(&p_motor_observer->neso_config, &p_motor_observer->neso_status, pos_fb, acc_ref);

        p_motor_observer->observer_outputs.pos_estimate         = p_motor_observer->neso_status.z1;
        p_motor_observer->observer_outputs.vel_estimate         = p_motor_observer->neso_status.z2;
        p_motor_observer->observer_outputs.acc_estimate         = acc_ref + p_motor_observer->neso_status.z3;
        p_motor_observer->observer_outputs.tor_out_estimate     = p_motor_observer->observer_outputs.acc_estimate * total_inertia;
        p_motor_observer->observer_outputs.tor_disturb_estimate = p_motor_observer->neso_status.z3 * total_inertia;
    }
    break;

    default:
        break;
    }
}
