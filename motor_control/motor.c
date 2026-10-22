/**
 * @file Motor.c
 * @author jinchl (jcl709905683@gmail.com)
 * @brief 
 * @version 0.1
 * @date 2026-03-11
 * 
 * @copyright Copyright (c) 2026
 * 
 * 直线电机单位为m m/s m/s^2
 * 旋转电机单位为rad rad/s rad/s^2
 * 电流单位为A
 * 扭矩单位为N或N/M
 */


#include "motor.h"
#include "motor_api.h"
#include "global.h"
#include "inverter.h"
#include "board.h"
#include "foc.h"
#include "dc.h"
#include "motion_planner.h"
#include "observer.h"
#include <stddef.h>

/* 记录最近几次出现的警告,避免警告循环写入日志 */
#define MOTOR_WARNING_LIST_CAPACITY (5U)

typedef struct
{
    uint32_t codes[MOTOR_WARNING_LIST_CAPACITY];
    uint32_t index;
} Motor_Warning_List_t;


static void motor_filter_init(int index)
{
    const Loop_Filter_Config_t *p_loop_filter_config = &global_config.loop_filter_config[index];
    Loop_Filter_t *p_loop_filter_status              = &loop_filter_status[index];
    Motor_Status_t *p_motor_status                   = &global_status.motor_status[index];

    if (Init_Filter(&p_loop_filter_status->current_fb_filter, &p_loop_filter_config->current_fb_filter_config, (float)BOARD_RT_FREQ) != Filter_Status_OK)
        motor_set_ext_err(index, EXT_FAULT_CODE_CFG_CONTROL_CURRENT_FB_FILTER);
    if (Init_Filter(&p_loop_filter_status->current_ref_filter, &p_loop_filter_config->current_ref_filter_config, (float)BOARD_RT_FREQ) != Filter_Status_OK)
        motor_set_ext_err(index, EXT_FAULT_CODE_CFG_CONTROL_CURRENT_REF_FILTER);
    if (Init_Filter(&p_loop_filter_status->torque_fb_filter, &p_loop_filter_config->current_fb_filter_config, (float)BOARD_RT_FREQ) != Filter_Status_OK)
        motor_set_ext_err(index, EXT_FAULT_CODE_CFG_CONTROL_CURRENT_FB_FILTER);
    if (Init_Filter(&p_loop_filter_status->torque_ref_filter, &p_loop_filter_config->current_ref_filter_config, (float)BOARD_RT_FREQ) != Filter_Status_OK)
        motor_set_ext_err(index, EXT_FAULT_CODE_CFG_CONTROL_CURRENT_REF_FILTER);
    if (Init_Filter(&p_loop_filter_status->speed_out_filter, &p_loop_filter_config->speed_out_filter_config, (float)BOARD_RT_FREQ) != Filter_Status_OK)
        motor_set_ext_err(index, EXT_FAULT_CODE_CFG_CONTROL_SPEED_OUT_FILTER);
    if (Init_Filter(&p_loop_filter_status->speed_ref_filter, &p_loop_filter_config->speed_ref_filter_config, (float)BOARD_RT_FREQ) != Filter_Status_OK)
        motor_set_ext_err(index, EXT_FAULT_CODE_CFG_CONTROL_SPEED_REF_FILTER);
    if (Init_Filter(&p_loop_filter_status->pos_ref_filter, &p_loop_filter_config->pos_ref_filter_config, (float)BOARD_RT_FREQ) != Filter_Status_OK)
        motor_set_ext_err(index, EXT_FAULT_CODE_CFG_CONTROL_POS_REF_FILTER);
    if (Init_Filter(&p_loop_filter_status->force_fb_filter, &p_loop_filter_config->force_fb_filter_config, (float)BOARD_RT_FREQ) != Filter_Status_OK)
        motor_set_ext_err(index, EXT_FAULT_CODE_CFG_CONTROL_FORCE_FB_FILTER);

    Prime_Filter(&p_loop_filter_status->current_fb_filter, p_motor_status->current_fb);
    Prime_Filter(&p_loop_filter_status->current_ref_filter, p_motor_status->current_ref);
    Prime_Filter(&p_loop_filter_status->torque_fb_filter, p_motor_status->torque_fb);
    Prime_Filter(&p_loop_filter_status->torque_ref_filter, p_motor_status->torque_ref);
    Prime_Filter(&p_loop_filter_status->speed_out_filter, p_motor_status->vel_out);
    Prime_Filter(&p_loop_filter_status->speed_ref_filter, p_motor_status->vel_ref);
    Prime_Filter(&p_loop_filter_status->pos_ref_filter, p_motor_status->pos_ref);
    Prime_Filter(&p_loop_filter_status->force_fb_filter, p_motor_status->force_fb);
}

/**
 * @brief 电机获取力反馈
 * 
 * @param index 电机编号
 * @return float 力反馈
 */
static inline float motot_get_force_fb(int index)
{
    Motor_Observer_t *p_motor_observer = &global_status.motor_observer[index];

    float force_fb = -p_motor_observer->observer_outputs.tor_disturb_estimate;

    return force_fb;
}

/**
 * @brief 电机层编码器位置速度更新
 * 
 * @param index 电机编号
 */
static inline void motor_encoder_update_rt(int index)
{
    Motor_Config_t *p_motor_config     = &global_config.motor_config[index];
    Motor_Status_t *p_motor_status     = &global_status.motor_status[index];
    Encoder_Status_t *p_encoder_status = &global_status.encoder_status[index];

    switch (p_motor_config->motor_struct)
    {
    case Motor_Struct_Linear:
        switch (p_motor_config->motor_type)
        {
        case Motor_Type_DC:
            p_motor_status->pos_fb = p_encoder_status->encoder_pos / 1e3f;
            p_motor_status->vel_fb = p_encoder_status->encoder_speed / 1e3f;
            p_motor_status->acc_fb = p_encoder_status->encoder_acc / 1e3f;
            break;

        case Motor_Type_BLDC_3_Phase:
            p_motor_status->pos_fb = p_encoder_status->encoder_pos * p_motor_config->pp / 1e3f;
            p_motor_status->vel_fb = p_encoder_status->encoder_speed * p_motor_config->pp / 1e3f;
            p_motor_status->acc_fb = p_encoder_status->encoder_acc * p_motor_config->pp / 1e3f;
            break;

        default:
            break;
        }
        break;

    case Motor_Struct_Rotary:
        p_motor_status->pos_fb = p_encoder_status->encoder_pos * _2PI;
        p_motor_status->vel_fb = p_encoder_status->encoder_speed * _2PI;
        p_motor_status->acc_fb = p_encoder_status->encoder_acc * _2PI;
        break;

        break;

    default:
        break;
    }

    p_motor_status->acc_abs     = fabsf(p_motor_status->acc_fb);
    p_motor_status->acc_err     = p_motor_status->acc_ref - p_motor_status->acc_fb;
    p_motor_status->acc_err_abs = fabsf(p_motor_status->acc_err);
    p_motor_status->vel_abs     = fabsf(p_motor_status->vel_fb);
    p_motor_status->vel_err     = p_motor_status->vel_ref_f - p_motor_status->vel_fb;
    p_motor_status->vel_err_abs = fabsf(p_motor_status->vel_err);
    p_motor_status->pos_abs     = fabsf(p_motor_status->pos_fb);
    p_motor_status->pos_err     = p_motor_status->pos_ref_f - p_motor_status->pos_fb;
    p_motor_status->pos_err_abs = fabsf(p_motor_status->pos_err);
}

/**
 * @brief 电机层电流和扭矩反馈更新
 * 
 * @param index 电机编号
 */
static inline void motor_torque_update_rt(int index)
{
    Motor_Config_t *p_motor_config                       = &global_config.motor_config[index];
    Motor_Controller_Config_t *p_motor_controller_config = &global_config.motor_controller_config[index];
    Motor_Status_t *p_motor_status                       = &global_status.motor_status[index];
    Motor_Basic_Status_t *p_motor_basic_status           = &global_status.motor_basic_status[index];
    Loop_Filter_t *p_loop_filter_status                  = &loop_filter_status[index];

    switch (p_motor_config->motor_type)
    {
    case Motor_Type_DC:
        p_motor_status->current_fb = dc_get_current_fb(index);
        p_motor_status->torque_fb  = dc_get_torque_fb(index);
        break;

    case Motor_Type_BLDC_3_Phase:
        switch (p_motor_controller_config->motor_controller_type)
        {
        case Motor_Controller_Type_FOC:
            p_motor_status->current_fb = foc_get_current_fb(index);
            p_motor_status->torque_fb  = foc_get_torque_fb(index);
            break;

        case Motor_Controller_Type_DTC:
            break;

        default:
            break;
        }
        break;

    default:
        break;
    }

    p_motor_status->current_fb_f = Process_Filter(&p_loop_filter_status->current_fb_filter, p_motor_status->current_fb);
    p_motor_status->torque_fb_f  = Process_Filter(&p_loop_filter_status->torque_fb_filter, p_motor_status->torque_fb);

    p_motor_status->current_abs     = fabsf(p_motor_status->current_fb);
    p_motor_status->current_err     = p_motor_status->current_ref - p_motor_status->current_fb;
    p_motor_status->current_err_abs = fabsf(p_motor_status->current_err);
    p_motor_status->torque_abs      = fabsf(p_motor_status->torque_fb);
    p_motor_status->torque_err      = p_motor_status->torque_ref - p_motor_status->torque_fb;
    p_motor_status->torque_err_abs  = fabsf(p_motor_status->torque_err);

    p_motor_basic_status->m_power = p_motor_status->torque_fb * p_motor_status->vel_fb;

    const float min_power = 0.1f;
    float e_power         = p_motor_basic_status->e_power;
    float m_power         = p_motor_basic_status->m_power;
    if (e_power > min_power && m_power > min_power)
    {
        p_motor_basic_status->efficiency = m_power / e_power;
    }
    else if (e_power < -min_power && m_power < -min_power)
    {
        p_motor_basic_status->efficiency = e_power / m_power;
    }
    else
    {
        p_motor_basic_status->efficiency = 0.0f;
    }
}

/**
 * @brief 电机位置跟随误差状态更新
 * 
 * @param index 电机编号
 */
static inline void pos_follow_err_update_rt(int index)
{
    Motor_Status_t *p_motor_status = &global_status.motor_status[index];

    if (p_motor_status->follow_err_window && p_motor_status->follow_err_time)
    {
        if (p_motor_status->pos_err_abs > p_motor_status->follow_err_window)
        {
            if (++p_motor_status->follow_err_tick > MS_TO_RT_TICKS(p_motor_status->follow_err_time))
            {
                p_motor_status->is_follow_err = 1;
                motor_set_err(index, FAULT_CODE_POSITION_FOLLOW_ERR);
            }
        }
        else
        {
            p_motor_status->follow_err_tick = 0;
        }
    }
    else
    {
        p_motor_status->is_follow_err   = 0;
        p_motor_status->follow_err_tick = 0;
    }
}

/**
 * @brief 电机速度跟随误差状态更新
 * 
 * @param index 电机编号
 */
static inline void vel_follow_err_update_rt(int index)
{
    Motor_Status_t *p_motor_status = &global_status.motor_status[index];

    float ref_abs     = fabsf(p_motor_status->vel_ref_f);
    float vel_max_abs = cfg_peak_speed_to_motor_speed(index);
    float err_limit   = fmaxf(vel_max_abs * PEAK_VELOCITY_ERR_RATE, ref_abs * REF_VELOCITY_ERR_RATE);

    bool is_vel_err = false;

    switch (p_motor_status->vel_monitor_mode)
    {
    case Vel_Monitor_Normal:
        is_vel_err = p_motor_status->vel_err_abs > err_limit;
        break;

    case Vel_Monitor_AllowBlock:
        is_vel_err = (p_motor_status->vel_err_abs > err_limit) && (p_motor_status->vel_abs > p_motor_status->block_vel);
        break;

    default:
        break;
    }

    if (is_vel_err)
    {
        if (++p_motor_status->follow_err_tick > MS_TO_RT_TICKS(VELOCITY_ERR_MS))
        {
            p_motor_status->is_follow_err = 1;
            motor_set_err(index, FAULT_CODE_VELOCITY_FOLLOW_ERR);
        }
    }
    else
    {
        p_motor_status->is_follow_err   = 0;
        p_motor_status->follow_err_tick = 0;
    }
}

/**
 * @brief 电机到位与堵转状态更新
 * 
 * @param index 电机编号
 */
static inline void motor_status_update_rt(int index)
{
    Motor_Status_t *p_motor_status = &global_status.motor_status[index];

    if (p_motor_status->is_planning || p_motor_status->vel_abs > p_motor_status->block_vel)
    {
        p_motor_status->is_moving = 1;
    }
    else
    {
        p_motor_status->is_moving = 0;
    }

    if (p_motor_status->arrive_window && p_motor_status->arrive_time)
    {
        if (p_motor_status->pos_err_abs <= p_motor_status->arrive_window && !p_motor_status->is_planning)
        {
            if (p_motor_status->arrive_tick < 0xfffffff)
            {
                p_motor_status->arrive_tick++;
            }
            if (p_motor_status->arrive_tick > MS_TO_RT_TICKS(p_motor_status->arrive_time))
            {
                p_motor_status->is_arrived = 1;
            }
        }
        else
        {
            p_motor_status->is_arrived  = 0;
            p_motor_status->arrive_tick = 0;
        }
    }
    else
    {
        p_motor_status->is_arrived  = 0;
        p_motor_status->arrive_tick = 0;
    }

    if (p_motor_status->vel_abs <= p_motor_status->block_vel && p_motor_status->torque_abs >= p_motor_status->block_tor)
    {
        if (p_motor_status->block_tick < 0xfffffff)
        {
            p_motor_status->block_tick++;
        }
        if (p_motor_status->block_time && p_motor_status->block_tick > MS_TO_RT_TICKS(p_motor_status->block_time))
        {
            p_motor_status->is_blocked = 1;
        }
    }
    else
    {
        p_motor_status->is_blocked = 0;
        p_motor_status->block_tick = 0;
    }
}

/**
 * @brief 电机层检查错误
 * 
 * @param index 电机编号
 */
static inline void motor_check_err_rt(int index)
{
    Power_Status_t *p_power_status                   = &global_status.power_status;
    Peripheral_Raw_Status_t *p_peripheral_raw_status = &global_status.peripheral_raw_status;
    Motor_Config_t *p_motor_config                   = &global_config.motor_config[index];
    Motor_Basic_Status_t *p_motor_basic_status       = &global_status.motor_basic_status[index];
    Motor_Status_t *p_motor_status                   = &global_status.motor_status[index];

    // 过压检测
    if (p_motor_basic_status->v_bus > BOARD_MAX_BUS_VOLTAGE)
    {
        if (++p_motor_status->voltage_up_cnt > MS_TO_RT_TICKS(10))
        {
            motor_set_err(index, FAULT_CODE_OVER_VOLTAGE);
        }
    }
    else
    {
        p_motor_status->voltage_up_cnt = 0;
    }

    // 欠压检测
    if (p_motor_basic_status->v_bus < BOARD_MIN_BUS_VOLTAGE)
    {
        if (++p_motor_status->voltage_down_cnt > MS_TO_RT_TICKS(10))
        {
            motor_set_err(index, FAULT_CODE_UNDER_VOLTAGE);
        }
    }
    else
    {
        p_motor_status->voltage_down_cnt = 0;
    }

    // 过温检测
    if (p_power_status->core_temp > BOARD_MAX_TEMP)
    {
        if (++p_motor_status->core_temp_high_cnt > S_TO_RT_TICKS(5))
        {
            motor_set_err(index, FAULT_CODE_OVER_TEMP_CORE);
        }
    }
    else
    {
        p_motor_status->core_temp_high_cnt = 0;
    }

    if (p_power_status->board_temp > BOARD_MAX_TEMP)
    {
        if (++p_motor_status->core_temp_low_cnt > S_TO_RT_TICKS(5))
        {
            motor_set_err(index, FAULT_CODE_OVER_TEMP_BOARD);
        }
    }
    else
    {
        p_motor_status->core_temp_low_cnt = 0;
    }

    // 电机母线过流检测
    if (fabsf(p_motor_basic_status->i_bus) > p_motor_config->bus_over_current_threshold)
    {
        if (++p_motor_status->bus_over_current_cnt > 1)
        {
            motor_set_err(index, FAULT_CODE_BUS_OVER_CURRENT);
        }
    }
    else
    {
        p_motor_status->bus_over_current_cnt = 0;
    }

    // 板母线过流检测
    if (fabsf(p_power_status->i_bus) > BOARD_MAX_BUS_CURRENT)
    {
        if (++p_motor_status->board_over_current_cnt > 1)
        {
            motor_set_err(index, FAULT_CODE_BOARD_BUS_OVER_CURRENT);
        }
    }
    else
    {
        p_motor_status->board_over_current_cnt = 0;
    }

    // 三相电流不平衡检测
    if (fabsf(p_motor_basic_status->i_sum) > 0.5f)
    {
        if (++p_motor_status->current_unbalance_cnt > MS_TO_RT_TICKS(5))
        {
            motor_set_err(index, FAULT_CODE_UNBALANCED_CURRENTS);
        }
    }
    else
    {
        p_motor_status->current_unbalance_cnt = 0;
    }

    /*
        带热时间常数的一阶热模型
        dH / dt = (I^2 - Irate^2) - H / τ
        H ：热积累量
        τ：热时间常数
    */
    float ir = p_motor_config->rated_current * _1_SQRT2;
    float ip = p_motor_config->peak_current * _1_SQRT2;
#ifndef FAST_CURRENT_LOOP_ENABLE
    float i2 = SQ(p_motor_basic_status->i_rms);
#else
    float i2 = p_motor_basic_status->i_rms;
#endif
    float ir2       = SQ(ir);
    float ip2       = SQ(ip);
    float peak_time = p_motor_config->peak_current_time_s;
    float heat_gen  = (i2 - ir2) / (ip2 - ir2);
    if (heat_gen < 0.0f)
    {
        heat_gen = 0.0f;
    }
    float dthermal = heat_gen / peak_time - p_motor_status->thermal_accumulator / p_motor_config->cooling_time_s;

    p_motor_status->thermal_accumulator += dthermal * BOARD_RT_DT;
    p_motor_status->thermal_accumulator  = CONSTRAIN(p_motor_status->thermal_accumulator, 0.0f, 2.0f);

    if (p_motor_status->thermal_accumulator > p_motor_status->thermal_limit)
    {
        motor_set_err(index, FAULT_CODE_OVER_LOAD);
    }

#if SUPPORTED_STOP_NUMBER > 0
    // 急停错误
    if (p_peripheral_raw_status->sto_in_status == 1)
    {
        motor_set_err(index, FAULT_CODE_STOP);
    }
#endif
}

/**
 * @brief 电机配置参数检查
 * 
 * @param index 电机编号
 */
static inline void motor_config_check(int index)
{
    Motor_Config_t *p_motor_config                       = &global_config.motor_config[index];
    Motor_Controller_Config_t *p_motor_controller_config = &global_config.motor_controller_config[index];

    if (p_motor_config->Ld > FLOAT_MIN)
    {
        p_motor_config->L_diff         = p_motor_config->Ld - p_motor_config->Lq;
        p_motor_config->saliency_ratio = p_motor_config->Lq / p_motor_config->Ld;
    }
    else
    {
        p_motor_config->L_diff         = 0.0f;
        p_motor_config->saliency_ratio = 0.0f;
    }

    if (p_motor_config->motor_type < Motor_Type_BLDC_3_Phase || p_motor_config->motor_type >= Motor_Type_MAX)
    {
        motor_set_ext_err(index, EXT_FAULT_CODE_CFG_MOTOR_TYPE);
    }

    if (p_motor_config->motor_struct < Motor_Struct_Linear || p_motor_config->motor_struct >= Motor_Struct_MAX)
    {
        motor_set_ext_err(index, EXT_FAULT_CODE_CFG_MOTOR_STRUCT);
    }

    if (p_motor_config->rated_voltage < FLOAT_EPSILON)
    {
        motor_set_ext_err(index, EXT_FAULT_CODE_CFG_MOTOR_RATED_VOLTAGE);
    }

    if (p_motor_config->rated_current < FLOAT_EPSILON)
    {
        motor_set_ext_err(index, EXT_FAULT_CODE_CFG_MOTOR_RATED_CURRENT);
    }

    if (p_motor_config->peak_current < FLOAT_EPSILON || p_motor_config->peak_current <= p_motor_config->rated_current)
    {
        motor_set_ext_err(index, EXT_FAULT_CODE_CFG_MOTOR_PEAK_CURRENT);
    }

    if (p_motor_config->rated_speed < FLOAT_EPSILON)
    {
        motor_set_ext_err(index, EXT_FAULT_CODE_CFG_MOTOR_RATED_SPEED);
    }

    if (p_motor_config->peak_speed < FLOAT_EPSILON)
    {
        motor_set_ext_err(index, EXT_FAULT_CODE_CFG_MOTOR_PEAK_SPEED);
    }

    if ((p_motor_config->motor_type == Motor_Type_DC && p_motor_config->kt <= FLOAT_MIN)
        || (p_motor_config->motor_type == Motor_Type_BLDC_3_Phase && p_motor_config->kt <= FLOAT_MIN && p_motor_config->Flux <= FLOAT_MIN))
    {
        motor_set_ext_err(index, EXT_FAULT_CODE_CFG_MOTOR_KT);
    }

    if (p_motor_config->motor_type == Motor_Type_BLDC_3_Phase && p_motor_config->pp < FLOAT_EPSILON)
    {
        motor_set_ext_err(index, EXT_FAULT_CODE_CFG_MOTOR_PP);
    }

    if (p_motor_config->Rs < FLOAT_MIN)
    {
        motor_set_ext_err(index, EXT_FAULT_CODE_CFG_MOTOR_RS);
    }

    if (p_motor_config->Ld < FLOAT_MIN)
    {
        motor_set_ext_err(index, EXT_FAULT_CODE_CFG_MOTOR_LD);
    }

    if (p_motor_config->Lq < FLOAT_MIN)
    {
        motor_set_ext_err(index, EXT_FAULT_CODE_CFG_MOTOR_LQ);
    }

    if (p_motor_config->inertia_rotor < FLOAT_MIN)
    {
        motor_set_ext_err(index, EXT_FAULT_CODE_CFG_MOTOR_ROTOR_INERTIA);
    }

    if (p_motor_config->inertia_load < 0)
    {
        motor_set_ext_err(index, EXT_FAULT_CODE_CFG_MOTOR_LOAD_INERTIA);
    }

    if (p_motor_config->inertia_rotor + p_motor_config->inertia_load < FLOAT_MIN)
    {
        motor_set_ext_err(index, EXT_FAULT_CODE_CFG_MOTOR_INERTIA);
    }

    if (p_motor_config->peak_current_time_s < FLOAT_EPSILON)
    {
        motor_set_ext_err(index, EXT_FAULT_CODE_CFG_MOTOR_PEAK_CURRENT_TIME);
    }

    if (p_motor_config->phase_over_current_threshold < FLOAT_EPSILON)
    {
        motor_set_ext_err(index, EXT_FAULT_CODE_CFG_MOTOR_PHASE_OVER_CURRENT_THRESHOLD);
    }

    if (p_motor_config->bus_over_current_threshold < FLOAT_EPSILON)
    {
        motor_set_ext_err(index, EXT_FAULT_CODE_CFG_MOTOR_BUS_OVER_CURRENT_THRESHOLD);
    }

    if (p_motor_config->cooling_time_s < FLOAT_EPSILON)
    {
        motor_set_ext_err(index, EXT_FAULT_CODE_CFG_MOTOR_COOLING_TIME);
    }

    if (p_motor_config->motor_dir != 1 && p_motor_config->motor_dir != -1)
    {
        motor_set_ext_err(index, EXT_FAULT_CODE_CFG_MOTOR_DIR);
    }

    if (p_motor_config->have_brake != 1 && p_motor_config->have_brake != 0)
    {
        motor_set_ext_err(index, EXT_FAULT_CODE_CFG_MOTOR_HAVE_BRAKE);
    }
    else if (p_motor_config->have_brake == 1)
    {
        if (p_motor_config->brake_open_time < FLOAT_EPSILON)
        {
            motor_set_ext_err(index, EXT_FAULT_CODE_CFG_MOTOR_BRAKE_OPEN_TIME);
        }

        if (p_motor_config->brake_close_time < FLOAT_EPSILON)
        {
            motor_set_ext_err(index, EXT_FAULT_CODE_CFG_MOTOR_BRAKE_CLOSE_TIME);
        }
    }

    if (p_motor_controller_config->vel_feedforward_rate < 0.0f || p_motor_controller_config->vel_feedforward_rate > 1.0f)
    {
        motor_set_ext_err(index, EXT_FAULT_CODE_CFG_CONTROL_VEL_FEED_FORWARD_RATE);
    }

    if (p_motor_controller_config->acc_feedforward_rate < 0.0f || p_motor_controller_config->acc_feedforward_rate > 1.0f)
    {
        motor_set_ext_err(index, EXT_FAULT_CODE_CFG_CONTROL_ACC_FEED_FORWARD_RATE);
    }

    if (p_motor_controller_config->motor_controller_type != Motor_Controller_Type_FOC)
    {
        motor_set_ext_err(index, EXT_FAULT_CODE_CFG_CONTROL_MOTOR_CONTROLLER_TYPE);
    }

    if (p_motor_controller_config->dead_zone_compensation_enable != 0 && p_motor_controller_config->dead_zone_compensation_enable != 1)
    {
        motor_set_ext_err(index, EXT_FAULT_CODE_CFG_CONTROL_DEAD_ZONE_COMP_ENABLE);
    }

    if (p_motor_controller_config->current_decoupling_type < Current_Decoupling_Type_None || p_motor_controller_config->current_decoupling_type >= Current_Decoupling_Type_MAX)
    {
        motor_set_ext_err(index, EXT_FAULT_CODE_CFG_CONTROL_CURRENT_DECOUPLING_TYPE);
    }

    if (p_motor_controller_config->cogging_compensation_enable != 0 && p_motor_controller_config->cogging_compensation_enable != 1)
    {
        motor_set_ext_err(index, EXT_FAULT_CODE_CFG_CONTROL_COGGING_COMP_ENABLE);
    }

    if (p_motor_controller_config->friction_compensation_enable != 0 && p_motor_controller_config->friction_compensation_enable != 1)
    {
        motor_set_ext_err(index, EXT_FAULT_CODE_CFG_CONTROL_FRICTION_COMP_ENABLE);
    }

    if (p_motor_controller_config->spring_compensation_enable != 0 && p_motor_controller_config->spring_compensation_enable != 1)
    {
        motor_set_ext_err(index, EXT_FAULT_CODE_CFG_CONTROL_SPRING_COMP_ENABLE);
    }

    if (p_motor_controller_config->gravity_compensation_enable != 0 && p_motor_controller_config->gravity_compensation_enable != 1)
    {
        motor_set_ext_err(index, EXT_FAULT_CODE_CFG_CONTROL_GRAVITY_COMP_ENABLE);
    }


    switch (p_motor_config->motor_type)
    {
    case Motor_Type_BLDC_3_Phase:
        switch (p_motor_controller_config->motor_controller_type)
        {
        case Motor_Controller_Type_FOC:
            if ((p_motor_controller_config->mtpa_enable || p_motor_controller_config->mtpv_enable || p_motor_controller_config->current_decoupling_type >= Current_Decoupling_Type_Bemf) && p_motor_config->Flux <= FLOAT_MIN)
            {
                motor_set_ext_err(index, EXT_FAULT_CODE_CFG_MOTOR_FLUX);
            }

            if (p_motor_controller_config->max_modulation_ratio > _2_SQRT3 || p_motor_controller_config->max_modulation_ratio < FLOAT_EPSILON)
            {
                motor_set_ext_err(index, EXT_FAULT_CODE_CFG_CONTROL_MODULATION_RATIO);
            }

            if (p_motor_controller_config->mtpa_enable != 0 && p_motor_controller_config->mtpa_enable != 1)
            {
                motor_set_ext_err(index, EXT_FAULT_CODE_CFG_CONTROL_MTPA_ENABLE);
            }

            if (p_motor_controller_config->fw_enable != 0 && p_motor_controller_config->fw_enable != 1)
            {
                motor_set_ext_err(index, EXT_FAULT_CODE_CFG_CONTROL_FW_ENABLE);
            }

            if (p_motor_controller_config->mtpv_enable != 0 && p_motor_controller_config->mtpv_enable != 1)
            {
                motor_set_ext_err(index, EXT_FAULT_CODE_CFG_CONTROL_MTPV_ENABLE);
            }

            if (p_motor_controller_config->align_method < Align_Method_Fast || p_motor_controller_config->align_method >= Align_Method_MAX)
            {
                motor_set_ext_err(index, EXT_FAULT_CODE_CFG_CONTROL_ALIGN_METHOD);
            }

            if (p_motor_controller_config->align_force < FLOAT_EPSILON || p_motor_controller_config->align_force > 1.0f)
            {
                motor_set_ext_err(index, EXT_FAULT_CODE_CFG_CONTROL_ALIGN_FORCE);
            }

            if (p_motor_controller_config->align_time <= 0)
            {
                motor_set_ext_err(index, EXT_FAULT_CODE_CFG_CONTROL_ALIGN_TIME);
            }

            if (p_motor_controller_config->align_threshold < FLOAT_EPSILON)
            {
                motor_set_ext_err(index, EXT_FAULT_CODE_CFG_CONTROL_ALIGN_THRESHOLD);
            }
            break;

        case Motor_Controller_Type_DTC:
            break;

        default:
            break;
        }
        break;

    case Motor_Type_DC:
        if (p_motor_controller_config->current_decoupling_type >= Current_Decoupling_Type_Bemf && p_motor_config->ke <= FLOAT_MIN)
        {
            motor_set_ext_err(index, EXT_FAULT_CODE_CFG_MOTOR_KE);
        }

        if (p_motor_controller_config->current_decoupling_type == Current_Decoupling_Type_Cross || p_motor_controller_config->current_decoupling_type == Current_Decoupling_Type_RL_Cross || p_motor_controller_config->current_decoupling_type == Current_Decoupling_Type_Cross_Bemf || p_motor_controller_config->current_decoupling_type == Current_Decoupling_Type_RL_Cross_Bemf)
        {
            motor_set_ext_err(index, EXT_FAULT_CODE_CFG_CONTROL_CURRENT_DECOUPLING_TYPE);
        }

        if (p_motor_controller_config->max_modulation_ratio > 1.0f || p_motor_controller_config->max_modulation_ratio < FLOAT_EPSILON)
        {
            motor_set_ext_err(index, EXT_FAULT_CODE_CFG_CONTROL_MODULATION_RATIO);
        }
        break;

    default:
        break;
    }
}

/**
 * @brief 根据电机类型和控制方式计算额定/峰值扭矩
 *
 * @param index 电机编号
 */
static void motor_calculate_torque_limits(int index)
{
    Motor_Config_t *p_motor_config                       = &global_config.motor_config[index];
    Motor_Controller_Config_t *p_motor_controller_config = &global_config.motor_controller_config[index];
    Motor_Status_t *p_motor_status                       = &global_status.motor_status[index];

    p_motor_status->rated_torque = 0.0f;
    p_motor_status->peak_torque  = 0.0f;

    switch (p_motor_config->motor_type)
    {
    case Motor_Type_DC:
        dc_calculate_torque_limits(index);
        break;

    case Motor_Type_BLDC_3_Phase:
        switch (p_motor_controller_config->motor_controller_type)
        {
        case Motor_Controller_Type_FOC:
            foc_calculate_torque_limits(index);
            break;

        case Motor_Controller_Type_DTC:
        default:
            /* DTC尚未实现，保留为零以禁止使用无效扭矩能力。 */
            break;
        }
        break;

    default:
        break;
    }
}

/**
 * @brief 规划电机位置轨迹
 * 
 * @param index 电机编号
 * @param pos 目标位置
 * @param vel 目标速度
 * @param acc 目标加速度
 * @param jerk 目标加加速度
 */
void motor_plan_pos(int index, float pos, float vel, float acc, float jerk)
{
    Motor_Status_t *p_motor_status     = &global_status.motor_status[index];
    Motion_Planner_t *p_motion_planner = &motion_planner[index];

    memset(p_motion_planner, 0, sizeof(Motion_Planner_t));
    p_motor_status->is_planning = motion_plan(p_motion_planner, p_motor_status->pos_fb, pos, acc, vel,
                                              jerk, BOARD_RT_DT, 1);
}

/**
 * @brief 电机位置规划实时更新
 * 
 * @param index 电机编号
 */
static inline void motor_pos_plan_update_rt(int index)
{
    Motor_Config_t *p_motor_config                       = &global_config.motor_config[index];
    Motor_Controller_Config_t *p_motor_controller_config = &global_config.motor_controller_config[index];
    Motor_Status_t *p_motor_status                       = &global_status.motor_status[index];
    Motion_Planner_t *p_motion_planner                   = &motion_planner[index];

    if (p_motor_status->is_planning && p_motion_planner->plan_target == Plan_Target_Pos)
    {
        if (motion_update(p_motion_planner))
        {
            p_motor_status->pos_ref = p_motion_planner->pos_output;
            p_motor_status->vel_ff  = p_motion_planner->vel_output * p_motor_controller_config->vel_feedforward_rate;
            p_motor_status->acc_ref = p_motion_planner->acc_output;
        }
        else
        {
            p_motor_status->pos_ref     = p_motion_planner->target;
            p_motor_status->vel_ff      = 0;
            p_motor_status->acc_ref     = 0;
            p_motor_status->is_planning = false;
        }

        p_motor_status->torque_ff = p_motor_status->acc_ref * (p_motor_config->inertia_rotor + p_motor_config->inertia_load) * p_motor_controller_config->acc_feedforward_rate;
    }
}

/**
 * @brief 规划电机速度轨迹
 * 
 * @param index 电机编号
 * @param vel 目标速度
 * @param acc 目标加速度
 * @param jerk 目标加加速度
 */
void motor_plan_vel(int index, float vel, float acc, float jerk)
{
    Motor_Status_t *p_motor_status     = &global_status.motor_status[index];
    Motion_Planner_t *p_motion_planner = &motion_planner[index];

    memset(p_motion_planner, 0, sizeof(Motion_Planner_t));
    p_motor_status->is_planning = velocity_plan(p_motion_planner, p_motor_status->vel_fb, vel, acc, jerk,
                                                BOARD_RT_DT, 1);
}

/**
 * @brief 电机速度规划实时更新
 * 
 * @param index 电机编号
 */
static inline void motor_vel_plan_update_rt(int index)
{
    Motor_Config_t *p_motor_config                       = &global_config.motor_config[index];
    Motor_Controller_Config_t *p_motor_controller_config = &global_config.motor_controller_config[index];
    Motor_Status_t *p_motor_status                       = &global_status.motor_status[index];
    Motion_Planner_t *p_motion_planner                   = &motion_planner[index];
    float total_inertia                                  = p_motor_config->inertia_rotor + p_motor_config->inertia_load;

    if (p_motor_status->is_planning && p_motion_planner->plan_target == Plan_Target_Vel)
    {
        if (velocity_update(p_motion_planner))
        {
            p_motor_status->vel_ref = p_motion_planner->vel_output;
            p_motor_status->acc_ref = p_motion_planner->acc_output;
        }
        else
        {
            p_motor_status->vel_ref     = p_motion_planner->target;
            p_motor_status->acc_ref     = 0;
            p_motor_status->is_planning = false;
        }

        p_motor_status->torque_ff = p_motor_status->acc_ref * total_inertia * p_motor_controller_config->acc_feedforward_rate;
    }
}

/**
 * @brief 电流环实时更新
 * 
 * @param index 电机编号
 */
static inline void current_loop_update_rt(int index)
{
    Motor_Config_t *p_motor_config                       = &global_config.motor_config[index];
    Motor_Controller_Config_t *p_motor_controller_config = &global_config.motor_controller_config[index];
    Motor_Status_t *p_motor_status                       = &global_status.motor_status[index];
    Loop_Filter_t *p_loop_filter_status                  = &loop_filter_status[index];

    p_motor_status->current_ref_f = Process_Filter(&p_loop_filter_status->current_ref_filter, p_motor_status->current_ref);

    switch (p_motor_config->motor_type)
    {
    case Motor_Type_DC:
        dc_set_current_ref(index, p_motor_status->current_ref_f);
        break;

    case Motor_Type_BLDC_3_Phase:
        switch (p_motor_controller_config->motor_controller_type)
        {
        case Motor_Controller_Type_FOC:
            foc_set_current_ref(index, p_motor_status->current_ref_f);
            break;

        default:
            break;
        }
        break;

    default:
        break;
    }
}

/**
 * @brief d 电流扫频实时更新
 * 
 * @param index 电机编号
 */
static inline void current_d_sweep_mode_update_rt(int index)
{
    Motor_Config_t *p_motor_config                       = &global_config.motor_config[index];
    Motor_Controller_Config_t *p_motor_controller_config = &global_config.motor_controller_config[index];
    Motor_Status_t *p_motor_status                       = &global_status.motor_status[index];

    switch (p_motor_config->motor_type)
    {
    case Motor_Type_DC:
        dc_set_current_ref(index, p_motor_status->current_ref);
        break;

    case Motor_Type_BLDC_3_Phase:
        switch (p_motor_controller_config->motor_controller_type)
        {
        case Motor_Controller_Type_FOC:
            foc_controller_set_manual_mode(index, p_motor_status->current_ref, 0.0f);
            break;

        default:
            break;
        }
        break;

    default:
        break;
    }
}

/**
 * @brief q 电流扫频实时更新
 * 
 * @param index 电机编号
 */
static inline void current_q_sweep_mode_update_rt(int index)
{
    Motor_Config_t *p_motor_config                       = &global_config.motor_config[index];
    Motor_Controller_Config_t *p_motor_controller_config = &global_config.motor_controller_config[index];
    Motor_Status_t *p_motor_status                       = &global_status.motor_status[index];

    switch (p_motor_config->motor_type)
    {
    case Motor_Type_DC:
        dc_set_current_ref(index, p_motor_status->current_ref);
        break;

    case Motor_Type_BLDC_3_Phase:
        switch (p_motor_controller_config->motor_controller_type)
        {
        case Motor_Controller_Type_FOC:
            foc_controller_set_manual_mode(index, 0.0f, p_motor_status->current_ref);
            break;

        default:
            break;
        }
        break;

    default:
        break;
    }
}

/**
 * @brief 扭矩环实时更新
 * 
 * @param index 电机编号
 */
static inline void torque_loop_update_rt(int index)
{
    Motor_Config_t *p_motor_config                       = &global_config.motor_config[index];
    Motor_Controller_Config_t *p_motor_controller_config = &global_config.motor_controller_config[index];
    Motor_Status_t *p_motor_status                       = &global_status.motor_status[index];
    Loop_Filter_t *p_loop_filter_status                  = &loop_filter_status[index];

    p_motor_status->torque_ref_f = Process_Filter(&p_loop_filter_status->torque_ref_filter, p_motor_status->torque_ref);

    switch (p_motor_config->motor_type)
    {
    case Motor_Type_DC:
        dc_set_torque_ref(index, p_motor_status->torque_ref_f);
        break;

    case Motor_Type_BLDC_3_Phase:
        switch (p_motor_controller_config->motor_controller_type)
        {
        case Motor_Controller_Type_FOC:
            foc_set_torque_ref(index, p_motor_status->torque_ref_f);
            break;

        default:
            break;
        }
        break;

    default:
        break;
    }
}

/**
 * @brief 速度环实时更新
 * 
 * @param index 电机编号
 */
static inline void speed_loop_update_rt(int index)
{
    Loop_Controller_Config_t *p_loop_controller_config = &global_config.loop_controller_config[index];
    Motor_Observer_Config_t *p_motor_observer_config   = &global_config.motor_observer_config[index];
    Motor_Status_t *p_motor_status                     = &global_status.motor_status[index];
    Loop_Controller_t *p_loop_controller_status        = &global_status.loop_controller_status[index];
    Motor_Observer_t *p_motor_observer                 = &global_status.motor_observer[index];
    Loop_Filter_t *p_loop_filter_status                = &loop_filter_status[index];
    float vel_fb                                       = p_motor_status->vel_fb;
    float torque_ff                                    = p_motor_status->torque_ff;

    switch (p_motor_observer_config->observser_config.observer_type)
    {
    case Observer_Type_NESO2:
    case Observer_Type_NESO3:
        vel_fb     = p_motor_observer->observer_outputs.vel_estimate;
        torque_ff -= p_motor_observer->observer_outputs.tor_disturb_estimate;
        break;

    default:
        break;
    }

    p_motor_status->vel_ref_f = Process_Filter(&p_loop_filter_status->speed_ref_filter, p_motor_status->vel_ref);

    switch (p_loop_controller_config->speed_controller_config.controller_type)
    {
    case Controller_Type_Parallel_PID:
        p_motor_status->vel_out = Parallel_PID_Position_Update(&p_loop_controller_status->speed_pid, torque_ff, p_motor_status->vel_ref_f, vel_fb);
        break;

    case Controller_Type_Serial_PID:
        p_motor_status->vel_out = Serial_PID_Position_Update(&p_loop_controller_status->speed_pid, torque_ff, p_motor_status->vel_ref_f, vel_fb);
        break;

    default:
        break;
    }

    p_motor_status->vel_out_f  = Process_Filter(&p_loop_filter_status->speed_out_filter, p_motor_status->vel_out);
    p_motor_status->torque_ref = p_motor_status->vel_out_f;

    torque_loop_update_rt(index);

    if (p_loop_controller_status->speed_pid.status.saturation)
    {
        motor_set_warning(index, WARN_CODE_SPEED_LOOP_SATURATION);
    }
}

/**
 * @brief 阻抗控制实时更新
 * 
 * @param index 电机编号
 * @param pos_err 位置误差
 * @param vel_err 速度误差
 * @param acc_err 加速度误差
 * @param torque_ff 扭矩前馈值
 */
static inline void impedance_update_rt(int index, float pos_err, float vel_err, float acc_err, float torque_ff, bool use_md)
{
    Motor_Status_t *p_motor_status              = &global_status.motor_status[index];
    Loop_Controller_t *p_loop_controller_status = &global_status.loop_controller_status[index];

    Impedance_Update(&p_loop_controller_status->force_compliance, pos_err, vel_err, acc_err, torque_ff, use_md);
    p_motor_status->torque_ref = p_loop_controller_status->force_compliance.status.force_cmd;
    torque_loop_update_rt(index);
}

/**
 * @brief 位置环实时更新
 * 
 * @param index 电机编号
 */
static inline void pos_loop_update_rt(int index)
{
    Loop_Controller_Config_t *p_loop_controller_config = &global_config.loop_controller_config[index];
    Motor_Observer_Config_t *p_motor_observer_config   = &global_config.motor_observer_config[index];
    Motor_Status_t *p_motor_status                     = &global_status.motor_status[index];
    Loop_Controller_t *p_loop_controller_status        = &global_status.loop_controller_status[index];
    Motor_Observer_t *p_motor_observer                 = &global_status.motor_observer[index];
    Loop_Filter_t *p_loop_filter_status                = &loop_filter_status[index];
    float pos_fb                                       = p_motor_status->pos_fb;

    p_motor_status->pos_ref_f = Process_Filter(&p_loop_filter_status->pos_ref_filter, p_motor_status->pos_ref);

    if (p_motor_observer_config->observser_config.observer_type == Observer_Type_NESO3)
    {
        pos_fb = p_motor_observer->observer_outputs.pos_estimate;
    }

    switch (p_loop_controller_config->pos_controller_config.controller_type)
    {
    case Controller_Type_Parallel_PID:
        p_motor_status->vel_ref = Parallel_PID_Position_Update(&p_loop_controller_status->pos_pid, p_motor_status->vel_ff, p_motor_status->pos_ref_f, pos_fb);
        speed_loop_update_rt(index);
        break;

    case Controller_Type_Serial_PID:
        p_motor_status->vel_ref = Serial_PID_Position_Update(&p_loop_controller_status->pos_pid, p_motor_status->vel_ff, p_motor_status->pos_ref_f, pos_fb);
        speed_loop_update_rt(index);
        break;

    case Controller_Type_Impedance:
        p_motor_status->vel_ref_f                                  = Process_Filter(&p_loop_filter_status->speed_ref_filter, p_motor_status->vel_ff);
        p_motor_status->vel_ref_f                                  = CONSTRAIN(p_motor_status->vel_ref_f, p_motor_status->negative_vel_limit, p_motor_status->positive_vel_limit);
        p_loop_controller_status->force_compliance.limit.max_force = p_motor_status->positive_tor_limit;
        p_loop_controller_status->force_compliance.limit.min_force = p_motor_status->negative_tor_limit;
        p_motor_status->acc_err                                    = p_motor_status->acc_ref - p_motor_status->acc_fb;
        p_motor_status->vel_err                                    = p_motor_status->vel_ref_f - p_motor_status->vel_fb;
        p_motor_status->pos_err                                    = p_motor_status->pos_ref_f - p_motor_status->pos_fb;
        impedance_update_rt(index, p_motor_status->pos_err, p_motor_status->vel_err, p_motor_status->acc_err, p_motor_status->torque_ff, true);
        break;

    default:
        break;
    }

    if (p_loop_controller_status->pos_pid.status.saturation)
    {
        motor_set_warning(index, WARN_CODE_POS_LOOP_SATURATION);
    }
}

/**
 * @brief 寻相模式实时更新
 * 
 * @param index 电机编号
 */
static inline void align_mode_update_rt(int index)
{
    Motor_Config_t *p_motor_config                       = &global_config.motor_config[index];
    Motor_Controller_Config_t *p_motor_controller_config = &global_config.motor_controller_config[index];

    switch (p_motor_config->motor_type)
    {
    case Motor_Type_DC:
        dc_align_update_rt(index);
        break;

    case Motor_Type_BLDC_3_Phase:
        switch (p_motor_controller_config->motor_controller_type)
        {
        case Motor_Controller_Type_FOC:
            foc_align_update_rt(index);
            break;

        default:
            break;
        }
        break;

    default:
        break;
    }
}

/**
 * @brief 电流模式实时更新
 * 
 * @param index 电机编号
 */
static inline void current_mode_update_rt(int index)
{
    current_loop_update_rt(index);
}

/**
 * @brief 扭矩模式实时更新
 * 
 * @param index 电机编号
 */
static inline void torque_mode_update_rt(int index)
{
    torque_loop_update_rt(index);
}

/**
 * @brief 速度模式实时更新
 * 
 * @param index 电机编号
 */
static inline void speed_mode_update_rt(int index)
{
    motor_vel_plan_update_rt(index);
    speed_loop_update_rt(index);
    vel_follow_err_update_rt(index);
}

/**
 * @brief 位置模式实时更新
 * 
 * @param index 电机编号
 */
static inline void pos_mode_update_rt(int index)
{
    motor_pos_plan_update_rt(index);
    pos_loop_update_rt(index);
    pos_follow_err_update_rt(index);
}

/**
 * @brief 导纳控制实时更新
 * 
 * @param index 电机编号
 * @param force_err 力误差
 * @param use_kd 使用虚拟刚度
 */
static inline void admittance_update_rt(int index, float force_err, bool use_kd)
{
    Motor_Status_t *p_motor_status              = &global_status.motor_status[index];
    Loop_Controller_t *p_loop_controller_status = &global_status.loop_controller_status[index];

    Admittance_Update(&p_loop_controller_status->force_compliance, force_err, use_kd);

    if (use_kd)
    {
        p_motor_status->pos_ref = p_loop_controller_status->force_compliance.status.base_pos + p_loop_controller_status->force_compliance.status.x_pos;

        pos_loop_update_rt(index);
    }
    else
    {
        p_motor_status->vel_ref = p_loop_controller_status->force_compliance.status.x_vel;
        speed_loop_update_rt(index);
    }
}

/**
 * @brief 示教控制实时更新
 * 
 * @param index 电机编号
 * @param force_err 力误差
 * @param use_kd 使用虚拟刚度
 */
static inline void teaching_update_rt(int index, float force_err)
{
    Motor_Status_t *p_motor_status              = &global_status.motor_status[index];
    Loop_Controller_t *p_loop_controller_status = &global_status.loop_controller_status[index];

    Admittance_Update(&p_loop_controller_status->force_compliance, force_err, false);

    if (fabsf(force_err) < p_motor_status->force_ref)
    {
        p_loop_controller_status->force_compliance.status.x_vel = 0;
        p_loop_controller_status->force_compliance.status.x_acc = 0;
    }

    float pos_ref = p_motor_status->pos_ref + p_loop_controller_status->force_compliance.status.x_vel * BOARD_RT_DT;

    p_motor_status->pos_ref = CONSTRAIN(pos_ref, p_loop_controller_status->force_compliance.limit.min_position, p_loop_controller_status->force_compliance.limit.max_position);

    pos_loop_update_rt(index);
}

/**
 * @brief 软着陆实时更新
 * 
 * @param index 电机编号
 */
static inline void softland_update_rt(int index)
{
    Loop_Controller_Config_t *p_loop_controller_config = &global_config.loop_controller_config[index];
    Motor_Status_t *p_motor_status                     = &global_status.motor_status[index];

    switch (p_loop_controller_config->force_controller_config.controller_type)
    {
    case Force_Controller_Type_Admittance:
        admittance_update_rt(index, p_motor_status->force_err, false);
        break;

    case Force_Controller_Type_Impedance:
        p_motor_status->acc_err = p_motor_status->acc_ref - p_motor_status->acc_fb;
        p_motor_status->vel_err = p_motor_status->vel_ref_f - p_motor_status->vel_fb;
        p_motor_status->pos_err = p_motor_status->pos_ref_f - p_motor_status->pos_fb;

        impedance_update_rt(index, p_motor_status->pos_err, p_motor_status->vel_err, p_motor_status->acc_err, p_motor_status->torque_ff, false);
        break;

    default:
        break;
    }
}

/**
 * @brief 软着陆模式速度规划实时更新
 * 
 * @param index 电机编号
 * @param vel_ref 速度参考值
 */
static inline void softland_plan_rt(int index, float vel_ref)
{
    Loop_Controller_Config_t *p_loop_controller_config = &global_config.loop_controller_config[index];
    Motor_Status_t *p_motor_status                     = &global_status.motor_status[index];
    Loop_Filter_t *p_loop_filter_status                = &loop_filter_status[index];

    switch (p_loop_controller_config->force_controller_config.controller_type)
    {
    case Force_Controller_Type_Impedance:
        p_motor_status->pos_ref   += vel_ref * BOARD_RT_DT;
        p_motor_status->vel_ref    = vel_ref;
        p_motor_status->acc_ref    = 0;
        p_motor_status->pos_ref_f  = Process_Filter(&p_loop_filter_status->pos_ref_filter, p_motor_status->pos_ref);
        p_motor_status->vel_ref_f  = Process_Filter(&p_loop_filter_status->speed_ref_filter, p_motor_status->vel_ref);
        break;

    default:
        break;
    }
}

/**
 * @brief 软着陆复位
 * 
 * @param index 电机编号
 */
void softland_reset(int index)
{
    Loop_Controller_Config_t *p_loop_controller_config = &global_config.loop_controller_config[index];
    Motor_Status_t *p_motor_status                     = &global_status.motor_status[index];
    Loop_Controller_t *p_loop_controller_status        = &global_status.loop_controller_status[index];

    Compliance_Controller_Reset(&p_loop_controller_status->force_compliance);

    switch (p_loop_controller_config->force_controller_config.controller_type)
    {
    case Force_Controller_Type_Admittance:
        p_loop_controller_status->force_compliance.limit.max_velocity     = p_motor_status->force_status.max_vel;
        p_loop_controller_status->force_compliance.limit.min_velocity     = -p_motor_status->force_status.vel;
        p_loop_controller_status->force_compliance.limit.max_acceleration = p_motor_status->force_status.acc;
        p_loop_controller_status->force_compliance.limit.min_acceleration = -p_motor_status->force_status.acc;
        break;

    case Force_Controller_Type_Impedance:
        p_loop_controller_status->force_compliance.limit.max_force   = p_motor_status->force_status.max_force;
        p_loop_controller_status->force_compliance.limit.min_force   = p_motor_status->negative_tor_limit;
        p_loop_controller_status->force_compliance.limit.max_pos_err = 1.0f;
        p_loop_controller_status->force_compliance.limit.max_vel_err = 1.0f;
        p_loop_controller_status->force_compliance.limit.max_acc_err = 1.0f;
        break;

    default:
        break;
    }
}

/**
 * @brief 开环力控模式实时更新
 * 
 * @param index 电机编号
 */
static inline void ol_force_mode_update_rt(int index)
{
    Motor_Status_t *p_motor_status = &global_status.motor_status[index];
    float hold_toruqe              = torque_to_torque_rate(index, p_motor_status->torque_abs * 1.1f);

    switch (p_motor_status->force_status.force_stage)
    {
    case Force_Stage_Pos:
        pos_mode_update_rt(index);
        if (p_motor_status->arrive_tick > MS_TO_RT_TICKS(1))
        {
            p_motor_status->force_status.force_stage = Force_Stage_Vel;
            motor_set_vel_monitor_mode(index, Vel_Monitor_AllowBlock);
            motor_set_torque_limit(index, p_motor_status->force_status.max_tor, -p_motor_status->force_status.max_tor);
            motor_plan_vel(index, p_motor_status->force_status.max_vel, p_motor_status->force_status.acc, p_motor_status->force_status.jerk);
        }
        else if (p_motor_status->block_tick > MS_TO_RT_TICKS(1) && p_motor_status->pos_fb < p_motor_status->force_status.block_pos && p_motor_status->force_status.block_pos > FLOAT_EPSILON)
        {
            p_motor_status->force_status.force_stage = Force_Stage_Block;
            motor_reset_torque_limit(index);
            motor_plan_pos(index, p_motor_status->force_status.lift_pos, p_motor_status->force_status.vel, p_motor_status->force_status.acc, p_motor_status->force_status.jerk);
        }
        break;

    case Force_Stage_Vel:
        speed_mode_update_rt(index);
        if (p_motor_status->pos_fb >= p_motor_status->force_status.max_pos && p_motor_status->force_status.max_pos > FLOAT_EPSILON)
        {
            p_motor_status->force_status.force_stage = Force_Stage_Pos_Done;
            motor_set_torque_limit(index, hold_toruqe, -hold_toruqe);
            motor_plan_pos(index, p_motor_status->force_status.max_pos, p_motor_status->force_status.vel, p_motor_status->force_status.acc, p_motor_status->force_status.jerk);
        }
        else if (p_motor_status->block_tick > MS_TO_RT_TICKS(1))
        {
            if (p_motor_status->pos_fb < p_motor_status->force_status.block_pos && p_motor_status->force_status.block_pos > FLOAT_EPSILON)
            {
                p_motor_status->force_status.force_stage = Force_Stage_Block;
                motor_reset_torque_limit(index);
                motor_plan_pos(index, p_motor_status->force_status.lift_pos, p_motor_status->force_status.vel, p_motor_status->force_status.acc, p_motor_status->force_status.jerk);
            }
            else
            {
                p_motor_status->force_status.force_stage = Force_Stage_Vel_Done;
            }
        }
        break;

    case Force_Stage_Pos_Done:
        pos_mode_update_rt(index);
        break;

    case Force_Stage_Vel_Done:
        speed_mode_update_rt(index);
        if (p_motor_status->pos_fb >= p_motor_status->force_status.max_pos && p_motor_status->force_status.max_pos > FLOAT_EPSILON)
        {
            p_motor_status->force_status.force_stage = Force_Stage_Pos_Done;
            motor_set_torque_limit(index, hold_toruqe, -hold_toruqe);
            motor_plan_pos(index, p_motor_status->force_status.max_pos, p_motor_status->force_status.vel, p_motor_status->force_status.acc, p_motor_status->force_status.jerk);
        }
        break;

    case Force_Stage_Block:
    case Force_Stage_Err:
        pos_mode_update_rt(index);
        break;

    default:
        break;
    }
}

/**
 * @brief 闭环力控模式实时更新
 * 
 * @param index 电机编号
 */
static inline void cl_force_mode_update_rt(int index)
{
    Motor_Status_t *p_motor_status      = &global_status.motor_status[index];
    Loop_Filter_t *p_loop_filter_status = &loop_filter_status[index];

    float hold_toruqe   = torque_to_torque_rate(index, p_motor_status->torque_abs * 1.1f);
    float force_fb      = motot_get_force_fb(index);
    float force_fb_f    = Process_Filter(&p_loop_filter_status->force_fb_filter, force_fb);
    float force_err     = p_motor_status->force_status.max_force - force_fb_f;
    float force_err_abs = fabsf(force_err);
    bool is_block_pos   = p_motor_status->pos_fb < p_motor_status->force_status.block_pos && p_motor_status->force_status.block_pos > FLOAT_EPSILON;
    bool is_arrived     = p_motor_status->arrive_tick > MS_TO_RT_TICKS(1);
    bool is_blocked     = p_motor_status->block_tick > MS_TO_RT_TICKS(1) && is_block_pos;
    bool is_max_pos     = p_motor_status->pos_fb >= p_motor_status->force_status.max_pos && p_motor_status->force_status.max_pos > FLOAT_EPSILON;

    p_motor_status->force_ref     = p_motor_status->force_status.max_force;
    p_motor_status->force_fb      = force_fb;
    p_motor_status->force_fb_f    = force_fb_f;
    p_motor_status->force_err     = force_err;
    p_motor_status->force_abs     = fabsf(force_fb_f);
    p_motor_status->force_err_abs = force_err_abs;

    if (force_err < -p_motor_status->force_status.max_force_err && p_motor_status->force_status.max_force_err_time > FLOAT_EPSILON)
    {
        p_motor_status->force_status.force_over_cnt++;
    }
    else
    {
        p_motor_status->force_status.force_over_cnt = 0;
    }

    if (force_err_abs < p_motor_status->force_status.max_force_err)
    {
        p_motor_status->force_status.force_reach_cnt++;
    }
    else
    {
        p_motor_status->force_status.force_reach_cnt = 0;
    }

    bool is_force_over  = p_motor_status->force_status.force_over_cnt > MS_TO_RT_TICKS(p_motor_status->force_status.max_force_err_time);
    bool is_force_reach = p_motor_status->force_status.force_reach_cnt > MS_TO_RT_TICKS(p_motor_status->force_status.max_force_err_time);

    if (is_force_over && p_motor_status->force_status.force_stage < Force_Stage_Pos_Done)
    {
        p_motor_status->force_status.force_stage = Force_Stage_Err;
        motor_reset_torque_limit(index);
        motor_plan_pos(index, p_motor_status->force_status.lift_pos, p_motor_status->force_status.vel, p_motor_status->force_status.acc, p_motor_status->force_status.jerk);
    }

    switch (p_motor_status->force_status.force_stage)
    {
    case Force_Stage_Pos:
        pos_mode_update_rt(index);
        if (is_arrived)
        {
            p_motor_status->force_status.force_stage = Force_Stage_Force;
            softland_reset(index);
            // motor_set_torque_limit(index, p_motor_status->force_status.max_tor, -p_motor_status->force_status.max_tor);
        }
        else if (is_blocked)
        {
            p_motor_status->force_status.force_stage = Force_Stage_Block;
            motor_reset_torque_limit(index);
            motor_plan_pos(index, p_motor_status->force_status.lift_pos, p_motor_status->force_status.vel, p_motor_status->force_status.acc, p_motor_status->force_status.jerk);
        }
        break;

    case Force_Stage_Force:
        softland_plan_rt(index, p_motor_status->force_status.max_vel);
        softland_update_rt(index);
        if (is_max_pos)
        {
            p_motor_status->force_status.force_stage = Force_Stage_Pos_Done;
            motor_set_torque_limit(index, hold_toruqe, -hold_toruqe);
            motor_plan_pos(index, p_motor_status->force_status.max_pos, p_motor_status->force_status.vel, p_motor_status->force_status.acc, p_motor_status->force_status.jerk);
        }
        else if (is_blocked)
        {
            p_motor_status->force_status.force_stage = Force_Stage_Block;
            motor_reset_torque_limit(index);
            motor_plan_pos(index, p_motor_status->force_status.lift_pos, p_motor_status->force_status.vel, p_motor_status->force_status.acc, p_motor_status->force_status.jerk);
        }
        else if (is_force_reach)
        {
            if (is_block_pos)
            {
                p_motor_status->force_status.force_stage = Force_Stage_Block;
                motor_reset_torque_limit(index);
                motor_plan_pos(index, p_motor_status->force_status.lift_pos, p_motor_status->force_status.vel, p_motor_status->force_status.acc, p_motor_status->force_status.jerk);
            }
            else
            {
                p_motor_status->force_status.force_stage = Force_Stage_Force_Done;
            }
        }
        break;

    case Force_Stage_Pos_Done:
        pos_mode_update_rt(index);
        break;

    case Force_Stage_Force_Done:
        softland_plan_rt(index, 0);
        softland_update_rt(index);
        if (is_max_pos)
        {
            p_motor_status->force_status.force_stage = Force_Stage_Pos_Done;
            motor_set_torque_limit(index, hold_toruqe, -hold_toruqe);
            motor_plan_pos(index, p_motor_status->force_status.max_pos, p_motor_status->force_status.vel, p_motor_status->force_status.acc, p_motor_status->force_status.jerk);
        }
        break;

    case Force_Stage_Block:
    case Force_Stage_Err:
        pos_mode_update_rt(index);
        break;

    default:
        break;
    }
}

/**
 * @brief 阻抗模式实时更新
 * 
 * @param index 电机编号
 */
static inline void impedance_mode_update_rt(int index)
{
    Motor_Status_t *p_motor_status      = &global_status.motor_status[index];
    Loop_Filter_t *p_loop_filter_status = &loop_filter_status[index];

    motor_pos_plan_update_rt(index);

    p_motor_status->pos_ref_f = Process_Filter(&p_loop_filter_status->pos_ref_filter, p_motor_status->pos_ref);
    p_motor_status->vel_ref_f = Process_Filter(&p_loop_filter_status->speed_ref_filter, p_motor_status->vel_ff);

    p_motor_status->acc_err = p_motor_status->acc_ref - p_motor_status->acc_fb;
    p_motor_status->vel_err = p_motor_status->vel_ref_f - p_motor_status->vel_fb;
    p_motor_status->pos_err = p_motor_status->pos_ref_f - p_motor_status->pos_fb;

    impedance_update_rt(index, p_motor_status->pos_err, p_motor_status->vel_err, p_motor_status->acc_err, p_motor_status->torque_ff, true);
}

/**
 * @brief 导纳模式实时更新
 * 
 * @param index 电机编号
 */
static inline void admittance_mode_update_rt(int index)
{
    Motor_Status_t *p_motor_status              = &global_status.motor_status[index];
    Loop_Controller_t *p_loop_controller_status = &global_status.loop_controller_status[index];
    Loop_Filter_t *p_loop_filter_status         = &loop_filter_status[index];

    float force_fb    = motot_get_force_fb(index);
    float force_fb_f  = Process_Filter(&p_loop_filter_status->force_fb_filter, force_fb);
    float force_err   = p_motor_status->force_ref - force_fb_f;
    bool was_planning = p_motor_status->is_planning;

    motor_pos_plan_update_rt(index);

    p_motor_status->force_fb      = force_fb;
    p_motor_status->force_fb_f    = force_fb_f;
    p_motor_status->force_err     = force_err;
    p_motor_status->force_abs     = fabsf(force_fb_f);
    p_motor_status->force_err_abs = fabsf(p_motor_status->force_err);

    if (was_planning)
    {
        p_loop_controller_status->force_compliance.status.base_pos = p_motor_status->pos_ref;
    }
    admittance_update_rt(index, force_err, true);
}

/**
 * @brief 示教模式实时更新
 * 
 * @param index 电机编号
 */
static inline void teaching_mode_update_rt(int index)
{
    Motor_Status_t *p_motor_status      = &global_status.motor_status[index];
    Loop_Filter_t *p_loop_filter_status = &loop_filter_status[index];

    float force_fb   = motot_get_force_fb(index);
    float force_fb_f = Process_Filter(&p_loop_filter_status->force_fb_filter, force_fb);
    float force_err  = p_motor_status->force_ref - force_fb_f;

    p_motor_status->force_fb      = force_fb;
    p_motor_status->force_fb_f    = force_fb_f;
    p_motor_status->force_err     = force_err;
    p_motor_status->force_abs     = fabsf(force_fb_f);
    p_motor_status->force_err_abs = fabsf(p_motor_status->force_err);

    teaching_update_rt(index, force_err);
}

/**
 * @brief 周期同步扭矩模式实时更新
 * 
 * @param index 电机编号
 */
static inline void cst_mode_update_rt(int index)
{
    speed_loop_update_rt(index);
}

/**
 * @brief 周期同步速度模式实时更新
 * 
 * @param index 电机编号
 */
static inline void csv_mode_update_rt(int index)
{
    vel_follow_err_update_rt(index);
    speed_loop_update_rt(index);
}

/**
 * @brief 周期同步位置模式实时更新
 * 
 * @param index 电机编号
 */
static inline void csp_mode_update_rt(int index)
{
    pos_follow_err_update_rt(index);
    pos_loop_update_rt(index);
}

/**
 * @brief 速度扫频实时更新
 * 
 * @param index 电机编号
 */
static inline void vel_sweep_mode_update_rt(int index)
{
    speed_loop_update_rt(index);
}

/**
 * @brief 位置扫频实时更新
 * 
 * @param index 电机编号
 */
static inline void pos_sweep_mode_update_rt(int index)
{
    pos_loop_update_rt(index);
}

/**
 * @brief 电压开环相位模式实时更新
 * 
 * @param index 电机编号
 */
static inline void vf_theta_mode_update_rt(int index)
{
    Motor_Config_t *p_motor_config                       = &global_config.motor_config[index];
    Motor_Controller_Config_t *p_motor_controller_config = &global_config.motor_controller_config[index];
    Motor_Status_t *p_motor_status                       = &global_status.motor_status[index];

    switch (p_motor_config->motor_type)
    {
    case Motor_Type_DC:
        dc_set_vf_voltage(index, p_motor_status->current_ref);
        break;

    case Motor_Type_BLDC_3_Phase:
        switch (p_motor_controller_config->motor_controller_type)
        {
        case Motor_Controller_Type_FOC:
            foc_set_vf_voltage(index, p_motor_status->current_ref, 0.0f, &p_motor_status->pos_ref, NULL);
            break;

        default:
            break;
        }
        break;

    default:
        break;
    }
}

/**
 * @brief 电压开环速度模式实时更新
 * 
 * @param index 电机编号
 */
static inline void vf_speed_mode_update_rt(int index)
{
    Motor_Config_t *p_motor_config                       = &global_config.motor_config[index];
    Motor_Controller_Config_t *p_motor_controller_config = &global_config.motor_controller_config[index];
    Motor_Status_t *p_motor_status                       = &global_status.motor_status[index];

    switch (p_motor_config->motor_type)
    {
    case Motor_Type_DC:
        dc_set_vf_voltage(index, p_motor_status->current_ref);
        break;

    case Motor_Type_BLDC_3_Phase:
        switch (p_motor_controller_config->motor_controller_type)
        {
        case Motor_Controller_Type_FOC:
            foc_set_vf_voltage(index, p_motor_status->current_ref, 0.0f, NULL, &p_motor_status->vel_ref);
            break;

        default:
            break;
        }
        break;

    default:
        break;
    }
}

/**
 * @brief 电流开环相位模式实时更新
 * 
 * @param index 电机编号
 */
static inline void if_theta_mode_update_rt(int index)
{
    Motor_Config_t *p_motor_config                       = &global_config.motor_config[index];
    Motor_Controller_Config_t *p_motor_controller_config = &global_config.motor_controller_config[index];
    Motor_Status_t *p_motor_status                       = &global_status.motor_status[index];

    switch (p_motor_config->motor_type)
    {
    case Motor_Type_DC:
        dc_set_if_current(index, p_motor_status->current_ref);
        break;

    case Motor_Type_BLDC_3_Phase:
        switch (p_motor_controller_config->motor_controller_type)
        {
        case Motor_Controller_Type_FOC:
            foc_set_if_current(index, p_motor_status->current_ref, 0.0f, &p_motor_status->pos_ref, NULL);
            break;

        default:
            break;
        }
        break;

    default:
        break;
    }
}

/**
 * @brief 电流开环速度模式实时更新
 * 
 * @param index 电机编号
 */
static inline void if_speed_mode_update_rt(int index)
{
    Motor_Config_t *p_motor_config                       = &global_config.motor_config[index];
    Motor_Controller_Config_t *p_motor_controller_config = &global_config.motor_controller_config[index];
    Motor_Status_t *p_motor_status                       = &global_status.motor_status[index];

    switch (p_motor_config->motor_type)
    {
    case Motor_Type_DC:
        dc_set_if_current(index, p_motor_status->current_ref);
        break;

    case Motor_Type_BLDC_3_Phase:
        switch (p_motor_controller_config->motor_controller_type)
        {
        case Motor_Controller_Type_FOC:
            foc_set_if_current(index, p_motor_status->current_ref, 0.0f, NULL, &p_motor_status->vel_ref);
            break;

        default:
            break;
        }
        break;

    default:
        break;
    }
}

/**
 * @brief HFI开环模式实时更新
 * 
 * @param index 电机编号
 */
static inline void vf_hfi_mode_update_rt(int index)
{
    Motor_Config_t *p_motor_config                       = &global_config.motor_config[index];
    Motor_Controller_Config_t *p_motor_controller_config = &global_config.motor_controller_config[index];
    Motor_Status_t *p_motor_status                       = &global_status.motor_status[index];

    switch (p_motor_config->motor_type)
    {
    case Motor_Type_DC:
        break;

    case Motor_Type_BLDC_3_Phase:
        switch (p_motor_controller_config->motor_controller_type)
        {
        case Motor_Controller_Type_FOC:
            foc_set_hfi_vf_voltage(index, p_motor_status->current_ref, &p_motor_status->pos_ref, &p_motor_status->vel_ref);
            break;

        default:
            break;
        }
        break;

    default:
        break;
    }
}

/**
 * @brief 相电阻辨识实时更新
 * 
 * @param index 电机编号
 */
static inline void rs_identify_mode_update_rt(int index)
{
    Motor_Config_t *p_motor_config                       = &global_config.motor_config[index];
    Motor_Controller_Config_t *p_motor_controller_config = &global_config.motor_controller_config[index];

    switch (p_motor_config->motor_type)
    {
    case Motor_Type_DC:
        dc_rs_identify_update_rt(index);
        break;

    case Motor_Type_BLDC_3_Phase:
        switch (p_motor_controller_config->motor_controller_type)
        {
        case Motor_Controller_Type_FOC:
            foc_rs_identify_update_rt(index);
            break;

        default:
            break;
        }
        break;

    default:
        break;
    }
}

/**
 * @brief 相电感辨识实时更新
 * 
 * @param index 电机编号
 */
static inline void ls_identify_mode_update_rt(int index)
{
    Motor_Config_t *p_motor_config                       = &global_config.motor_config[index];
    Motor_Controller_Config_t *p_motor_controller_config = &global_config.motor_controller_config[index];

    switch (p_motor_config->motor_type)
    {
    case Motor_Type_DC:
        dc_ls_identify_update_rt(index);
        break;

    case Motor_Type_BLDC_3_Phase:
        switch (p_motor_controller_config->motor_controller_type)
        {
        case Motor_Controller_Type_FOC:
            foc_ls_identify_update_rt(index);
            break;

        default:
            break;
        }
        break;

    default:
        break;
    }
}

/**
 * @brief 极对数辨识实时更新
 * 
 * @param index 电机编号
 */
static inline void pp_identify_mode_update_rt(int index)
{
    Motor_Config_t *p_motor_config                       = &global_config.motor_config[index];
    Motor_Controller_Config_t *p_motor_controller_config = &global_config.motor_controller_config[index];

    switch (p_motor_config->motor_type)
    {
    case Motor_Type_DC:
        dc_pp_identify_update_rt(index);
        break;

    case Motor_Type_BLDC_3_Phase:
        switch (p_motor_controller_config->motor_controller_type)
        {
        case Motor_Controller_Type_FOC:
            foc_pp_identify_update_rt(index);
            break;

        default:
            break;
        }
        break;

    default:
        break;
    }
}

/**
 * @brief 磁链辨识实时更新
 * 
 * @param index 电机编号
 */
static inline void flux_identify_mode_update_rt(int index)
{
    Motor_Config_t *p_motor_config                       = &global_config.motor_config[index];
    Motor_Controller_Config_t *p_motor_controller_config = &global_config.motor_controller_config[index];

    switch (p_motor_config->motor_type)
    {
    case Motor_Type_DC:
        dc_flux_identify_update_rt(index);
        break;

    case Motor_Type_BLDC_3_Phase:
        switch (p_motor_controller_config->motor_controller_type)
        {
        case Motor_Controller_Type_FOC:
            foc_flux_identify_update_rt(index);
            break;

        default:
            break;
        }
        break;

    default:
        break;
    }
}

/**
 * @brief 齿槽辨识实时更新
 * 
 * @param index 电机编号
 */
static inline void cogg_identify_mode_update_rt(int index)
{
}

/**
 * @brief 惯量辨识实时更新
 * 
 * @param index 电机编号
 */
static inline void inertia_identify_mode_update_rt(int index)
{
}

/**
 * @brief 摩擦辨识实时更新
 * 
 * @param index 电机编号
 */
static inline void friction_identify_mode_update_rt(int index)
{
}

/**
 * @brief 重力辨识实时更新
 * 
 * @param index 电机编号
 */
static inline void gravity_identify_mode_update_rt(int index)
{
}

/**
 * @brief 计算 PWM 影子寄存器更新时间
 *
 * @param index 电机编号
 * @return 更新时间延时，单位 us
 */
void pwm_update_delay_record(int index)
{
    uint32_t pwm_counter = get_pwm_counter(0);
#if SUMODE == SSSU
    global_status.core_status.pwm_update_time[index] = BOARD_MOTO_CNT_TO_US(pwm_counter);
#elif SUMODE == DSSU
    global_status.core_status.pwm_update_time[index] = BOARD_MOTO_CNT_TO_US(pwm_counter);
#elif SUMODE == DSDU
    if (pwm_counter > MOTOR0_BLDCPWM_TRIG_CMP1_VALUE)
    {
        global_status.core_status.pwm_update_time[index] = BOARD_MOTO_CNT_TO_US(pwm_counter) - BOARD_HRT_DT * 1e6f;
    }
    else
    {
        global_status.core_status.pwm_update_time[index] = BOARD_MOTO_CNT_TO_US(pwm_counter);
    }
#endif
}

float current_loop_delay_calculate(int index)
{
#if SUMODE == SSSU
    return BOARD_HRT_DT * 1.5f;
#elif SUMODE == DSSU
    return BOARD_HRT_DT * 1.75f;
#elif SUMODE == DSDU
    return BOARD_HRT_DT * 1.5f;
#endif
}

/**
 * @brief 环路控制器参数非实时更新
 * 
 * @param index 电机编号
 */
static inline void loop_controller_config_update_nrt(int index)
{
    Motor_Config_t *p_motor_config                     = &global_config.motor_config[index];
    Loop_Controller_Config_t *p_loop_controller_config = &global_config.loop_controller_config[index];
    Loop_Controller_t *p_loop_controller_status        = &global_status.loop_controller_status[index];

    if (p_loop_controller_config->current_controller_config.controller_type < Controller_Type_Parallel_PID || p_loop_controller_config->current_controller_config.controller_type >= Controller_Type_MAX)
    {
        motor_set_ext_err(index, EXT_FAULT_CODE_CFG_CONTROL_CURRENT_CONTROLLER_TYPE);
    }

    if (p_loop_controller_config->current_controller_config.auto_tune == 0)
    {
        memcpy(&p_loop_controller_status->id_pid.config, &p_loop_controller_config->id_pid_config, sizeof(PID_Config_t));
        memcpy(&p_loop_controller_status->iq_pid.config, &p_loop_controller_config->iq_pid_config, sizeof(PID_Config_t));
    }
    else if (p_loop_controller_config->current_controller_config.auto_tune == 1)
    {
        float current_loop_delay = current_loop_delay_calculate(index);

        if (p_loop_controller_config->current_controller_config.controller_band_width < FLOAT_EPSILON)
        {
            motor_set_ext_err(index, EXT_FAULT_CODE_CFG_CONTROL_CURRENT_CONTROLLER_BAND_WIDTH);
        }

        switch (p_loop_controller_config->current_controller_config.controller_type)
        {
        case Controller_Type_Parallel_PID:
            Current_Loop_Parallel_PID_Tune(&p_loop_controller_status->id_pid, p_loop_controller_config->current_controller_config.controller_band_width, p_motor_config->Ld, p_motor_config->Rs, current_loop_delay);
            Current_Loop_Parallel_PID_Tune(&p_loop_controller_status->iq_pid, p_loop_controller_config->current_controller_config.controller_band_width, p_motor_config->Lq, p_motor_config->Rs, current_loop_delay);
            break;

        case Controller_Type_Serial_PID:
            Current_Loop_Serial_PID_Tune(&p_loop_controller_status->id_pid, p_loop_controller_config->current_controller_config.controller_band_width, p_motor_config->Ld, p_motor_config->Rs, current_loop_delay);
            Current_Loop_Serial_PID_Tune(&p_loop_controller_status->iq_pid, p_loop_controller_config->current_controller_config.controller_band_width, p_motor_config->Lq, p_motor_config->Rs, current_loop_delay);
            break;

        default:
            motor_set_ext_err(index, EXT_FAULT_CODE_CFG_CONTROL_CURRENT_CONTROLLER_TYPE);
            break;
        }
    }
    else
    {
        motor_set_ext_err(index, EXT_FAULT_CODE_CFG_CONTROL_CURRENT_CONTROLLER_AUTO_TUNE);
    }

    if (p_loop_controller_config->fw_controller_config.controller_type < Controller_Type_Parallel_PID || p_loop_controller_config->fw_controller_config.controller_type >= Controller_Type_MAX)
    {
        motor_set_ext_err(index, EXT_FAULT_CODE_CFG_CONTROL_FW_CONTROLLER_TYPE);
    }

    if (p_loop_controller_config->fw_controller_config.auto_tune == 0)
    {
        memcpy(&p_loop_controller_status->fw_pid.config, &p_loop_controller_config->fw_pid_config, sizeof(PID_Config_t));
    }
    else if (p_loop_controller_config->fw_controller_config.auto_tune == 1)
    {
        if (p_loop_controller_config->fw_controller_config.controller_band_width < FLOAT_EPSILON)
        {
            motor_set_ext_err(index, EXT_FAULT_CODE_CFG_CONTROL_FW_CONTROLLER_BAND_WIDTH);
        }

        if (p_loop_controller_config->fw_controller_config.controller_integral_time < FLOAT_EPSILON)
        {
            motor_set_ext_err(index, EXT_FAULT_CODE_CFG_CONTROL_FW_CONTROLLER_INTEGRAL_TIME);
        }
    }
    else
    {
        motor_set_ext_err(index, EXT_FAULT_CODE_CFG_CONTROL_FW_CONTROLLER_AUTO_TUNE);
    }

    if (p_loop_controller_config->speed_controller_config.controller_type < Controller_Type_Parallel_PID || p_loop_controller_config->speed_controller_config.controller_type >= Controller_Type_MAX)
    {
        motor_set_ext_err(index, EXT_FAULT_CODE_CFG_CONTROL_SPEED_CONTROLLER_TYPE);
    }

    if (p_loop_controller_config->speed_controller_config.auto_tune == 0)
    {
        memcpy(&p_loop_controller_status->speed_pid.config, &p_loop_controller_config->speed_pid_config, sizeof(PID_Config_t));
    }
    else if (p_loop_controller_config->speed_controller_config.auto_tune == 1)
    {
        if (p_loop_controller_config->speed_controller_config.controller_band_width < FLOAT_EPSILON)
        {
            motor_set_ext_err(index, EXT_FAULT_CODE_CFG_CONTROL_SPEED_CONTROLLER_BAND_WIDTH);
        }

        if (p_loop_controller_config->speed_controller_config.controller_integral_time < FLOAT_EPSILON)
        {
            motor_set_ext_err(index, EXT_FAULT_CODE_CFG_CONTROL_SPEED_CONTROLLER_INTEGRAL_TIME);
        }

        switch (p_loop_controller_config->speed_controller_config.controller_type)
        {
        case Controller_Type_Parallel_PID:
            Speed_Loop_Parallel_PID_Tune(&p_loop_controller_status->speed_pid, p_loop_controller_config->speed_controller_config.controller_band_width, MS_TO_S(p_loop_controller_config->speed_controller_config.controller_integral_time), p_motor_config->inertia_load + p_motor_config->inertia_rotor);
            break;

        case Controller_Type_Serial_PID:
            Speed_Loop_Serial_PID_Tune(&p_loop_controller_status->speed_pid, p_loop_controller_config->speed_controller_config.controller_band_width, MS_TO_S(p_loop_controller_config->speed_controller_config.controller_integral_time), p_motor_config->inertia_load + p_motor_config->inertia_rotor);
            break;

        default:
            motor_set_ext_err(index, EXT_FAULT_CODE_CFG_CONTROL_SPEED_CONTROLLER_TYPE);
            break;
        }
    }
    else
    {
        motor_set_ext_err(index, EXT_FAULT_CODE_CFG_CONTROL_SPEED_CONTROLLER_AUTO_TUNE);
    }

    if (p_loop_controller_config->pos_controller_config.controller_type < Controller_Type_Parallel_PID || (p_loop_controller_config->pos_controller_config.controller_type >= Controller_Type_MAX && p_loop_controller_config->pos_controller_config.controller_type != Controller_Type_Impedance))
    {
        motor_set_ext_err(index, EXT_FAULT_CODE_CFG_CONTROL_POS_CONTROLLER_TYPE);
    }

    if (p_loop_controller_config->pos_controller_config.auto_tune == 0)
    {
        memcpy(&p_loop_controller_status->pos_pid.config, &p_loop_controller_config->pos_pid_config, sizeof(PID_Config_t));
    }
    else if (p_loop_controller_config->pos_controller_config.auto_tune == 1)
    {
        if (p_loop_controller_config->pos_controller_config.controller_band_width < FLOAT_EPSILON)
        {
            motor_set_ext_err(index, EXT_FAULT_CODE_CFG_CONTROL_POS_CONTROLLER_BAND_WIDTH);
        }

        switch (p_loop_controller_config->pos_controller_config.controller_type)
        {
        case Controller_Type_Parallel_PID:
        case Controller_Type_Serial_PID:
            Pos_Loop_PID_Tune(&p_loop_controller_status->pos_pid, p_loop_controller_config->pos_controller_config.controller_band_width);
            break;

        case Controller_Type_Impedance:
            // do nothing ,tune by force loop
            break;

        default:
            motor_set_ext_err(index, EXT_FAULT_CODE_CFG_CONTROL_POS_CONTROLLER_TYPE);
            break;
        }
    }
    else
    {
        motor_set_ext_err(index, EXT_FAULT_CODE_CFG_CONTROL_POS_CONTROLLER_AUTO_TUNE);
    }

    if (p_loop_controller_config->force_controller_config.controller_type < Force_Controller_Type_Admittance || p_loop_controller_config->force_controller_config.controller_type >= Force_Controller_Type_MAX)
    {
        motor_set_ext_err(index, EXT_FAULT_CODE_CFG_CONTROL_FORCE_CONTROLLER_TYPE);
    }

    if (p_loop_controller_config->force_controller_config.auto_tune == 0)
    {
        memcpy(&p_loop_controller_status->force_compliance.config, &p_loop_controller_config->compliance_config, sizeof(Compliance_Config_t));
    }
    else if (p_loop_controller_config->force_controller_config.auto_tune == 1)
    {
        if (p_loop_controller_config->force_controller_config.controller_band_width < FLOAT_EPSILON)
        {
            motor_set_ext_err(index, EXT_FAULT_CODE_CFG_CONTROL_FORCE_CONTROLLER_BAND_WIDTH);
        }

        if (p_loop_controller_config->force_controller_config.Md < FLOAT_EPSILON)
        {
            motor_set_ext_err(index, EXT_FAULT_CODE_CFG_CONTROL_FORCE_CONTROLLER_MD);
        }

        switch (p_loop_controller_config->force_controller_config.controller_type)
        {
        case Force_Controller_Type_Impedance:
        case Force_Controller_Type_Admittance:
            Compliance_Controller_Tune(&p_loop_controller_status->force_compliance, p_loop_controller_config->force_controller_config.controller_band_width, p_loop_controller_config->force_controller_config.Md);
            break;

        default:
            motor_set_ext_err(index, EXT_FAULT_CODE_CFG_CONTROL_FORCE_CONTROLLER_TYPE);
            break;
        }
    }
    else
    {
        motor_set_ext_err(index, EXT_FAULT_CODE_CFG_CONTROL_FORCE_CONTROLLER_AUTO_TUNE);
    }
}

/**
 * @brief 排除观测器增益的观测器配置参数更新
 * 
 * @param p_dst 目标结构体
 * @param p_src 源结构体
 */
static inline void neso_config_update_without_gain(NESO_Config_t *p_dst, NESO_Config_t *p_src)
{
    uint32_t offset = offsetof(NESO_Config_t, neso_k_smooth);

    memcpy((uint8_t *)p_dst + offset, (uint8_t *)p_src + offset, sizeof(NESO_Config_t) - offset);
}

/**
 * @brief 观测器参数非实时更新
 * 
 * @param index 电机编号
 */
static inline void observer_config_update_nrt(int index)
{
    Motor_Observer_Config_t *p_motor_observer_config = &global_config.motor_observer_config[index];
    Motor_Observer_t *p_motor_observer               = &global_status.motor_observer[index];

    if (p_motor_observer_config->observser_config.auto_tune == 0)
    {
        switch (p_motor_observer_config->observser_config.observer_type)
        {
        case Observer_Type_None:
            break;

        case Observer_Type_NESO2:
        case Observer_Type_NESO3:
            memcpy(&p_motor_observer->neso_config, &p_motor_observer_config->neso_config, sizeof(NESO_Config_t));

            if (p_motor_observer->neso_config.neso_gain < FLOAT_EPSILON)
            {
                motor_set_ext_err(index, EXT_FAULT_CODE_CFG_CONTROL_NESO_GAIN);
            }
            break;

        default:
            motor_set_ext_err(index, EXT_FAULT_CODE_CFG_CONTROL_OBSERVER_TYPE);
            break;
        }
    }
    else if (p_motor_observer_config->observser_config.auto_tune == 1)
    {
        switch (p_motor_observer_config->observser_config.observer_type)
        {
        case Observer_Type_None:
            break;

        case Observer_Type_NESO2:
        case Observer_Type_NESO3:
            if (p_motor_observer_config->observser_config.observer_band_width < FLOAT_EPSILON)
            {
                motor_set_ext_err(index, EXT_FAULT_CODE_CFG_CONTROL_OBSERVER_BAND_WIDTH);
            }

            neso_config_update_without_gain(&p_motor_observer->neso_config, &p_motor_observer_config->neso_config);
            break;

        default:
            motor_set_ext_err(index, EXT_FAULT_CODE_CFG_CONTROL_OBSERVER_TYPE);
            break;
        }
    }
    else
    {
        motor_set_ext_err(index, EXT_FAULT_CODE_CFG_CONTROL_OBSERVER_AUTO_TUNE);
    }
}

/**
 * @brief 电机非实时更新
 * 
 * @param index 电机编号
 */
void motor_update_nrt(int index)
{
    loop_controller_config_update_nrt(index);
    observer_config_update_nrt(index);
}

/**
 * @brief 电机实时更新
 * 
 * @param index 电机编号
 */
void motor_update_rt(int index)
{
    Motor_Config_t *p_motor_config                       = &global_config.motor_config[index];
    Motor_Controller_Config_t *p_motor_controller_config = &global_config.motor_controller_config[index];
    Motor_Status_t *p_motor_status                       = &global_status.motor_status[index];

    motor_encoder_update_rt(index);
    motor_torque_update_rt(index);
    motor_status_update_rt(index);

    switch (p_motor_status->motor_state)
    {
    case MS_Running:
        observer_update_rt(index);

        switch (p_motor_status->motor_ctrl_mode)
        {
        case MCM_Align:
            align_mode_update_rt(index);
            break;

        case MCM_Current:
            current_mode_update_rt(index);
            break;

        case MCM_Torque:
            torque_mode_update_rt(index);
            break;

        case MCM_Speed:
            speed_mode_update_rt(index);
            break;

        case MCM_Pos:
            pos_mode_update_rt(index);
            break;

        case MCM_Ol_Force:
            ol_force_mode_update_rt(index);
            break;

        case MCM_Cl_Force:
            cl_force_mode_update_rt(index);
            break;

        case MCM_Impedance:
            impedance_mode_update_rt(index);
            break;

        case MCM_Admittance:
            admittance_mode_update_rt(index);
            break;

        case MCM_Teaching:
            teaching_mode_update_rt(index);
            break;

        case MCM_CST:
            cst_mode_update_rt(index);
            break;

        case MCM_CSV:
            csv_mode_update_rt(index);
            break;

        case MCM_CSP:
            csp_mode_update_rt(index);
            break;

        case MCM_VF_Theta:
            vf_theta_mode_update_rt(index);
            break;

        case MCM_VF_Speed:
            vf_speed_mode_update_rt(index);
            break;

        case MCM_IF_Theta:
            if_theta_mode_update_rt(index);
            break;

        case MCM_IF_Speed:
            if_speed_mode_update_rt(index);
            break;

        case MCM_HFI_VF:
            vf_hfi_mode_update_rt(index);
            break;

        case MCM_Current_D_Sweep:
            current_d_sweep_mode_update_rt(index);
            break;

        case MCM_Current_Q_Sweep:
            current_q_sweep_mode_update_rt(index);
            break;

        case MCM_Vel_Sweep:
            vel_sweep_mode_update_rt(index);
            break;

        case MCM_Pos_Sweep:
            pos_sweep_mode_update_rt(index);
            break;

        case MCM_RS_IDENTIFY:
            rs_identify_mode_update_rt(index);
            break;

        case MCM_LS_IDENTIFY:
            ls_identify_mode_update_rt(index);
            break;

        case MCM_PP_IDENTIFY:
            pp_identify_mode_update_rt(index);
            break;

        case MCM_FLUX_IDENTIFY:
            flux_identify_mode_update_rt(index);
            break;

        case MCM_Cogg_IDENTIFY:
            cogg_identify_mode_update_rt(index);
            break;

        case MCM_Inertia_IDENTIFY:
            inertia_identify_mode_update_rt(index);
            break;

        case MCM_Friction_IDENTIFY:
            friction_identify_mode_update_rt(index);
            break;

        case MCM_Gravity_IDENTIFY:
            gravity_identify_mode_update_rt(index);
            break;

        default:
            break;
        }

        switch (p_motor_config->motor_type)
        {
        case Motor_Type_DC:
            dc_update_rt(index);
            break;

        case Motor_Type_BLDC_3_Phase:
            switch (p_motor_controller_config->motor_controller_type)
            {
            case Motor_Controller_Type_FOC:
                foc_update_rt(index);
                break;

            case Motor_Controller_Type_DTC:
                break;

            default:
                break;
            }
            break;

        default:
            break;
        }

        motor_check_err_rt(index);
        break;

    case MS_IDLE:
        motor_check_err_rt(index);
        break;

    case MS_Err:
        motor_check_err_rt(index);
        break;

    default:
        break;
    }
}

/**
 * @brief 电机高频实时更新
 * 
 * @param index 电机编号
 */
void motor_update_frt(int index)
{
    Power_Status_t *p_power_status                       = &global_status.power_status;
    Peripheral_Raw_Status_t *p_peripheral_raw_status     = &global_status.peripheral_raw_status;
    Motor_Config_t *p_motor_config                       = &global_config.motor_config[index];
    Motor_Controller_Config_t *p_motor_controller_config = &global_config.motor_controller_config[index];
    Motor_Basic_Status_t *p_motor_basic_status           = &global_status.motor_basic_status[index];
    Motor_Status_t *p_motor_status                       = &global_status.motor_status[index];

    if (p_motor_config->motor_dir == 1)
    {
        p_motor_basic_status->phase_current.U = (p_peripheral_raw_status->ia_adc_raw[index] - p_motor_basic_status->phase_current_offset.U) * BOARD_PHASE_CURRENT_CONVERT_COFFCIENT;
        p_motor_basic_status->phase_current.V = (p_peripheral_raw_status->ib_adc_raw[index] - p_motor_basic_status->phase_current_offset.V) * BOARD_PHASE_CURRENT_CONVERT_COFFCIENT;
        p_motor_basic_status->phase_current.W = (p_peripheral_raw_status->ic_adc_raw[index] - p_motor_basic_status->phase_current_offset.W) * BOARD_PHASE_CURRENT_CONVERT_COFFCIENT;

#ifndef FAST_CURRENT_LOOP_ENABLE
        p_motor_basic_status->phase_voltage.U = (p_peripheral_raw_status->va_adc_raw[index] - p_motor_basic_status->phase_voltage_offset.U) * BOARD_TERMINAL_VOLTAGE_CONVERT_COFFCIENT;
        p_motor_basic_status->phase_voltage.V = (p_peripheral_raw_status->vb_adc_raw[index] - p_motor_basic_status->phase_voltage_offset.V) * BOARD_TERMINAL_VOLTAGE_CONVERT_COFFCIENT;
        p_motor_basic_status->phase_voltage.W = (p_peripheral_raw_status->vc_adc_raw[index] - p_motor_basic_status->phase_voltage_offset.W) * BOARD_TERMINAL_VOLTAGE_CONVERT_COFFCIENT;
#endif
    }
    else if (p_motor_config->motor_dir == -1) // 电机反向 交换U相和V相采样值
    {
        p_motor_basic_status->phase_current.V = (p_peripheral_raw_status->ia_adc_raw[index] - p_motor_basic_status->phase_current_offset.U) * BOARD_PHASE_CURRENT_CONVERT_COFFCIENT;
        p_motor_basic_status->phase_current.U = (p_peripheral_raw_status->ib_adc_raw[index] - p_motor_basic_status->phase_current_offset.V) * BOARD_PHASE_CURRENT_CONVERT_COFFCIENT;
        p_motor_basic_status->phase_current.W = (p_peripheral_raw_status->ic_adc_raw[index] - p_motor_basic_status->phase_current_offset.W) * BOARD_PHASE_CURRENT_CONVERT_COFFCIENT;

#ifndef FAST_CURRENT_LOOP_ENABLE
        p_motor_basic_status->phase_voltage.V = (p_peripheral_raw_status->va_adc_raw[index] - p_motor_basic_status->phase_voltage_offset.U) * BOARD_TERMINAL_VOLTAGE_CONVERT_COFFCIENT;
        p_motor_basic_status->phase_voltage.U = (p_peripheral_raw_status->vb_adc_raw[index] - p_motor_basic_status->phase_voltage_offset.V) * BOARD_TERMINAL_VOLTAGE_CONVERT_COFFCIENT;
        p_motor_basic_status->phase_voltage.W = (p_peripheral_raw_status->vc_adc_raw[index] - p_motor_basic_status->phase_voltage_offset.W) * BOARD_TERMINAL_VOLTAGE_CONVERT_COFFCIENT;
#endif
    }

    p_motor_basic_status->v_bus     = p_power_status->v_bus;
    p_motor_basic_status->inv_v_bus = p_power_status->inv_v_bus;

    if (p_motor_status->is_calibrated)
    {
        // 相线过流检测
        if (fabsf(p_motor_basic_status->phase_current.U) > p_motor_config->phase_over_current_threshold || fabsf(p_motor_basic_status->phase_current.V) > p_motor_config->phase_over_current_threshold || fabsf(p_motor_basic_status->phase_current.W) > p_motor_config->phase_over_current_threshold)
        {
            if (++p_motor_status->phase_over_current_cnt > 1)
            {
                motor_set_err(index, FAULT_CODE_PHASE_OVER_CURRENT);
            }
        }
        else
        {
            p_motor_status->phase_over_current_cnt = 0;
        }

        if (p_peripheral_raw_status->ia_adc_raw[index] >= BOARD_ADC_RESOLUTION || p_peripheral_raw_status->ia_adc_raw[index] == 0 || p_peripheral_raw_status->ib_adc_raw[index] >= BOARD_ADC_RESOLUTION || p_peripheral_raw_status->ib_adc_raw[index] == 0 || p_peripheral_raw_status->ic_adc_raw[index] >= BOARD_ADC_RESOLUTION || p_peripheral_raw_status->ic_adc_raw[index] == 0)
        {
            motor_set_err(index, FAULT_CODE_BOARD_PHASE_OVER_CURRENT);
        }

        switch (p_motor_config->motor_type)
        {
        case Motor_Type_DC:
            p_motor_basic_status->i_sum = p_motor_basic_status->phase_current.U + p_motor_basic_status->phase_current.V;
#ifndef FAST_CURRENT_LOOP_ENABLE
            p_motor_basic_status->i_rms = sqrtf((SQ(p_motor_basic_status->phase_current.U) + SQ(p_motor_basic_status->phase_current.V)) / 2.0f);
#else
            p_motor_basic_status->i_rms = (SQ(p_motor_basic_status->phase_current.U) + SQ(p_motor_basic_status->phase_current.V)) / 2.0f;
#endif
            break;

        case Motor_Type_BLDC_3_Phase:
            p_motor_basic_status->i_sum = p_motor_basic_status->phase_current.U + p_motor_basic_status->phase_current.V + p_motor_basic_status->phase_current.W;
#ifndef FAST_CURRENT_LOOP_ENABLE
            p_motor_basic_status->i_rms = sqrtf((SQ(p_motor_basic_status->phase_current.U) + SQ(p_motor_basic_status->phase_current.V) + SQ(p_motor_basic_status->phase_current.W)) / 3.0f);
#else
            p_motor_basic_status->i_rms = (SQ(p_motor_basic_status->phase_current.U) + SQ(p_motor_basic_status->phase_current.V) + SQ(p_motor_basic_status->phase_current.W)) / 3.0f;
#endif
            break;

        default:
            break;
        }
    }

    switch (p_motor_status->motor_state)
    {
    case MS_Init:
        if (p_motor_status->motor_cnt == 0)
        {
            motor_pwm_set(index, 0.0f, 0.0f, 0.0f);
            enable_inverter(index);
        }
        if (++p_motor_status->motor_cnt > BOARD_ADC_CALI_CNT)
        {
            p_motor_status->motor_cnt   = 0;
            p_motor_status->motor_state = MS_Calibrate;
        }
        break;

    case MS_Calibrate:
        if (++p_motor_status->motor_cnt < BOARD_ADC_CALI_CNT)
        {
#ifndef FAST_CURRENT_LOOP_ENABLE
            FirstOrder_LPF_Cal(p_peripheral_raw_status->va_adc_raw[index], p_motor_basic_status->phase_voltage_offset.U, BOARD_ADC_CALI_COFFCIENT);
            FirstOrder_LPF_Cal(p_peripheral_raw_status->vb_adc_raw[index], p_motor_basic_status->phase_voltage_offset.V, BOARD_ADC_CALI_COFFCIENT);
            FirstOrder_LPF_Cal(p_peripheral_raw_status->vc_adc_raw[index], p_motor_basic_status->phase_voltage_offset.W, BOARD_ADC_CALI_COFFCIENT);
#endif

            FirstOrder_LPF_Cal(p_peripheral_raw_status->ia_adc_raw[index], p_motor_basic_status->phase_current_offset.U, BOARD_ADC_CALI_COFFCIENT);
            FirstOrder_LPF_Cal(p_peripheral_raw_status->ib_adc_raw[index], p_motor_basic_status->phase_current_offset.V, BOARD_ADC_CALI_COFFCIENT);
            FirstOrder_LPF_Cal(p_peripheral_raw_status->ic_adc_raw[index], p_motor_basic_status->phase_current_offset.W, BOARD_ADC_CALI_COFFCIENT);
        }
        else
        {
            disable_inverter(index);

#ifndef FAST_CURRENT_LOOP_ENABLE
            if (p_motor_basic_status->phase_voltage_offset.U > BOARD_TERMINAL_VOLTAGE_BIAS_ERROR_MAX)
            {
                motor_set_err(index, FAULT_CODE_VOLTAGE_SENSOR_OFFSET_U);
            }
            if (p_motor_basic_status->phase_voltage_offset.V > BOARD_TERMINAL_VOLTAGE_BIAS_ERROR_MAX)
            {
                motor_set_err(index, FAULT_CODE_VOLTAGE_SENSOR_OFFSET_V);
            }
            if (p_motor_basic_status->phase_voltage_offset.W > BOARD_TERMINAL_VOLTAGE_BIAS_ERROR_MAX)
            {
                motor_set_err(index, FAULT_CODE_VOLTAGE_SENSOR_OFFSET_W);
            }
#endif
            if (fabsf(p_motor_basic_status->phase_current_offset.U - BOARD_ADC_RESOLUTION_HALF) > BOARD_PHASE_CURRENT_BIAS_ERROR_MAX)
            {
                motor_set_err(index, FAULT_CODE_CURRENT_SENSOR_OFFSET_U);
            }
            if (fabsf(p_motor_basic_status->phase_current_offset.V - BOARD_ADC_RESOLUTION_HALF) > BOARD_PHASE_CURRENT_BIAS_ERROR_MAX)
            {
                motor_set_err(index, FAULT_CODE_CURRENT_SENSOR_OFFSET_V);
            }
            if (fabsf(p_motor_basic_status->phase_current_offset.W - BOARD_ADC_RESOLUTION_HALF) > BOARD_PHASE_CURRENT_BIAS_ERROR_MAX)
            {
                motor_set_err(index, FAULT_CODE_CURRENT_SENSOR_OFFSET_W);
            }

            if (p_motor_status->motor_state != MS_Err)
            {
                p_motor_status->is_calibrated = 1;
                p_motor_status->motor_state   = MS_IDLE;
            }

            p_motor_status->motor_cnt = 0;
        }
        break;

    case MS_Running:
        switch (p_motor_config->motor_type)
        {
        case Motor_Type_DC:
            dc_update_frt(index);
            break;

        case Motor_Type_BLDC_3_Phase:
            switch (p_motor_controller_config->motor_controller_type)
            {
            case Motor_Controller_Type_FOC:
                foc_update_frt(index);
                break;

            case Motor_Controller_Type_DTC:
                break;

            default:
                break;
            }
            break;

        default:
            break;
        }
        break;

    default:
        break;
    }
}

/**
 * @brief 设置电机三相占空比
 * 
 * @param index 电机编号
 * @param u u相占空比
 * @param v v相占空比
 * @param w w相占空比
 */
void motor_pwm_set(int index, float u, float v, float w)
{
    Motor_Config_t *p_motor_config = &global_config.motor_config[index];

    // 电机反向 交换U相和V相占空比
    if (p_motor_config->motor_dir == -1)
    {
        float temp = u;

        u = v;
        v = temp;
    }

    pwm_duty_set(index, u, v, w);
}

/**
 * @brief 初始化电机控制
 * 
 */
void motor_init(void)
{
    for (int index = 0; index < BOARD_SUPPORTED_MOTOR_NUMBER; index++)
    {
        Motor_Config_t *p_motor_config                       = &global_config.motor_config[index];
        Motor_Controller_Config_t *p_motor_controller_config = &global_config.motor_controller_config[index];
        Loop_Controller_Config_t *p_loop_controller_config   = &global_config.loop_controller_config[index];
        Motor_Status_t *p_motor_status                       = &global_status.motor_status[index];
        Loop_Controller_t *p_loop_controller_status          = &global_status.loop_controller_status[index];

        p_motor_status->thermal_limit = 1.1f;

        motor_config_check(index);
        motor_calculate_torque_limits(index);

        // 电流环初始化
        // 限制最大输出电压
        float max_vlotage               = p_motor_config->rated_voltage * _1_SQRT3 * p_motor_controller_config->max_modulation_ratio;
        PID_Limit_t phase_voltage_limit = {.limit_max = max_vlotage, .limit_min = -max_vlotage};
        float current_loop_delay        = current_loop_delay_calculate(index);

        switch (p_loop_controller_config->current_controller_config.controller_type)
        {
        case Controller_Type_Parallel_PID:
            PID_Init(&p_loop_controller_status->id_pid, BOARD_HRT_DT, &p_loop_controller_config->id_pid_config, &phase_voltage_limit);
            if (p_loop_controller_config->current_controller_config.auto_tune)
            {
                Current_Loop_Parallel_PID_Tune(&p_loop_controller_status->id_pid, p_loop_controller_config->current_controller_config.controller_band_width, p_motor_config->Ld, p_motor_config->Rs, current_loop_delay);
            }

            PID_Init(&p_loop_controller_status->iq_pid, BOARD_HRT_DT, &p_loop_controller_config->iq_pid_config, &phase_voltage_limit);
            if (p_loop_controller_config->current_controller_config.auto_tune)
            {
                Current_Loop_Parallel_PID_Tune(&p_loop_controller_status->iq_pid, p_loop_controller_config->current_controller_config.controller_band_width, p_motor_config->Lq, p_motor_config->Rs, current_loop_delay);
            }
            break;

        case Controller_Type_Serial_PID:
            PID_Init(&p_loop_controller_status->id_pid, BOARD_HRT_DT, &p_loop_controller_config->id_pid_config, &phase_voltage_limit);
            if (p_loop_controller_config->current_controller_config.auto_tune)
            {
                Current_Loop_Serial_PID_Tune(&p_loop_controller_status->id_pid, p_loop_controller_config->current_controller_config.controller_band_width, p_motor_config->Ld, p_motor_config->Rs, current_loop_delay);
            }

            PID_Init(&p_loop_controller_status->iq_pid, BOARD_HRT_DT, &p_loop_controller_config->iq_pid_config, &phase_voltage_limit);
            if (p_loop_controller_config->current_controller_config.auto_tune)
            {
                Current_Loop_Serial_PID_Tune(&p_loop_controller_status->iq_pid, p_loop_controller_config->current_controller_config.controller_band_width, p_motor_config->Lq, p_motor_config->Rs, current_loop_delay);
            }
            break;

        default:
            motor_set_ext_err(index, EXT_FAULT_CODE_CFG_CONTROL_CURRENT_CONTROLLER_TYPE);
            break;
        }

        // 弱磁环初始化
        // 限制最大弱磁电流防止退磁
        float demagnetization_current = p_motor_config->Ld > FLOAT_MIN ? -fabsf(p_motor_config->Flux / p_motor_config->Ld) : -fabsf(p_motor_config->peak_current);
        float peak_fw_current         = -fabsf(p_motor_config->peak_current) * 0.9f;
        float max_fw_current          = MAX(demagnetization_current, peak_fw_current);
        PID_Limit_t fw_current_limit  = {.limit_max = 0, .limit_min = max_fw_current};

        switch (p_loop_controller_config->fw_controller_config.controller_type)
        {
        case Controller_Type_Parallel_PID:
            PID_Init(&p_loop_controller_status->fw_pid, BOARD_HRT_DT, &p_loop_controller_config->fw_pid_config, &fw_current_limit);
            break;

        case Controller_Type_Serial_PID:
            PID_Init(&p_loop_controller_status->fw_pid, BOARD_HRT_DT, &p_loop_controller_config->fw_pid_config, &fw_current_limit);
            break;

        default:
            motor_set_ext_err(index, EXT_FAULT_CODE_CFG_CONTROL_FW_CONTROLLER_TYPE);
            break;
        }

        // 速度环初始化
        // 限制最大扭矩
        float max_torque         = p_motor_status->peak_torque;
        PID_Limit_t torque_limit = {.limit_max = max_torque, .limit_min = -max_torque};

        p_motor_status->positive_tor_limit = max_torque;
        p_motor_status->negative_tor_limit = -max_torque;

        switch (p_loop_controller_config->speed_controller_config.controller_type)
        {
        case Controller_Type_Parallel_PID:
            PID_Init(&p_loop_controller_status->speed_pid, BOARD_RT_DT, &p_loop_controller_config->speed_pid_config, &torque_limit);
            if (p_loop_controller_config->speed_controller_config.auto_tune)
            {
                Speed_Loop_Parallel_PID_Tune(&p_loop_controller_status->speed_pid, p_loop_controller_config->speed_controller_config.controller_band_width, MS_TO_S(p_loop_controller_config->speed_controller_config.controller_integral_time), p_motor_config->inertia_load + p_motor_config->inertia_rotor);
            }
            break;

        case Controller_Type_Serial_PID:
            PID_Init(&p_loop_controller_status->speed_pid, BOARD_RT_DT, &p_loop_controller_config->speed_pid_config, &torque_limit);
            if (p_loop_controller_config->speed_controller_config.auto_tune)
            {
                Speed_Loop_Serial_PID_Tune(&p_loop_controller_status->speed_pid, p_loop_controller_config->speed_controller_config.controller_band_width, MS_TO_S(p_loop_controller_config->speed_controller_config.controller_integral_time), p_motor_config->inertia_load + p_motor_config->inertia_rotor);
            }
            break;

        default:
            motor_set_ext_err(index, EXT_FAULT_CODE_CFG_CONTROL_SPEED_CONTROLLER_TYPE);
            break;
        }

        // 观测器初始化
        observer_init(index);

        // 位置环初始化
        // 限制最大速度
        float peak_vel          = cfg_peak_speed_to_motor_speed(index);
        PID_Limit_t speed_limit = {.limit_max = peak_vel, .limit_min = -peak_vel};

        // 力控环初始化
        Compliance_Limit_t limit =
            {
                .max_position     = 10,
                .min_position     = -10,
                .max_velocity     = 0.1,
                .max_acceleration = 1,
                .max_force        = 0,
                .max_pos_err      = 1.0,
                .max_vel_err      = 1.0,
                .max_acc_err      = 1.0,
            };

        switch (p_loop_controller_config->pos_controller_config.controller_type)
        {
        case Controller_Type_Parallel_PID:
        case Controller_Type_Serial_PID:
        case Controller_Type_Impedance: // 阻抗控制也初始化PID防止由阻抗控制切换回PID时出现错误
            PID_Init(&p_loop_controller_status->pos_pid, BOARD_RT_DT, &p_loop_controller_config->pos_pid_config, &speed_limit);
            if (p_loop_controller_config->pos_controller_config.auto_tune)
            {
                Pos_Loop_PID_Tune(&p_loop_controller_status->pos_pid, p_loop_controller_config->pos_controller_config.controller_band_width);
            }
            break;

        default:
            motor_set_ext_err(index, EXT_FAULT_CODE_CFG_CONTROL_POS_CONTROLLER_TYPE);
            break;
        }

        switch (p_loop_controller_config->force_controller_config.controller_type)
        {
        case Force_Controller_Type_Impedance:
        case Force_Controller_Type_Admittance:
            Compliance_Controller_Init(&p_loop_controller_status->force_compliance, BOARD_RT_DT, &p_loop_controller_config->compliance_config, &limit);
            if (p_loop_controller_config->force_controller_config.auto_tune)
            {
                Compliance_Controller_Tune(&p_loop_controller_status->force_compliance, p_loop_controller_config->force_controller_config.controller_band_width, p_loop_controller_config->force_controller_config.Md);
            }
            break;

        default:
            motor_set_ext_err(index, EXT_FAULT_CODE_CFG_CONTROL_FORCE_CONTROLLER_TYPE);
            break;
        }
        motor_filter_init(index);
    }
}

/**
 * @brief 复位电机控制器
 * 
 * @param index 电机编号
 */
void motor_controller_reset(int index)
{
    Motor_Config_t *p_motor_config                       = &global_config.motor_config[index];
    Motor_Controller_Config_t *p_motor_controller_config = &global_config.motor_controller_config[index];

    switch (p_motor_config->motor_type)
    {
    case Motor_Type_DC:
        dc_controller_reset(index);
        break;

    case Motor_Type_BLDC_3_Phase:
        switch (p_motor_controller_config->motor_controller_type)
        {
        case Motor_Controller_Type_FOC:
            foc_controller_reset(index);
            break;

        case Motor_Controller_Type_DTC:
            break;

        default:
            break;
        }
        break;

    default:
        break;
    }
}

/**
 * @brief 复位电机控制
 * 
 * @param index 电机编号
 */
void motor_reset(int index)
{
    Motor_Config_t *p_motor_config                     = &global_config.motor_config[index];
    Loop_Controller_Config_t *p_loop_controller_config = &global_config.loop_controller_config[index];
    Motor_Status_t *p_motor_status                     = &global_status.motor_status[index];
    Loop_Controller_t *p_loop_controller_status        = &global_status.loop_controller_status[index];
    Motion_Planner_t *p_motion_planner                 = &motion_planner[index];

    if (!p_motor_status->is_calibrated)
    {
        p_motor_status->motor_state = MS_Init;
    }
    else
    {
        p_motor_status->motor_state = MS_IDLE;
    }

    p_motor_status->motor_ctrl_mode = MCM_None;

    motor_config_check(index);
    motor_calculate_torque_limits(index);
    motor_controller_reset(index);
    observer_reset(index);

    p_motor_status->current_ref      = 0;
    p_motor_status->current_ref_f    = 0;
    p_motor_status->current_fb_f     = p_motor_status->current_fb;
    p_motor_status->torque_ref       = 0;
    p_motor_status->torque_ref_f     = 0;
    p_motor_status->torque_fb_f      = p_motor_status->torque_fb;
    p_motor_status->acc_ref          = 0;
    p_motor_status->vel_ref          = 0;
    p_motor_status->vel_ref_f        = 0;
    p_motor_status->vel_out          = 0;
    p_motor_status->vel_out_f        = 0;
    p_motor_status->pos_ref          = 0;
    p_motor_status->pos_ref_f        = 0;
    p_motor_status->force_ref        = 0;
    p_motor_status->force_fb_f       = p_motor_status->force_fb;
    p_motor_status->follow_err_tick  = 0;
    p_motor_status->vel_monitor_mode = Vel_Monitor_Disabled;
    memset(p_motion_planner, 0, sizeof(Motion_Planner_t));

    if (p_loop_controller_config->current_controller_config.auto_tune)
    {
        float current_loop_delay = current_loop_delay_calculate(index);

        switch (p_loop_controller_config->current_controller_config.controller_type)
        {
        case Controller_Type_Parallel_PID:
            Current_Loop_Parallel_PID_Tune(&p_loop_controller_status->id_pid, p_loop_controller_config->current_controller_config.controller_band_width, p_motor_config->Ld, p_motor_config->Rs, current_loop_delay);
            Current_Loop_Parallel_PID_Tune(&p_loop_controller_status->iq_pid, p_loop_controller_config->current_controller_config.controller_band_width, p_motor_config->Lq, p_motor_config->Rs, current_loop_delay);
            break;

        case Controller_Type_Serial_PID:
            Current_Loop_Serial_PID_Tune(&p_loop_controller_status->id_pid, p_loop_controller_config->current_controller_config.controller_band_width, p_motor_config->Ld, p_motor_config->Rs, current_loop_delay);
            Current_Loop_Serial_PID_Tune(&p_loop_controller_status->iq_pid, p_loop_controller_config->current_controller_config.controller_band_width, p_motor_config->Lq, p_motor_config->Rs, current_loop_delay);
            break;

        default:
            break;
        }
    }

    if (p_loop_controller_config->speed_controller_config.auto_tune)
    {
        switch (p_loop_controller_config->speed_controller_config.controller_type)
        {
        case Controller_Type_Parallel_PID:
            Speed_Loop_Parallel_PID_Tune(&p_loop_controller_status->speed_pid, p_loop_controller_config->speed_controller_config.controller_band_width, MS_TO_S(p_loop_controller_config->speed_controller_config.controller_integral_time), p_motor_config->inertia_load + p_motor_config->inertia_rotor);
            break;

        case Controller_Type_Serial_PID:
            Speed_Loop_Serial_PID_Tune(&p_loop_controller_status->speed_pid, p_loop_controller_config->speed_controller_config.controller_band_width, MS_TO_S(p_loop_controller_config->speed_controller_config.controller_integral_time), p_motor_config->inertia_load + p_motor_config->inertia_rotor);
            break;

        default:
            motor_set_ext_err(index, EXT_FAULT_CODE_CFG_CONTROL_SPEED_CONTROLLER_TYPE);
            break;
        }
    }

    if (p_loop_controller_config->pos_controller_config.auto_tune)
    {
        switch (p_loop_controller_config->pos_controller_config.controller_type)
        {
        case Controller_Type_Parallel_PID:
        case Controller_Type_Serial_PID:
            Pos_Loop_PID_Tune(&p_loop_controller_status->pos_pid, p_loop_controller_config->pos_controller_config.controller_band_width);
            break;

        case Controller_Type_Impedance:
            // do nothing ,init by force loop
            break;

        default:
            motor_set_ext_err(index, EXT_FAULT_CODE_CFG_CONTROL_POS_CONTROLLER_TYPE);
            break;
        }
    }

    Compliance_Controller_Reset(&p_loop_controller_status->force_compliance);
    if (p_loop_controller_config->force_controller_config.auto_tune)
    {
        switch (p_loop_controller_config->force_controller_config.controller_type)
        {
        case Force_Controller_Type_Impedance:
        case Force_Controller_Type_Admittance:
            Compliance_Controller_Tune(&p_loop_controller_status->force_compliance, p_loop_controller_config->force_controller_config.controller_band_width, p_loop_controller_config->force_controller_config.Md);
            break;

        default:
            motor_set_ext_err(index, EXT_FAULT_CODE_CFG_CONTROL_FORCE_CONTROLLER_TYPE);
            break;
        }
    }

    motor_reset_current_control(index);
    motor_reset_speed_control(index);
    motor_reset_torque_limit(index);
    motor_reset_pos_control(index);

    motor_filter_init(index);
}

/**
 * @brief 使能电机
 * 
 * @param index 电机编号
 * @return true 成功
 * @return false 失败
 */
bool motor_enable(int index)
{
    Motor_Status_t *p_motor_status = &global_status.motor_status[index];

    if (!p_motor_status->is_calibrated || p_motor_status->is_error || p_motor_status->is_braked)
    {
        return false;
    }

    motor_pwm_set(index, 0.5f, 0.5f, 0.5f);
    enable_inverter(index);
    p_motor_status->is_enabled      = 1;
    p_motor_status->motor_state     = MS_Running;
    p_motor_status->motor_ctrl_mode = MCM_None;

    return true;
}

/**
 * @brief 失能电机
 * 
 * @param index 电机编号
 * @return true 成功
 * @return false 失败
 */
bool motor_disable(int index)
{
    Motor_Status_t *p_motor_status = &global_status.motor_status[index];

    disable_inverter(index);
    p_motor_status->is_enabled      = 0;
    p_motor_status->motor_state     = MS_IDLE;
    p_motor_status->motor_ctrl_mode = MCM_None;

    return true;
}

/**
 * @brief 设置电机错误状态
 * 
 * @param index 电机编号
 * @param code 错误代码
 */
void motor_set_err(int index, uint16_t code)
{
    Power_Status_t *p_power_status             = &global_status.power_status;
    Motor_Basic_Status_t *p_motor_basic_status = &global_status.motor_basic_status[index];
    Motor_Status_t *p_motor_status             = &global_status.motor_status[index];

    if (p_motor_status->motor_state == MS_Err)
    {
        return;
    }
    disable_inverter(index);
    p_motor_status->is_enabled  = 0;
    p_motor_status->is_error    = 1;
    p_motor_status->motor_state = MS_Err;
    p_motor_status->err_code    = code;
}

/**
 * @brief 设置电机扩展错误状态
 * 
 * @param index 电机编号
 * @param ext_code 扩展错误代码
 */
void motor_set_ext_err(int index, uint32_t ext_code)
{
    Power_Status_t *p_power_status             = &global_status.power_status;
    Motor_Basic_Status_t *p_motor_basic_status = &global_status.motor_basic_status[index];
    Motor_Status_t *p_motor_status             = &global_status.motor_status[index];

    if (p_motor_status->motor_state == MS_Err)
    {
        return;
    }
    disable_inverter(index);
    p_motor_status->is_enabled   = 0;
    p_motor_status->is_error     = 1;
    p_motor_status->motor_state  = MS_Err;
    p_motor_status->err_code     = FAULT_CODE_EXTEND;
    p_motor_status->ext_err_code = ext_code;
}

/**
 * @brief 设置电机警告状态
 * 
 * @param index 电机编号
 * @param warn_code 警告代码
 */
void motor_set_warning(int index, uint32_t warn_code)
{
    Power_Status_t *p_power_status             = &global_status.power_status;
    Motor_Basic_Status_t *p_motor_basic_status = &global_status.motor_basic_status[index];
    Motor_Status_t *p_motor_status             = &global_status.motor_status[index];

    p_motor_status->ext_warn_code = warn_code;
}

/**
 * @brief 复位电机故障
 * 
 * @param index 电机编号
 */
void motor_reset_err(int index)
{
    Motor_Status_t *p_motor_status = &global_status.motor_status[index];

    p_motor_status->is_enabled = 0;
    p_motor_status->is_error   = 0;
    if (!p_motor_status->is_calibrated)
    {
        p_motor_status->motor_state = MS_Init;
    }
    else
    {
        p_motor_status->motor_state = MS_IDLE;
    }
    p_motor_status->err_code      = FAULT_CODE_NONE;
    p_motor_status->ext_err_code  = EXT_FAULT_CODE_NONE;
    p_motor_status->ext_warn_code = WARN_CODE_NONE;

    encoder_set_reset_err(index);
}

/**
 * @brief 刷新电机位置信息
 * 
 * @param index 电机编号
 */
void motor_refresh_pos(int index)
{
    motor_encoder_update_rt(index);
}


/**
 * @brief 设置电机编码器偏移
 * 
 * @param index 电机编号
 * @param offset 编码器偏移
 */
void motor_set_encoder_offset(int index, float offset)
{
    Motor_Config_t *p_motor_config                       = &global_config.motor_config[index];
    Motor_Controller_Config_t *p_motor_controller_config = &global_config.motor_controller_config[index];

    switch (p_motor_config->motor_type)
    {
    case Motor_Type_BLDC_3_Phase:
        switch (p_motor_controller_config->motor_controller_type)
        {
        case Motor_Controller_Type_FOC:
            foc_set_encoder_offset(index, offset);
            break;

        case Motor_Controller_Type_DTC:
            break;

        default:
            break;
        }
        break;

    default:
        break;
    }
}

/**
 * @brief 同步电机编码器偏移
 * 
 * @param index 电机编号
 * @param offset 编码器偏移
 */
void motor_sync_encoder_offset(int index, float offset)
{
    Motor_Config_t *p_motor_config                       = &global_config.motor_config[index];
    Motor_Controller_Config_t *p_motor_controller_config = &global_config.motor_controller_config[index];
    Encoder_Status_t *p_encoder_status                   = &global_status.encoder_status[index];

    switch (p_motor_config->motor_type)
    {
    case Motor_Type_BLDC_3_Phase:
        switch (p_motor_controller_config->motor_controller_type)
        {
        case Motor_Controller_Type_FOC: {
            float theta_offset = Normalize_Rad((p_encoder_status->encoder_pos_fractional - p_encoder_status->encoder_pos_fractional_second) * _2PI + offset);
            foc_set_encoder_offset(index, theta_offset);
        }
        break;

        case Motor_Controller_Type_DTC:
            break;

        default:
            break;
        }
        break;

    default:
        break;
    }
}
