/**
 * @file dc.c
 * @author jinchl (jcl709905683@gmail.com)
 * @brief 
 * @version 0.1
 * @date 2026-03-30
 * 
 * @copyright Copyright (c) 2026
 * 
 */


#include "foc.h"
#include "board.h"
#include "foc_utils.h"
#include "global.h"
#include "../drivers/multicore/ipc.h"
#include "../drivers/pwm/pwm_driver.h"
#include "motor.h"
#include "motor_api.h"
#include "encoder.h"
#include "hpm_rng_drv.h"

/**
 * @brief dc编码器更新
 * 
 * @param index 电机编号
 */
static void dc_encoder_update(int index)
{
    Motor_Config_t *p_motor_config     = &global_config_in_core1.motor_config[index];
    FOC_Status_t *p_foc_status         = &global_status.foc_status[index];
    Encoder_Status_t *p_encoder_status = &global_status.encoder_status[index];

    switch (p_motor_config->motor_struct)
    {
    case Motor_Struct_Linear:
        p_foc_status->we_encoder = p_encoder_status->encoder_speed / 1e3f;
        break;

    case Motor_Struct_Rotary:
        p_foc_status->we_encoder = p_encoder_status->encoder_speed * _2PI;
        break;

    default:
        break;
    }
}

/**
 * @brief dc电压电流更新
 * 
 * @param index 电机编号
 */
static void dc_u_i_update(int index)
{
    Motor_Controller_Config_t *p_motor_controller_config = &global_config_in_core1.motor_controller_config[index];
    Motor_Basic_Status_t *p_motor_basic_status           = &global_status.motor_basic_status[index];
    FOC_Status_t *p_foc_status                           = &global_status.foc_status[index];

    memcpy(&p_foc_status->u_dq_last, &p_foc_status->u_dq, sizeof(DQ_Axis_t));
    memcpy(&p_foc_status->i_dq_last, &p_foc_status->i_dq, sizeof(DQ_Axis_t));

    p_foc_status->u_dq.D = 0;
    p_foc_status->u_dq.Q = p_motor_basic_status->phase_voltage.U - p_motor_basic_status->phase_voltage.V;
    p_foc_status->i_dq.D = 0;
    p_foc_status->i_dq.Q = (p_motor_basic_status->phase_current.U - p_motor_basic_status->phase_current.V) / 2.0f;

    /*
        us_max 为能够调制出的最大电压矢量
        uo_max 为在当前调制比下能够调制出的最大电压矢量
    */
    p_foc_status->us_max = p_motor_basic_status->v_bus;
    p_foc_status->uo_max = p_motor_controller_config->max_modulation_ratio * p_foc_status->us_max;
}

/**
 * @brief dc电压开环模式更新
 * 
 * @param index 电机编号
 */
static void dc_vf_update(int index)
{
    FOC_Status_t *p_foc_status = &global_status.foc_status[index];

    p_foc_status->u_dq_ref.D = 0;
    p_foc_status->u_dq_ref.Q = CONSTRAIN(p_foc_status->u_dq_ref.Q, -p_foc_status->uo_max, p_foc_status->uo_max);
    memcpy(&p_foc_status->u_dq_out, &p_foc_status->u_dq_ref, sizeof(DQ_Axis_t));
}

/**
 * @brief dc电流环前馈解耦更新
 * 
 * @param index 电机编号
 */
static void dc_decoupling_update(int index)
{
    Motor_Config_t *p_motor_config                       = &global_config_in_core1.motor_config[index];
    Motor_Controller_Config_t *p_motor_controller_config = &global_config_in_core1.motor_controller_config[index];
    FOC_Status_t *p_foc_status                           = &global_status.foc_status[index];

    if (p_motor_controller_config->current_decoupling_type > Current_Decoupling_Type_None)
    {
        p_foc_status->i_dq_delta.D   = 0;
        p_foc_status->i_dq_delta.Q   = p_foc_status->i_dq.Q - p_foc_status->i_dq_last.Q;
        p_foc_status->i_dq_delta_f.D = 0;
        FirstOrder_LPF_Cal(p_foc_status->i_dq_delta.Q, p_foc_status->i_dq_delta_f.Q, 0.1f);
        p_foc_status->u_dq_rl.D    = 0;
        p_foc_status->u_dq_rl.Q    = p_foc_status->i_dq.Q * p_motor_config->Rs + p_motor_config->Lq * p_foc_status->i_dq_delta_f.Q / BOARD_HRT_DT;
        p_foc_status->u_dq_cross.D = 0;
        p_foc_status->u_dq_cross.Q = 0;
        p_foc_status->u_bemf       = p_foc_status->we * p_motor_config->ke;

        switch (p_motor_controller_config->current_decoupling_type)
        {
        case Current_Decoupling_Type_RL:
            p_foc_status->u_dq_feed_forward.D = 0;
            p_foc_status->u_dq_feed_forward.Q = p_foc_status->u_dq_rl.Q;
            break;

        case Current_Decoupling_Type_Bemf:
            p_foc_status->u_dq_feed_forward.D = 0;
            p_foc_status->u_dq_feed_forward.Q = p_foc_status->u_bemf;
            break;

        case Current_Decoupling_Type_RL_Bemf:
            p_foc_status->u_dq_feed_forward.D = 0;
            p_foc_status->u_dq_feed_forward.Q = p_foc_status->u_dq_rl.Q + p_foc_status->u_bemf;
            break;

        default:
            p_foc_status->u_dq_feed_forward.D = 0;
            p_foc_status->u_dq_feed_forward.Q = 0;
            break;
        }
    }
    else
    {
        p_foc_status->u_dq_feed_forward.D = 0;
        p_foc_status->u_dq_feed_forward.Q = 0;
    }
}

/**
 * @brief dc电流环更新
 * 
 * @param index 电机编号
 */
static void dc_current_loop_update(int index)
{
    Loop_Controller_Config_t *p_loop_controller_config = &global_config_in_core1.loop_controller_config[index];
    Loop_Controller_t *p_loop_controller_status        = &global_status.loop_controller_status[index];
    FOC_Status_t *p_foc_status                         = &global_status.foc_status[index];

    float iq_err = p_foc_status->i_dq_ref.Q - p_foc_status->i_dq.Q;

    if (FOC_Current_Follow_Err_Exceeded(p_foc_status->i_dq_ref.Q, iq_err))
    {
        if (++p_foc_status->current_err_cnt > MS_TO_HRT_TICKS(CURRENT_ERR_MS))
        {
            motor_set_err(index, FAULT_CODE_CURRENT_FOLLOW_ERR);
        }
    }
    else
    {
        p_foc_status->current_err_cnt = 0;
    }

    PID_Set_Abs_Limit(&p_loop_controller_status->iq_pid, p_foc_status->uo_max);

    switch (p_loop_controller_config->current_controller_config.controller_type)
    {
    case Controller_Type_Parallel_PID:
        p_foc_status->u_dq_out_raw.Q = Parallel_PI_Update(&p_loop_controller_status->iq_pid, p_foc_status->u_dq_feed_forward.Q, iq_err);
        break;

    case Controller_Type_Serial_PID:
        p_foc_status->u_dq_out_raw.Q = Serial_PI_Update(&p_loop_controller_status->iq_pid, p_foc_status->u_dq_feed_forward.Q, iq_err);
        break;

    default:
        break;
    }

    if (p_loop_controller_status->iq_pid.status.saturation)
    {
        motor_set_warning(index, WARN_CODE_CURRENT_LOOP_SATURATION);
    }
}

/**
 * @brief dc电压极限圆限制更新
 * 
 * @param index 电机编号
 */
static void dc_limit_circle_update(int index)
{
    FOC_Status_t *p_foc_status = &global_status.foc_status[index];

    memcpy(&p_foc_status->u_dq_out, &p_foc_status->u_dq_out_raw, sizeof(DQ_Axis_t));
    p_foc_status->us_out_raw = p_foc_status->u_dq_out_raw.Q;
    FirstOrder_LPF_Cal(p_foc_status->us_out_raw, p_foc_status->us_out_raw_f, 0.1f);
    p_foc_status->us_out_rate_raw = p_foc_status->us_out_raw / p_foc_status->uo_max;

    p_foc_status->u_dq_out.Q = CONSTRAIN(p_foc_status->u_dq_out.Q, -p_foc_status->uo_max, p_foc_status->uo_max);
    p_foc_status->us_out     = p_foc_status->u_dq_out.Q;
    FirstOrder_LPF_Cal(p_foc_status->us_out, p_foc_status->us_out_f, 0.1f);
    p_foc_status->us_out_rate = p_foc_status->us_out_f / p_foc_status->uo_max;
}

/**
 * @brief dc功率参数更新
 * 
 * @param index 电机编号
 */
static void dc_power_update(int index)
{
    Motor_Config_t *p_motor_config             = &global_config_in_core1.motor_config[index];
    Motor_Basic_Status_t *p_motor_basic_status = &global_status.motor_basic_status[index];
    FOC_Status_t *p_foc_status                 = &global_status.foc_status[index];

    p_foc_status->us      = p_foc_status->u_dq.Q;
    p_foc_status->us_duty = p_foc_status->us / p_foc_status->uo_max;
    p_foc_status->is      = p_foc_status->i_dq.Q;
    p_foc_status->is_duty = p_foc_status->is / p_motor_config->peak_current;

    p_foc_status->current_fb = p_foc_status->i_dq.Q;
    p_foc_status->torque_fb  = p_motor_config->kt * p_foc_status->i_dq.Q;

    p_motor_basic_status->e_power = p_foc_status->u_dq.Q * p_foc_status->i_dq.Q;
    p_motor_basic_status->i_bus   = p_motor_basic_status->e_power * p_motor_basic_status->inv_v_bus;
}

/**
 * @brief dc pwm更新
 * 
 * @param index 电机编号
 * @param dt_comp_enable 死区补偿使能
 */
static void dc_pwm_update(int index, int dt_comp_enable)
{
    Motor_Basic_Status_t *p_motor_basic_status = &global_status.motor_basic_status[index];
    FOC_Status_t *p_foc_status                 = &global_status.foc_status[index];

    float half_uq_rate = p_foc_status->u_dq_out.Q / p_foc_status->us_max * 0.5f;

    p_motor_basic_status->pwm_duty_raw.U = 0.5f + half_uq_rate;
    p_motor_basic_status->pwm_duty_raw.V = 0.5f - half_uq_rate;
    p_motor_basic_status->pwm_duty_raw.W = 0.0f;

    if (dt_comp_enable)
    {
        // 电流角度和扇区仅用于诊断显示，不参与死区补偿。
        p_foc_status->dt_compensation_value = PWM_DEAD_ZONE_COMPENSATION_DUTY;

        PWM_Dead_Zone_Compensation(&p_motor_basic_status->phase_current,
                                   p_foc_status->dt_compensation_value,
                                   PWM_DEAD_ZONE_TRANSITION_CURRENT,
                                   &p_motor_basic_status->pwm_duty_raw,
                                   &p_motor_basic_status->pwm_duty_comp,
                                   &p_motor_basic_status->pwm_duty_out);

        // DC H桥仅使用U、V两相。
        p_motor_basic_status->pwm_duty_comp.W = 0.0f;
        p_motor_basic_status->pwm_duty_out.W  = 0.0f;
    }
    else
    {
        p_foc_status->dt_compensation_value = 0.0f;
        memset(&p_motor_basic_status->pwm_duty_comp, 0, sizeof(UVW_Axis_t));
        memcpy(&p_motor_basic_status->pwm_duty_out, &p_motor_basic_status->pwm_duty_raw, sizeof(UVW_Axis_t));
    }

    p_motor_basic_status->phase_voltage_out.U = p_motor_basic_status->pwm_duty_raw.U * p_motor_basic_status->v_bus;
    p_motor_basic_status->phase_voltage_out.V = p_motor_basic_status->pwm_duty_raw.V * p_motor_basic_status->v_bus;
    p_motor_basic_status->phase_voltage_out.W = p_motor_basic_status->pwm_duty_raw.W * p_motor_basic_status->v_bus;

    motor_pwm_set(index, p_motor_basic_status->pwm_duty_out.U, p_motor_basic_status->pwm_duty_out.V, p_motor_basic_status->pwm_duty_out.W);
}

/**
 * @brief dc慢速寻相实时更新
 * 
 * @param index 电机编号
 */
static inline void dc_align_slow_update_rt(int index)
{
    Motor_Config_t *p_motor_config                       = &global_config_in_core1.motor_config[index];
    Motor_Controller_Config_t *p_motor_controller_config = &global_config_in_core1.motor_controller_config[index];
    Motor_Status_t *p_motor_status                       = &global_status.motor_status[index];
    FOC_Status_t *p_foc_status                           = &global_status.foc_status[index];
    Motor_Basic_Status_t *p_motor_basic_status           = &global_status.motor_basic_status[index];

    p_foc_status->us_max = p_motor_basic_status->v_bus;
    p_foc_status->uo_max = p_motor_controller_config->max_modulation_ratio * p_foc_status->us_max;

    float pos_err_threshold = p_motor_controller_config->align_threshold;
    float delta_pos         = 0;
    float abs_delta_pos     = 0;

    float force  = CONSTRAIN(p_motor_controller_config->align_force, 0.05f, 1.0f);
    float ud_ref = force * p_foc_status->uo_max;

    switch (p_foc_status->align_state)
    {
    case 0: // 进入电压开环控制
        p_foc_status->run_state              = FOC_VF_OpenLoop;
        p_foc_status->u_dq_ref.D             = 0;
        p_foc_status->u_dq_ref.Q             = 0;
        p_foc_status->align_tick             = 0;
        p_foc_status->align_state            = 1;
        p_foc_status->align_initial          = 0;
        p_foc_status->align_offset_candidate = 0;
        p_foc_status->align_dir_detected     = 0;
        break;

    case 1: // 等待位置稳定并设置初始位置反馈
        if (++p_foc_status->align_tick > MS_TO_RT_TICKS(p_motor_controller_config->align_time / 2))
        {
            p_foc_status->align_tick    = 0;
            p_foc_status->align_state   = 2;
            p_foc_status->align_initial = p_motor_status->pos_fb;
        }
        break;

    case 2: // 正向增加电压并检查位置反馈
        if (p_foc_status->u_dq_ref.Q < ud_ref)
        {
            p_foc_status->u_dq_ref.Q += ud_ref / MS_TO_S((float)p_motor_controller_config->align_time) * BOARD_RT_DT;
        }

        delta_pos     = p_motor_status->pos_fb - p_foc_status->align_initial;
        abs_delta_pos = fabsf(delta_pos);
        if (abs_delta_pos > pos_err_threshold)
        {
            // 第一次位移判断：记录运动方向（正向驱动下delta_pos应为正）
            if (p_foc_status->align_dir_detected == 0)
            {
                p_foc_status->align_dir_detected = (delta_pos > 0) ? 1 : -1;
            }

            p_foc_status->align_tick  = 0;
            p_foc_status->align_state = 3;
            p_foc_status->u_dq_ref.Q  = 0;
        }
        else if (p_foc_status->u_dq_ref.Q >= ud_ref)
        {
            p_foc_status->align_tick++;
        }

        if (p_foc_status->align_tick > MS_TO_RT_TICKS(20))
        {
            p_foc_status->align_tick  = 0;
            p_foc_status->align_state = 5;
            p_foc_status->u_dq_ref.Q  = 0;
        }
        break;

    case 3: // 等待位置稳定
        if (++p_foc_status->align_tick > MS_TO_RT_TICKS(p_motor_controller_config->align_time))
        {
            p_foc_status->align_tick    = 0;
            p_foc_status->align_state   = 4;
            p_foc_status->align_initial = p_motor_status->pos_fb;
        }
        break;

    case 4: // 正向增加电压并检查位置反馈
        if (p_foc_status->u_dq_ref.Q < ud_ref)
        {
            p_foc_status->u_dq_ref.Q += ud_ref / MS_TO_S((float)p_motor_controller_config->align_time) * BOARD_RT_DT;
        }

        delta_pos     = p_motor_status->pos_fb - p_foc_status->align_initial;
        abs_delta_pos = fabsf(delta_pos);
        if (abs_delta_pos > pos_err_threshold)
        {
            // 第二次位移判断：与第一次结果对比，一致时确认方向检测有效
            int dir_step = (delta_pos > 0) ? 1 : -1;

            if (p_foc_status->align_dir_detected == dir_step)
            {
                p_foc_status->align_dir_detected = dir_step * 2; // 两次一致，确认: 2=正确, -2=需修正
                p_foc_status->align_tick         = 0;
                p_foc_status->align_state        = 9;
                p_foc_status->u_dq_ref.Q         = 0;
            }
            else
            {
                p_foc_status->align_dir_detected = 0; // 两次不一致，放弃修正
                p_foc_status->align_tick         = 0;
                p_foc_status->align_state        = 1;
                p_foc_status->u_dq_ref.Q         = 0;
            }
        }
        else if (p_foc_status->u_dq_ref.Q >= ud_ref)
        {
            p_foc_status->align_tick++;
        }

        if (p_foc_status->align_tick > MS_TO_RT_TICKS(20))
        {
            p_foc_status->align_tick  = 0;
            p_foc_status->align_state = 1;
            p_foc_status->u_dq_ref.Q  = 0;
        }
        break;

    case 5: // 等待位置稳定
        if (++p_foc_status->align_tick > MS_TO_RT_TICKS(p_motor_controller_config->align_time))
        {
            p_foc_status->align_tick    = 0;
            p_foc_status->align_state   = 6;
            p_foc_status->align_initial = p_motor_status->pos_fb;
        }
        break;

    case 6: // 反向增加电压并检查位置反馈
        if (p_foc_status->u_dq_ref.Q > -ud_ref)
        {
            p_foc_status->u_dq_ref.Q -= ud_ref / MS_TO_S((float)p_motor_controller_config->align_time) * BOARD_RT_DT;
        }

        delta_pos     = p_motor_status->pos_fb - p_foc_status->align_initial;
        abs_delta_pos = fabsf(delta_pos);
        if (abs_delta_pos > pos_err_threshold)
        {
            // 第一次位移判断：记录运动方向（正向驱动下delta_pos应为正）
            if (p_foc_status->align_dir_detected == 0)
            {
                p_foc_status->align_dir_detected = (delta_pos > 0) ? -1 : 1;
            }

            p_foc_status->align_tick  = 0;
            p_foc_status->align_state = 7;
            p_foc_status->u_dq_ref.Q  = 0;
        }
        else if (p_foc_status->u_dq_ref.Q <= -ud_ref)
        {
            p_foc_status->align_tick++;
        }

        if (p_foc_status->align_tick > MS_TO_RT_TICKS(20))
        {
            p_foc_status->align_tick  = 0;
            p_foc_status->align_state = 10;
            p_foc_status->u_dq_ref.Q  = 0;
        }
        break;

    case 7: // 等待位置稳定
        if (++p_foc_status->align_tick > MS_TO_RT_TICKS(p_motor_controller_config->align_time))
        {
            p_foc_status->align_tick    = 0;
            p_foc_status->align_state   = 8;
            p_foc_status->align_initial = p_motor_status->pos_fb;
        }
        break;

    case 8: // 反向增加电压并检查位置反馈
        if (p_foc_status->u_dq_ref.Q > -ud_ref)
        {
            p_foc_status->u_dq_ref.Q -= ud_ref / MS_TO_S((float)p_motor_controller_config->align_time) * BOARD_RT_DT;
        }

        delta_pos     = p_motor_status->pos_fb - p_foc_status->align_initial;
        abs_delta_pos = fabsf(delta_pos);
        if (abs_delta_pos > pos_err_threshold)
        {
            // 第二次位移判断：与第一次结果对比，一致时确认方向检测有效
            int dir_step = (delta_pos > 0) ? -1 : 1;

            if (p_foc_status->align_dir_detected == dir_step)
            {
                p_foc_status->align_dir_detected = dir_step * 2; // 两次一致，确认: 2=正确, -2=需修正
                p_foc_status->align_tick         = 0;
                p_foc_status->align_state        = 9;
                p_foc_status->u_dq_ref.Q         = 0;
            }
            else
            {
                p_foc_status->align_dir_detected = 0; // 两次不一致，放弃修正
                p_foc_status->align_tick         = 0;
                p_foc_status->align_state        = 1;
                p_foc_status->u_dq_ref.Q         = 0;
            }
        }
        else if (p_foc_status->u_dq_ref.Q <= -ud_ref)
        {
            p_foc_status->align_tick++;
        }

        if (p_foc_status->align_tick > MS_TO_RT_TICKS(20))
        {
            p_foc_status->align_tick  = 0;
            p_foc_status->align_state = 1;
            p_foc_status->u_dq_ref.Q  = 0;
        }
        break;

    case 9: // 应用寻相结果
        // 若两次位移判断均检测到电机方向与配置不符，自动修正motor_dir并通知core0，然后重新寻相
        if (p_foc_status->align_dir_detected == -2)
        {
            int corrected_dir = -p_motor_config->motor_dir;

            if (!ipc_send_message(1, index, MOTOR_CMD_SET_MOTOR_DIR, (uint8_t *)&corrected_dir, sizeof(corrected_dir), 0))
            {
                p_motor_config->motor_dir = corrected_dir;

                // 使用修正后的方向重新寻相
                p_foc_status->align_state        = 0;
                p_foc_status->align_dir_detected = 0;
            }
            break;
        }

        // 两次方向判断不一致，放弃本次寻相结果，回到state 1重试
        if (p_foc_status->align_dir_detected == 0)
        {
            p_foc_status->align_initial = p_motor_status->pos_fb;
            p_foc_status->align_state   = 1;
            break;
        }

        p_motor_status->is_aligned = 1;
        p_foc_status->u_dq_ref.D   = 0;
        p_foc_status->u_dq_ref.Q   = 0;
        p_foc_status->align_state  = 11;
        break;

    case 10:
        motor_set_err(index, FAULT_CODE_ALIGN_NO_CONVERGENCE);
        break;

    default:
        break;
    }
}

/**
 * @brief dc寻相实时更新
 * 
 * @param index 电机编号
 */
void dc_align_update_rt(int index)
{
    Motor_Controller_Config_t *p_motor_controller_config = &global_config_in_core1.motor_controller_config[index];
    Motor_Status_t *p_motor_status                       = &global_status.motor_status[index];

    switch (p_motor_controller_config->align_method)
    {
    case Align_Method_Fast:
        p_motor_status->is_aligned = 1;
        break;

    case Align_Method_Slow:
        dc_align_slow_update_rt(index);
        break;

    case Align_Method_Bisection:
        p_motor_status->is_aligned = 1;
        break;

    default:
        break;
    }
}

/**
 * @brief dc相电阻辨识实时更新
 * 
 * @param index 电机编号
 */
void dc_rs_identify_update_rt(int index)
{
    motor_set_err(index, FAULT_CODE_MOTOR_TYPE_IDENTIFY);
    return;
}

/**
 * @brief dc相电感辨识实时更新
 * 
 * @param index 电机编号
 */
void dc_ls_identify_update_rt(int index)
{
    motor_set_err(index, FAULT_CODE_MOTOR_TYPE_IDENTIFY);
    return;
}

/**
 * @brief dc磁链辨识实时更新
 * 
 * @param index 电机编号
 */
void dc_flux_identify_update_rt(int index)
{
    motor_set_err(index, FAULT_CODE_MOTOR_TYPE_IDENTIFY);
    return;
}

/**
 * @brief dc极对数辨识实时更新
 * 
 * @param index 电机编号
 */
void dc_pp_identify_update_rt(int index)
{
    motor_set_err(index, FAULT_CODE_MOTOR_TYPE_IDENTIFY);
    return;
}

/**
 * @brief dc实时更新
 * 
 * @param index 电机编号
 */
void dc_update_rt(int index)
{
}

/**
 * @brief dc高频实时更新
 * 
 * @param index 电机编号
 */
void dc_update_frt(int index)
{
    Motor_Controller_Config_t *p_motor_controller_config = &global_config_in_core1.motor_controller_config[index];
    FOC_Status_t *p_foc_status                           = &global_status.foc_status[index];

    dc_encoder_update(index);

    p_foc_status->we = p_foc_status->we_encoder;

    dc_u_i_update(index);

    switch (p_foc_status->run_state)
    {
    case FOC_VF_OpenLoop:
        dc_vf_update(index);
        dc_pwm_update(index, p_motor_controller_config->dead_zone_compensation_enable);
        break;

    case FOC_IF_OpenLoop:
    case FOC_CloseLoop:
        dc_decoupling_update(index);
        dc_current_loop_update(index);
        dc_limit_circle_update(index);
        dc_power_update(index);
        dc_pwm_update(index, p_motor_controller_config->dead_zone_compensation_enable);
        break;

    default:
        break;
    }
}

/**
 * @brief 复位DC电机控制器
 * 
 * @param index 电机编号
 */
void dc_controller_reset(int index)
{
    FOC_Status_t *p_foc_status = &global_status.foc_status[index];

    p_foc_status->run_state       = FOC_IDLE;
    p_foc_status->align_state     = 0;
    p_foc_status->align_tick      = 0;
    p_foc_status->u_dq_ref.D      = 0;
    p_foc_status->u_dq_ref.Q      = 0;
    p_foc_status->i_dq_ref.D      = 0;
    p_foc_status->i_dq_ref.Q      = 0;
    p_foc_status->current_err_cnt = 0;
}

/**
 * @brief 计算并缓存DC电机额定扭矩和峰值扭矩
 *
 * DC电机转矩与电枢电流成正比：T = kt * I。
 *
 * @param index 电机编号
 */
void dc_calculate_torque_limits(int index)
{
    Motor_Config_t *p_motor_config = &global_config_in_core1.motor_config[index];
    Motor_Status_t *p_motor_status = &global_status.motor_status[index];
    float kt_abs                   = fabsf(p_motor_config->kt);

    p_motor_status->rated_torque = fabsf(p_motor_config->rated_current) * kt_abs;
    p_motor_status->peak_torque  = fabsf(p_motor_config->peak_current) * kt_abs;
}

/**
 * @brief 设置DC电流参考
 * 
 * @param index 电机编号
 * @param current_ref 电流参考
 */
void dc_set_current_ref(int index, float current_ref)
{
    FOC_Status_t *p_foc_status = &global_status.foc_status[index];

    p_foc_status->run_state   = FOC_CloseLoop;
    p_foc_status->current_ref = current_ref;
    p_foc_status->i_dq_ref.Q  = current_ref;
}

/**
 * @brief 获取DC电流反馈
 * 
 * @param index 电机编号
 * @return float 电流反馈
 */
float dc_get_current_fb(int index)
{
    FOC_Status_t *p_foc_status = &global_status.foc_status[index];

    return p_foc_status->current_fb;
}

/**
 * @brief 设置DC扭矩参考
 * 
 * @param index 电机编号
 * @param torque_ref 设置FOC扭矩参考
 */
void dc_set_torque_ref(int index, float torque_ref)
{
    Motor_Config_t *p_motor_config = &global_config_in_core1.motor_config[index];
    Motor_Status_t *p_motor_status = &global_status.motor_status[index];
    FOC_Status_t *p_foc_status     = &global_status.foc_status[index];

    p_foc_status->run_state  = FOC_CloseLoop;
    p_foc_status->torque_ref = torque_ref;

    if (p_motor_config->kt > FLOAT_MIN)
    {
        p_foc_status->i_dq_ref.Q = torque_ref / p_motor_config->kt;
    }
    else
    {
        /* DC 电机无法使用 PMSM 转子磁链公式换算扭矩。 */
        p_foc_status->i_dq_ref.Q = 0.0f;
        motor_set_ext_err(index, EXT_FAULT_CODE_CFG_MOTOR_KT);
    }

    p_motor_status->current_ref = p_foc_status->current_ref = p_foc_status->i_dq_ref.Q;
}

/**
 * @brief 获取DC扭矩反馈
 * 
 * @param index 电机编号
 * @return float 扭矩反馈
 */
float dc_get_torque_fb(int index)
{
    FOC_Status_t *p_foc_status = &global_status.foc_status[index];

    return p_foc_status->torque_fb;
}

/**
 * @brief 设置DC开环电压
 * 
 * @param index 电机编号
 * @param us 电压
 */
void dc_set_vf_voltage(int index, float us)
{
    FOC_Status_t *p_foc_status = &global_status.foc_status[index];

    p_foc_status->run_state  = FOC_VF_OpenLoop;
    p_foc_status->u_dq_ref.D = 0.0f;
    p_foc_status->u_dq_ref.Q = us;
}

/**
 * @brief 设置DC开环电流
 * 
 * @param index 电机编号
 * @param current 电流
 */
void dc_set_if_current(int index, float current)
{
    FOC_Status_t *p_foc_status = &global_status.foc_status[index];

    p_foc_status->run_state  = FOC_IF_OpenLoop;
    p_foc_status->i_dq_ref.D = 0.0f;
    p_foc_status->i_dq_ref.Q = current;
}
