/**
 * @file motor_api.c
 * @author jinchl (jcl709905683@gmail.com)
 * @brief 
 * @version 0.1
 * @date 2026-08-04
 * 
 * @copyright Copyright (c) 2026
 * 
 */


#include "motor_api.h"
#include "global.h"
#include <stddef.h>


/**
 * @brief 判断是否是旋转电机
 *
 * @param index 轴编号
 * @return 是否为旋转电机
 */
bool is_rotary_motor(int index)
{
    Motor_Config_t *p_motor_config = &global_config.motor_config[index];

    return p_motor_config->motor_struct == Motor_Struct_Rotary;
}

/**
 * @brief 轴单位转化为电机单位
 * 
 * @param index 电机编号
 * @param units 轴单位
 * @return float 电机单位
 */
float axis_units_to_motor_units(int index, float units)
{
    Motor_Config_t *p_motor_config = &global_config.motor_config[index];

    switch (p_motor_config->motor_struct)
    {
    case Motor_Struct_Linear:
        return MM_TO_M(units);

    case Motor_Struct_Rotary:
        return DEG_TO_RAD(units);

    default:
        return units;
    }
}

/**
 * @brief 电机速度单位转化为轴速度单位
 * 
 * @param index 电机编号
 * @param units 电机单位
 * @return float 轴单位
 */
float motor_units_to_axis_units(int index, float units)
{
    Motor_Config_t *p_motor_config = &global_config.motor_config[index];

    switch (p_motor_config->motor_struct)
    {
    case Motor_Struct_Linear:
        return M_TO_MM(units);

    case Motor_Struct_Rotary:
        return RAD_TO_DEG(units);

    default:
        return units;
    }
}

/**
 * @brief 电机配置峰值速度转化为电机速度
 * 
 * @param index 
 * @return float 
 */
float cfg_peak_speed_to_motor_speed(int index)
{
    Motor_Config_t *p_motor_config = &global_config.motor_config[index];

    switch (p_motor_config->motor_struct)
    {
    case Motor_Struct_Linear:
        return p_motor_config->peak_speed;

    case Motor_Struct_Rotary:
        return RPM_TO_OMEGA(p_motor_config->peak_speed);

    default:
        return p_motor_config->peak_speed;
    }
}

/**
 * @brief 扭矩比例转化为扭矩
 * 
 * @param index 电机编号
 * @param torque_rate 扭矩比例
 * @return float 扭矩
 */
float torque_rate_to_torque(int index, int torque_rate)
{
    Motor_Status_t *p_motor_status = &global_status.motor_status[index];

    float torque = 0;

    torque = (float)torque_rate / RATED_TORQUE_RATE * p_motor_status->rated_torque;
    torque = CONSTRAIN(torque, -p_motor_status->peak_torque, p_motor_status->peak_torque);

    return torque;
}

/**
 * @brief 扭矩转化为扭矩比例
 * 
 * @param index 电机编号
 * @param torque 扭矩
 * @return int 扭矩比例
 */
int torque_to_torque_rate(int index, float torque)
{
    Motor_Status_t *p_motor_status = &global_status.motor_status[index];

    float torque_rate = 0;

    if (p_motor_status->rated_torque < FLOAT_EPSILON)
    {
        return 0;
    }

    torque_rate = torque / p_motor_status->rated_torque * RATED_TORQUE_RATE;

    return torque_rate;
}

/**
 * @brief 设置电机跟随误差判断条件
 * 
 * @param index 电机编号
 * @param err_window 误差窗口
 * @param err_time 误差时间
 */
void motor_set_follow_err_condition(int index, float err_window, int err_time)
{
    Motor_Status_t *p_motor_status = &global_status.motor_status[index];

    p_motor_status->follow_err_window = axis_units_to_motor_units(index, err_window);
    p_motor_status->follow_err_time   = err_time;
}

/**
 * @brief 设置电机到位判断条件
 * 
 * @param index 电机编号
 * @param arrive_window 到位窗口
 * @param arrive_time 到位时间
 */
void motor_set_arrive_condition(int index, float arrive_window, int arrive_time)
{
    Motor_Status_t *p_motor_status = &global_status.motor_status[index];

    p_motor_status->arrive_window = axis_units_to_motor_units(index, arrive_window);
    p_motor_status->arrive_time   = arrive_time;
}

/**
 * @brief 设置电机堵判转断条件
 * 
 * @param index 电机编号
 * @param block_vel 堵转速度
 * @param block_tor_rate 堵转扭矩比例
 * @param block_time 堵转时间
 */
void motor_set_block_condition(int index, float block_vel, float block_tor_rate, int block_time)
{
    Motor_Status_t *p_motor_status = &global_status.motor_status[index];

    p_motor_status->block_vel  = axis_units_to_motor_units(index, block_vel);
    p_motor_status->block_tor  = torque_rate_to_torque(index, block_tor_rate);
    p_motor_status->block_time = block_time;
    p_motor_status->is_blocked = 0;
    p_motor_status->block_tick = 0;
}

/**
 * @brief 设置电机电流控制模式
 * 
 * @param index 电机编号
 * @param current 目标电流
 * @return true 成功
 * @return false 失败
 */
bool motor_set_current(int index, float current)
{
    Motor_Status_t *p_motor_status = &global_status.motor_status[index];

    if (!p_motor_status->is_calibrated || !p_motor_status->is_enabled || p_motor_status->is_error || !p_motor_status->is_aligned || p_motor_status->is_braked)
    {
        return false;
    }

    current = CONSTRAIN(current, p_motor_status->negative_current_limit, p_motor_status->positive_current_limit);

    p_motor_status->motor_ctrl_mode = MCM_Current;
    p_motor_status->current_ref     = current;
    p_motor_status->is_blocked      = 0;
    p_motor_status->block_tick      = 0;

    return true;
}

/**
 * @brief 设置电机电流限制
 * 
 * @param index 电机编号
 * @param max_current 最大电流
 * @param min_current 最小电流
 */
void motor_set_current_limit(int index, float max_current, float min_current)
{
    Motor_Config_t *p_motor_config = &global_config.motor_config[index];
    Motor_Status_t *p_motor_status = &global_status.motor_status[index];

    max_current = CONSTRAIN(max_current, 0, p_motor_config->peak_current);
    min_current = CONSTRAIN(min_current, -p_motor_config->peak_current, 0);

    p_motor_status->positive_current_limit = max_current;
    p_motor_status->negative_current_limit = min_current;
}

/**
 * @brief 复位电机电流限制
 * 
 * @param index 电机编号
 */
void motor_reset_current_limit(int index)
{
    Motor_Config_t *p_motor_config = &global_config.motor_config[index];
    Motor_Status_t *p_motor_status = &global_status.motor_status[index];

    p_motor_status->positive_current_limit = p_motor_config->peak_current;
    p_motor_status->negative_current_limit = -p_motor_config->peak_current;
}

/**
 * @brief 复位电流控制模式
 * 
 * @param index 电机编号
 */
void motor_reset_current_control(int index)
{
    Motor_Controller_Config_t *p_motor_controller_config = &global_config.motor_controller_config[index];
    Loop_Controller_t *p_loop_controller_status          = &global_status.loop_controller_status[index];

    switch (p_motor_controller_config->motor_controller_type)
    {
    case Motor_Controller_Type_FOC:
        PID_Reset(&p_loop_controller_status->id_pid);
        PID_Reset(&p_loop_controller_status->iq_pid);
        break;

    case Motor_Controller_Type_DTC:
        break;

    default:
        break;
    }
}

/**
 * @brief 设置电机扭矩控制模式
 * 
 * @param index 电机编号
 * @param torque_rate 目标扭矩
 * @return true 成功
 * @return false 失败
 */
bool motor_set_torque(int index, float torque_rate)
{
    Motor_Status_t *p_motor_status = &global_status.motor_status[index];

    if (!p_motor_status->is_calibrated || !p_motor_status->is_enabled || p_motor_status->is_error || !p_motor_status->is_aligned || p_motor_status->is_braked)
    {
        return false;
    }

    float torque = torque_rate_to_torque(index, torque_rate);
    torque       = CONSTRAIN(torque, p_motor_status->negative_tor_limit, p_motor_status->positive_tor_limit);

    p_motor_status->motor_ctrl_mode = MCM_Torque;
    p_motor_status->torque_ref      = torque;
    p_motor_status->is_blocked      = 0;
    p_motor_status->block_tick      = 0;

    return true;
}

/**
 * @brief 设置电机扭矩限制
 * 
 * @param index 电机编号
 * @param max_torque_rate 最大扭矩
 * @param max_torque_rate 最小扭矩
 */
void motor_set_torque_limit(int index, float max_torque_rate, float min_torque_rate)
{
    Motor_Status_t *p_motor_status              = &global_status.motor_status[index];
    Loop_Controller_t *p_loop_controller_status = &global_status.loop_controller_status[index];

    p_motor_status->positive_tor_limit = torque_rate_to_torque(index, max_torque_rate);
    p_motor_status->negative_tor_limit = torque_rate_to_torque(index, min_torque_rate);

    PID_Set_Limit(&p_loop_controller_status->speed_pid, p_motor_status->positive_tor_limit, p_motor_status->negative_tor_limit);
}

/**
 * @brief 复位电机扭矩限制
 * 
 * @param index 电机编号
 */
void motor_reset_torque_limit(int index)
{
    Motor_Status_t *p_motor_status              = &global_status.motor_status[index];
    Loop_Controller_t *p_loop_controller_status = &global_status.loop_controller_status[index];

    p_motor_status->positive_tor_limit = p_motor_status->peak_torque;
    p_motor_status->negative_tor_limit = -p_motor_status->peak_torque;

    PID_Set_Limit(&p_loop_controller_status->speed_pid, p_motor_status->peak_torque, -p_motor_status->peak_torque);
}

/**
 * @brief 复位扭矩控制模式
 * 
 * @param index 电机编号
 */
void motor_reset_torque_control(int index)
{
    motor_reset_current_control(index);
}

/**
 * @brief 设置电机速度控制模式
 * 
 * @param index 电机编号
 * @param speed 目标速度
 * @param acc 目标加速度
 * @param jerk 目标加加速度
 * @param allow_block 允许堵转
 * @return true 成功
 * @return false 失败
 */
bool motor_set_speed(int index, float speed, float acc, float jerk, bool allow_block)
{
    Motor_Status_t *p_motor_status = &global_status.motor_status[index];

    if (!p_motor_status->is_calibrated || !p_motor_status->is_enabled || p_motor_status->is_error || !p_motor_status->is_aligned || p_motor_status->is_braked)
    {
        return false;
    }

    speed = axis_units_to_motor_units(index, speed);
    acc   = axis_units_to_motor_units(index, acc);
    jerk  = axis_units_to_motor_units(index, jerk);

    speed = CONSTRAIN(speed, p_motor_status->negative_vel_limit, p_motor_status->positive_vel_limit);

    if (allow_block)
    {
        motor_set_vel_monitor_mode(index, Vel_Monitor_AllowBlock);
    }
    else
    {
        motor_set_vel_monitor_mode(index, Vel_Monitor_Normal);
    }

    motor_plan_vel(index, speed, acc, jerk);

    p_motor_status->motor_ctrl_mode = MCM_Speed;
    p_motor_status->vel_ref         = speed;
    p_motor_status->is_blocked      = 0;
    p_motor_status->block_tick      = 0;
    p_motor_status->follow_err_tick = 0;

    return true;
}

/**
 * @brief 设置电机速度限制
 * 
 * @param index 电机编号
 * @param max_speed 最大速度限制
 * @param min_speed 最小速度限制
 */
void motor_set_speed_limit(int index, float max_speed, float min_speed)
{
    Motor_Status_t *p_motor_status              = &global_status.motor_status[index];
    Loop_Controller_t *p_loop_controller_status = &global_status.loop_controller_status[index];

    float peak_vel = cfg_peak_speed_to_motor_speed(index);

    max_speed = axis_units_to_motor_units(index, max_speed);
    min_speed = axis_units_to_motor_units(index, min_speed);

    max_speed = CONSTRAIN(max_speed, 0, peak_vel);
    min_speed = CONSTRAIN(min_speed, -peak_vel, 0);

    p_motor_status->positive_vel_limit = max_speed;
    p_motor_status->negative_vel_limit = min_speed;

    PID_Set_Limit(&p_loop_controller_status->pos_pid, max_speed, min_speed);
}

/**
 * @brief 复位速度扭矩限制
 * 
 * @param index 电机编号
 */
void motor_reset_speed_limit(int index)
{
    Motor_Status_t *p_motor_status              = &global_status.motor_status[index];
    Loop_Controller_t *p_loop_controller_status = &global_status.loop_controller_status[index];

    float peak_vel = cfg_peak_speed_to_motor_speed(index);

    p_motor_status->positive_vel_limit = peak_vel;
    p_motor_status->negative_vel_limit = -peak_vel;

    PID_Set_Limit(&p_loop_controller_status->pos_pid, peak_vel, -peak_vel);
}

/**
 * @brief 复位速度控制模式
 * 
 * @param index 电机编号
 */
void motor_reset_speed_control(int index)
{
    Loop_Controller_t *p_loop_controller_status = &global_status.loop_controller_status[index];

    PID_Reset(&p_loop_controller_status->speed_pid);
}

/**
 * @brief 设置电机位置控制模式
 * 
 * @param index 电机编号
 * @param pos 目标位置
 * @param speed 目标速度
 * @param acc 目标加速度
 * @param jerk 目标加加速度
 * @return true 成功
 * @return false 失败
 */
bool motor_set_pos(int index, float pos, float speed, float acc, float jerk)
{
    Motor_Status_t *p_motor_status = &global_status.motor_status[index];

    if (!p_motor_status->is_calibrated || !p_motor_status->is_enabled || p_motor_status->is_error || !p_motor_status->is_aligned || p_motor_status->is_braked)
    {
        return false;
    }

    pos   = axis_units_to_motor_units(index, pos);
    speed = axis_units_to_motor_units(index, speed);
    acc   = axis_units_to_motor_units(index, acc);
    jerk  = axis_units_to_motor_units(index, jerk);

    motor_plan_pos(index, pos, speed, acc, jerk);

    p_motor_status->motor_ctrl_mode = MCM_Pos;
    p_motor_status->is_blocked      = 0;
    p_motor_status->block_tick      = 0;
    p_motor_status->is_arrived      = 0;
    p_motor_status->arrive_tick     = 0;
    p_motor_status->follow_err_tick = 0;

    return true;
}

/**
 * @brief 复位位置控制模式
 * 
 * @param index 电机编号
 */
void motor_reset_pos_control(int index)
{
    Loop_Controller_t *p_loop_controller_status = &global_status.loop_controller_status[index];

    PID_Reset(&p_loop_controller_status->pos_pid);
}

/**
 * @brief 设置电机开环力控模式
 * 
 * @param index 电机编号
 * @param pos 目标位置
 * @param second_pos 第二目标位置
 * @param block_pos 叠料报警位置
 * @param speed 目标速度
 * @param acc 目标加速度
 * @param jerk 目标加加速度
 * @param max_pos 最大位置限制
 * @param max_vel 最大速度限制
 * @param max_force 最大压力限制
 * @param max_force_err 压力限制误差阈值
 * @param max_force_err_time 压力限制阈值时间
 * @param close_loop 闭环力控使能
 * @return true 成功
 * @return false 失败
 */
bool motor_set_force(int index, float pos, float second_pos, float block_pos, float speed, float acc, float jerk, float max_pos, float max_vel, float max_force, float max_force_err, float max_force_err_time, bool close_loop)
{
    Motor_Status_t *p_motor_status = &global_status.motor_status[index];

    if (!p_motor_status->is_calibrated || !p_motor_status->is_enabled || p_motor_status->is_error || !p_motor_status->is_aligned || p_motor_status->is_braked)
    {
        return false;
    }

    pos        = axis_units_to_motor_units(index, pos);
    second_pos = axis_units_to_motor_units(index, second_pos);
    block_pos  = axis_units_to_motor_units(index, block_pos);
    speed      = axis_units_to_motor_units(index, speed);
    acc        = axis_units_to_motor_units(index, acc);
    jerk       = axis_units_to_motor_units(index, jerk);
    max_pos    = axis_units_to_motor_units(index, max_pos);
    max_vel    = axis_units_to_motor_units(index, max_vel);

    if (max_vel < FLOAT_EPSILON)
    {
        max_vel = 0.01f;
    }
    if (max_force_err < FLOAT_EPSILON)
    {
        max_force_err = max_force * 0.01f;
    }
    if (max_force_err_time < FLOAT_EPSILON)
    {
        max_force_err_time = 10.0f;
    }

    p_motor_status->is_blocked  = 0;
    p_motor_status->block_tick  = 0;
    p_motor_status->is_arrived  = 0;
    p_motor_status->arrive_tick = 0;

    p_motor_status->force_status.lift_pos        = second_pos;
    p_motor_status->force_status.block_pos       = block_pos;
    p_motor_status->force_status.vel             = speed;
    p_motor_status->force_status.acc             = acc;
    p_motor_status->force_status.jerk            = jerk;
    p_motor_status->force_status.max_pos         = max_pos;
    p_motor_status->force_status.max_vel         = max_vel;
    p_motor_status->force_status.force_over_cnt  = 0;
    p_motor_status->force_status.force_reach_cnt = 0;

    if (!close_loop)
    {
        p_motor_status->force_status.max_tor = max_force;
        p_motor_status->motor_ctrl_mode      = MCM_Ol_Force;
    }
    else
    {
        // p_motor_status->force_status.max_tor            = torque_rate_to_torque(index, max_force * 2.0f); // 将最大扭矩限制为目标扭矩的2倍,但考虑到不一定是直线电机直驱场景所以注释掉
        p_motor_status->force_status.max_force          = max_force;
        p_motor_status->force_status.max_force_err      = max_force_err;
        p_motor_status->force_status.max_force_err_time = max_force_err_time;
        p_motor_status->motor_ctrl_mode                 = MCM_Cl_Force;
    }

    if (p_motor_status->pos_fb >= max_pos && max_pos > FLOAT_EPSILON)
    {
        motor_plan_pos(index, second_pos, speed, acc, jerk);

        p_motor_status->force_status.force_stage = Force_Stage_Err;
    }
    else if (p_motor_status->pos_fb < pos)
    {
        motor_plan_pos(index, pos, speed, acc, jerk);

        p_motor_status->force_status.force_stage = Force_Stage_Pos;
    }
    else
    {
        if (!close_loop)
        {
            motor_set_torque_limit(index, p_motor_status->force_status.max_tor, -p_motor_status->force_status.max_tor);
            motor_plan_vel(index, p_motor_status->force_status.max_vel, p_motor_status->force_status.acc, p_motor_status->force_status.jerk);
            p_motor_status->force_status.force_stage = Force_Stage_Vel;
        }
        else
        {
            p_motor_status->force_status.force_stage = Force_Stage_Force;
            softland_reset(index);
        }
    }

    return true;
}

/**
 * @brief 设置电机阻抗控制模式
 * 
 * @param index 电机编号
 * @param pos 目标位置
 * @param speed 目标速度
 * @param acc 目标加速度
 * @param jerk 目标加加速度
 * @return true 成功
 * @return false 失败
 */
bool motor_set_impedance(int index, float pos, float speed, float acc, float jerk)
{
    Motor_Status_t *p_motor_status              = &global_status.motor_status[index];
    Loop_Controller_t *p_loop_controller_status = &global_status.loop_controller_status[index];

    if (!p_motor_status->is_calibrated || !p_motor_status->is_enabled || p_motor_status->is_error || !p_motor_status->is_aligned || p_motor_status->is_braked)
    {
        return false;
    }

    pos   = axis_units_to_motor_units(index, pos);
    speed = axis_units_to_motor_units(index, speed);
    acc   = axis_units_to_motor_units(index, acc);
    jerk  = axis_units_to_motor_units(index, jerk);

    motor_plan_pos(index, pos, speed, acc, jerk);

    p_motor_status->motor_ctrl_mode = MCM_Impedance;
    p_motor_status->vel_ff          = 0;
    p_motor_status->acc_ref         = 0;
    p_motor_status->torque_ff       = 0;
    p_motor_status->is_blocked      = 0;
    p_motor_status->block_tick      = 0;
    p_motor_status->is_arrived      = 0;
    p_motor_status->arrive_tick     = 0;
    p_motor_status->follow_err_tick = 0;

    Compliance_Controller_Reset(&p_loop_controller_status->force_compliance);

    p_loop_controller_status->force_compliance.limit.max_force = p_motor_status->positive_tor_limit;
    p_loop_controller_status->force_compliance.limit.min_force = p_motor_status->negative_tor_limit;

    return true;
}

/**
 * @brief 设置电机导纳控制模式
 * 
 * @param index 电机编号
 * @param pos 目标位置
 * @param speed 目标速度
 * @param acc 目标加速度
 * @param jerk 目标加加速度
 * @param displacement 虚拟位移限制
 * @param force 目标力
 * @return true 成功
 * @return false 失败
 */
bool motor_set_admittance(int index, float pos, float speed, float acc, float jerk, float displacement, float force)
{
    Motor_Status_t *p_motor_status              = &global_status.motor_status[index];
    Loop_Controller_t *p_loop_controller_status = &global_status.loop_controller_status[index];

    if (!p_motor_status->is_calibrated || !p_motor_status->is_enabled || p_motor_status->is_error || !p_motor_status->is_aligned || p_motor_status->is_braked)
    {
        return false;
    }

    pos          = axis_units_to_motor_units(index, pos);
    speed        = axis_units_to_motor_units(index, speed);
    acc          = axis_units_to_motor_units(index, acc);
    jerk         = axis_units_to_motor_units(index, jerk);
    displacement = axis_units_to_motor_units(index, displacement);

    motor_plan_pos(index, pos, speed, acc, jerk);

    p_motor_status->motor_ctrl_mode = MCM_Admittance;
    p_motor_status->force_ref       = force;
    p_motor_status->vel_ff          = 0;
    p_motor_status->acc_ref         = 0;
    p_motor_status->torque_ff       = 0;
    p_motor_status->is_blocked      = 0;
    p_motor_status->block_tick      = 0;
    p_motor_status->is_arrived      = 0;
    p_motor_status->arrive_tick     = 0;
    p_motor_status->follow_err_tick = 0;

    Compliance_Controller_Reset(&p_loop_controller_status->force_compliance);

    float base_pos = p_motor_status->pos_fb;

    p_loop_controller_status->force_compliance.status.base_pos        = base_pos;
    p_loop_controller_status->force_compliance.limit.max_position     = displacement;
    p_loop_controller_status->force_compliance.limit.min_position     = -displacement;
    p_loop_controller_status->force_compliance.limit.max_velocity     = speed;
    p_loop_controller_status->force_compliance.limit.min_velocity     = -speed;
    p_loop_controller_status->force_compliance.limit.max_acceleration = acc;
    p_loop_controller_status->force_compliance.limit.min_acceleration = -acc;

    return true;
}

/**
 * @brief 设置电机示教控制模式
 * 
 * @param index 电机编号
 * @param max_pos 最大位置限制
 * @param min_pos 最小位置限制
 * @param speed 速度限制
 * @param acc 加速度限制
 * @param dead_zone 力死区
 * @return true 成功
 * @return false 失败
 */
bool motor_set_teaching(int index, float max_pos, float min_pos, float speed, float acc, float dead_zone)
{
    Motor_Status_t *p_motor_status              = &global_status.motor_status[index];
    Loop_Controller_t *p_loop_controller_status = &global_status.loop_controller_status[index];

    if (!p_motor_status->is_calibrated || !p_motor_status->is_enabled || p_motor_status->is_error || !p_motor_status->is_aligned || p_motor_status->is_braked)
    {
        return false;
    }

    max_pos = axis_units_to_motor_units(index, max_pos);
    min_pos = axis_units_to_motor_units(index, min_pos);
    speed   = axis_units_to_motor_units(index, speed);
    acc     = axis_units_to_motor_units(index, acc);

    p_motor_status->motor_ctrl_mode = MCM_Teaching;
    p_motor_status->pos_ref         = p_motor_status->pos_fb;
    p_motor_status->vel_ref         = 0;
    p_motor_status->vel_ff          = 0;
    p_motor_status->acc_ref         = 0;
    p_motor_status->torque_ff       = 0;
    p_motor_status->is_planning     = false;
    p_motor_status->force_ref       = dead_zone;
    p_motor_status->is_blocked      = 0;
    p_motor_status->block_tick      = 0;
    p_motor_status->is_arrived      = 0;
    p_motor_status->arrive_tick     = 0;
    p_motor_status->follow_err_tick = 0;

    Compliance_Controller_Reset(&p_loop_controller_status->force_compliance);

    p_loop_controller_status->force_compliance.limit.max_position     = max_pos;
    p_loop_controller_status->force_compliance.limit.min_position     = min_pos;
    p_loop_controller_status->force_compliance.limit.max_velocity     = speed;
    p_loop_controller_status->force_compliance.limit.min_velocity     = -speed;
    p_loop_controller_status->force_compliance.limit.max_acceleration = acc;
    p_loop_controller_status->force_compliance.limit.min_acceleration = -acc;

    return true;
}

/**
 * @brief 设置电机周期同步扭矩控制模式
 * 
 * @param index 电机编号
 * @param torque_rate 目标扭矩
 * @param speed_limit 速度限制
 * @param tor_limit 扭矩限制
 * @param pos_tor_limit 正向扭矩限制
 * @param neg_tor_limit 负向扭矩限制
 * @return true 成功
 * @return false 失败
 */
bool motor_set_cst(int index, int torque_rate, float speed_limit, int tor_limit, int pos_tor_limit, int neg_tor_limit)
{
    Motor_Status_t *p_motor_status = &global_status.motor_status[index];

    if (!p_motor_status->is_calibrated || !p_motor_status->is_enabled || p_motor_status->is_error || !p_motor_status->is_aligned || p_motor_status->is_braked)
    {
        return false;
    }

    int dir             = SIGN(torque_rate);
    float peak_vel      = cfg_peak_speed_to_motor_speed(index);
    speed_limit         = axis_units_to_motor_units(index, speed_limit);
    speed_limit         = fabsf(speed_limit);
    speed_limit         = CONSTRAIN(speed_limit, 0, peak_vel);
    torque_rate         = CONSTRAIN(torque_rate, -neg_tor_limit, pos_tor_limit);
    torque_rate         = CONSTRAIN(torque_rate, -tor_limit, tor_limit);
    int torque_rate_abs = ABS(torque_rate);

    p_motor_status->motor_ctrl_mode = MCM_CST;
    p_motor_status->vel_ref         = dir * speed_limit;
    p_motor_status->is_planning     = 0;

    motor_set_torque_limit(index, torque_rate_abs, -torque_rate_abs);

    return true;
}

/**
 * @brief 设置电机周期同步速度控制模式
 * 
 * @param index 电机编号
 * @param speed 目标速度
 * @param tor_limit 扭矩限制
 * @param pos_tor_limit 正向扭矩限制
 * @param neg_tor_limit 负向扭矩限制
 * @return true 成功
 * @return false 失败
 */
bool motor_set_csv(int index, float speed, int tor_limit, int pos_tor_limit, int neg_tor_limit)
{
    Motor_Status_t *p_motor_status = &global_status.motor_status[index];

    if (!p_motor_status->is_calibrated || !p_motor_status->is_enabled || p_motor_status->is_error || !p_motor_status->is_aligned || p_motor_status->is_braked)
    {
        return false;
    }

    float peak_vel = cfg_peak_speed_to_motor_speed(index);
    speed          = axis_units_to_motor_units(index, speed);
    speed          = CONSTRAIN(speed, -peak_vel, peak_vel);
    pos_tor_limit  = CONSTRAIN(pos_tor_limit, 0, tor_limit);
    neg_tor_limit  = CONSTRAIN(neg_tor_limit, 0, tor_limit);

    p_motor_status->motor_ctrl_mode = MCM_CSV;
    p_motor_status->vel_ref         = speed;
    p_motor_status->is_planning     = 0;

    motor_set_vel_monitor_mode(index, Vel_Monitor_Normal);
    motor_set_torque_limit(index, pos_tor_limit, -neg_tor_limit);

    return true;
}

/**
 * @brief 设置电机周期同步位置控制模式
 * 
 * @param index 电机编号
 * @param pos 目标位置
 * @param vel_ff 速度前馈值
 * @param acc_ff 加速度前馈值
 * @param speed_limit 速度限制
 * @param tor_limit 扭矩限制
 * @param pos_tor_limit 正向扭矩限制
 * @param neg_tor_limit 负向扭矩限制
 * @return true 成功
 * @return false 失败
 */
bool motor_set_csp(int index, float pos, float vel_ff, float acc_ff, float speed_limit, int tor_limit, int pos_tor_limit, int neg_tor_limit)
{
    Motor_Controller_Config_t *p_motor_controller_config = &global_config.motor_controller_config[index];
    Motor_Status_t *p_motor_status                       = &global_status.motor_status[index];

    if (!p_motor_status->is_calibrated || !p_motor_status->is_enabled || p_motor_status->is_error || !p_motor_status->is_aligned || p_motor_status->is_braked)
    {
        return false;
    }

    pos                   = axis_units_to_motor_units(index, pos);
    vel_ff                = axis_units_to_motor_units(index, vel_ff);
    acc_ff                = axis_units_to_motor_units(index, acc_ff);
    float speed_limit_abs = fabsf(speed_limit);
    pos_tor_limit         = CONSTRAIN(pos_tor_limit, 0, tor_limit);
    neg_tor_limit         = CONSTRAIN(neg_tor_limit, 0, tor_limit);

    p_motor_status->motor_ctrl_mode = MCM_CSP;
    p_motor_status->pos_ref         = pos;
    p_motor_status->vel_ff          = vel_ff * p_motor_controller_config->vel_feedforward_rate;
    p_motor_status->acc_ref         = acc_ff;
    p_motor_status->is_planning     = 0;

    motor_set_speed_limit(index, speed_limit_abs, -speed_limit_abs);
    motor_set_torque_limit(index, pos_tor_limit, -neg_tor_limit);

    return true;
}

/**
 * @brief 设置电机电压开环相位控制模式
 * 
 * @param index 电机编号
 * @param voltage 开环电压
 * @param deg 开环角度
 * @return true 成功
 * @return false 失败
 */
bool motor_set_vf_theta(int index, float voltage, float deg)
{
    Motor_Status_t *p_motor_status = &global_status.motor_status[index];

    if (!p_motor_status->is_calibrated || !p_motor_status->is_enabled || p_motor_status->is_error || p_motor_status->is_braked)
    {
        return false;
    }

    p_motor_status->motor_ctrl_mode = MCM_VF_Theta;
    p_motor_status->pos_ref         = DEG_TO_RAD(deg);
    p_motor_status->current_ref     = voltage;

    return true;
}

/**
 * @brief 设置电机电压开环速度控制模式
 * 
 * @param index 电机编号
 * @param voltage 开环电压
 * @param speed 开环速度
 * @return true 成功
 * @return false 失败
 */
bool motor_set_vf_speed(int index, float voltage, float speed)
{
    Motor_Status_t *p_motor_status = &global_status.motor_status[index];

    if (!p_motor_status->is_calibrated || !p_motor_status->is_enabled || p_motor_status->is_error || p_motor_status->is_braked)
    {
        return false;
    }

    p_motor_status->motor_ctrl_mode = MCM_VF_Speed;
    p_motor_status->vel_ref         = DEG_TO_RAD(speed);
    p_motor_status->current_ref     = voltage;

    return true;
}

/**
 * @brief 设置电机电流开环相位控制模式
 * 
 * @param index 电机编号
 * @param current 开环电流
 * @param deg 开环角度
 * @return true 成功
 * @return false 失败
 */
bool motor_set_if_theta(int index, float current, float deg)
{
    Motor_Status_t *p_motor_status = &global_status.motor_status[index];

    if (!p_motor_status->is_calibrated || !p_motor_status->is_enabled || p_motor_status->is_error || p_motor_status->is_braked)
    {
        return false;
    }

    p_motor_status->motor_ctrl_mode = MCM_IF_Theta;
    p_motor_status->pos_ref         = DEG_TO_RAD(deg);
    p_motor_status->current_ref     = current;

    return true;
}

/**
 * @brief 设置电机电流开环速度控制模式
 * 
 * @param index 电机编号
 * @param current 开环电流
 * @param speed 开环速度
 * @return true 成功
 * @return false 失败
 */
bool motor_set_if_speed(int index, float current, float speed)
{
    Motor_Status_t *p_motor_status = &global_status.motor_status[index];

    if (!p_motor_status->is_calibrated || !p_motor_status->is_enabled || p_motor_status->is_error || p_motor_status->is_braked)
    {
        return false;
    }

    p_motor_status->motor_ctrl_mode = MCM_IF_Speed;
    p_motor_status->vel_ref         = DEG_TO_RAD(speed);
    p_motor_status->current_ref     = current;

    return true;
}

/**
 * @brief 设置电机HFI开环控制模式
 * 
 * @param index 电机编号
 * @param voltage 开环电压
 * @param deg 开环角度
 * @param speed 开环速度
 * @return true 成功
 * @return false 失败
 */
bool motor_set_hfi_vf_speed(int index, float voltage, float deg, float speed)
{
    Motor_Status_t *p_motor_status = &global_status.motor_status[index];

    if (!p_motor_status->is_calibrated || !p_motor_status->is_enabled || p_motor_status->is_error || p_motor_status->is_braked)
    {
        return false;
    }

    p_motor_status->motor_ctrl_mode = MCM_HFI_VF;
    p_motor_status->pos_ref         = DEG_TO_RAD(deg);
    p_motor_status->vel_ref         = DEG_TO_RAD(speed);
    p_motor_status->current_ref     = voltage;

    return true;
}

/**
 * @brief 设置电机 d 轴电流扫频参考
 * 
 * @param index 电机编号
 * @param current_d d 轴电流参考
 * @return true 成功
 * @return false 失败
 */
bool motor_set_current_d_sweep(int index, float current_d)
{
    Motor_Status_t *p_motor_status = &global_status.motor_status[index];

    if (!p_motor_status->is_calibrated || !p_motor_status->is_enabled || p_motor_status->is_error || !p_motor_status->is_aligned || p_motor_status->is_braked)
    {
        return false;
    }

    current_d = CONSTRAIN(current_d, p_motor_status->negative_current_limit, p_motor_status->positive_current_limit);

    if (p_motor_status->motor_ctrl_mode != MCM_Current_D_Sweep)
    {
        p_motor_status->is_blocked = 0;
        p_motor_status->block_tick = 0;
    }

    p_motor_status->motor_ctrl_mode = MCM_Current_D_Sweep;
    p_motor_status->current_ref     = current_d;

    return true;
}

/**
 * @brief 设置电机 q 轴电流扫频参考
 * 
 * @param index 电机编号
 * @param current_q q 轴电流参考
 * @return true 成功
 * @return false 失败
 */
bool motor_set_current_q_sweep(int index, float current_q)
{
    Motor_Status_t *p_motor_status = &global_status.motor_status[index];

    if (!p_motor_status->is_calibrated || !p_motor_status->is_enabled || p_motor_status->is_error || !p_motor_status->is_aligned || p_motor_status->is_braked)
    {
        return false;
    }

    current_q = CONSTRAIN(current_q, p_motor_status->negative_current_limit, p_motor_status->positive_current_limit);

    if (p_motor_status->motor_ctrl_mode != MCM_Current_Q_Sweep)
    {
        p_motor_status->is_blocked = 0;
        p_motor_status->block_tick = 0;
    }

    p_motor_status->motor_ctrl_mode = MCM_Current_Q_Sweep;
    p_motor_status->current_ref     = current_q;

    return true;
}

/**
 * @brief 设置电机速度扫频参考
 * 
 * @param index 电机编号
 * @param speed 目标速度
 * @return true 成功
 * @return false 失败
 */
bool motor_set_speed_sweep(int index, float speed)
{
    Motor_Status_t *p_motor_status = &global_status.motor_status[index];

    if (!p_motor_status->is_calibrated || !p_motor_status->is_enabled || p_motor_status->is_error || !p_motor_status->is_aligned || p_motor_status->is_braked)
    {
        return false;
    }

    speed = axis_units_to_motor_units(index, speed);
    speed = CONSTRAIN(speed, p_motor_status->negative_vel_limit, p_motor_status->positive_vel_limit);

    if (p_motor_status->motor_ctrl_mode != MCM_Vel_Sweep)
    {
        p_motor_status->is_blocked      = 0;
        p_motor_status->block_tick      = 0;
        p_motor_status->follow_err_tick = 0;
    }

    p_motor_status->motor_ctrl_mode = MCM_Vel_Sweep;
    p_motor_status->vel_ref         = speed;
    p_motor_status->acc_ref         = 0.0f;
    p_motor_status->torque_ff       = 0.0f;
    p_motor_status->is_planning     = 0;

    motor_set_vel_monitor_mode(index, Vel_Monitor_Disabled);

    return true;
}

/**
 * @brief 设置电机位置扫频参考
 * 
 * @param index 电机编号
 * @param pos 目标位置
 * @return true 成功
 * @return false 失败
 */
bool motor_set_pos_sweep(int index, float pos)
{
    Motor_Status_t *p_motor_status = &global_status.motor_status[index];

    if (!p_motor_status->is_calibrated || !p_motor_status->is_enabled || p_motor_status->is_error || !p_motor_status->is_aligned || p_motor_status->is_braked)
    {
        return false;
    }

    pos = axis_units_to_motor_units(index, pos);

    if (p_motor_status->motor_ctrl_mode != MCM_Pos_Sweep)
    {
        p_motor_status->is_blocked      = 0;
        p_motor_status->block_tick      = 0;
        p_motor_status->is_arrived      = 0;
        p_motor_status->arrive_tick     = 0;
        p_motor_status->follow_err_tick = 0;
    }

    p_motor_status->motor_ctrl_mode = MCM_Pos_Sweep;
    p_motor_status->pos_ref         = pos;
    p_motor_status->vel_ff          = 0.0f;
    p_motor_status->acc_ref         = 0.0f;
    p_motor_status->torque_ff       = 0.0f;
    p_motor_status->is_planning     = 0;

    return true;
}

/**
 * @brief 设置电机速度监测模式
 * 
 * @param index 电机编号
 * @param vel_monitor_mode 速度监测模式
 */
void motor_set_vel_monitor_mode(int index, Vel_Monitor_Mode_t vel_monitor_mode)
{
    Motor_Status_t *p_motor_status = &global_status.motor_status[index];

    p_motor_status->vel_monitor_mode = vel_monitor_mode;
}

/**
 * @brief 开始电机寻相
 * 
 * @param index 电机编号
 * @return true 成功
 * @return false 失败
 */
bool motor_set_align(int index)
{
    Motor_Status_t *p_motor_status = &global_status.motor_status[index];

    if (!p_motor_status->is_calibrated || p_motor_status->is_error || p_motor_status->is_braked)
    {
        return false;
    }

    p_motor_status->motor_ctrl_mode = MCM_Align;
    p_motor_status->is_aligned      = 0;

    return true;
}

/**
 * @brief 开始电机相电阻辨识
 * 
 * @param index 电机编号
 * @return true 成功
 * @return false 失败
 */
bool motor_set_rs_identify(int index)
{
    Motor_Status_t *p_motor_status = &global_status.motor_status[index];

    if (!p_motor_status->is_calibrated || p_motor_status->is_error || p_motor_status->is_braked)
    {
        return false;
    }

    p_motor_status->motor_ctrl_mode = MCM_RS_IDENTIFY;
    p_motor_status->is_aligned      = 0;

    return true;
}

/**
 * @brief 开始电机相电感辨识
 * 
 * @param index 电机编号
 * @return true 成功
 * @return false 失败
 */
bool motor_set_ls_identify(int index)
{
    Motor_Status_t *p_motor_status = &global_status.motor_status[index];

    if (!p_motor_status->is_calibrated || p_motor_status->is_error || p_motor_status->is_braked)
    {
        return false;
    }

    p_motor_status->motor_ctrl_mode = MCM_LS_IDENTIFY;
    p_motor_status->is_aligned      = 0;

    return true;
}

/**
 * @brief 开始电机极对数辨识
 * 
 * @param index 电机编号
 * @return true 成功
 * @return false 失败
 */
bool motor_set_pp_identify(int index)
{
    Motor_Status_t *p_motor_status = &global_status.motor_status[index];

    if (!p_motor_status->is_calibrated || p_motor_status->is_error || p_motor_status->is_braked)
    {
        return false;
    }

    p_motor_status->motor_ctrl_mode = MCM_PP_IDENTIFY;
    p_motor_status->is_aligned      = 0;

    return true;
}

/**
 * @brief 开始电机磁链辨识
 * 
 * @param index 电机编号
 * @return true 成功
 * @return false 失败
 */
bool motor_set_flux_identify(int index)
{
    Motor_Status_t *p_motor_status = &global_status.motor_status[index];

    if (!p_motor_status->is_aligned || !p_motor_status->is_calibrated || p_motor_status->is_error || p_motor_status->is_braked)
    {
        return false;
    }

    p_motor_status->motor_ctrl_mode = MCM_FLUX_IDENTIFY;

    return true;
}

/**
 * @brief 开始电机齿槽转矩辨识
 * 
 * @param index 电机编号
 * @return true 成功
 * @return false 失败
 */
bool motor_set_cogg_identify(int index)
{
    Motor_Status_t *p_motor_status = &global_status.motor_status[index];

    if (!p_motor_status->is_aligned || !p_motor_status->is_calibrated || p_motor_status->is_error || p_motor_status->is_braked)
    {
        return false;
    }

    p_motor_status->motor_ctrl_mode = MCM_Cogg_IDENTIFY;

    return true;
}

/**
 * @brief 开始电机惯量辨识
 * 
 * @param index 电机编号
 * @return true 成功
 * @return false 失败
 */
bool motor_set_inertia_identify(int index)
{
    Motor_Status_t *p_motor_status = &global_status.motor_status[index];

    if (!p_motor_status->is_aligned || !p_motor_status->is_calibrated || p_motor_status->is_error || p_motor_status->is_braked)
    {
        return false;
    }

    p_motor_status->motor_ctrl_mode = MCM_Inertia_IDENTIFY;

    return true;
}

/**
 * @brief 开始电机摩擦力辨识
 * 
 * @param index 电机编号
 * @return true 成功
 * @return false 失败
 */
bool motor_set_friction_identify(int index)
{
    Motor_Status_t *p_motor_status = &global_status.motor_status[index];

    if (!p_motor_status->is_aligned || !p_motor_status->is_calibrated || p_motor_status->is_error || p_motor_status->is_braked)
    {
        return false;
    }

    p_motor_status->motor_ctrl_mode = MCM_Friction_IDENTIFY;

    return true;
}

/**
 * @brief 开始电机重力辨识
 * 
 * @param index 电机编号
 * @return true 成功
 * @return false 失败
 */
bool motor_set_gravity_identify(int index)
{
    Motor_Status_t *p_motor_status = &global_status.motor_status[index];

    if (!p_motor_status->is_aligned || !p_motor_status->is_calibrated || p_motor_status->is_error || p_motor_status->is_braked)
    {
        return false;
    }

    p_motor_status->motor_ctrl_mode = MCM_Gravity_IDENTIFY;

    return true;
}

