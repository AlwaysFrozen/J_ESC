/**
 * @file motor_api.h
 * @author jinchl (jcl709905683@gmail.com)
 * @brief 
 * @version 0.1
 * @date 2026-08-04
 * 
 * @copyright Copyright (c) 2026
 * 
 */


#pragma once
#include "../math/my_math.h"
#include "motor.h"

/** 更新弹簧辨识参数；调用方须防止控制中断读取到一半更新的数据。 */
bool motor_set_spring_cali(int index, const Spring_Cali_t *cali);

/** 当前闭环运动所需的弹簧保持力，辨识期间及未使能补偿时为零。 */
float motor_get_spring_compensation(int index);

/**
 * @brief 判断是否是旋转电机
 * 
 * @param index 电机编号
 * @return true 是
 * @return false 否
 */
bool is_rotary_motor(int index);

/**
 * @brief 轴单位转化为电机单位
 * 
 * @param index 电机编号
 * @param units 轴单位
 * @return float 电机单位
 */
float axis_units_to_motor_units(int index, float units);

/**
 * @brief 电机速度单位转化为轴速度单位
 * 
 * @param index 电机编号
 * @param units 电机单位
 * @return float 轴单位
 */
float motor_units_to_axis_units(int index, float units);

/**
 * @brief 电机配置峰值速度转化为电机速度
 * 
 * @param index 
 * @return float 
 */
float cfg_peak_speed_to_motor_speed(int index);

/**
 * @brief 扭矩比例转化为扭矩
 * 
 * @param index 电机编号
 * @param torque_rate 扭矩比例
 * @return float 扭矩
 */
float torque_rate_to_torque(int index, int torque_rate);

/**
 * @brief 扭矩转化为扭矩比例
 * 
 * @param index 电机编号
 * @param torque 扭矩
 * @return int 扭矩比例
 */
int torque_to_torque_rate(int index, float torque);

/**
 * @brief 设置电机跟随误差判断条件
 * 
 * @param index 电机编号
 * @param err_window 误差窗口
 * @param err_time 误差时间
 */
void motor_set_follow_err_condition(int index, float err_window, int err_time);

/**
 * @brief 设置电机到位判断条件
 * 
 * @param index 电机编号
 * @param arrive_window 到位窗口
 * @param arrive_time 到位时间
 */
void motor_set_arrive_condition(int index, float arrive_window, int arrive_time);

/**
 * @brief 设置电机堵判转断条件
 * 
 * @param index 电机编号
 * @param block_vel 堵转速度
 * @param block_tor_rate 堵转扭矩比例
 * @param block_time 堵转时间
 */
void motor_set_block_condition(int index, float block_vel, float block_tor_rate, int block_time);

/**
 * @brief 设置电机电流控制模式
 * 
 * @param index 电机编号
 * @param current 目标电流
 * @return true 成功
 * @return false 失败
 */
bool motor_set_current(int index, float current);

/**
 * @brief 设置电机电流限制
 * 
 * @param index 电机编号
 * @param max_current 最大电流
 * @param min_current 最小电流
 */
void motor_set_current_limit(int index, float max_current, float min_current);

/**
 * @brief 复位电机电流限制
 * 
 * @param index 电机编号
 */
void motor_reset_current_limit(int index);

/**
 * @brief 复位电流控制模式
 * 
 * @param index 电机编号
 */
void motor_reset_current_control(int index);

/**
 * @brief 设置电机扭矩控制模式
 * 
 * @param index 电机编号
 * @param torque_rate 目标扭矩
 * @return true 成功
 * @return false 失败
 */
bool motor_set_torque(int index, float torque_rate);

/**
 * @brief 设置电机扭矩限制
 * 
 * @param index 电机编号
 * @param max_torque 最大扭矩
 * @param min_torque 最小扭矩
 */
void motor_set_torque_limit(int index, float max_torque_rate, float min_torque_rate);

/**
 * @brief 复位电机扭矩限制
 * 
 * @param index 电机编号
 */
void motor_reset_torque_limit(int index);

/**
 * @brief 复位扭矩控制模式
 * 
 * @param index 电机编号
 */
void motor_reset_torque_control(int index);

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
bool motor_set_speed(int index, float speed, float acc, float jerk, bool allow_block);

/**
 * @brief 设置电机速度限制
 * 
 * @param index 电机编号
 * @param max_speed 最大速度限制
 * @param min_speed 最小速度限制
 */
void motor_set_speed_limit(int index, float max_speed, float min_speed);

/**
 * @brief 复位速度扭矩限制
 * 
 * @param index 电机编号
 */
void motor_reset_speed_limit(int index);

/**
 * @brief 复位速度控制模式
 * 
 * @param index 电机编号
 */
void motor_reset_speed_control(int index);

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
bool motor_set_pos(int index, float pos, float speed, float acc, float jerk);

/**
 * @brief 复位位置控制模式
 * 
 * @param index 电机编号
 */
void motor_reset_pos_control(int index);

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
bool motor_set_force(int index, float pos, float second_pos, float block_pos, float speed, float acc, float jerk, float max_pos, float max_vel, float max_force, float max_force_err, float max_force_err_time, bool close_loop);

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
bool motor_set_impedance(int index, float pos, float speed, float acc, float jerk);

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
bool motor_set_admittance(int index, float pos, float speed, float acc, float jerk, float displacement, float force);

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
bool motor_set_teaching(int index, float max_pos, float min_pos, float speed, float acc, float dead_zone);

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
bool motor_set_cst(int index, int torque_rate, float speed_limit, int tor_limit, int pos_tor_limit, int neg_tor_limit);

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
bool motor_set_csv(int index, float speed, int tor_limit, int pos_tor_limit, int neg_tor_limit);

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
bool motor_set_csp(int index, float pos, float vel_ff, float acc_ff, float speed_limit, int tor_limit, int pos_tor_limit, int neg_tor_limit);

/**
 * @brief 设置电机电压开环相位控制模式
 * 
 * @param index 电机编号
 * @param voltage 开环电压
 * @param deg 开环角度
 * @return true 成功
 * @return false 失败
 */
bool motor_set_vf_theta(int index, float voltage, float deg);

/**
 * @brief 设置电机电压开环速度控制模式
 * 
 * @param index 电机编号
 * @param voltage 开环电压
 * @param speed 开环速度
 * @return true 成功
 * @return false 失败
 */
bool motor_set_vf_speed(int index, float voltage, float speed);

/**
 * @brief 设置电机电流开环相位控制模式
 * 
 * @param index 电机编号
 * @param current 开环电流
 * @param deg 开环角度
 * @return true 成功
 * @return false 失败
 */
bool motor_set_if_theta(int index, float current, float deg);

/**
 * @brief 设置电机电流开环速度控制模式
 * 
 * @param index 电机编号
 * @param current 开环电流
 * @param speed 开环速度
 * @return true 成功
 * @return false 失败
 */
bool motor_set_if_speed(int index, float current, float speed);

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
bool motor_set_hfi_vf_speed(int index, float voltage, float deg, float speed);

/**
 * @brief 设置电机 d 轴电流扫频参考
 * 
 * @param index 电机编号
 * @param current_d d 轴电流参考
 * @return true 成功
 * @return false 失败
 */
bool motor_set_current_d_sweep(int index, float current_d);

/**
 * @brief 设置电机 q 轴电流扫频参考
 * 
 * @param index 电机编号
 * @param current_q q 轴电流参考
 * @return true 成功
 * @return false 失败
 */
bool motor_set_current_q_sweep(int index, float current_q);

/**
 * @brief 设置电机速度扫频参考
 * 
 * @param index 电机编号
 * @param speed 目标速度
 * @return true 成功
 * @return false 失败
 */
bool motor_set_speed_sweep(int index, float speed);

/**
 * @brief 设置电机位置扫频参考
 * 
 * @param index 电机编号
 * @param pos 目标位置
 * @return true 成功
 * @return false 失败
 */
bool motor_set_pos_sweep(int index, float pos);

/**
 * @brief 设置电机速度监测模式
 * 
 * @param index 电机编号
 * @param vel_monitor_mode 速度监测模式
 */
void motor_set_vel_monitor_mode(int index, Vel_Monitor_Mode_t vel_monitor_mode);

/**
 * @brief 开始电机寻相
 * 
 * @param index 电机编号
 * @return true 成功
 * @return false 失败
 */
bool motor_set_align(int index);

/**
 * @brief 开始电机相电阻辨识
 * 
 * @param index 电机编号
 * @return true 成功
 * @return false 失败
 */
bool motor_set_rs_identify(int index);

/**
 * @brief 开始电机相电感辨识
 * 
 * @param index 电机编号
 * @return true 成功
 * @return false 失败
 */
bool motor_set_ls_identify(int index);

/**
 * @brief 开始电机极对数辨识
 * 
 * @param index 电机编号
 * @return true 成功
 * @return false 失败
 */
bool motor_set_pp_identify(int index);

/**
 * @brief 开始电机磁链辨识
 * 
 * @param index 电机编号
 * @return true 成功
 * @return false 失败
 */
bool motor_set_flux_identify(int index);

/**
 * @brief 开始电机齿槽转矩辨识
 * 
 * @param index 电机编号
 * @return true 成功
 * @return false 失败
 */
bool motor_set_cogg_identify(int index);

/**
 * @brief 开始电机惯量辨识
 * 
 * @param index 电机编号
 * @return true 成功
 * @return false 失败
 */
bool motor_set_inertia_identify(int index);

/**
 * @brief 开始电机摩擦力辨识
 * 
 * @param index 电机编号
 * @return true 成功
 * @return false 失败
 */
bool motor_set_friction_identify(int index);

/**
 * @brief 开始电机重力辨识
 * 
 * @param index 电机编号
 * @return true 成功
 * @return false 失败
 */
bool motor_set_gravity_identify(int index);
