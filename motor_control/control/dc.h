/**
 * @file dc.h
 * @author jinchl (jcl709905683@gmail.com)
 * @brief 
 * @version 0.1
 * @date 2026-03-30
 * 
 * @copyright Copyright (c) 2026
 * 
 */


#pragma once
/**
 * @brief dc寻相实时更新
 * 
 * @param index 电机编号
 */
void dc_align_update_rt(int index);

/**
 * @brief dc相电阻辨识实时更新
 * 
 * @param index 电机编号
 */
void dc_rs_identify_update_rt(int index);

/**
 * @brief dc相电感辨识实时更新
 * 
 * @param index 电机编号
 */
void dc_ls_identify_update_rt(int index);

/**
 * @brief dc磁链辨识实时更新
 * 
 * @param index 电机编号
 */
void dc_flux_identify_update_rt(int index);

/**
 * @brief dc极对数辨识实时更新
 * 
 * @param index 电机编号
 */
void dc_pp_identify_update_rt(int index);

/**
 * @brief dc实时更新
 * 
 * @param index 电机编号
 */
void dc_update_rt(int index);

/**
 * @brief dc高频实时更新
 * 
 * @param index 电机编号
 */
void dc_update_frt(int index);

/**
 * @brief 复位DC电机控制器
 * 
 * @param index 电机编号
 */
void dc_controller_reset(int index);

/**
 * @brief 计算并缓存DC电机额定扭矩和峰值扭矩
 *
 * @param index 电机编号
 */
void dc_calculate_torque_limits(int index);

/**
 * @brief 设置DC电流参考
 * 
 * @param index 电机编号
 * @param current_ref 电流参考
 */
void dc_set_current_ref(int index, float current_ref);

/**
 * @brief 获取DC电流反馈
 * 
 * @param index 电机编号
 * @return float 电流反馈
 */
float dc_get_current_fb(int index);

/**
 * @brief 设置DC扭矩参考
 * 
 * @param index 电机编号
 * @param torque_ref 设置FOC扭矩参考
 */
void dc_set_torque_ref(int index, float torque_ref);

/**
 * @brief 获取DC扭矩反馈
 * 
 * @param index 电机编号
 * @return float 扭矩反馈
 */
float dc_get_torque_fb(int index);

/**
 * @brief 设置DC开环电压
 * 
 * @param index 电机编号
 * @param us 电压
 */
void dc_set_vf_voltage(int index, float us);

/**
 * @brief 设置DC开环电流
 * 
 * @param index 电机编号
 * @param current 电流
 */
void dc_set_if_current(int index, float current);
