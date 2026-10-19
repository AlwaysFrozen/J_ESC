/**
 * @file inverter.h
 * @author jinchl (jinchl@jodell.com)
 * @brief 
 * @version 0.1
 * @date 2026-10-19
 * 
 * @copyright Copyright (c) 2026
 * 
 */

#pragma once
#include <stdint.h>

/**
* @brief 使能逆变器
* 
* @param index 电机编号
*/
void enable_inverter(int index);

/**
* @brief 失能逆变器
* 
* @param index 电机编号
*/
void disable_inverter(int index);

/**
   * @brief 设置三相PWM输出
   * @param index 电机编号
   * @param duty_u U相PWM占空比(0.0-1.0)
   * @param duty_v V相PWM占空比(0.0-1.0)
   * @param duty_w W相PWM占空比(0.0-1.0)
   */
void pwm_duty_set(int index, float duty_u, float duty_v, float duty_w);

/**
 * @brief 获取pwm发生器计数值
 * 
 * @param index 电机编号
 * @return uint32_t 计数值
 */
uint32_t get_pwm_counter(int index);
