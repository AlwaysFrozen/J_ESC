/**
 * @file inverter.c
 * @author jinchl (jinchl@jodell.com)
 * @brief 
 * @version 0.1
 * @date 2026-10-19
 * 
 * @copyright Copyright (c) 2026
 * 
 */

#include "inverter.h"

#include "board.h"

#include "main.h"
#include "stm32f4xx_hal.h"
#include "tim.h"
#include "stm32f4xx_ll_tim.h"

/**
* @brief 使能逆变器
* 
* @param index 电机编号
*/
void enable_inverter(int index)
{
    HAL_GPIO_WritePin(DRIVER_EN_GPIO_Port,DRIVER_EN_Pin,GPIO_PIN_SET);

    LL_TIM_CC_EnableChannel(TIM1,
    LL_TIM_CHANNEL_CH1  | LL_TIM_CHANNEL_CH1N |
    LL_TIM_CHANNEL_CH2  | LL_TIM_CHANNEL_CH2N |
    LL_TIM_CHANNEL_CH3  | LL_TIM_CHANNEL_CH3N);
}

/**
* @brief 失能逆变器
* 
* @param index 电机编号
*/
void disable_inverter(int index)
{
    HAL_GPIO_WritePin(DRIVER_EN_GPIO_Port,DRIVER_EN_Pin,GPIO_PIN_RESET);

    LL_TIM_CC_DisableChannel(TIM1,
    LL_TIM_CHANNEL_CH1  | LL_TIM_CHANNEL_CH1N |
    LL_TIM_CHANNEL_CH2  | LL_TIM_CHANNEL_CH2N |
    LL_TIM_CHANNEL_CH3  | LL_TIM_CHANNEL_CH3N);

}

/**
   * @brief 设置三相PWM输出
   * @param index 电机编号
   * @param duty_u U相PWM占空比(0.0-1.0)
   * @param duty_v V相PWM占空比(0.0-1.0)
   * @param duty_w W相PWM占空比(0.0-1.0)
   */
void pwm_duty_set(int index, float duty_u, float duty_v, float duty_w)
{
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, BOARD_PWM_TIM_RELOAD * duty_u);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, BOARD_PWM_TIM_RELOAD * duty_v);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, BOARD_PWM_TIM_RELOAD * duty_w);
}

/**
 * @brief 获取pwm发生器计数值
 * 
 * @param index 电机编号
 * @return uint32_t 计数值
 */
uint32_t get_pwm_counter(int index)
{
    return __HAL_TIM_GET_COUNTER(&htim8);
}