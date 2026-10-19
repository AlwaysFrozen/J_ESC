/**
 * @file hw_vesc_6_4.h
 * @author jinchl (jcl709905683@gmail.com)
 * @brief 
 * @version 0.1
 * @date 2026-10-16
 * 
 * @copyright Copyright (c) 2026
 * 
 */


#pragma once

#include "stm32f4xx_hal.h"

/* Board Config */
#define BOARD_SUPPORTED_MOTOR_NUMBER                (1U)
#define BOARD_PHASE_CURRENT_SENSOR_NUM              (3U)
#define BOARD_MAX_BUS_VOLTAGE                       (48.0f)
#define BOARD_MIN_BUS_VOLTAGE                       (8.0f)
#define BOARD_MAX_BUS_CURRENT                       (10.0f)
#define BOARD_MAX_PHASE_CURRENT                     (50.0f)
#define BOARD_MAX_TEMP                              (105.0f)

/* Clock Config */
#define BOARD_HCLK_FREQ                             (SystemCoreClock)
#define BOARD_CPU_FREQ                              (BOARD_HCLK_FREQ)
#define BOARD_APB1_FREQ                             (BOARD_HCLK_FREQ / 4)
#define BOARD_APB1_TIMER_FREQ                       (BOARD_APB1_FREQ * 2)
#define BOARD_APB2_FREQ                             (BOARD_HCLK_FREQ / 2)
#define BOARD_APB2_TIMER_FREQ                       (BOARD_APB2_FREQ * 2)

/* Timer & ADC Config */
#define BOARD_PWM_FREQ                              (20000U)
#define BOARD_HRT_FREQ                              (20000U)
#define BOARD_RT_FREQ                               (10000U)

#define BOARD_PWM_TIM_BASE                          (TIM1)
#define BOARD_PWM_TIM_CLK_FREQ                      (BOARD_APB2_TIMER_FREQ)
#define BOARD_PWM_TIM_PRESCALER                     (1U)
#define BOARD_PWM_TIM_FREQ                          (BOARD_PWM_TIM_CLK_FREQ / BOARD_PWM_TIM_PRESCALER)
#define BOARD_PWM_TIM_MAX_CNT                       (0xFFFFUL)
#define BOARD_PWM_TIM_RELOAD                        (BOARD_PWM_TIM_FREQ / (2U * BOARD_PWM_FREQ))
#define BOARD_PWM_DEAD_TIME_NS                      (200U)

#define BOARD_HRT_TIM_BASE                          (TIM8)
#define BOARD_HRT_TIM_CLK_FREQ                      (BOARD_APB2_TIMER_FREQ)
#define BOARD_HRT_TIM_PRESCALER                     (1U)
#define BOARD_HRT_TIM_FREQ                          (BOARD_HRT_TIM_CLK_FREQ / BOARD_HRT_TIM_PRESCALER)
#define BOARD_HRT_TIM_MAX_CNT                       (0xFFFFUL)
#define BOARD_HRT_TIM_RELOAD                        (BOARD_HRT_TIM_FREQ / (2U * BOARD_HRT_FREQ))
#define BOARD_ADC_TRIGGER_COMPARE                   (1U)

#define BOARD_ADC_CALI_CNT                          (BOARD_HRT_FREQ / 10)
#define BOARD_ADC_CALI_COFFCIENT                    (0.01f)
#define BOARD_ADC_RESOLUTION                        (4095.0f)
#define BOARD_ADC_RESOLUTION_HALF                   (BOARD_ADC_RESOLUTION / 2)
#define BOARD_ADC_REFERENCE_VOLTAGE                 (3.3f)
#define BOARD_PHASE_CURRENT_GAIN                    (20.0f)
#define BOARD_PHASE_CURRENT_SHUNT                   (0.0005f)
#define BOARD_BUS_VOLTAGE_CONVERT_COFFCIENT         (BOARD_ADC_REFERENCE_VOLTAGE / BOARD_ADC_RESOLUTION / (2.2f / (2.2f + 39.0f)))
#define BOARD_TERMINAL_VOLTAGE_CONVERT_COFFCIENT    (BOARD_ADC_REFERENCE_VOLTAGE / BOARD_ADC_RESOLUTION / (2.2f / (2.2f + 39.0f)))
#define BOARD_TERMINAL_VOLTAGE_BIAS_ERROR_MAX       (20.0f)
#define BOARD_PHASE_CURRENT_CONVERT_COFFCIENT       (BOARD_ADC_REFERENCE_VOLTAGE / BOARD_ADC_RESOLUTION / BOARD_PHASE_CURRENT_SHUNT / BOARD_PHASE_CURRENT_GAIN)
#define BOARD_PHASE_CURRENT_BIAS_ERROR_MAX          (BOARD_ADC_RESOLUTION_HALF * 0.05f)
#define BOARD_ADC_SAMPLE_AT_HIGH_SIDE               (1)

#define BOARD_RT_TIM_BASE                           (TIM2)
#define BOARD_RT_TIM_CLK_FREQ                       (BOARD_APB1_TIMER_FREQ)
#define BOARD_RT_TIM_PRESCALER                      (27U)
#define BOARD_RT_TIM_FREQ                           (BOARD_RT_TIM_CLK_FREQ / BOARD_RT_TIM_PRESCALER)
#define BOARD_RT_TIM_MAX_CNT                        (0xFFFFFFFFUL)
#define BOARD_RT_TIM_RELOAD                         ((BOARD_RT_TIM_FREQ / BOARD_RT_FREQ) - 1)

#define DBG_TIME_INIT()                             (DWT_Init())
#define GET_DBG_TIM_CNT                             (DWT->CYCCNT)
#define DBG_CNT_TO_US(CNT)                          ((float)(CNT) * 1e6f / SystemCoreClock)
