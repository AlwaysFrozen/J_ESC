/**
 * @file global.h
 * @author jinchl (jcl709905683@gmail.com)
 * @brief 
 * @version 0.1
 * @date 2026-03-11
 * 
 * @copyright Copyright (c) 2026
 * 
 */

#pragma once
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include "board.h"


typedef struct
{
    Motor_Config_t motor_config;
    Encoder_Config_t encoder_config;
    Motor_Controller_Config_t motor_controller_config;
    Motor_Observer_Config_t motor_observer_config;
    Loop_Controller_Config_t loop_controller_config;
    Loop_Filter_Config_t loop_filter_config;
    HFI_Config_t hfi_config;
} Global_Configs_t;

typedef struct
{
    uint32_t ia_adc_raw;
    uint32_t ib_adc_raw;
    uint32_t ic_adc_raw;
    uint32_t va_adc_raw;
    uint32_t vb_adc_raw;
    uint32_t vc_adc_raw;
    uint32_t v_bus_adc_raw;
    uint32_t board_temp_adc_raw;
    uint32_t motor_temp_adc_raw;
    uint32_t ain_adc_raw;
} Peripheral_Raw_Status_t;

typedef struct
{
    float hrt_interval_us;
    float hrt_consume_us;
    float hrt_consume_us_max;
    uint32_t hrt_cnt;
    float rt_interval_us;
    float rt_consume_us;
    float rt_consume_us_max;
    uint32_t rt_cnt;
    float adc_isr_time;
    float pwm_update_time;
} Core_Status_t;

typedef struct
{
    float v_bus;
    float v_bus_f;
    float i_bus;
    float i_bus_f;
    float core_temp;
    float board_temp;
    float board_power;
    float inv_v_bus;
} Power_Status_t;

typedef struct 
{
    Core_Status_t core_status;
    Power_Status_t power_status;
    Peripheral_Raw_Status_t peripheral_raw_status;
    Motor_Basic_Status_t motor_basic_status;
    Encoder_Status_t encoder_status;
    Motor_Status_t motor_status;
    Motor_Observer_t motor_observer;
    Loop_Controller_t loop_controller_status;
    FOC_Status_t foc_status;
    HFI_Status_t hfi_status;
} Global_Status_t;

extern Global_Configs_t global_config;
extern volatile Global_Status_t global_status;
// extern Loop_Filter_t loop_filter_status;
// extern Motion_Planner_t motion_planner;
// extern Sine_Generator_t sine_generator;
// extern PLL_t hfi_pll;

void global_config_init_default(void);

