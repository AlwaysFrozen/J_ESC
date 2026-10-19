/**
 * @file global.c
 * @author jinchl (jcl709905683@gmail.com)
 * @brief
 * @version 0.1
 * @date 2026-03-11
 *
 * @copyright Copyright (c) 2026
 *
 */

#include "global.h"

__attribute__((section(".ccmram"))) Global_Configs_t global_config = {0};
__attribute__((section(".ccmram"))) volatile Global_Status_t global_status = {0};
__attribute__((section(".ccmram"))) Loop_Filter_t loop_filter_status[BOARD_SUPPORTED_MOTOR_NUMBER] = {0};
__attribute__((section(".ccmram"))) Motion_Planner_t motion_planner[BOARD_SUPPORTED_MOTOR_NUMBER] = {0};
__attribute__((section(".ccmram"))) Sine_Generator_t sine_generator[BOARD_SUPPORTED_MOTOR_NUMBER] = {0};
__attribute__((section(".ccmram"))) PLL_t hfi_pll[BOARD_SUPPORTED_MOTOR_NUMBER] = {0};

void global_init_default(void)
{
  memset((void *)&global_config, 0, sizeof(global_config));

  memset((void *)&global_status, 0, sizeof(global_status));
}
