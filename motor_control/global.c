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

void global_config_init_default(void)
{
  memset(&global_config, 0, sizeof(Global_Configs_t));
}
