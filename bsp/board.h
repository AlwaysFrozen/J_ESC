/**
 * @file board.h
 * @author jinchl (jcl709905683@gmail.com)
 * @brief 
 * @version 0.1
 * @date 2026-10-15
 * 
 * @copyright Copyright (c) 2026
 * 
 */


#pragma once

#ifdef HW_VESC_6_4
#include "hw_vesc_6_4.h"
#endif

#define BOARD_CPU_CNT_TO_US(CNT)                    ((float)(CNT) * 1e6f / BOARD_CPU_FREQ)
#define BOARD_MOTO_CNT_TO_US(CNT)                   ((float)(CNT) * 1e6f / BOARD_PWM_TIM_FREQ)

#define BOARD_HRT_DT                                (1.0f / (float)BOARD_HRT_FREQ)
#define BOARD_RT_DT                                 (1.0f / (float)BOARD_RT_FREQ)

#define US_TO_RT_TICKS(US)                          ((int)((float)(US) / 1e6f / BOARD_RT_DT))
#define MS_TO_RT_TICKS(MS)                          ((int)((float)(MS) / 1e3f / BOARD_RT_DT))
#define S_TO_RT_TICKS(S)                            ((int)((float)(S) / BOARD_RT_DT))

#define US_TO_HRT_TICKS(US)                         ((int)((float)(US) / 1e6f / BOARD_HRT_DT))
#define MS_TO_HRT_TICKS(MS)                         ((int)((float)(MS) / 1e3f / BOARD_HRT_DT))
#define S_TO_HRT_TICKS(S)                           ((int)((float)(S) / BOARD_HRT_DT))