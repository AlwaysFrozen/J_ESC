/**
 * @file realtime.h
 * @author jinchl (jcl709905683@gmail.com)
 * @brief 
 * @version 0.1
 * @date 2026-10-16
 * 
 * @copyright Copyright (c) 2026
 * 
 */

#pragma once
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

/**
 * @brief 高频实时中断回调
 * 
 */
void high_realtime_callback(void);

/**
 * @brief 实时中断回调
 * 
 */
void realtime_callback(void);


/**
 * @brief 非实时中断回调
 * 
 */
void none_realtime_callback(void);

/**
 * @brief 初始化高频实时中断
 * 
 */
void high_realtime_init(void);

/**
 * @brief 初始化实时中断
 * 
 */
void realtime_init(void);
