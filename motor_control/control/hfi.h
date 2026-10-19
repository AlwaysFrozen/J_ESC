/**
 * @file hfi.h
 * @author jinchl (jcl709905683@gmail.com)
 * @brief 
 * @version 0.1
 * @date 2026-08-07
 * 
 * @copyright Copyright (c) 2026
 * 
 */


#pragma once
#include <stdbool.h>
#include <stdint.h>

#include "my_math.h"

#ifndef POLE_IDENTIFY_CURRENT_SAMPLE_TIMES
#define POLE_IDENTIFY_CURRENT_SAMPLE_TIMES 5
#endif

#ifndef POLE_IDENTIFY_MAX_RETRY_TIMES
#define POLE_IDENTIFY_MAX_RETRY_TIMES 5
#endif

#ifndef HFI_LOW_SPEED_THRESHOLD
#define HFI_LOW_SPEED_THRESHOLD _2PI
#endif

typedef enum
{
    HFI_Pole_Identify_Init,
    HFI_Pole_Identify_Wait_Stable_Zero,
    HFI_Pole_Identify_Positive_pulse,
    HFI_Pole_Identify_Wait_Stable_Positive,
    HFI_Pole_Identify_Negative_pulse,
    HFI_Pole_Identify_Wait_Stable_Negative,
    HFI_Pole_Identify_Retry,
    HFI_Pole_Identify_Sync_With_External,
    HFI_Pole_Identify_Done,
} HFI_Pole_Identify_State_t;

typedef struct
{
    uint32_t div;
    float pll_fc;
    float amp;
    float pole_identify_current;
    float pole_identify_amp;
    uint32_t pole_identify_stable_us;
    uint32_t pole_identify_pulse_us;
    uint32_t pole_identify_interval_us;

    float switch_on_hz;
    float switch_off_hz;
} HFI_Config_t;

typedef struct
{
    uint32_t div_cnt;
    int32_t sign;      // 本次更新施加的注入电压极性
    int32_t sign_last; // 当前采样响应对应的注入电压极性

    int32_t enable_pole_identify;
    HFI_Pole_Identify_State_t pole_identify_status;
    int32_t pole_identify_sign;
    uint32_t pole_identify_stable_cnt;
    uint32_t pole_identify_pulse_cnt;
    uint32_t pole_identify_interval_cnt;
    uint32_t pole_identify_cnt;
    uint32_t pole_identify_retry_cnt;

    DQ_Axis_t udq_excitation;

    AB_Axis_t uab_base;          // 缓慢变化的电压基准分量
    AB_Axis_t uab_response;      // 提取出的高频电压响应分量

    AB_Axis_t iab_base;          // 传回普通 FOC 电流环的电流基准分量
    AB_Axis_t iab_response;      // 采样得到的高频电流差分量
    AB_Axis_t iab_response_last; // 上一次响应，三次采样模式下用于再次差分

    AB_Axis_t iab_input;         // 经符号解调后送入 QPLL 的 alpha-beta 响应矢量

    float id_positive_sum;
    float id_negative_sum;
    float id_positive;
    float id_negative;

    float theta; // QPLL 输出的最新 HFI 角度估计值

    /* not in od */
    AB_Axis_t uab_k0;
    AB_Axis_t uab_k1;

    AB_Axis_t iab_k0;
    AB_Axis_t iab_k1;

    bool is_open_loop;
    float vf_amp;
} HFI_Status_t;

/**
 * @brief HFI初始化
 * 
 * @param index 电机编号
 * @param enable_pole_identify 是否进行极性辨识.true:用于无感零速启动 false:用于无感低速启动,实际角度与观测器同步无需极性辨识
 */

void hfi_init(int index, int enable_pole_identify);

/**
 * @brief HFI更新
 * 
 * @param index 电机编号
 */
void hfi_update(int index);

/**
 * @brief HFI极性辨识更新
 * 
 * @param index 电机编号
 */
void hfi_pole_identify_update(int index);

/**
 * @brief 获取最新高频响应的序号
 * 每当 hfi_update() 完成一次新的电压/电流响应解调时递增，用于避免低频任务对同一份高频响应重复处理。
 * @param index 电机编号
 * @return uint32_t 响应序号
 */
uint32_t hfi_get_response_sequence(int index);

/**
 * @brief 判断HFI结果是否有效
 * 
 * @param index 电机编号
 * @return true 结果有效
 * @return false 结果无效
 */
bool is_hfi_valid(int index);

/**
 * @brief 判断HFI是否已经在低速下建立稳定跟踪
 * 
 * @param index 电机编号
 * @return true 已经稳定跟踪
 * @return false 没有稳定跟踪
 */
bool is_hfi_locked(int index);
