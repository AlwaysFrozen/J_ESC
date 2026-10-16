/**
 * @file observer.h
 * @author jinchl (jcl709905683@gmail.com)
 * @brief 
 * @version 0.1
 * @date 2026-03-19
 * 
 * @copyright Copyright (c) 2026
 * 
 */


#pragma once
#include "../math/my_math.h"

typedef enum
{
    Observer_Type_None,
    Observer_Type_NESO2,
    Observer_Type_NESO3,
    Observer_Type_KF,
    Observer_Type_EKF,
    Observer_Type_MAX
} Observer_Type_t;

/**
 * @brief 非线性扩张观测器配置结构体
 * 
 */
typedef struct
{
    float neso_gain;
    float neso_k_smooth;
    float neso_z1_speed_threshold;
    float neso_z2_speed_threshold;
    float neso_z3_speed_threshold;
} NESO_Config_t;

typedef struct
{
    float err;
    float err_abs;
    float l1;
    float l2;
    float l3;
    float dz1;
    float dz2;
    float dz3;
    float z1;
    float z2;
    float z3;
} NESO_Status_t;

typedef struct
{
    float pos_estimate;
    float vel_estimate;
    float acc_estimate;
    float tor_out_estimate;
    float tor_disturb_estimate;
} Observer_Output_t;

/**
 * @brief 观测器配置结构体
 * 
 */
typedef struct
{
    Observer_Type_t observer_type;
    int auto_tune;
    float observer_band_width;
} Observer_Config_t;

/**
 * @brief 观测器初始化
 * 
 * @param index 电机编号
 */
void observer_init(int index);

/**
 * @brief 观测器复位
 * 
 * @param index 电机编号
 */
void observer_reset(int index);

/**
 * @brief 观测器实时更新
 * 
 * @param index 电机编号
 */
void observer_update_rt(int index);
