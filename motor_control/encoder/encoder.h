/**
 * @file encoder.h
 * @author jinchl (jcl709905683@gmail.com)
 * @brief 
 * @version 0.1
 * @date 2026-03-11
 * 
 * @copyright Copyright (c) 2026
 * 
 */


#pragma once
#include <stdbool.h>
#include <stdint.h>

#define ENCODER_SPECIAL_FUNCTION_RESET_ERROR (1)
#define ENCODER_SPECIAL_FUNCTION_RESET_ABS   (2)
#define ENCODER_SPECIAL_FUNCTION_RESET_ABM   (3)


/**
 * @brief 编码器索引二级回调函数指针定义
 * 
 */
typedef void (*encoder_index_callback)(int index);

/**
 * @brief 编码器类型枚举
 * 
 */
typedef enum
{
    Encoder_Type_ABZ,
    Encoder_Type_TAMAGAWA,
    Encoder_Type_HCFA,
	Encoder_Type_SPI_AS5048A,
    Encoder_Type_MAX
} Encoder_Type_t;

/**
 * @brief 编码器结构枚举
 * 
 */
typedef enum
{
    Encoder_Struct_Linear,
    Encoder_Struct_Rotary,
    Encoder_Struct_MAX
} Encoder_Struct_t;

/**
 * @brief 速度求解器类型枚举
 * 
 */
typedef enum
{
    Velocity_Solver_M,
    Velocity_Solver_Estimator,
    Velocity_Solver_MAX
} Velocity_Solver_Type_t;

/**
 * @brief 编码器配置结构体
 * 
 */
typedef struct
{
    Encoder_Type_t encoder_type;
    Encoder_Struct_t encoder_struct;
    int encoder_delta_cpr;
    int encoder_abs_cpr;
    int encoder_baudrate;
    int encoder_single_turn_bits;
    int encoder_mult_turn_bits;
    int encoder_delta_dir;
    int encoder_abs_dir;

    Velocity_Solver_Type_t velocity_solver_type;
    float velocity_solver_band_width;
} Encoder_Config_t;

/**
 * @brief 编码器状态结构体
 * 
 */
typedef struct
{
    uint32_t encoder_delta_max_cnt;
    uint32_t encoder_delta_cnt;
    uint32_t encoder_delta_cnt_last;
    int32_t encoder_delta_turn;
    float encoder_delta_fractional;
    float encoder_delta_home_offset;
    float encoder_delta_pos;
    float encoder_delta_pos_last;

    uint32_t encoder_abs_max_cnt;
    uint32_t encoder_abs_cnt;
    uint32_t encoder_abs_cnt_last;
    int32_t encoder_abs_turn;
    float encoder_abs_fractional;
    float encoder_abs_zero_offset; // 零点偏置
    float encoder_abs_home_offset; // home偏置
    float encoder_abs_pos;
    float encoder_abs_pos_last;

    int32_t encoder_index_cnt;
    float encoder_pos;            // 编码器的首要位置,负责参与FOC控制.如自研编码器的增量值接口或多摩川编码器.
    float encoder_pos_fractional; // 编码器的首要单圈位置(0~1),负责参与FOC控制.如自研编码器的增量值接口或多摩川编码器.
    float encoder_pos_last;
    float encoder_speed;
    float encoder_speed_last;
    float encoder_acc;
    float encoder_acc_last;
    float encoder_pos_second;            // 编码器的次要位置,负责矫正首要位置.如自研编码器的绝对值接口.
    float encoder_pos_fractional_second; // 编码器的次要单圈位置(0~1),负责矫正首要单圈位置.如自研编码器的绝对值接口.

    /* not in od */
    float encoder_delta_cnt_inv;
    float encoder_abs_cnt_inv;
    uint32_t encoder_position_jump_error_cnt;
    volatile bool encoder_delta_position_valid;
    volatile bool encoder_abs_position_valid;
    volatile uint32_t encoder_special_function_flag;
    float delay_time;
} Encoder_Status_t;

/**
 * @brief 编码器实时更新
 * 
 * @param index 编码器编号
 */
void encoder_update_rt(int index);

/**
 * @brief 编码器高频实时更新
 * 
 * @param index 编码器编号
 */
void encoder_update_frt(int index);

/**
 * @brief 初始化编码器
 * 
 */
void encoder_init(void);

/**
 * @brief 复位编码器
 * 
 * @param index 编码器编号
 */
void encoder_reset(int index);

/**
 * @brief 复位编码器错误
 * 
 * @param index 编码器编号
 */
void encoder_set_reset_err(int index);

/**
 * @brief 复位编码器ABS
 * 
 * @param index 编码器编号
 */
void encoder_set_reset_abs(int index);

/**
 * @brief 复位编码器ABM
 * 
 * @param index 编码器编号
 */
void encoder_set_reset_abm(int index);

/**
 * @brief 注册编码器二级回调
 * 
 * @param index 编码器编号
 * @param handler 回调函数
 */
void encoder_register_index_handle(int index, encoder_index_callback handler);

/**
 * @brief 取消注册编码器二级回调
 * 
 * @param index 编码器编号
 */
void encoder_unregister_index_handle(int index);

/**
 * @brief 设置编码器零点偏置
 * 
 * @param index 编码器编号
 * @param offset 零点偏置
 */
void encoder_set_zero_offset(int index, float offset);

/**
 * @brief 获取编码器零点偏置
 * 
 * @param index 编码器编号
 * @return float 零点偏置
 */
float encoder_get_zero_offset(int index);

/**
 * @brief 设置编码器回零偏置
 * 
 * @param index 编码器编号
 * @param offset 位置偏置
 */
void encoder_set_home_offset(int index, float offset);

/**
 * @brief 设置编码器回零偏置
 * 
 * @param index 编码器编号
 * @return float 位置偏置
 */
float encoder_get_home_offset(int index);

/**
 * @brief 刷新编码器位置信息
 * 
 * @param index 编码器编号
 */
void encoder_refresh_pos(int index);

/**
 * @brief 将编码器位置向上或向下取整
 * 
 * @param index 编码器编号
 * @param dir 取整方向
 * @param home_offset 回零偏置
 */
void encoder_round_pos(int index, int dir, float home_offset);

/**
 * @brief 同步编码器的绝对位置和增量位置
 * 
 * @param index 编码器编号
 */
void encoder_sync_pos(int index);
