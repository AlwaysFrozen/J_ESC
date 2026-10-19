/**
 * @file as5048a.h
 * @author jinchl (jcl709905683@gmail.com)
 * @brief 
 * @version 0.1
 * @date 2026-10-19
 * 
 * @copyright Copyright (c) 2026
 * 
 */

#pragma once
#include <stdint.h>
#include <stdbool.h>
#include "board.h"
#include "global.h"

#define AS5048A_MAX_VALUE    16383
#define AS5048A_ANGLE        0xffff
#define AS5048A_AGC          0x7ffd
#define AS5048A_MAG          0x7ffe
#define AS5048A_CLEAR        0x4001
#define AS5048A_NOP          0xc000

#define AS5048A_DATA_MSK     0x3fff
#define AS5048A_ERR_MSK      0x4000

typedef struct
{
    uint32_t abs;
    int wdg_cnt;
    bool err;
    bool wdg_err;
    bool pos_valid;
    bool err_reset_done;
} AS5048A_t;

extern AS5048A_t as5048a_encoer[BOARD_SUPPORTED_MOTOR_NUMBER];

/**
 * @brief AS5048a编码器初始化
 * 
 * @param index 编码器编号
 * @return true 初始化成功
 * @return false 初始化失败
 */
bool as5048a_encoder_init(int index);

/**
 * @brief AS5048a编码器读单圈绝对值
 * 
 * @param index 编码器编号
 */
void as5048a_encoder_read_single_turn(int index);

/**
 * @brief AS5048a编码器复位错误
 * 
 * @param index 编码器编号
 */
void as5048a_encoder_reset_error(int index);

static inline int as5048a_encoder_get_abs(int index)
{
    return as5048a_encoer[index].abs;
}

static inline int as5048a_encoder_get_error(int index)
{
    return as5048a_encoer[index].err;
}

static inline bool as5048a_encoder_get_wdg_error(int index)
{
    return as5048a_encoer[index].wdg_err;
}

static inline bool as5048a_encoder_get_pos_valid(int index)
{
    return as5048a_encoer[index].pos_valid;
}

static inline bool as5048a_encoder_get_error_reset_done(int index)
{
    return as5048a_encoer[index].err_reset_done;
}

static inline void as5048a_encoder_clear_error_reset_done(int index)
{
    as5048a_encoer[index].err_reset_done = false;
}