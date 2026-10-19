/**
 * @file hcfa.h
 * @author jinchl (jcl709905683@gmail.com)
 * @brief 
 * @version 0.1
 * @date 2026-07-28
 * 
 * @copyright Copyright (c) 2026
 * 
 */


#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "board.h"

#define HCFA_CF_ID0 (0x02)
#define HCFA_CF_ID1 (0x8A)
#define HCFA_CF_ID2 (0x92)
#define HCFA_CF_ID3 (0x1A)
#define HCFA_CF_ID7 (0xBA)
#define HCFA_CF_ID8 (0xC2)
#define HCFA_CF_IDC (0x62)

/*
|         位 | 名称        | 含义                    |
| --------: | --------- | --------------------- |
| bit0～bit3 | `dd0～dd3` | 保留信息位，正常固定为 0         |
|      bit4 | `ea0`     | 计数错误 `Counting error` |
|      bit5 | `ea1`     | 编码器综合错误               |
|      bit6 | `ca0`     | 请求帧 Data ID 奇偶校验错误    |
|      bit7 | `ca1`     | 请求帧结束位错误              |
*/
#define HCFA_SF_EA0_MASK  (UINT8_C(1) << 4)
#define HCFA_SF_EA1_MASK  (UINT8_C(1) << 5)
#define HCFA_SF_CA0_MASK  (UINT8_C(1) << 6)
#define HCFA_SF_CA1_MASK  (UINT8_C(1) << 7)
#define HCFA_SF_INFO_MASK UINT8_C(0x0F)

/*
|    位 | 符号   | 错误                           |
| ---: | ---- | ---------------------------- |
| bit0 | `OS` | 超速错误                         |
| bit1 | `RS` | 原始信号错误 |
| bit2 | `MT` | 多圈错误                         |
| bit3 | `PE` | 位置错误                      |
| bit4 | `ME` | 多圈未使能                           |
| bit5 | `EP` | EEPROM报警                         |
| bit6 | `OH` | 过热                         |
| bit7 | `BE` | 电池报警                         |
*/
#define HCFA_ALMC_OS_MASK (UINT8_C(1) << 0)
#define HCFA_ALMC_RS_MASK (UINT8_C(1) << 1)
#define HCFA_ALMC_MT_MASK (UINT8_C(1) << 2)
#define HCFA_ALMC_PE_MASK (UINT8_C(1) << 3)
#define HCFA_ALMC_ME_MASK (UINT8_C(1) << 4)
#define HCFA_ALMC_EP_MASK (UINT8_C(1) << 5)
#define HCFA_ALMC_OH_MASK (UINT8_C(1) << 6)
#define HCFA_ALMC_BE_MASK (UINT8_C(1) << 7)
#define HCFA_ALMC_MASK    (HCFA_ALMC_OS_MASK | HCFA_ALMC_RS_MASK | HCFA_ALMC_MT_MASK | HCFA_ALMC_PE_MASK | HCFA_ALMC_EP_MASK | HCFA_ALMC_OH_MASK)
#define HCFA_BATT_MASK    (HCFA_ALMC_BE_MASK)

typedef struct
{
    uint8_t cf;
    uint8_t sf;
    uint32_t abs;
    int32_t abm;
    uint8_t id;
    uint8_t almc;
    uint8_t rx_buf[11];
    uint32_t single_turn_bits;
    uint32_t mult_turn_bits;
    int wdg_cnt;
    int reset_abs_tx_cnt;
    int reset_abs_rx_cnt;
    int reset_abm_tx_cnt;
    int reset_abm_rx_cnt;
    bool wdg_err;
    bool pos_valid;
    bool err_reset_done;
    bool abs_reset_done;
    bool abm_reset_done;
} HCFA_t;

extern HCFA_t hcfa_encoer[BOARD_SUPPORTED_MOTOR_NUMBER];

/**
 * @brief 禾川编码器初始化
 * 
 * @param index 编码器编号
 * @param baudrate 波特率
 * @param single_turn_bits 单圈位数
 * @param mult_turn_bits 多圈位数
 * @return true 初始化成功
 * @return false 初始化失败
 */
bool hcfa_encoder_init(int index, int baudrate, int single_turn_bits, int mult_turn_bits);

/**
 * @brief 禾川编码器读单圈绝对值
 * 
 * @param index 编码器编号
 */
void hcfa_encoder_read_single_turn(int index);

/**
 * @brief 禾川编码器读多圈绝对值
 * 
 * @param index 编码器编号
 */
void hcfa_encoder_read_mult_turn(int index);

/**
 * @brief 禾川编码器读全部信息
 * 
 * @param index 编码器编号
 */
void hcfa_encoder_read_all(int index);

/**
 * @brief 禾川编码器复位错误
 * 
 * @param index 编码器编号
 */
void hcfa_encoder_reset_error(int index);

/**
 * @brief 禾川编码器复位ABS
 * 
 * @param index 编码器编号
 */
void hcfa_encoder_reset_abs(int index);

/**
 * @brief 禾川编码器复位ABM
 * 
 * @param index 编码器编号
 */
void hcfa_encoder_reset_abm(int index);

static inline int hcfa_encoder_get_abs(int index)
{
    return hcfa_encoer[index].abs;
}

static inline int hcfa_encoder_get_abm(int index)
{
    return hcfa_encoer[index].abm;
}

static inline int hcfa_encoder_get_id(int index)
{
    return hcfa_encoer[index].id;
}

static inline int hcfa_encoder_get_almc(int index)
{
    return hcfa_encoer[index].almc;
}

static inline int hcfa_encoder_get_error(int index)
{
    return (hcfa_encoer[index].sf & 0x30) >> 4;
}

static inline int hcfa_encoder_get_alarm(int index)
{
    return (hcfa_encoer[index].sf & 0xC0) >> 6;
}

static inline int hcfa_encoder_get_sf_error(int index)
{
    return (hcfa_encoer[index].sf & 0xF0) >> 4;
}

static inline bool hcfa_encoder_get_wdg_error(int index)
{
    return hcfa_encoer[index].wdg_err;
}

static inline bool hcfa_encoder_get_pos_valid(int index)
{
    return hcfa_encoer[index].pos_valid;
}

static inline bool hcfa_encoder_get_error_reset_done(int index)
{
    return hcfa_encoer[index].err_reset_done;
}

static inline void hcfa_encoder_clear_error_reset_done(int index)
{
    hcfa_encoer[index].err_reset_done = false;
}

static inline bool hcfa_encoder_get_abs_reset_done(int index)
{
    return hcfa_encoer[index].abs_reset_done;
}

static inline void hcfa_encoder_clear_abs_reset_done(int index)
{
    hcfa_encoer[index].abs_reset_done   = false;
    hcfa_encoer[index].reset_abs_tx_cnt = 0;
    hcfa_encoer[index].reset_abs_rx_cnt = 0;
}

static inline bool hcfa_encoder_get_abm_reset_done(int index)
{
    return hcfa_encoer[index].abm_reset_done;
}

static inline void hcfa_encoder_clear_abm_reset_done(int index)
{
    hcfa_encoer[index].abm_reset_done   = false;
    hcfa_encoer[index].reset_abm_tx_cnt = 0;
    hcfa_encoer[index].reset_abm_rx_cnt = 0;
}

