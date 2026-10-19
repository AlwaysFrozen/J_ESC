/**
 * @file tamagawa.c
 * @author jinchl (jcl709905683@gmail.com)
 * @brief 
 * @version 0.1
 * @date 2026-07-28
 * 
 * @copyright Copyright (c) 2026
 * 
 */

#include "tamagawa.h"

#include <string.h>


#define DEFAULT_WDG_VALUE       (3UL)
#define DEFAULT_RESET_ABS_VALUE (10UL)
#define DEFAULT_RESET_ABM_VALUE (10UL)

Tamagawa_t tamagawa_encoer[BOARD_SUPPORTED_MOTOR_NUMBER] = {0};

static inline uint8_t tamagawa_calc_crc(volatile uint8_t *buf, uint16_t len)
{
    uint8_t crc = 0;
    while (len--)
    {
        crc ^= buf[len];
    }
    return crc;
}

static inline int32_t tamagawa_calc_abs(uint32_t raw, uint32_t bits)
{
    if (bits > 24)
    {
        bits = 24;
    }
    else if (bits == 0)
    {
        return 0;
    }

    const uint32_t mask = (UINT32_C(1) << bits) - UINT32_C(1);

    raw &= mask;

    return raw;
}

static inline int32_t tamagawa_calc_abm(uint32_t raw, uint32_t bits)
{
    if (bits >= 24)
    {
        bits = 24;
    }
    else if (bits == 0)
    {
        return 0;
    }

    const uint32_t mask = (UINT32_C(1) << bits) - UINT32_C(1);
    const uint32_t sign = UINT32_C(1) << (bits - 1U);

    raw &= mask;

    return (int32_t)((raw ^ sign) - sign);
}

static inline void tamagawa_encoder_read_prepare(int index)
{
    if (tamagawa_encoer[index].wdg_cnt != DEFAULT_WDG_VALUE)
    {
        
    }

    if (--tamagawa_encoer[index].wdg_cnt < 0)
    {
        tamagawa_encoer[index].wdg_err   = true;
        tamagawa_encoer[index].pos_valid = false;
    }
}

void tamagawa_read_single_turn_callback(int index, volatile uint8_t *buf, int len)
{
    if (len == 6 && buf[0] == TAMAGAWA_CF_ID0)
    {
        if (tamagawa_calc_crc(buf, len - 1) == buf[len - 1])
        {
            tamagawa_encoer[index].wdg_cnt   = DEFAULT_WDG_VALUE;
            tamagawa_encoer[index].wdg_err   = false;
            tamagawa_encoer[index].pos_valid = true;
            tamagawa_encoer[index].cf        = buf[0];
            tamagawa_encoer[index].sf        = buf[1];
            uint32_t abs_raw                 = ((uint32_t)buf[2]) | ((uint32_t)buf[3] << 8) | ((uint32_t)buf[4] << 16);
            tamagawa_encoer[index].abs       = tamagawa_calc_abs(abs_raw, tamagawa_encoer[index].single_turn_bits);
        }
        else
        {
        }
    }
    else
    {
    }
}

void tamagawa_read_mult_turn_callback(int index, volatile uint8_t *buf, int len)
{
    if (len == 6 && buf[0] == TAMAGAWA_CF_ID1)
    {
        if (tamagawa_calc_crc(buf, len - 1) == buf[len - 1])
        {
            tamagawa_encoer[index].wdg_cnt   = DEFAULT_WDG_VALUE;
            tamagawa_encoer[index].wdg_err   = false;
            tamagawa_encoer[index].pos_valid = true;
            tamagawa_encoer[index].cf        = buf[0];
            tamagawa_encoer[index].sf        = buf[1];
            uint32_t abm_raw                 = buf[2] | buf[3] << 8 | buf[4] << 16;
            tamagawa_encoer[index].abm       = tamagawa_calc_abm(abm_raw, tamagawa_encoer[index].mult_turn_bits);
        }
        else
        {
        }
    }
    else
    {
    }
}

void tamagawa_read_all_callback(int index, volatile uint8_t *buf, int len)
{
    if (len == 11 && buf[0] == TAMAGAWA_CF_ID3)
    {
        if (tamagawa_calc_crc(buf, len - 1) == buf[len - 1])
        {
            tamagawa_encoer[index].wdg_cnt   = DEFAULT_WDG_VALUE;
            tamagawa_encoer[index].wdg_err   = false;
            tamagawa_encoer[index].pos_valid = true;
            tamagawa_encoer[index].cf        = buf[0];
            tamagawa_encoer[index].sf        = buf[1];
            uint32_t abs_raw                 = ((uint32_t)buf[2]) | ((uint32_t)buf[3] << 8) | ((uint32_t)buf[4] << 16);
            tamagawa_encoer[index].abs       = tamagawa_calc_abs(abs_raw, tamagawa_encoer[index].single_turn_bits);
            tamagawa_encoer[index].id        = buf[5];
            uint32_t abm_raw                 = ((uint32_t)buf[6]) | ((uint32_t)buf[7] << 8) | ((uint32_t)buf[8] << 16);
            tamagawa_encoer[index].abm       = tamagawa_calc_abm(abm_raw, tamagawa_encoer[index].mult_turn_bits);
            tamagawa_encoer[index].almc      = buf[9];
        }
        else
        {
        }
    }
    else
    {
    }
}

void tamagawa_read_reset_error_callback(int index, volatile uint8_t *buf, int len)
{
    if (len == 6 && buf[0] == TAMAGAWA_CF_ID7)
    {
        if (tamagawa_calc_crc(buf, len - 1) == buf[len - 1])
        {
            tamagawa_encoer[index].wdg_cnt        = DEFAULT_WDG_VALUE;
            tamagawa_encoer[index].wdg_err        = false;
            tamagawa_encoer[index].pos_valid      = false;
            tamagawa_encoer[index].err_reset_done = true;
            tamagawa_encoer[index].cf             = buf[0];
            tamagawa_encoer[index].sf             = buf[1];
            uint32_t abs_raw                      = ((uint32_t)buf[2]) | ((uint32_t)buf[3] << 8) | ((uint32_t)buf[4] << 16);
            tamagawa_encoer[index].abs            = tamagawa_calc_abs(abs_raw, tamagawa_encoer[index].single_turn_bits);
            tamagawa_encoer[index].almc           = 0;
        }
        else
        {
        }
    }
    else
    {
    }
}

void tamagawa_read_reset_abs_callback(int index, volatile uint8_t *buf, int len)
{
    if (len == 6 && buf[0] == TAMAGAWA_CF_ID8)
    {
        if (tamagawa_calc_crc(buf, len - 1) == buf[len - 1])
        {
            tamagawa_encoer[index].wdg_cnt           = DEFAULT_WDG_VALUE;
            tamagawa_encoer[index].wdg_err           = false;
            tamagawa_encoer[index].pos_valid         = false;
            tamagawa_encoer[index].reset_abs_rx_cnt += 1;
            if (tamagawa_encoer[index].reset_abs_rx_cnt >= DEFAULT_RESET_ABS_VALUE)
            {
                tamagawa_encoer[index].abs_reset_done = true;
            }
            tamagawa_encoer[index].cf  = buf[0];
            tamagawa_encoer[index].sf  = buf[1];
            uint32_t abs_raw           = ((uint32_t)buf[2]) | ((uint32_t)buf[3] << 8) | ((uint32_t)buf[4] << 16);
            tamagawa_encoer[index].abs = tamagawa_calc_abs(abs_raw, tamagawa_encoer[index].single_turn_bits);
        }
        else
        {
        }
    }
    else
    {
    }
}

void tamagawa_read_reset_abm_callback(int index, volatile uint8_t *buf, int len)
{
    if (len == 6 && buf[0] == TAMAGAWA_CF_IDC)
    {
        if (tamagawa_calc_crc(buf, len - 1) == buf[len - 1])
        {
            tamagawa_encoer[index].wdg_cnt           = DEFAULT_WDG_VALUE;
            tamagawa_encoer[index].wdg_err           = false;
            tamagawa_encoer[index].pos_valid         = false;
            tamagawa_encoer[index].reset_abm_rx_cnt += 1;
            if (tamagawa_encoer[index].reset_abm_rx_cnt >= DEFAULT_RESET_ABM_VALUE)
            {
                tamagawa_encoer[index].abm_reset_done = true;
            }
            tamagawa_encoer[index].cf  = buf[0];
            tamagawa_encoer[index].sf  = buf[1];
            uint32_t abs_raw           = ((uint32_t)buf[2]) | ((uint32_t)buf[3] << 8) | ((uint32_t)buf[4] << 16);
            tamagawa_encoer[index].abs = tamagawa_calc_abs(abs_raw, tamagawa_encoer[index].single_turn_bits);
        }
        else
        {
        }
    }
    else
    {
    }
}


/**
 * @brief 多摩川编码器初始化
 * 
 * @param index 编码器编号
 * @param baudrate 波特率
 * @param single_turn_bits 单圈位数
 * @param mult_turn_bits 多圈位数
 * @return true 初始化成功
 * @return false 初始化失败
 */
bool tamagawa_encoder_init(int index, int baudrate, int single_turn_bits, int mult_turn_bits)
{
    bool res = false;
    memset((void *)&tamagawa_encoer[index], 0, sizeof(Tamagawa_t));
    tamagawa_encoer[index].single_turn_bits = single_turn_bits;
    tamagawa_encoer[index].mult_turn_bits   = mult_turn_bits;
    tamagawa_encoer[index].wdg_cnt          = DEFAULT_WDG_VALUE;

    return res;
}

/**
 * @brief 多摩川编码器读单圈绝对值
 * 
 * @param index 编码器编号
 */
void tamagawa_encoder_read_single_turn(int index)
{
    tamagawa_encoder_read_prepare(index);
    tamagawa_encoer[index].cf = TAMAGAWA_CF_ID0;
}

/**
 * @brief 多摩川编码器读多圈绝对值
 * 
 * @param index 编码器编号
 */
void tamagawa_encoder_read_mult_turn(int index)
{
    tamagawa_encoder_read_prepare(index);
    tamagawa_encoer[index].cf = TAMAGAWA_CF_ID1;
}

/**
 * @brief 多摩川编码器读全部信息
 * 
 * @param index 编码器编号
 */
void tamagawa_encoder_read_all(int index)
{
    tamagawa_encoder_read_prepare(index);
    tamagawa_encoer[index].cf = TAMAGAWA_CF_ID3;
}

/**
 * @brief 多摩川编码器复位错误
 * 
 * @param index 编码器编号
 */
void tamagawa_encoder_reset_error(int index)
{
    tamagawa_encoder_read_prepare(index);
    tamagawa_encoer[index].err_reset_done = false;
    tamagawa_encoer[index].cf             = TAMAGAWA_CF_ID7;
}

/**
 * @brief 多摩川编码器复位ABS
 * 
 * @param index 编码器编号
 */
void tamagawa_encoder_reset_abs(int index)
{
    tamagawa_encoder_read_prepare(index);
    if (tamagawa_encoer[index].reset_abs_rx_cnt != tamagawa_encoer[index].reset_abs_tx_cnt)
    {
        tamagawa_encoer[index].reset_abs_rx_cnt = tamagawa_encoer[index].reset_abs_tx_cnt = 0;
    }
    tamagawa_encoer[index].reset_abs_tx_cnt += 1;
    tamagawa_encoer[index].cf                = TAMAGAWA_CF_ID8;
}

/**
 * @brief 多摩川编码器复位ABM
 * 
 * @param index 编码器编号
 */
void tamagawa_encoder_reset_abm(int index)
{
    tamagawa_encoder_read_prepare(index);
    if (tamagawa_encoer[index].reset_abm_rx_cnt != tamagawa_encoer[index].reset_abm_tx_cnt)
    {
        tamagawa_encoer[index].reset_abm_rx_cnt = tamagawa_encoer[index].reset_abm_tx_cnt = 0;
    }
    tamagawa_encoer[index].reset_abm_tx_cnt += 1;
    tamagawa_encoer[index].cf                = TAMAGAWA_CF_IDC;
}
