/**
 * @file as5048a.c
 * @author jinchl (jcl709905683@gmail.com)
 * @brief 
 * @version 0.1
 * @date 2026-10-19
 * 
 * @copyright Copyright (c) 2026
 * 
 */

#include "as5048a.h"
#include "encoder_spi.h"
#include <stdbool.h>
#include <string.h>

#define DEFAULT_WDG_VALUE       (3UL)

//https://ams.com/documents/20143/36005/AS5048_DS000298_4-00.pdf/910aef1f-6cd3-cbda-9d09-41f152104832
//bit15 Parity bit (EVEN) 
//bit14 1:read 0:write
//bit0~13 data or command

//0x0000    NOP
//0x0001    Clear Error Flag
//0x3FFE    Magnitude
//0x3FFF    Angle

AS5048A_t as5048a_encoer[BOARD_SUPPORTED_MOTOR_NUMBER] = {0};

static inline bool as5048a_read_raw(int index, uint16_t *data)
{
    bool sta = true;
    uint8_t parity_cnt = 0;
    uint16_t rx = encoder_spi_transfer16(index, AS5048A_ANGLE);

    *data = rx;

    // Parity check (EVEN)
    for(uint8_t i = 0;i < 16;i++)
    {
        if(rx & 1)
        {
            parity_cnt++;
        }
        rx >>= 1;
    }
    if(parity_cnt % 2)
    {
        sta = false;
    }

    return sta;
}

/**
 * @brief AS5048a编码器初始化
 * 
 * @param index 编码器编号
 * @return true 初始化成功
 * @return false 初始化失败
 */
bool as5048a_encoder_init(int index)
{
    bool res = true;
    memset((void *)&as5048a_encoer[index], 0, sizeof(AS5048A_t));
    as5048a_encoer[index].wdg_cnt          = DEFAULT_WDG_VALUE;

    return res;
}

/**
 * @brief AS5048a编码器读单圈绝对值
 * 
 * @param index 编码器编号
 */
void as5048a_encoder_read_single_turn(int index)
{
    uint16_t data = 0;
    bool valid = as5048a_read_raw(index, &data);
    if (!as5048a_encoer[index].primed) {
        as5048a_encoer[index].primed = true;
        return; /* SPI response is for the preceding command, not this one. */
    }
    if (valid)
    {
        // check err
        if(data & AS5048A_ERR_MSK)
        {
            as5048a_encoer[index].err = true;
            as5048a_encoer[index].pos_valid = false;
        }
        else
        {
            as5048a_encoer[index].err = false;
            as5048a_encoer[index].err_reset_done = true;
            as5048a_encoer[index].pos_valid = true;
            as5048a_encoer[index].abs = data & AS5048A_DATA_MSK;
        }

        as5048a_encoer[index].wdg_cnt = DEFAULT_WDG_VALUE;
        as5048a_encoer[index].wdg_err = false;
    }
    else
    {
        as5048a_encoer[index].pos_valid = false;
        if (as5048a_encoer[index].wdg_cnt > 0) --as5048a_encoer[index].wdg_cnt;
        else
        {
            as5048a_encoer[index].wdg_err = true;
            as5048a_encoer[index].pos_valid = false;
        }
    }
}

/**
 * @brief AS5048a编码器复位错误
 * 
 * @param index 编码器编号
 */
void as5048a_encoder_reset_error(int index)
{
    as5048a_encoer[index].err_reset_done = false;
    (void)encoder_spi_transfer16(index, AS5048A_CLEAR);
    /* Fetch the clear-error register response while requesting the next angle.
     * The following regular transfer therefore has a typed angle response. */
    uint16_t flags = encoder_spi_transfer16(index, AS5048A_ANGLE);
    as5048a_encoer[index].last_error_flags = flags & 7U;
    as5048a_encoer[index].primed = true;
    as5048a_encoer[index].pos_valid = false;
}


