#include "encoder_rs485.h"

bool encoder_rs485_init(int index, int baudrate)
{
    (void)index;
    (void)baudrate;
    /* 当前板未配置编码器 UART 和 DE，硬件初始化留空。 */
    return false;
}

void encoder_rs485_abort(int index)
{
    (void)index;
    /* 待适配：停止 UART/DMA 收发，清除接收状态并释放 DE。 */
}

bool encoder_rs485_transfer(int index, const uint8_t *tx_buf, uint16_t tx_len,
                           uint8_t *rx_buf, uint16_t rx_len,
                           Encoder_RS485_Receive_Callback_t callback)
{
    (void)index;
    (void)tx_buf;
    (void)tx_len;
    (void)rx_buf;
    (void)rx_len;
    (void)callback;
    /* 当前板无 UART/DE 配置，启动接收、请求发送及 DE 控制留空。 */
    return false;
}
