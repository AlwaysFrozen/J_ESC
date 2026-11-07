#pragma once

#include <stdbool.h>
#include <stdint.h>

typedef void (*Encoder_RS485_Receive_Callback_t)(int index, volatile uint8_t *buf, int len);

/**
 * @brief 配置编码器 RS485 的 UART 波特率、收发模式及 DE 引脚。
 * @return 初始化是否成功；当前板未配置 UART/DE，返回 false。
 */
bool encoder_rs485_init(int index, int baudrate);

/**
 * @brief 中止未完成的收发，清除接收状态并释放 DE 至接收方向。
 * @note 返回后，旧事务不得再访问缓冲区或触发回调。
 */
void encoder_rs485_abort(int index);

/**
 * @brief 启动异步请求/应答事务，由 HAL 层管理 UART 收发和 DE 切换。
 * @param tx_buf 请求帧，发送完成前必须保持有效且内容不变。
 * @param tx_len 请求帧字节数。
 * @param rx_buf 应答缓冲区，容量至少为 rx_len，事务完成/中止前必须有效。
 * @param rx_len 预期应答帧字节数。
 * @param callback 收到完整应答后回调，长度为实际接收字节数；协议校验由调用方负责。
 * @return 是否成功启动；false 时不访问缓冲区、不触发回调。
 * @note 后端须在请求末位发送完成后释放 DE，并及时接收应答；事务失败时
 *       不得伪造应答，由编码器层看门狗处理超时。同一通道的访问须串行化。
 *       当前板未配置 UART/DE，实际操作留空并返回 false。
 */
bool encoder_rs485_transfer(int index, const uint8_t *tx_buf, uint16_t tx_len,
                           uint8_t *rx_buf, uint16_t rx_len,
                           Encoder_RS485_Receive_Callback_t callback);
