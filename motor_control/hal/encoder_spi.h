#pragma once

#include <stdint.h>

/**
 * @brief 编码器 SPI 同步交换一个 16 位数据帧，包含片选控制。
 * @param index 编码器编号；当前单电机板使用 SPI1。
 * @param tx 发送的数据帧。
 * @return 同一帧接收到的数据。
 * @note SPI 由板级初始化配置为 16 位模式。本接口阻塞等待传输完成，
 *       同一 SPI 总线的访问须由调用方串行化。
 */
uint16_t encoder_spi_transfer16(int index, uint16_t tx);
