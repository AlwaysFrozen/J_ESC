#include "encoder_spi.h"
#include "main.h"
#include "board.h"

uint16_t encoder_spi_transfer16(int index, uint16_t tx)
{
    /* 当前板只有一路编码器，连接到 SPI1。 */
    (void)index;

    SPI1->CR1 |= SPI_CR1_SPE; /* HAL_SPI_Init configures but does not start SPI. */
    ENCODER_SPI_CS_GPIO_Port->BSRR = (uint32_t)ENCODER_SPI_CS_Pin << 16U;
    const uint32_t started = DWT->CYCCNT;
    unsigned budget = 256;
    SPI1->DR = tx;
    while ((SPI1->SR & SPI_SR_RXNE) == 0U) {
        if (--budget == 0 || DWT->CYCCNT - started > BOARD_CPU_FREQ / 100000U) goto failed;
    }
    uint16_t rx = SPI1->DR;
    while (SPI1->SR & SPI_SR_BSY) {
        if (--budget == 0 || DWT->CYCCNT - started > BOARD_CPU_FREQ / 100000U) goto failed;
    }
    ENCODER_SPI_CS_GPIO_Port->BSRR = ENCODER_SPI_CS_Pin;

    return rx;
failed:
    ENCODER_SPI_CS_GPIO_Port->BSRR = ENCODER_SPI_CS_Pin;
    /* Odd parity sentinel is rejected by the AS5048A driver. Never hang the
     * control ISR when a peripheral stops responding. */
    return 0x8000U;
}
