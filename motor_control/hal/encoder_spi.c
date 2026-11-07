#include "encoder_spi.h"
#include "main.h"

uint16_t encoder_spi_transfer16(int index, uint16_t tx)
{
    /* 当前板只有一路编码器，连接到 SPI1。 */
    (void)index;

    ENCODER_SPI_CS_GPIO_Port->BSRR = (uint32_t)ENCODER_SPI_CS_Pin << 16U;
    SPI1->DR = tx;
    while ((SPI1->SR & SPI_SR_RXNE) == 0U);
    uint16_t rx = SPI1->DR;
    while (SPI1->SR & SPI_SR_BSY);
    ENCODER_SPI_CS_GPIO_Port->BSRR = ENCODER_SPI_CS_Pin;

    return rx;
}
