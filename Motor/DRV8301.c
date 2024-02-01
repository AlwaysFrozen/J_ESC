#include "DRV8301.h"
#include "main.h"

extern SPI_HandleTypeDef hspi3;

uint16_t DRV8301_tx_data = 0x8800;
uint16_t DRV8301_rx_data = 0x00;
uint8_t transmit_en = 0;

void DRV8301_Transmit(void)
{
    if(transmit_en)
    {
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_RESET);
        HAL_SPI_TransmitReceive(&hspi3,(uint8_t *)&DRV8301_tx_data,(uint8_t *)&DRV8301_rx_data,1,1);
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_SET);
    }
}
