#include "AS5048a.h"
#include "FOC.h"
#include "Sensor.h"
#include "Encoder.h"


//https://ams.com/documents/20143/36005/AS5048_DS000298_4-00.pdf/910aef1f-6cd3-cbda-9d09-41f152104832
//bit15 Parity bit (EVEN) 
//bit14 1:read 0:write
//bit0~13 data or command

//0x0000    NOP
//0x0001    Clear Error Flag
//0x3FFE    Magnitude
//0x3FFF    Angle
extern SPI_HandleTypeDef hspi1;
uint16_t AS5048a_Read_Raw(void)
{
    uint16_t tx = 0xffff,rx = 0;

    // HAL_GPIO_WritePin(GPIOA,GPIO_PIN_4,GPIO_PIN_RESET);
    // HAL_SPI_TransmitReceive(&hspi1,(uint8_t *)&tx,(uint8_t *)&rx,1,1);
    // HAL_GPIO_WritePin(GPIOA,GPIO_PIN_4,GPIO_PIN_SET);
    
    GPIOA->BSRR = (uint32_t)GPIO_PIN_4 << 16U;
    // send
    SPI1->DR = tx;
    // receive
    rx = SPI1->DR;
    // wait until transmit end
    while(SPI1->SR & 0x80);
    GPIOA->BSRR = GPIO_PIN_4;

    return rx & 0x3fff;
}

void AS5048_Read_M_Ang(void)
{
    AS5048_para.raw_value = AS5048a_Read_Raw();
    AS5048_para.m_angle = Normalize_Angle((float)AS5048_para.raw_value / AS5048_MAX_VALUE * _2PI - AS5048_para.m_angle_offset);
    if (AS5048_para.reverse)
    {
        AS5048_para.m_angle = _2PI - AS5048_para.m_angle;
    }
}
