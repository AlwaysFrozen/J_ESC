#include "DataTransmit.h"

uint8_t IS_Buff_FUll(Ring_Buff_t *buff)
{
    return (buff->tail + 1) % BUFF_SIZE == buff->head;
}

uint8_t IS_Buff_Empty(Ring_Buff_t *buff)
{
    return buff->head == buff->tail;
}

uint8_t Get_Buff_Len(Ring_Buff_t *buff)
{
    return (buff->tail + BUFF_SIZE - buff->head) % BUFF_SIZE;
}

uint8_t Push_Buff(Ring_Buff_t *buff,uint8_t *data,uint16_t len)
{
	for(uint16_t i = 0;i < len;i++)
	{
		if (IS_Buff_FUll(buff)) 
		{
			return 0;
		}
		buff->buff[buff->tail] = data[i];
		buff->tail = (buff->tail + 1) % BUFF_SIZE;
	}
    return 1;
}

uint8_t Pop_Buff(Ring_Buff_t *buff,uint8_t *data,uint16_t len)
{
	for(uint16_t i = 0;i < len;i++)
	{
		if (IS_Buff_Empty(buff)) 
		{
			return 0;
		}
		data[i] = buff->buff[buff->head];
		buff->buff[buff->head] = 0;
		buff->head = (buff->head + 1) % BUFF_SIZE;
	}
    return 1;
}

void Clear_Buff(Ring_Buff_t *buff)
{
    memset(buff,0,sizeof(Ring_Buff_t));
}

uint32_t Buff_Add_Int32(uint8_t *buff,int32_t data)
{
    buff[0] = data;
    buff[1] = data >> 8;
    buff[2] = data >> 16;
    buff[3] = data >> 24;

    return 4;
}

uint32_t Buff_Add_Int16(uint8_t *buff,int16_t data)
{
    buff[0] = data;
    buff[1] = data >> 8;

    return 2;
}

uint32_t Buff_Add_Int8(uint8_t *buff,int8_t data)
{
    buff[0] = data;

    return 1;
}
