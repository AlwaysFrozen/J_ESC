#ifndef __DATA_TRANSMIT_H__
#define __DATA_TRANSMIT_H__

#include "stdbool.h"
#include "stdint.h"
#include "stdlib.h"
#include "string.h"

#define BUFF_SIZE   0xff
typedef struct 
{
    uint8_t buff[BUFF_SIZE];
    uint32_t head;
    uint32_t tail;
}Ring_Buff_t;


uint8_t IS_Buff_FUll(Ring_Buff_t *buff);
uint8_t IS_Buff_Empty(Ring_Buff_t *buff);
uint8_t Get_Buff_Len(Ring_Buff_t *buff);
uint8_t Push_Buff(Ring_Buff_t *buff,uint8_t *data,uint16_t len);
uint8_t Pop_Buff(Ring_Buff_t *buff,uint8_t *data,uint16_t len);
void Clear_Buff(Ring_Buff_t *buff);

uint32_t Buff_Add_Int32(uint8_t *buff,int32_t data);
uint32_t Buff_Add_Int16(uint8_t *buff,int16_t data);
uint32_t Buff_Add_Int8(uint8_t *buff,int8_t data);


#endif
