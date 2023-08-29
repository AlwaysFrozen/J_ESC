#ifndef AS5048A_H_
#define AS5048A_H_

#include "stdint.h"

#define AS5048_MAX_VALUE    16383
#define AS5048_ANGLE        0xffff
#define AS5048_AGC          0x7ffd
#define AS5048_MAG          0x7ffe
#define AS5048_CLEAR        0x4001
#define AS5048_NOP          0xc000

uint16_t AS5048a_Read_Raw(void);
void AS5048_Read_M_Ang(void);

#endif

