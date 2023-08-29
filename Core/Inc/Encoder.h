#ifndef __ENCODER_H__
#define __ENCODER_H__

#include "stm32f4xx_hal.h"

#include "stdbool.h"
#include "stdint.h"
#include "stdlib.h"
#include "string.h"
#include "My_Math.h"
#include "math.h"

#include "Sensor.h"

//Encoder
typedef struct 
{
    uint16_t bits;
    uint8_t reverse;
    uint16_t raw_value;

    float m_angle_offset;
    float m_angle;
    float m_angle_last;
    float m_angle_delta;
    float m_angle_calibrated;
    float m_rpm;
    float m_rpm_f;

    float e_angle;
    float e_rpm;
    float e_rpm_f;
}Encoder_Para_t;

extern Encoder_Para_t AS5048_para;

Sensor_Cali_Err_t Encoder_Calibration(float current_limit);

#endif
