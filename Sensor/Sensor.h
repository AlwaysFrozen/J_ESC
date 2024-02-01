#ifndef __SENSOR_H__
#define __SENSOR_H__

#include "stm32f4xx_hal.h"

#include "stdbool.h"
#include "stdint.h"
#include "stdlib.h"
#include "string.h"
#include "My_Math.h"
#include "math.h"



typedef enum
{
    SENSOR_LESS,
    SENSOR_LESS_VIS,
    SENSOR_LESS_HFI,
    HALL_120_SENSOR,
    HALL_60_SENSOR,
    IIC_ENCODER,
    SPI_ENCODER,
    ABI_ENCODER
}SENSOR_Type_t;



typedef enum
{
    Sensor_Calibrating,
    Sensor_Calibrated,
    Sensor_CalibrateFail,
}Sensor_Cali_t;

typedef enum
{
    Sensor_Cali_Err_NONE                 = 0x00000000,
    //ADC
    Sensor_Cali_Err_ADC_V_IDLE           = 0x00000001,
    Sensor_Cali_Err_ADC_I_IDLE           = 0x00000002,
    Sensor_Cali_Err_PHASE_V_ORDER        = 0x00000004,
    Sensor_Cali_Err_PHASE_I_ORDER        = 0x00000008,
    //HALL
    Sensor_Cali_Err_HALL_ALIGN           = 0x00000100,
    Sensor_Cali_Err_HALL_TYPE            = 0x00000200,
    Sensor_Cali_Err_HALL_SEQ             = 0x00000400,
    //Encoder
    Sensor_Cali_Err_NO_MOVE              = 0x00010000,
    Sensor_Cali_Err_ALIGN                = 0x00020000,
    Sensor_Cali_Err_DIRECTION            = 0x00040000,
}Sensor_Cali_Err_t;

extern Sensor_Cali_t sensor_calibration_status;
extern Sensor_Cali_Err_t sensor_calibration_err;
extern float sensor_calibration_current;

Sensor_Cali_Err_t Sensor_Calibration(float current_limit);

#endif
