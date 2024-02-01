#include "Board_Config.h"

#include "Sensor.h"
#include "VoltageCurrentSensor.h"
#include "HALL.h"
#include "Encoder.h"

#include "Motor.h"
#include "BLDC_Motor.h"
#include "FOC_Motor.h"
#include "FOC.h"
#include "FOC_Config.h"

#include "cmsis_os.h"


Sensor_Cali_t sensor_calibration_status = Sensor_Calibrating;
Sensor_Cali_Err_t sensor_calibration_err = Sensor_Cali_Err_NONE;

float sensor_calibration_current = 3;

// Do not use this function in irq!!!
Sensor_Cali_Err_t Sensor_Calibration(float current_limit)
{
    Sensor_Cali_Err_t err = Sensor_Cali_Err_NONE;

    err |= Voltage_Current_Sensor_Calibration();

    #if !FOC_DEBUG_HALL
    if((Motor_Config.moto_type == BLDC && (bldc_ctrl.sensor_type == HALL_120_SENSOR || bldc_ctrl.sensor_type == HALL_60_SENSOR)) ||
        (Motor_Config.moto_type == FOC && (foc_ctrl.sensor_type == HALL_120_SENSOR || foc_ctrl.sensor_type == HALL_60_SENSOR)))
    #endif
    {
        err |= HALL_Calibration(current_limit);
    }

    osDelay(1000);

    #if !FOC_DEBUG_ENCODER
    if(Motor_Config.moto_type == FOC && foc_ctrl.sensor_type >= IIC_ENCODER)
    #endif
    {
        err |= Encoder_Calibration(current_limit);
    }


    return err;
}
