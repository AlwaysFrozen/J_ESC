/*
    对 DQ 轴磁场方向的定义并不会影响到传感器校准和电机转向。
    无论如何定义磁场方向与电流方向的关系（相同或相反），只要Q轴滞后D轴90°，那么给定Q轴电流的情况下电机一定逆时针旋转（CCW）。
*/

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

// Do not use this function in irq!!!
Sensor_Cali_Err_t Sensor_Calibration(float current_limit)
{
    Sensor_Cali_Err_t err = Sensor_Cali_Err_NONE;

    MOS_Driver_Disable();
    osDelay(10);

    err |= Voltage_Current_Sensor_Calibration();

    #if !FOC_DEBUG_HALL
    if((Motor_Config.moto_type == BLDC && (bldc_ctrl.sensor_type == HALL_120_SENSOR || bldc_ctrl.sensor_type == HALL_60_SENSOR)) ||
        (Motor_Config.moto_type == FOC && (foc_ctrl.sensor_type == HALL_120_SENSOR || foc_ctrl.sensor_type == HALL_60_SENSOR)))
    #endif
    {
        err |= HALL_Calibration(current_limit);
    }

    osDelay(100);

    #if !FOC_DEBUG_ENCODER
    if(Motor_Config.moto_type == FOC && foc_ctrl.sensor_type >= IIC_ENCODER)
    #endif
    {
        err |= Encoder_Calibration(current_limit);
    }


    return err;
}
