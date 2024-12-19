#ifndef __VOLTAGECURRENTSENSOR_H__
#define __VOLTAGECURRENTSENSOR_H__

#include "stm32f4xx_hal.h"

#include "stdbool.h"
#include "stdint.h"
#include "stdlib.h"
#include "string.h"
#include "My_Math.h"
#include "math.h"

#include "Sensor.h"

typedef struct
{
  uint8_t ADC_Cali_Cnt;
  float ADC_Cali_Value[6];
}ADC_Cali_t;

extern ADC_Cali_t adc_cali;
extern uint16_t Vbus_ADC;
extern float adc_value[6];
extern UVW_Axis_t phase_voltage_V;
extern UVW_Axis_t phase_voltage_V_f;
extern UVW_Axis_t phase_current_A;
extern UVW_Axis_t phase_current_A_f;
extern UVW_Axis_t *p_phase_voltage_V;
extern UVW_Axis_t *p_phase_current_A;

Sensor_Cali_Err_t Voltage_Current_Sensor_Calibration(void);

#endif
