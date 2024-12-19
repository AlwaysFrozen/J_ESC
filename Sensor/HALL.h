#ifndef __HALL_H__
#define __HALL_H__

#include "stm32f4xx_hal.h"

#include "stdbool.h"
#include "stdint.h"
#include "stdlib.h"
#include "string.h"
#include "My_Math.h"
#include "math.h"

#include "Sensor.h"

//Hall Sensor
#define HALL_A_READ()   ((GPIOC->IDR & GPIO_PIN_6) != GPIO_PIN_RESET)
#define HALL_B_READ()   ((GPIOC->IDR & GPIO_PIN_7) != GPIO_PIN_RESET)
#define HALL_C_READ()   ((GPIOC->IDR & GPIO_PIN_8) != GPIO_PIN_RESET)

typedef struct 
{
    uint32_t hall_queue[3];
    uint32_t hall_queue_f[3];
    uint32_t hall_filter_len;
    uint32_t hall_filter_value;
    uint32_t hall_filter_res[3];

    uint8_t hall_index;
    uint8_t hall_index_last;
    uint32_t hall_sector_cnt;
    uint32_t hall_sector_cnt_last;

    float e_angle;
    float e_angle_delta;
    float e_angle_observe;
    float e_speed;
    float e_speed_f;
    float e_rpm;

    //err
    uint32_t err;
}HALL_Sensor_t;

extern HALL_Sensor_t Hall_Sensor;


extern float hall_index_calibrated[8];
extern float hall_deg_calibrated[8];
extern float hall_rad_calibrated[8];

void Hall_Sensor_Init(void);
void Update_Hall_Filter_Value(uint8_t len);
Sensor_Cali_Err_t HALL_Calibration(float current_limit);

#endif
