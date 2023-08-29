#ifndef __MOTOR_H__
#define __MOTOR_H__

#include "stm32f4xx_hal.h"
#include "stm32f4xx_it.h"
#include "stdbool.h"
#include "stdint.h"
#include "stdlib.h"
#include "string.h"
#include "math.h"
#include "My_Math.h"
#include "main.h"

#define PWM_TIM_BASE_FREQ       (168000000U)

#define MAX_PHASE_CURRENT       (16000)

#define PHASE_VOLTAGE_RATIO     (15.091575f)    // adc_value / 4095 * 3300 / (2.2 / (2.2 + 39)) mv
#define PHASE_CURRENT_RATIO     (8.0586081f)    // (adc_value - 2048) / 4095 * 3300 / 20 / 0.005 ma     //5m ohm

#define MIN_LPF_FC              (200)
#define MAX_LPF_FC              (5000)

typedef enum
{
    BLDC,
    FOC,
    DC,
    Single_Phase_Inverter,
    Three_Phase_Inverter,
}MOTOR_Type_t;

// typedef enum
// {
//     CCW = 0,
//     CW
// }Direction_t;
#define CCW     (1)
#define CW      (-1)
typedef int8_t Direction_t;

typedef struct 
{
    MOTOR_Type_t moto_type;
    Direction_t dir;
    uint8_t reverse;
}MOTOR_Config_t;

typedef enum
{
    MS_IDLE,
    MS_Start,
    MS_Run,
    MS_Break,
    MS_Stop,
    MS_Protect,
}Motor_State_t;

typedef struct
{
    uint8_t is_running;
    Motor_State_t state;
    uint32_t sample_freq_now;
    float electronic_speed_hz;
    uint32_t V_bus_mv;
    uint32_t V_bus_mv_f;
}Virtual_Motor_t;

typedef struct 
{
    float phase_voltage_filter_rate;
    float phase_current_filter_rate;
    float bus_voltage_filter_rate;
    float bus_current_filter_rate;
    float RPM_filter_rate;
}Filter_Rate_t;

extern MOTOR_Config_t Moto_Config;
extern Filter_Rate_t Filter_Rate;
extern Virtual_Motor_t Virtual_Moto;

void MOS_Driver_Enable(void);
void MOS_Driver_Disable(void);
void Init_Motor(void);
void Stop_Motor(void);
void Start_Motor(void);
bool Motor_Is_Running(void);
bool Motor_Is_Rotating(void);

#endif

