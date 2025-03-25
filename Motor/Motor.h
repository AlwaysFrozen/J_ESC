#ifndef __MOTOR_H__
#define __MOTOR_H__

#include "stm32f4xx_hal.h"
#include "stm32f4xx_it.h"
#include "stdbool.h"
#include "stdint.h"
#include "stdlib.h"
#include "string.h"
#include "main.h"

#include "usbd_cdc_if.h"


#define PWM_TIM_BASE_FREQ           (168000000U)
#define PWN_DEAD_TIME_NS            (250)
#define BOARD_MAX_PHASE_CURRENT     (100)

#define BUS_VOLTAGE_RATIO           (0.015091575f) //adc_value / 4095 * 3300 / (2.2 / (2.2 + 39)) mv
#define PHASE_VOLTAGE_V_RATIO       (0.015091575f) //adc_value / 4095 * 3300 / (2.2 / (2.2 + 39)) / 1000 v
#define PHASE_CURRENT_A_RATIO       (0.080586081f)  //(adc_value - 2048) / 4095 * 3300 / 20 / 0.0005 / 1000 a     //0.5m ohm
// #define PHASE_CURRENT_A_RATIO       (0.0080586081f)  //(adc_value - 2048) / 4095 * 3300 / 20 / 0.005 / 1000 a     //5m ohm
// #define PHASE_CURRENT_A_RATIO       (0.002014652f)  //(adc_value - 2048) / 4095 * 3300 / 20 / 0.02 / 1000 a        //20m ohm

#define MIN_LPF_FC                  (200)
#define MAX_LPF_FC                  (20000)

typedef enum
{
    BLDC,
    FOC,
    DC,
    Single_Phase_Inverter,
    Three_Phase_Inverter,
    VIS,
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

typedef enum
{
    Quad_0,
    Quad_1,
    Quad_2,
    Quad_3,
    Quad_4,
}Quadrant_t;

typedef struct
{
    float U;
    float V;
    float W;
}UVW_Axis_t;

typedef struct
{
    float Alpha;
    float Beta;
}AB_Axis_t;

typedef struct
{
    float D;
    float Q;
}DQ_Axis_t;

typedef struct
{
    uint32_t Reload;
    uint32_t CMP1;
    uint32_t CMP2;
    uint32_t CMP3;
}TIM_t;

typedef struct
{
    uint8_t is_running;
    Motor_State_t state;
    Quadrant_t run_quadrant;
    float dt;
    float electronic_speed_hz;
    float V_bus_v;
    float V_bus_v_f;
}Virtual_Motor_t;

typedef struct 
{
    float phase_voltage_filter_rate;
    float phase_current_filter_rate;
    float bus_voltage_filter_rate;
    float bus_current_filter_rate;
    float RPM_filter_rate;
}Filter_Rate_t;

extern MOTOR_Config_t Motor_Config;
extern MOTOR_Type_t motor_type_now;

extern Filter_Rate_t Filter_Rate;
extern Virtual_Motor_t Virtual_Moto;

extern uint16_t timer_cnt_arr[10];

void MOS_Driver_Enable(void);
void MOS_Driver_Disable(void);
void Init_Motor(void);
void Stop_Motor(void);
void Start_Motor(void);
bool Motor_Is_Running(void);
bool Motor_Is_Rotating(void);

#endif

