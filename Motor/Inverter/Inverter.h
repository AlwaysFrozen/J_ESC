#ifndef __INVERTER_H__
#define __INVERTER_H__

#include "stm32f4xx_hal.h"
#include "Motor.h"

typedef struct 
{
    //running status
    uint8_t is_running;

    //PWM
    uint32_t PWM_freq_now;
    uint16_t TIM1_ARR_now;

    //
    float angle;
    float angle_step;

    uint16_t CMP1;
    uint16_t CMP2;
    uint16_t CMP3;

    //err
    uint32_t err;
}Inverter_RUN_t;

typedef struct 
{
    float output;
    uint32_t carrier_freq;
    uint32_t fundamental_freq;
}Inverter_CONTROL_t;

extern Inverter_RUN_t inverter_run;
extern Inverter_CONTROL_t inverter_ctrl;

void Inverter_Process(void);
void Start_Inverter(void);
void Stop_Inverter(void);

#endif
