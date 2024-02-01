#ifndef __VIS_H__
#define __VIS_H__

#include "stm32f4xx_hal.h"
#include "Motor.h"
#include "Sensor.h"

typedef enum
{
    VIS_Stop,
    VIS_Run,
}VIS_Run_State_t;

typedef struct 
{
    /* running status */
    VIS_Run_State_t run_state;
    /* delay */
    uint32_t delay_Cnt;
    uint32_t delay_Cnt_Ref;
    /* step process */
    uint8_t step_index;
    uint8_t step_index_now;
    uint8_t step_index_next;
    uint32_t loop_num_cnt;
    /**/
    uint8_t VIS_credible;
    uint8_t VIS_index;
    float VIS_e_angle;
    /* PWM */
    uint32_t PWM_freq_now;
    uint16_t TIM1_ARR_now;
    uint16_t duty;
    /* err */
    uint32_t err;
}VIS_RUN_t;

typedef struct 
{
    /* PWM */
    uint32_t PWM_freq;
    /* output config */
    float output_min;
    float output_max;
    float output;
    /* step process */
    uint32_t step_us;
    uint32_t loop_num;
}VIS_CONTROL_t;

extern VIS_RUN_t vis_run;
extern VIS_CONTROL_t  vis_ctrl;

void Vector_0(uint16_t ARR_value,uint16_t CCR_value);
void Vector_1(uint16_t ARR_value,uint16_t CCR_value);
void Vector_2(uint16_t ARR_value,uint16_t CCR_value);
void Vector_3(uint16_t ARR_value,uint16_t CCR_value);
void Vector_4(uint16_t ARR_value,uint16_t CCR_value);
void Vector_5(uint16_t ARR_value,uint16_t CCR_value);
void Vector_6(uint16_t ARR_value,uint16_t CCR_value);


void VIS_Process(void);
void Clear_Vis(void);
void Start_VIS(void);
void Stop_VIS(void);
void Init_VIS(void);

#endif
