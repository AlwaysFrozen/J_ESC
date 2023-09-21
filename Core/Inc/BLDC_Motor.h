#ifndef __BLDC_MOTOR_H__
#define __BLDC_MOTOR_H__

#include "stm32f4xx_hal.h"
#include "BLDC_Config.h"
#include "Motor.h"
#include "Sensor.h"

typedef enum
{
    BLDC_Stop,
    BLDC_StartUp,
    BLDC_Run,
    BLDC_Brake,
    BLDC_FullBrake,
    BLDC_PlugBrake,
}BLDC_Run_State_t;

typedef enum
{
    BLDC_Start_Order,
    BLDC_Start_Order_Delay,
    BLDC_Start_First_Step,
    BLDC_Start_Step_Delay,
    BLDC_Start_Step,
    BLDC_Start_End,
}BLDC_Start_State_t;

typedef enum
{
    BLDC_VVVF_None,
    BLDC_VVVF_VF,
    BLDC_VVVF_FF,
    BLDC_VVVF_VFF,
}BLDC_VVVF_Method_t;

typedef struct 
{
    /* running status */
    BLDC_Run_State_t run_state;
    /* start process */
    BLDC_Start_State_t start_seq;
    uint8_t start_index;
    uint8_t start_step_index;
    uint16_t start_step_cnt;
    /* delay */
    uint32_t delay_Cnt;
    uint32_t delay_Cnt_Ref;
    /* step process */
    uint8_t befm_index;
    uint8_t befm_index_last;
    uint8_t step_index;
    uint8_t step_index_last;
    /* befm process */
    float virtual_mid;
    float virtual_mid_f;
    uint32_t befm_queue[3];
    uint32_t befm_queue_f[3];
    uint32_t befm_filter_len;
    uint32_t befm_filter_value;
    uint32_t befm_filter_res[3];
    /* delay angle and speed */
    uint32_t phase_cnt;
    uint32_t phase_cnt_last;
    uint32_t phase_cnt_f;
    uint32_t delay_angle_cnt_ref;
    uint32_t delay_angle_cnt;
    uint32_t period_cnt;
    /* speed */
    Direction_t dir;
    int32_t eletrical_rpm;
    int32_t eletrical_rpm_abs;
    int32_t machine_rpm;
    int32_t machine_rpm_abs;
    /* step time */
    uint8_t step_list[6];
    uint8_t step_cnt;
    int32_t step_time_cnt[6];
    int32_t step_time_sum;
    int32_t step_time_avg;
    int32_t step_time_mse;
    /* vol & cur */
    float I_bus_ma;
    float I_bus_ma_f;
    /* PWM */
    uint32_t PWM_freq_now;
    uint16_t TIM1_ARR_now;
    uint16_t duty;
    /* err */
    uint32_t err;
}BLDC_RUN_t;

typedef struct 
{
    /* sensor config */
    SENSOR_Type_t sensor_type;
    /* motor config */
    uint16_t KV;
    uint8_t pole_pairs;
    uint8_t delay_angle;
    /* start config */ 
    float start_force;
    uint32_t start_order_ms;
    uint32_t start_first_step_ms;
    float start_time_rate;
    float start_force_rate;
    uint16_t start_step_min;
    uint16_t start_step_max;
    uint16_t start_max_mse;
    /* stall protect */
    uint16_t stall_protect_ms;
    uint16_t stall_protect_mse;
    uint16_t run_max_mse;
    /* output config */
    float output_min;
    float output_max;
    float output;
    /* fliter config */
    float virtual_mid_filter_rate;
    /* VVVF config */
    BLDC_VVVF_Method_t VVVF_method;
    uint8_t VVVF_ratio;
    uint32_t PWM_freq_min;
    uint32_t PWM_freq_max;
}BLDC_CONTROL_t;

extern BLDC_RUN_t bldc_run;
extern BLDC_CONTROL_t  bldc_ctrl;


void BLDC_Process(void);
void Start_BLDC_Motor(void);
void Stop_BLDC_Motor(void);
void Init_BLDC_Motor(void);

void MOS_Q15PWM(uint16_t ARR_value,uint16_t CCR_value);
void MOS_Q16PWM(uint16_t ARR_value,uint16_t CCR_value);
void MOS_Q24PWM(uint16_t ARR_value,uint16_t CCR_value);
void MOS_Q26PWM(uint16_t ARR_value,uint16_t CCR_value);
void MOS_Q34PWM(uint16_t ARR_value,uint16_t CCR_value);
void MOS_Q35PWM(uint16_t ARR_value,uint16_t CCR_value);
void Brake(void);
void Full_Brake(void);
void Plug_Brake(void);

#endif
