#ifndef __FOC_MOTOR_H__
#define __FOC_MOTOR_H__

#include "stm32f4xx_hal.h"
#include "FOC_Config.h"
#include "Motor.h"
#include "PID.h"
#include "Sensor.h"
#include "VoltageCurrentSensor.h"
#include "HALL.h"
#include "Encoder.h"
#include "AS5048a.h"

typedef enum
{
    FOC_IDLE,
    FOC_Stop,
    FOC_Order,
    FOC_StartUp,
    FOC_CloseLoopRun,
    FOC_HFIRun,
}FOC_Run_State_t;

typedef struct
{
    /* phase to ground voltage */
    float U_ag;
    float U_bg;
    float U_cg;
    /* phase voltage */
    float U_an;
    float U_bn;
    float U_cn;
    /* phase current */
    float I_a;
    float I_b;
    float I_c;
    /* voltage and current in alpha-beta axis */
    float Ua;
    float Ub;
    float Ia;
    float Ib;
    /* voltage and current in D-Q axis */
    float Ud;
    float Uq;
    float Id;
    float Iq;
    /*  */
    float Ua_rate;
    float Ub_rate;
    float Ud_rate;
    float Uq_rate;
    float Us_rate;
    float Us_rate_f;
    float Us;
    float Us_f;
    float Is;
    float Is_f;
    /* PID reference */
    float Id_target;
    float Iq_target;
    float Speed_target;

    float e_angle;
    float m_angle;

    
    uint8_t sector;
    int32_t CMP1;
    int32_t CMP2;
    int32_t CMP3;

    #if USE_DT_COMPENSATION
    float current_vector_angle;
    uint8_t current_vector_sector;
    int16_t dt_compensation_value;
    #endif
}FOC_Para_t;

typedef struct
{
    uint8_t HFI_Div;
    uint16_t HFI_cnt;

    float HFI_Id_target;
    float HFI_Ud_sign;
    float HFI_Ia_now;
    float HFI_Ia_last;
    float HFI_Ib_now;
    float HFI_Ib_last;

    float HFI_Ia_base;
    float HFI_Ia_delta;
    float HFI_Ib_base;
    float HFI_Ib_delta;

    float HFI_e_angle;
}FOC_HFI_Para_t;

typedef struct 
{
    /* running status */
    uint8_t is_running;
    FOC_Run_State_t run_state;
    /* sensorless IF start up */
    int32_t start_iq_now;
    float start_rpp_now;
    float start_e_ang;
    uint32_t start_cnt;
    /* general cnt */
    uint32_t general_cnt;
    /* control loop div */
    uint32_t current_loop_cnt;
    uint32_t speed_loop_cnt;
    /* speed */
    float eletrical_rpm;
    float eletrical_rpm_f;
    float eletrical_Hz;
    float eletrical_Hz_f;
    float machine_rpm;
    float machine_rpm_f;
    /* PWM */
    uint32_t PWM_freq_now;
    uint16_t TIM1_ARR_now;
    /* err */
    uint32_t err;
}FOC_RUN_t;

typedef struct 
{
    SENSOR_Type_t sensor_type;
    /* sensorless IF start up */
    int32_t startup_erpm;
    int32_t startup_iq_max;
    int32_t startup_iq_min;
    uint32_t startup_order_ms;
    uint32_t startup_acc_ms;
    /* sensorless stall protection */
    int32_t erpm_min;
    /* motor  */
    uint8_t pole_pairs;
    /* control loop */
    uint8_t current_loop;
    uint8_t speed_loop;
    uint8_t current_loop_div;
    uint8_t speed_loop_div;
    /* PWM */
    uint32_t PWM_freq;
}FOC_CONTROL_t;

extern FOC_Para_t foc_para;
extern FOC_HFI_Para_t HFI_para;
extern FOC_RUN_t foc_run;
extern FOC_CONTROL_t  foc_ctrl;
extern PID_Position_t ID_PID;
extern PID_Position_t IQ_PID;
extern PID_Position_t Speed_PID;

void SVPWM_Update(float Ua_rate,float Ub_rate,float Ud_rate,float Uq_rate,float angle);
void FOC_Process(void);
void Start_FOC_Motor(void);
void Stop_FOC_Motor(void);
void Init_FOC_Motor(void);

#endif
