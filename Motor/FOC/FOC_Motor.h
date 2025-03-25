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
#include "PLL.h"


typedef enum
{
    SPM,
    IPM
}FOC_Motor_Type_t;

typedef enum
{
    FOC_IDLE,
    FOC_Stop,
    FOC_Order,
    FOC_StartUp,
    FOC_CloseLoopRun,
    FOC_HFIRun,
    FOC_VF_OpenLoopRun,
    FOC_IF_OpenLoopRun,
}FOC_Run_State_t;

typedef enum
{
    FOC_OP_ID0,
    FOC_OP_MTPA,
    FOC_OP_FW,
    FOC_OP_MTPV,
    FOC_OP_MANUAL,
}FOC_Operate_Area_t;

typedef enum
{
	FOC_DECOUPLING_DISABLED = 0,
	FOC_DECOUPLING_CROSS,
	FOC_DECOUPLING_BEMF,
	FOC_DECOUPLING_CROSS_BEMF
}FOC_Decoupling_Mode_t;


typedef struct
{
    /* phase to ground voltage */
    UVW_Axis_t V_phase_to_ground;
    /* phase voltage */
    UVW_Axis_t V_phase;
    /* phase current */
    UVW_Axis_t I_phase;
    /* voltage and current in alpha-beta axis */
    AB_Axis_t V_alpha_beta;
    AB_Axis_t I_alpha_beta;
    /* voltage and current in D-Q axis */
    DQ_Axis_t V_dq;
    DQ_Axis_t I_dq;
    /* PID output */
    DQ_Axis_t V_dq_feed_forward;
    DQ_Axis_t V_dq_out;
    AB_Axis_t V_alpha_beta_out;
    float V_out;
    float V_out_f;
    float V_out_rate;
    /* PID reference */
    DQ_Axis_t I_dq_target;
    float Speed_target;
    float Position_target;
    /* angle */
    float e_angle;
    float m_angle;
    float m_angle_multicycle;
    /* decoupling */
    DQ_Axis_t V_dq_cross;
    float V_befm;
    /* Operating Area */
    FOC_Operate_Area_t op_area;
    /* Uout limit*/
    float Uo_max;
    /* voltage vector and current vector */
    float Us;
    float Is;
    /* duty */
    float Us_duty;
    float Is_duty;
    /* Bus */
    float V_bus;
    float I_bus;
    /* power */
    float power;
    /* SVPWM */
    uint8_t sector;
    TIM_t tim;
    /* Dead Zone Compensation */
    float current_vector_angle;
    uint8_t current_vector_sector;
    int16_t dt_compensation_value;
}FOC_Para_t;

typedef struct 
{
    /* running status */
    uint8_t is_running;
    FOC_Run_State_t run_state;
    /* sensorless IF start up */
    float start_iq_now;
    float startup_target_rads_per_cycle;
    float startup_erpm_now;
    float start_e_ang;
    float startup_diff_angle;
    uint32_t start_cnt;
    /* general cnt */
    uint32_t general_cnt;
    /* control loop div */
    uint32_t current_loop_cnt;
    uint32_t speed_loop_cnt;
    uint32_t position_loop_cnt;
    /* speed */
    float eletrical_rpm;
    float eletrical_rpm_f;
    float eletrical_Hz;
    float eletrical_Hz_f;
    float we;
    float machine_rpm;
    float machine_rpm_f;
    /* PWM */
    uint32_t PWM_freq_now;
    uint16_t TIM_ARR_now;
    /* err */
    uint32_t err;
}FOC_RUN_t;

typedef struct 
{
    /* Timer */
    TIM_TypeDef * pTIM;
    /* motor */
    FOC_Motor_Type_t motor_type;
    SENSOR_Type_t sensor_type;
    uint8_t pole_pairs;
    float max_current;
    float max_rpm;
    float Rs;
    float Ld;
    float Lq;
    float Ldiff;
    float Ls;
    float Flux;
    /* sensorless IF start up */
    int32_t startup_erpm;
    float startup_diff_angle_max;
    float startup_iq_max;
    float startup_iq_min;
    uint32_t startup_order_ms;
    uint32_t startup_acc_ms;
    uint32_t startup_stable_ms;
    /* sensorless stall protection */
    int32_t erpm_min;
    /* control loop */
    uint8_t svpwm_update_en;
    uint8_t speed_loop_en;
    uint8_t position_loop_en;
    uint8_t current_loop_div;
    uint8_t speed_loop_div;
    uint8_t position_loop_div;
    uint32_t current_loop_freq;
    uint32_t speed_loop_freq;
    uint32_t position_loop_freq;
    float current_loop_dt;
    float speed_loop_dt;
    float position_loop_dt;
    /* modulation ratio */
    float modulation_ratio;
    /* PWM */
    uint32_t PWM_freq;
    /* function control */
    FOC_Decoupling_Mode_t decoupling_mode;
    uint8_t deadtime_compensate_enable;
    uint8_t MTPA_enable;
    uint8_t FW_enable;
    float FW_throttle;
    uint8_t MTPV_enable;
}FOC_CONTROL_t;

typedef enum
{
    HFI_Pole_Detect_Wait_Theta_Stable,
    HFI_Pole_Detect_Positive_Pluse,
    HFI_Pole_Detect_Wait_Current_Stable,
    HFI_Pole_Detect_Negative_Pluse,
    HFI_Pole_Detect_Done,
    HFI_Pole_Detect_Retry,
}HFI_Pole_Detect_State_t;

typedef struct
{
    uint32_t HFI_high_freq;
    float HFI_high_amplitude;

    float HFI_pole_detect_amplitude;
    uint32_t HFI_pole_detect_delay_us;
    uint32_t HFI_pole_detect_pluse_us;
    uint32_t HFI_pole_detect_pluse_interval_us;

    float HFI_switch_on_speed;
    float HFI_switch_off_speed;
}HFI_CONTROL_t;

typedef struct
{
    uint32_t HFI_high_freq_cnt;
    int32_t HFI_high_freq_sign;

    HFI_Pole_Detect_State_t HFI_pole_detect_status;
    int32_t HFI_pole_detect_sign;
    uint32_t HFI_pole_detect_delay_cnt;
    uint32_t HFI_pole_detect_pluse_cnt;
    uint32_t HFI_pole_detect_pluse_interval_cnt;
    uint32_t HFI_pole_detect_cnt;

    float HFI_Ud_excitation;

    DQ_Axis_t Udq_out;
    AB_Axis_t Uab_out;

    AB_Axis_t Uab_k0;
    AB_Axis_t Uab_k1;
    AB_Axis_t Uab_k2;

    AB_Axis_t Uab_base;
    AB_Axis_t Uab_response;

    AB_Axis_t Iab_k0;
    AB_Axis_t Iab_k1;
    AB_Axis_t Iab_k2;

    AB_Axis_t Iab_base;
    AB_Axis_t Iab_response;

    float HFI_Id_P;
    float HFI_Id_N;
    
    float theta;
}HFI_RUN_t;

extern FOC_Para_t foc_para;
extern FOC_RUN_t foc_run;
extern FOC_CONTROL_t  foc_ctrl;
extern PID_t ID_PID;
extern PID_t IQ_PID;
extern PID_t Speed_PID;
extern HFI_CONTROL_t HFI_ctrl;
extern HFI_RUN_t HFI_run;

void SVPWM_Update(FOC_CONTROL_t *ctrl,FOC_RUN_t *run_parameters,FOC_Para_t *parameters,HFI_RUN_t *hfi_parameters);

void FOC_Process(void);
void Reset_Machine_Angle_To_Zero(void);
void Start_FOC_Motor(void);
void Stop_FOC_Motor(void);
void Init_FOC_Motor(void);

#endif
