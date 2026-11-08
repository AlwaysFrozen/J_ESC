#ifndef TEST_OD_GLOBAL_H
#define TEST_OD_GLOBAL_H
#include <stdint.h>
#define BOARD_SUPPORTED_MOTOR_NUMBER 1U
#define BOARD_HRT_FREQ 20000U
#define BOARD_RT_FREQ 10000U
typedef enum { MS_Init, MS_Calibrate, MS_IDLE, MS_Err, MS_Running } Motor_State_t;
typedef enum { MCM_None, MCM_Align, MCM_RS_IDENTIFY, MCM_LS_IDENTIFY, MCM_PP_IDENTIFY, MCM_FLUX_IDENTIFY, MCM_Current,MCM_Speed,MCM_Pos } Motor_Control_Mode_t;
typedef enum { Motor_Type_DC, Motor_Type_BLDC_3_Phase } Motor_Type_t;
typedef enum { Motor_Controller_Type_FOC, Motor_Controller_Type_DTC } Motor_Controller_Type_t;
typedef struct {
    int is_enabled, is_error, is_braked, is_calibrated, is_aligned;
    Motor_State_t motor_state;
    Motor_Control_Mode_t motor_ctrl_mode;
    float pos_fb,vel_fb,negative_vel_limit,positive_vel_limit,negative_current_limit,positive_current_limit;
    float current_ref,vel_ref,pos_ref,vel_ff,acc_ref,torque_ff;
    int is_planning,is_arrived,arrive_tick,is_blocked,block_tick,follow_err_tick;
} Motor_Status_t;
typedef struct {float feedback,target,vmax,accmax,dt,tick,total_ticks,total_t,pos_output,vel_output,acc_output,jerk_output,dir,v2,dt1,dt2,dt3,x1,x2,cx1,cx2,a0,v0;} Motion_Planner_t;
extern Motion_Planner_t motion_planner[1];
typedef struct { Motor_Status_t motor_status[1]; struct { int identify_state; } foc_status[1]; } Global_Status_t;
typedef struct {
    struct { Motor_Type_t motor_type; } motor_config[1];
    struct { Motor_Controller_Type_t motor_controller_type; } motor_controller_config[1];
} Global_Configs_t;
extern volatile Global_Status_t global_status;
extern Global_Configs_t global_config;
extern uint32_t test_primask, test_ipsr;
static inline uint32_t __get_PRIMASK(void) { return test_primask; }
static inline uint32_t __get_IPSR(void) { return test_ipsr; }
static inline void __disable_irq(void) { test_primask = 1; }
static inline void __set_PRIMASK(uint32_t state) { test_primask = state; }
static inline void __DMB(void) { }
#endif
