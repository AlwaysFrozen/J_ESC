/**
 * @file foc.h
 * @author jinchl (jcl709905683@gmail.com)
 * @brief 
 * @version 0.1
 * @date 2026-03-30
 * 
 * @copyright Copyright (c) 2026
 * 
 */


#pragma once
#include <math.h>
#include <stdbool.h>

#include "my_math.h"
#include "motor.h"

#define RUNNING_MASK     (0x0000000F)
#define START_UP_MASK    (0x000000F0)
#define HFI_RUNNING_MASK (0x00000F00)
#define CLOSE_LOOP_MASK  (0x0000F000)
#define OPEN_LOOP_MASK   (0x0F000000)

#define CLOSE_LOOP_RUNNING_MASK (RUNNING_MASK | CLOSE_LOOP_MASK)

/** MTPA进入阈值，当电机凸极率大于时允许进入MTPA。 */
#define FOC_MTPA_SALIENCY_RATE (1.1f)
/** 弱磁环目标及进入阈值，占当前可用电压矢量的比例。 */
#define FOC_FW_VOLTAGE_RATE (0.95f)
/** 弱磁环退出阈值，与进入阈值形成滞环以避免运行区振荡。 */
#define FOC_FW_EXIT_VOLTAGE_RATE (0.90f)
/** 弱磁对象最小增益占“可用电压/峰值电流”的比例，用于限制自动整定最大增益。 */
#define FOC_FW_MIN_PLANT_GAIN_RATE (0.01f)

#ifndef MIN_CURRENT_ERR
#define MIN_CURRENT_ERR (0.1f)
#endif

#ifndef MAX_CURRENT_ERR_RATE
#define MAX_CURRENT_ERR_RATE (0.1f)
#endif

#ifndef CURRENT_ERR_MS
#define CURRENT_ERR_MS (10.0f)
#endif

/**
 * @brief 检测电流跟随误差是否超限
 * 
 * @param current_ref 电流参考
 * @param current_err 电流误差
 * @return int 误差超限
 */
static inline int FOC_Current_Follow_Err_Exceeded(float current_ref, float current_err)
{
    float err_abs   = fabsf(current_err);
    float ref_abs   = fabsf(current_ref);
    float err_limit = fmaxf(MIN_CURRENT_ERR, ref_abs * MAX_CURRENT_ERR_RATE);

    return err_abs > err_limit;
}

typedef enum
{
    FOC_IDLE          = 0x00000000,
    FOC_Order         = 0x00000011,
    FOC_StartUp       = 0x00000021,
    FOC_HFI_Init      = 0x00000101,
    FOC_HFI_Identify  = 0x00001201,
    FOC_HFI_CloseLoop = 0x00001301,
    FOC_CloseLoop     = 0x00001001,
    FOC_VF_OpenLoop   = 0x01000001,
    FOC_IF_OpenLoop   = 0x11000001,
    FOC_HFI_OpenLoop  = 0x21000001,
} FOC_Run_State_t;

typedef enum
{
    FOC_OP_ID0,
    FOC_OP_MTPA,
    FOC_OP_FW,
    FOC_OP_MTPV,
    FOC_OP_MANUAL,
} FOC_Operate_Area_t;

/**
 * @brief foc运行反馈结构体
 * 
 */
typedef struct
{
    /* running status */
    FOC_Run_State_t run_state;
    /* Operating Area */
    FOC_Operate_Area_t op_area;
    /* voltage and current in alpha-beta axis */
    AB_Axis_t u_ab;
    AB_Axis_t u_ab_last;
    AB_Axis_t i_ab;
    AB_Axis_t i_ab_last;
    /* voltage and current in D-Q axis */
    DQ_Axis_t u_dq;
    DQ_Axis_t u_dq_last;
    DQ_Axis_t i_dq;
    DQ_Axis_t i_dq_last;
    /* PID reference */
    DQ_Axis_t u_dq_ref;
    DQ_Axis_t i_dq_ref;
    /* PID output */
    DQ_Axis_t u_dq_feed_forward;
    DQ_Axis_t u_dq_out_raw;
    DQ_Axis_t u_dq_out;
    AB_Axis_t u_ab_out_raw;
    AB_Axis_t u_ab_out;
    float us_out_raw;
    float us_out_raw_f;
    float us_out;
    float us_out_f;
    float us_out_rate_raw;
    float us_out_rate;
    /* angle */
    float rads_encoder;
    float rads_encoder_offset;
    float theta_encoder;
    float we_encoder;
    float theta_hall;
    float we_hall;
    float theta_observer;
    float we_observer;
    float theta_hfi;
    float we_hfi;
    float theta_openloop;
    float we_openloop;
    float theta;
    float theta_comp;
    float we;
    Sin_Cos_t sin_cos;
    /* decoupling */
    DQ_Axis_t i_dq_delta;
    DQ_Axis_t i_dq_delta_f;
    DQ_Axis_t u_dq_rl;
    DQ_Axis_t u_dq_cross;
    float u_bemf;
    /* Uout limit*/
    float us_max;
    float uo_max;
    /* voltage vector and current vector */
    float us;
    float is;
    /* duty */
    float us_duty;
    float is_duty;
    /* align */
    int align_state;
    int align_tick;
    int align_retry_count;
    float align_initial;
    float align_offset_candidate;
    float align_correction;
    int align_dir_detected;
    /* Dead Zone Compensation */
    float dt_compensation_value;


    /* not in od */

    /* err check */
    int current_err_cnt;
    /* internal ref & fb */
    float current_ref;
    float current_fb;
    float torque_ref;
    float torque_fb;
    /* electrical parameters detect */
    int identify_state;
    int identify_tick;
    DQ_Axis_t u_dq_cache[2];
    DQ_Axis_t i_dq_cache[2];
    float encoder_pos_cache;
    float rs_identify;
    DQ_Axis_t l_identify;
    float pp_identify;
    float flux_identify;
    bool overide_decoupling_update;
    /* second encoder */
    float align_initial_second;
    int align_dir_detected_second;
    float rads_encoder_offset_second;
} FOC_Status_t;

/**
 * @brief MTPA计算
 * 
 * @param p_motor_config 电机本体配置结构体指针
 * @param p_foc_status foc运行反馈结构体指针
 */
void MTPA_Cal(Motor_Config_t *p_motor_config, FOC_Status_t *p_foc_status);

/**
 * @brief 根据目标扭矩计算MTPA总电流参考值
 *
 * @param motor 电机本体配置结构体指针
 * @param torque_ref 扭矩参考
 * @return 带扭矩方向的定子电流矢量幅值参考
 */
float IPM_MTPA_From_Torque(const Motor_Config_t *motor, float torque_ref);

/**
 * @brief MTPV计算
 * 
 * @param p_motor_config 电机本体配置结构体指针
 * @param p_foc_status foc运行反馈结构体指针
 */
void MTPV_Cal(Motor_Config_t *p_motor_config, FOC_Status_t *p_foc_status);

/**
 * @brief foc寻相实时更新
 * 
 * @param index 电机编号
 */
void foc_align_update_rt(int index);

/**
 * @brief foc相电阻辨识实时更新
 * 
 * @param index 电机编号
 */
void foc_rs_identify_update_rt(int index);

/**
 * @brief foc相电感辨识实时更新
 * 
 * @param index 电机编号
 */
void foc_ls_identify_update_rt(int index);

/**
 * @brief foc磁链辨识实时更新
 * 
 * @param index 电机编号
 */
void foc_flux_identify_update_rt(int index);

/**
 * @brief foc极对数辨识实时更新
 * 
 * @param index 电机编号
 */
void foc_pp_identify_update_rt(int index);

/**
 * @brief foc实时更新
 * 
 * @param index 电机编号
 */
void foc_update_rt(int index);

/**
 * @brief foc高频实时更新
 * 
 * @param index 电机编号
 */
void foc_update_frt(int index);

/**
 * @brief foc编码器复位
 * 
 * @param index 电机编号
 */
void foc_encoder_reset(int index);

/**
 * @brief 复位FOC电机控制器
 * 
 * @param index 电机编号
 */
void foc_controller_reset(int index);

/**
 * @brief 根据FOC控制方式计算并缓存额定扭矩和峰值扭矩
 *
 * @param index 电机编号
 */
void foc_calculate_torque_limits(int index);

/**
 * @brief 设置FOC控制器为手动模式
 * 
 * @param index 电机编号
 * @param id_ref d轴电流参考
 * @param iq_ref q轴电流参考
 */
void foc_controller_set_manual_mode(int index, float id_ref, float iq_ref);

/**
 * @brief 设置FOC控制器为自动模式
 * 
 * @param index 电机编号
 */
void foc_controller_set_auto_mode(int index);

/**
 * @brief 设置FOC电流参考
 * 
 * @param index 电机编号
 * @param current_ref 电流参考
 */
void foc_set_current_ref(int index, float current_ref);

/**
 * @brief 获取FOC电流反馈
 * 
 * @param index 电机编号
 * @return float 电流反馈
 */
float foc_get_current_fb(int index);

/**
 * @brief 设置FOC扭矩参考
 * 
 * @param index 电机编号
 * @param torque_ref 设置FOC扭矩参考
 */
void foc_set_torque_ref(int index, float torque_ref);

/**
 * @brief 获取FOC扭矩反馈
 * 
 * @param index 电机编号
 * @return float 扭矩反馈
 */
float foc_get_torque_fb(int index);

/**
 * @brief 设置FOC开环电压
 * 
 * @param index 电机编号
 * @param ud d轴电压
 * @param uq q轴电压
 * @param p_theta 开环电角度
 * @param p_we 开环电角速度
 */
void foc_set_vf_voltage(int index, float ud, float uq, float *p_theta, float *p_we);

/**
 * @brief 设置FOC开环电流
 * 
 * @param index 电机编号
 * @param id d轴电流
 * @param iq q轴电流
 * @param p_theta 开环电角度
 * @param p_we 开环电角速度
 */
void foc_set_if_current(int index, float id, float iq, float *p_theta, float *p_we);

/**
 * @brief 设置FOC HFI开环电压
 * 
 * @param index 电机编号
 * @param ud d轴电压
 * @param p_theta 开环电角度
 * @param p_we 开环电角速度
 */
void foc_set_hfi_vf_voltage(int index, float ud, float *p_theta, float *p_we);

/**
 * @brief 设置FOC编码器偏移
 * 
 * @param index 电机编号
 * @param offset 编码器偏移
 */
void foc_set_encoder_offset(int index, float offset);

/**
 * @brief 获取FOC编码器偏移
 * 
 * @param index 电机编号
 * @return float 编码器偏移
 */
float foc_get_encoder_offset(int index);
