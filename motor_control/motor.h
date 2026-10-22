/**
 * @file Motor.h
 * @author jinchl (jcl709905683@gmail.com)
 * @brief 
 * @version 0.1
 * @date 2026-03-11
 * 
 * @copyright Copyright (c) 2026
 * 
 */


#pragma once
#include "my_math.h"
#include "observer.h"

#define RATED_TORQUE_RATE (1e3f)

#ifndef PEAK_VELOCITY_ERR_RATE
#define PEAK_VELOCITY_ERR_RATE (0.01f)
#endif

#ifndef REF_VELOCITY_ERR_RATE
#define REF_VELOCITY_ERR_RATE (0.1f)
#endif

#ifndef VELOCITY_ERR_MS
#define VELOCITY_ERR_MS (100.0f)
#endif

#define PWM_DEAD_ZONE_COMPENSATION_DUTY ((float)BOARD_PWM_DEAD_TIME_NS / (1e9f / (float)BOARD_PWM_FREQ))

/* 各相电流在 +/- 此阈值内线性过渡，区外使用完整死区补偿；单位 A。 */
#ifndef PWM_DEAD_ZONE_TRANSITION_CURRENT
#define PWM_DEAD_ZONE_TRANSITION_CURRENT (0.1f)
#endif

// 电机层上行命令定义
#define MOTOR_CMD_SET_ERROR              0xEEEE
#define MOTOR_CMD_SET_EXT_ERROR          0x0EEE
#define MOTOR_CMD_SET_WARN               0x00EE
#define MOTOR_CMD_SET_ABS_OFFSET         0x0100
#define MOTOR_CMD_SET_ABM_OFFSET         0x0101
#define MOTOR_CMD_SET_MOTOR_DIR          0x1100
#define MOTOR_CMD_SET_RS                 0x1101
#define MOTOR_CMD_SET_LS                 0x1102
#define MOTOR_CMD_SET_PP                 0x1103
#define MOTOR_CMD_SET_FLUX               0x1104
#define MOTOR_CMD_SET_SECOND_ENCODER_DIR 0x1105
#define MOTOR_CMD_SET_COGGING            0x1106
#define MOTOR_CMD_SET_INERTIA            0x1107
#define MOTOR_CMD_SET_FRICTION           0x1108
#define MOTOR_CMD_SET_GRAVITY            0x1109
#define MOTOR_CMD_SET_SPRING             0x110A

/** 弹簧保持力模型：torque = k * pos + b，位置单位为 m 或 rad。 */
typedef struct
{
    int valid;
    float k; ///< N/m 或 Nm/rad
    float b; ///< N 或 Nm
} Spring_Cali_t;

/**
 * @brief 电机故障发生瞬间的状态快照，通过 IPC 从 Core1 发送至 Core0。
 */
typedef struct
{
    uint16_t code;
    int32_t axis_mode;
    int32_t motor_ctrl_mode;
    float v_bus;
    float core_temp;
    float board_temp;
    float pos_ref;
    float pos_fb;
    float vel_ref;
    float vel_fb;
    float torque_ref;
    float torque_fb;
} Motor_Error_Log_t;

_Static_assert(sizeof(Motor_Error_Log_t) <= 128U, "motor error log IPC payload too large");

/**
 * @brief 电机警告发生瞬间的状态快照，通过 IPC 从 Core1 发送至 Core0。
 */
typedef struct
{
    uint16_t code;
    int32_t axis_mode;
    int32_t motor_ctrl_mode;
    float v_bus;
    float core_temp;
    float board_temp;
    float pos_ref;
    float pos_fb;
    float vel_ref;
    float vel_fb;
    float torque_ref;
    float torque_fb;
} Motor_Warning_Log_t;

_Static_assert(sizeof(Motor_Warning_Log_t) <= 128U, "motor warning log IPC payload too large");

/**
 * @brief 故障代码定义。
 */
typedef enum
{
    FAULT_CODE_NONE = 0, ///< 无故障

    // 电机外设相关错误
    FAULT_CODE_VOLTAGE_SENSOR_OFFSET_U,    ///< U相电压偏置过高
    FAULT_CODE_VOLTAGE_SENSOR_OFFSET_V,    ///< V相电压偏置过高
    FAULT_CODE_VOLTAGE_SENSOR_OFFSET_W,    ///< W相电压偏置过高
    FAULT_CODE_CURRENT_SENSOR_OFFSET_U,    ///< U相电流偏置过高
    FAULT_CODE_CURRENT_SENSOR_OFFSET_V,    ///< V相电流偏置过高
    FAULT_CODE_CURRENT_SENSOR_OFFSET_W,    ///< W相电流偏置过高
    FAULT_CODE_CURRENT_SENSOR_UNCALIBRATE, ///< 电流偏置未校准
    FAULT_CODE_OVER_VOLTAGE,               ///< 过电压
    FAULT_CODE_UNDER_VOLTAGE,              ///< 低电压
    FAULT_CODE_OVER_TEMP_CORE,             ///< CPU高温
    FAULT_CODE_OVER_TEMP_BOARD,            ///< 驱动高温
    FAULT_CODE_UNDER_TEMP_CORE,            ///< CPU低温
    FAULT_CODE_UNDER_TEMP_BOARD,           ///< 驱动低温
    FAULT_CODE_PHASE_OVER_CURRENT,         ///< 电机相线过流
    FAULT_CODE_BUS_OVER_CURRENT,           ///< 电机母线过流
    FAULT_CODE_BOARD_PHASE_OVER_CURRENT,   ///< 板相线过流
    FAULT_CODE_BOARD_BUS_OVER_CURRENT,     ///< 板母线过流
    FAULT_CODE_UNBALANCED_CURRENTS,        ///< 电流不平衡
    FAULT_CODE_OVER_LOAD,                  ///< 电机过载
    FAULT_CODE_STOP,                       ///< 急停开关触发

    // 编码器相关错误
    FAULT_CODE_ENCODER_INIT = 50,     ///< 编码器初始化失败
    FAULT_CODE_ENCODER_FAULT,         ///< 编码器故障
    FAULT_CODE_ENCODER_POSITION_JUMP, ///< 编码器位置反馈跳变次数过多
    FAULT_CODE_ENCODER_LOST,          ///< 编码器丢失
    FAULT_CODE_ENCODER_OVERFLOW,      ///< 编码器计数溢出

    // 控制模式相关错误
    FAULT_CODE_BRAKE_OPEN_TIMEOUT = 100,    ///< 抱闸打开超时
    FAULT_CODE_BRAKE_CLOSE_TIMEOUT,         ///< 抱闸打开超时
    FAULT_CODE_SERVO_ON,                    ///< 使能错误
    FAULT_CODE_ALIGN,                       ///< 寻相故障
    FAULT_CODE_ALIGN_NO_CONVERGENCE,        ///< 寻相误差无法收敛
    FAULT_CODE_ALIGN_TIMEOUT,               ///< 寻相超时
    FAULT_CODE_HOMING_LIMIT_SWITCH_TIMEOUT, ///< 回零寻找限位开关超时
    FAULT_CODE_HOMING_INDEX_TIMEOUT,        ///< 回零寻找编码器索引超时
    FAULT_CODE_HOMING,                      ///< 回零错误
    FAULT_CODE_CHECK_TIMEOUT,               ///< 行程校准超时
    FAULT_CODE_EXT_HOMING_ABANDON,          ///< 放弃扩展回零
    FAULT_CODE_EXT_FORCE_ABANDON,           ///< 放弃扩展力控

    // 参数辨识错误
    FAULT_CODE_UNSUPPORTED_IDENTIFY = 130, ///< 不支持辨识错误
    FAULT_CODE_MOTOR_TYPE_IDENTIFY,        ///< 不支持辨识的电机类型错误
    FAULT_CODE_RS_IDENTIFY,                ///< 相电阻辨识错误
    FAULT_CODE_LS_IDENTIFY,                ///< 相电感辨识错误
    FAULT_CODE_PP_IDENTIFY,                ///< 极对数辨识错误
    FAULT_CODE_FLUX_IDENTIFY,              ///< 磁链辨识错误
    FAULT_CODE_COGGING_IDENTIFY,           ///< 齿槽辨识错误
    FAULT_CODE_INERTIA_IDENTIFY,           ///< 惯量辨识错误
    FAULT_CODE_FRICTION_IDENTIFY,          ///< 摩擦辨识错误
    FAULT_CODE_GRAVITY_IDENTIFY,           ///< 重力辨识错误
    FAULT_CODE_SPRING_IDENTIFY,            ///< 弹簧辨识错误

    // 运动状态错误
    FAULT_CODE_CURRENT_FOLLOW_ERR = 150, ///< 电流跟随误差错误
    FAULT_CODE_VELOCITY_FOLLOW_ERR,      ///< 速度跟随误差错误
    FAULT_CODE_POSITION_FOLLOW_ERR,      ///< 位置跟随误差错误
    FAULT_CODE_FORCE_ERR,                ///< 力控错误
    FAULT_CODE_BLOCKED,                  ///< 堵转错误
    FAULT_CODE_JITTER,                   ///< 异常抖动
    FAULT_CODE_HFI,                      ///< 高频注入错误

    // 通信错误
    FAULT_CODE_ERROR_ETHERCAT = 200, ///< ECAT通信中断
    FAULT_CODE_ERROR_IPC,            ///< IPC通信失败

    // 配置文件错误
    FAULT_CODE_CONFIG_VERSION = 220, ///< 配置版本与固件版本不匹配

    // 扩展错误
    FAULT_CODE_EXTEND = 255, ///< 扩展错误
} Motor_Err_Code_t;

/**
 * @brief 扩展故障代码定义。
 */
typedef enum
{
    EXT_FAULT_CODE_NONE = 0, ///< 无故障

    // 参数配置错误
    EXT_FAULT_CODE_CFG_MOTOR_TYPE = 1,                           ///< 电机类型配置错误
    EXT_FAULT_CODE_CFG_MOTOR_STRUCT,                             ///< 电机结构配置错误
    EXT_FAULT_CODE_CFG_MOTOR_RATED_VOLTAGE,                      ///< 电机额定电压配置错误
    EXT_FAULT_CODE_CFG_MOTOR_RATED_CURRENT,                      ///< 电机额定电流配置错误
    EXT_FAULT_CODE_CFG_MOTOR_PEAK_CURRENT,                       ///< 电机峰值电流配置错误
    EXT_FAULT_CODE_CFG_MOTOR_RATED_SPEED,                        ///< 电机额定速度配置错误
    EXT_FAULT_CODE_CFG_MOTOR_PEAK_SPEED,                         ///< 电机峰值速度配置错误
    EXT_FAULT_CODE_CFG_MOTOR_KT,                                 ///< 电机扭矩常数配置错误
    EXT_FAULT_CODE_CFG_MOTOR_KE,                                 ///< 电机反电势常数配置错误
    EXT_FAULT_CODE_CFG_MOTOR_PP,                                 ///< 电机极对数配置错误
    EXT_FAULT_CODE_CFG_MOTOR_RS,                                 ///< 电机相电阻配置错误
    EXT_FAULT_CODE_CFG_MOTOR_LD,                                 ///< 电机D轴电感配置错误
    EXT_FAULT_CODE_CFG_MOTOR_LQ,                                 ///< 电机Q轴电感配置错误
    EXT_FAULT_CODE_CFG_MOTOR_FLUX,                               ///< 电机转子磁链配置错误
    EXT_FAULT_CODE_CFG_MOTOR_INERTIA,                            ///< 电机惯量配置错误
    EXT_FAULT_CODE_CFG_MOTOR_ROTOR_INERTIA,                      ///< 电机转子惯量配置错误
    EXT_FAULT_CODE_CFG_MOTOR_LOAD_INERTIA,                       ///< 电机负载惯量配置错误
    EXT_FAULT_CODE_CFG_MOTOR_PEAK_CURRENT_TIME,                  ///< 电机峰值电流最大持续时间配置错误
    EXT_FAULT_CODE_CFG_MOTOR_PHASE_OVER_CURRENT_THRESHOLD,       ///< 电机相线过流阈值配置错误
    EXT_FAULT_CODE_CFG_MOTOR_BUS_OVER_CURRENT_THRESHOLD,         ///< 电机母线过流阈值配置错误
    EXT_FAULT_CODE_CFG_MOTOR_COOLING_TIME,                       ///< 电机冷却时间配置错误
    EXT_FAULT_CODE_CFG_MOTOR_DIR,                                ///< 电机方向配置错误
    EXT_FAULT_CODE_CFG_MOTOR_HAVE_BRAKE,                         ///< 电机有无抱闸配置错误
    EXT_FAULT_CODE_CFG_MOTOR_BRAKE_OPEN_TIME,                    ///< 电机抱闸打开时间配置错误
    EXT_FAULT_CODE_CFG_MOTOR_BRAKE_CLOSE_TIME,                   ///< 电机抱闸关闭时间配置错误

    EXT_FAULT_CODE_CFG_ENCODER_TYPE = 100,                       ///< 编码器类型配置错误
    EXT_FAULT_CODE_CFG_ENCODER_STRUCT,                           ///< 编码器结构配置错误
    EXT_FAULT_CODE_CFG_ENCODER_DELTA_CPR,                        ///< 编码器增量式分辨率配置错误
    EXT_FAULT_CODE_CFG_ENCODER_ABS_CPR,                          ///< 编码器绝对式分辨率配置错误
    EXT_FAULT_CODE_CFG_ENCODER_BAUDRATE,                         ///< 编码器波特率配置错误
    EXT_FAULT_CODE_CFG_ENCODER_SINGLE_TURN_BITS,                 ///< 编码器单圈位数配置错误
    EXT_FAULT_CODE_CFG_ENCODER_MULT_TURN_BITS,                   ///< 编码器多圈位数配置错误
    EXT_FAULT_CODE_CFG_ENCODER_DELTA_DIR,                        ///< 编码器增量式方向配置错误
    EXT_FAULT_CODE_CFG_ENCODER_ABS_DIR,                          ///< 编码器绝对式方向配置错误

    EXT_FAULT_CODE_CFG_ENCODER_VELOCITY_SOLVER_TYPE = 150,       ///< 速度求解器类型配置错误
    EXT_FAULT_CODE_CFG_ENCODER_VELOCITY_SOLVER_BAND_WIDTH,       ///< 速度求解器带宽配置错误

    EXT_FAULT_CODE_CFG_AXIS_DIR = 200,                           ///< 轴方向配置错误
    EXT_FAULT_CODE_CFG_AXIS_AUTO_HOMING,                         ///< 轴自动回零使能使能配置错误
    EXT_FAULT_CODE_CFG_AXIS_TRAJECTORY_TYPE,                     ///< 轴轨迹规划类型配置错误
    EXT_FAULT_CODE_CFG_AXIS_JERK_LIMIT,                          ///< 轴加加速度限制配置错误
    EXT_FAULT_CODE_CFG_AXIS_ACC_LIMIT,                           ///< 轴加速度限制配置错误
    EXT_FAULT_CODE_CFG_AXIS_VEL_LIMIT,                           ///< 轴速度限制配置错误
    EXT_FAULT_CODE_CFG_AXIS_POSITIVE_POS_LIMIT,                  ///< 轴正限位配置错误
    EXT_FAULT_CODE_CFG_AXIS_NEGATIVE_POS_LIMIT,                  ///< 轴负限位配置错误
    EXT_FAULT_CODE_CFG_AXIS_FOLLOW_ERR_WINDOW,                   ///< 轴跟随误差窗口配置错误
    EXT_FAULT_CODE_CFG_AXIS_FOLLOW_ERR_TIME,                     ///< 轴跟随误差窗口时间配置错误
    EXT_FAULT_CODE_CFG_AXIS_BLOCK_VEL,                           ///< 轴堵转速度配置错误
    EXT_FAULT_CODE_CFG_AXIS_BLOCK_TORQUE,                        ///< 轴堵转扭矩配置错误
    EXT_FAULT_CODE_CFG_AXIS_BLOCK_TIME,                          ///< 轴堵转时间配置错误
    EXT_FAULT_CODE_CFG_AXIS_DEFAULT_HOME_METHOD,                 ///< 轴回零方式配置错误
    EXT_FAULT_CODE_CFG_AXIS_DEFAULT_HOME_ACC,                    ///< 轴回零加速度配置错误
    EXT_FAULT_CODE_CFG_AXIS_DEFAULT_HOME_SPEED_H,                ///< 轴回零高速配置错误
    EXT_FAULT_CODE_CFG_AXIS_DEFAULT_HOME_SPEED_L,                ///< 轴回零低速配置错误
    EXT_FAULT_CODE_CFG_AXIS_DEFAULT_HOME_OFFSET,                 ///< 轴回零偏置配置错误
    EXT_FAULT_CODE_CFG_AXIS_DEFAULT_HOME_TORQUE,                 ///< 轴回零扭矩配置错误

    EXT_FAULT_CODE_CFG_CIA402_POS_RES_NUMERATOR = 300,           ///< CIA402位置编码器分辨率分子配置错误
    EXT_FAULT_CODE_CFG_CIA402_POS_RES_DENOMINATOR,               ///< CIA402位置编码器分辨率分母配置错误
    EXT_FAULT_CODE_CFG_CIA402_VEL_RES_NUMERATOR,                 ///< CIA402速度编码器分辨率分子配置错误
    EXT_FAULT_CODE_CFG_CIA402_VEL_RES_DENOMINATOR,               ///< CIA402速度编码器分辨率分母配置错误
    EXT_FAULT_CODE_CFG_CIA402_GEAR_RATIO_NUMERATOR,              ///< CIA402齿轮比分子配置错误
    EXT_FAULT_CODE_CFG_CIA402_GEAR_RATIO_DENOMINATOR,            ///< CIA402齿轮比分母配置错误
    EXT_FAULT_CODE_CFG_CIA402_FEED_CONSTANT_NUMERATOR,           ///< CIA402进给常数分子配置错误
    EXT_FAULT_CODE_CFG_CIA402_FEED_CONSTANT_DENOMINATOR,         ///< CIA402进给常数分母配置错误

    EXT_FAULT_CODE_CFG_CONTROL_VEL_FEED_FORWARD_RATE = 400,      ///< 速度前馈增益配置错误
    EXT_FAULT_CODE_CFG_CONTROL_ACC_FEED_FORWARD_RATE,            ///< 加速度前馈增益配置错误
    EXT_FAULT_CODE_CFG_CONTROL_MOTOR_CONTROLLER_TYPE,            ///< 电机控制方法配置错误
    EXT_FAULT_CODE_CFG_CONTROL_DEAD_ZONE_COMP_ENABLE,            ///< 死区补偿使能配置错误
    EXT_FAULT_CODE_CFG_CONTROL_MODULATION_RATIO,                 ///< 调制比配置错误
    EXT_FAULT_CODE_CFG_CONTROL_CURRENT_DECOUPLING_TYPE,          ///< 电流解耦方法配置错误
    EXT_FAULT_CODE_CFG_CONTROL_MTPA_ENABLE,                      ///< 最大转矩电流比控制使能配置错误
    EXT_FAULT_CODE_CFG_CONTROL_FW_ENABLE,                        ///< 弱磁控制使能配置错误
    EXT_FAULT_CODE_CFG_CONTROL_MTPV_ENABLE,                      ///< 最大转矩电压比控制使能配置错误
    EXT_FAULT_CODE_CFG_CONTROL_ALIGN_METHOD,                     ///< 寻相方法配置错误
    EXT_FAULT_CODE_CFG_CONTROL_ALIGN_FORCE,                      ///< 寻相力度配置错误
    EXT_FAULT_CODE_CFG_CONTROL_ALIGN_TIME,                       ///< 寻相时间配置错误
    EXT_FAULT_CODE_CFG_CONTROL_ALIGN_THRESHOLD,                  ///< 寻相误差阈值配置错误
    EXT_FAULT_CODE_CFG_CONTROL_COGGING_COMP_ENABLE,              ///< 齿槽效应补偿使能配置错误
    EXT_FAULT_CODE_CFG_CONTROL_FRICTION_COMP_ENABLE,             ///< 摩擦补偿使能配置错误
    EXT_FAULT_CODE_CFG_CONTROL_SPRING_COMP_ENABLE,               ///< 弹簧补偿使能配置错误
    EXT_FAULT_CODE_CFG_CONTROL_GRAVITY_COMP_ENABLE,              ///< 重力补偿使能配置错误
    EXT_FAULT_CODE_CFG_CONTROL_CURRENT_CONTROLLER_TYPE,          ///< 电流控制器类型配置错误
    EXT_FAULT_CODE_CFG_CONTROL_CURRENT_CONTROLLER_AUTO_TUNE,     ///< 电流控制器自动整定配置错误
    EXT_FAULT_CODE_CFG_CONTROL_CURRENT_CONTROLLER_BAND_WIDTH,    ///< 电流控制器带宽配置错误
    EXT_FAULT_CODE_CFG_CONTROL_CURRENT_CONTROLLER_INTEGRAL_TIME, ///< 电流控制器积分时间配置错误
    EXT_FAULT_CODE_CFG_CONTROL_FW_CONTROLLER_TYPE,               ///< 弱磁控制器类型配置错误
    EXT_FAULT_CODE_CFG_CONTROL_FW_CONTROLLER_AUTO_TUNE,          ///< 弱磁控制器自动整定配置错误
    EXT_FAULT_CODE_CFG_CONTROL_FW_CONTROLLER_BAND_WIDTH,         ///< 弱磁控制器带宽配置错误
    EXT_FAULT_CODE_CFG_CONTROL_FW_CONTROLLER_INTEGRAL_TIME,      ///< 弱磁控制器积分时间配置错误
    EXT_FAULT_CODE_CFG_CONTROL_SPEED_CONTROLLER_TYPE,            ///< 速度控制器类型配置错误
    EXT_FAULT_CODE_CFG_CONTROL_SPEED_CONTROLLER_AUTO_TUNE,       ///< 速度控制器自动整定配置错误
    EXT_FAULT_CODE_CFG_CONTROL_SPEED_CONTROLLER_BAND_WIDTH,      ///< 速度控制器带宽配置错误
    EXT_FAULT_CODE_CFG_CONTROL_SPEED_CONTROLLER_INTEGRAL_TIME,   ///< 速度控制器积分时间配置错误
    EXT_FAULT_CODE_CFG_CONTROL_POS_CONTROLLER_TYPE,              ///< 位置控制器类型配置错误
    EXT_FAULT_CODE_CFG_CONTROL_POS_CONTROLLER_AUTO_TUNE,         ///< 位置控制器自动整定配置错误
    EXT_FAULT_CODE_CFG_CONTROL_POS_CONTROLLER_BAND_WIDTH,        ///< 位置控制器带宽配置错误
    EXT_FAULT_CODE_CFG_CONTROL_POS_CONTROLLER_INTEGRAL_TIME,     ///< 位置控制器积分时间配置错误
    EXT_FAULT_CODE_CFG_CONTROL_FORCE_CONTROLLER_TYPE,            ///< 压力控制器类型配置错误
    EXT_FAULT_CODE_CFG_CONTROL_FORCE_CONTROLLER_AUTO_TUNE,       ///< 压力控制器自动整定配置错误
    EXT_FAULT_CODE_CFG_CONTROL_FORCE_CONTROLLER_BAND_WIDTH,      ///< 压力控制器带宽配置错误
    EXT_FAULT_CODE_CFG_CONTROL_FORCE_CONTROLLER_MD,              ///< 压力控制器虚拟质量配置错误
    EXT_FAULT_CODE_CFG_CONTROL_OBSERVER_TYPE,                    ///< 观测器类型配置错误
    EXT_FAULT_CODE_CFG_CONTROL_OBSERVER_AUTO_TUNE,               ///< 观测器自动整定配置错误
    EXT_FAULT_CODE_CFG_CONTROL_OBSERVER_BAND_WIDTH,              ///< 观测器带宽配置错误

    EXT_FAULT_CODE_CFG_CONTROL_CURRENT_FB_FILTER = 500,          ///< 电流反馈滤波器配置错误
    EXT_FAULT_CODE_CFG_CONTROL_CURRENT_REF_FILTER,               ///< 电流给定滤波器配置错误
    EXT_FAULT_CODE_CFG_CONTROL_SPEED_OUT_FILTER,                 ///< 速度输出滤波器配置错误
    EXT_FAULT_CODE_CFG_CONTROL_SPEED_REF_FILTER,                 ///< 速度给定滤波器配置错误
    EXT_FAULT_CODE_CFG_CONTROL_POS_REF_FILTER,                   ///< 位置给定滤波器配置错误
    EXT_FAULT_CODE_CFG_CONTROL_FORCE_FB_FILTER,                  ///< 压力反馈滤波器配置错误

    EXT_FAULT_CODE_CFG_CONTROL_NESO_GAIN = 800,                  ///< NESO 增益配置错误
    EXT_FAULT_CODE_CFG_CONTROL_NESO_K_SMOOTH,                    ///< NESO 平滑系数配置错误
    EXT_FAULT_CODE_CFG_CONTROL_NESO_Z1_THRESHOLD,                ///< NESO Z1阈值配置错误
    EXT_FAULT_CODE_CFG_CONTROL_NESO_Z2_THRESHOLD,                ///< NESO Z2阈值配置错误
    EXT_FAULT_CODE_CFG_CONTROL_NESO_Z3_THRESHOLD,                ///< NESO Z3阈值配置错误
} Motor_Ext_Err_Code_t;

/**
 * @brief 警告代码定义。
 */
typedef enum
{
    WARN_CODE_NONE = 0, ///< 无警告

    // 编码器报警
    WARN_CODE_ENCODER_BATTERY = 100, ///< 编码器电池报警

    // 环路饱和警告
    WARN_CODE_CURRENT_LOOP_SATURATION = 1000, ///< 电流环控制器饱和
    WARN_CODE_FW_LOOP_SATURATION,             ///< 弱磁环控制器饱和
    WARN_CODE_SPEED_LOOP_SATURATION,          ///< 速度环控制器饱和
    WARN_CODE_POS_LOOP_SATURATION,            ///< 位置环控制器饱和
    WARN_CODE_FORCE_LOOP_SATURATION,          ///< 力控环控制器饱和
} Motor_Warning_Code_t;

/**
 * @brief 电机状态枚举
 * 
 */
typedef enum
{
    MS_Init,
    MS_Calibrate,
    MS_IDLE,
    MS_Running,
    MS_Err,
} Motor_State_t;

/**
 * @brief 电机控制模式枚举
 * 
 */
typedef enum
{
    MCM_None,
    MCM_Align,
    MCM_Current,
    MCM_Torque,
    MCM_Speed,
    MCM_Pos,
    MCM_Ol_Force,
    MCM_Cl_Force,
    MCM_Impedance,
    MCM_Admittance,
    MCM_Teaching,
    MCM_CST,
    MCM_CSV,
    MCM_CSP,
    MCM_VF_Theta,
    MCM_VF_Speed,
    MCM_IF_Theta,
    MCM_IF_Speed,
    MCM_HFI_VF,
    MCM_Current_D_Sweep,
    MCM_Current_Q_Sweep,
    MCM_Vel_Sweep,
    MCM_Pos_Sweep,
    MCM_RS_IDENTIFY,
    MCM_LS_IDENTIFY,
    MCM_PP_IDENTIFY,
    MCM_FLUX_IDENTIFY,
    MCM_Cogg_IDENTIFY,
    MCM_Inertia_IDENTIFY,
    MCM_Friction_IDENTIFY,
    MCM_Gravity_IDENTIFY,
} Motor_Control_Mode_t;

typedef enum
{
    Brake_State_Closed,
    Brake_State_Opening,
    Brake_State_Opened,
    Brake_State_Closing,
} Brake_State_t;

typedef enum
{
    Vel_Monitor_Disabled,   // 关闭速度误差检测
    Vel_Monitor_Normal,     // 完整速度误差检测
    Vel_Monitor_AllowBlock, // 排除堵转的速度误差检测
} Vel_Monitor_Mode_t;

typedef enum
{
    Force_Stage_Pos        = 0,
    Force_Stage_Vel        = 1,
    Force_Stage_Force      = 1,
    Force_Stage_Pos_Done   = 2,
    Force_Stage_Vel_Done   = 3,
    Force_Stage_Force_Done = 3,
    Force_Stage_Block,
    Force_Stage_Err,
} Force_Stage_t;

typedef struct
{
    Force_Stage_t force_stage;
    float lift_pos;
    float block_pos;
    float vel;
    float acc;
    float jerk;
    float max_pos;
    float max_vel;
    float max_tor;
    float max_force;
    float max_force_err;
    float max_force_err_time;
    float force_over_cnt;
    float force_reach_cnt;
} Force_Control_Status_t;

/**
 * @brief 电机基础反馈结构体
 * 
 */
typedef struct
{
    UVW_Axis_t phase_voltage_offset;
    UVW_Axis_t phase_voltage;
    UVW_Axis_t phase_current_offset;
    UVW_Axis_t phase_current;
    UVW_Axis_t pwm_duty_raw;
    UVW_Axis_t pwm_duty_comp;
    UVW_Axis_t pwm_duty_out;
    UVW_Axis_t phase_voltage_out;
    float i_sum;
    float i_rms;
    float v_bus;
    float i_bus;
    float e_power;
    float m_power;
    float efficiency;
    float inv_v_bus;
} Motor_Basic_Status_t;

/**
 * @brief 电机运行反馈结构体
 * 
 */
typedef struct
{
    Motor_State_t motor_state;
    Motor_Control_Mode_t motor_ctrl_mode;
    Brake_State_t brake_state;
    uint32_t brake_cnt;

    int is_calibrated;
    int is_enabled;
    int is_error;
    int is_aligned;
    int is_braked;
    int is_homed;
    int is_planning;
    int is_moving;
    int is_arrived;
    int is_blocked;
    int is_follow_err;

    uint16_t err_code;

    uint32_t motor_cnt;

    uint32_t voltage_up_cnt;
    uint32_t voltage_down_cnt;
    uint32_t core_temp_high_cnt;
    uint32_t core_temp_low_cnt;
    uint32_t board_temp_high_cnt;
    uint32_t board_temp_low_cnt;
    uint32_t phase_over_current_cnt;
    uint32_t bus_over_current_cnt;
    uint32_t board_over_current_cnt;
    uint32_t current_unbalance_cnt;


    float thermal_limit;
    float thermal_accumulator;

    float follow_err_window;
    int follow_err_time;
    int follow_err_tick;

    float arrive_window;
    int arrive_time;
    int arrive_tick;

    float block_vel;
    float block_tor;
    int block_time;
    int block_tick;

    float positive_current_limit;
    float negative_current_limit;
    float positive_tor_limit;
    float negative_tor_limit;
    float positive_vel_limit;
    float negative_vel_limit;

    float current_ref;
    float current_ref_f;
    float current_fb;
    float current_fb_f;
    float current_abs;
    float current_err;
    float current_err_abs;
    float torque_ref;
    float torque_ref_f;
    float torque_ff;
    float torque_fb;
    float torque_fb_f;
    float torque_abs;
    float torque_err;
    float torque_err_abs;
    float acc_ref;
    float acc_fb;
    float acc_abs;
    float acc_err;
    float acc_err_abs;
    float vel_ref;
    float vel_ref_f;
    float vel_ff;
    float vel_fb;
    float vel_abs;
    float vel_err;
    float vel_err_abs;
    float vel_out;
    float vel_out_f;
    float pos_ref;
    float pos_ref_f;
    float pos_fb;
    float pos_abs;
    float pos_err;
    float pos_err_abs;
    float force_ref;
    float force_fb;
    float force_fb_f;
    float force_abs;
    float force_err;
    float force_err_abs;

    uint32_t ext_err_code;
    uint32_t ext_warn_code;

    /* 以下部分暂时不介入对象字典 */
    Force_Control_Status_t force_status;
    Vel_Monitor_Mode_t vel_monitor_mode;
    float rated_torque;
    float peak_torque;
    bool identify_done;
} Motor_Status_t;

/**
 * @brief 环路控制器结构体
 * 
 */
typedef struct
{
    PID_t id_pid;
    PID_t iq_pid;
    PID_t fw_pid;
    PID_t speed_pid;
    PID_t pos_pid;
    Compliance_Controller_t force_compliance;
} Loop_Controller_t;

typedef struct
{
    NESO_Config_t neso_config;
    NESO_Status_t neso_status;
    Observer_Output_t observer_outputs;
} Motor_Observer_t;

typedef struct
{
    Filter_t current_fb_filter;
    Filter_t current_ref_filter;
    Filter_t torque_fb_filter;
    Filter_t torque_ref_filter;
    Filter_t speed_out_filter;
    Filter_t speed_ref_filter;
    Filter_t pos_ref_filter;
    Filter_t force_fb_filter;
} Loop_Filter_t;

/**
 * @brief 电机类型枚举
 * 
 */
typedef enum
{
    Motor_Type_BLDC_3_Phase,
    Motor_Type_DC,
    Motor_Type_MAX
} Motor_Type_t;

/**
 * @brief 电机结构枚举
 * 
 */
typedef enum
{
    Motor_Struct_Linear,
    Motor_Struct_Rotary,
    Motor_Struct_MAX
} Motor_Struct_t;

/**
 * @brief 寻相方法枚举
 * 
 */
typedef enum
{
    Align_Method_Fast,
    Align_Method_Slow,
    Align_Method_Bisection,
#ifndef FAST_CURRENT_LOOP_ENABLE
    Align_Method_HFI,
#endif
    Align_Method_MAX
} Align_Method_t;

/**
 * @brief 电机本体配置结构体
 * 
 */
typedef struct
{
    Motor_Type_t motor_type;
    Motor_Struct_t motor_struct;
    float rated_voltage;
    float rated_current;
    float peak_current;
    float rated_speed;
    float peak_speed;
    float kt;
    float ke;
    float pp;
    float Rs;
    float Ld;
    float Lq;
    float Flux;
    float inertia_rotor;
    float inertia_load;

    float peak_current_time_s;
    float phase_over_current_threshold;
    float bus_over_current_threshold;
    float cooling_time_s;
    int motor_dir;

    int have_brake;
    int brake_open_time;
    int brake_close_time;

    float L_diff;
    float saliency_ratio;
} Motor_Config_t;

/**
 * @brief 电机控制方法枚举
 * 
 */
typedef enum
{
    Motor_Controller_Type_FOC,
    Motor_Controller_Type_DTC,
    Motor_Controller_Type_MAX
} Motor_Controller_Type_t;

/**
 * @brief 电流解耦方法枚举
 * 
 */
typedef enum
{
    Current_Decoupling_Type_None,
    Current_Decoupling_Type_RL,
    Current_Decoupling_Type_Cross,
    Current_Decoupling_Type_RL_Cross,
    Current_Decoupling_Type_Bemf,
    Current_Decoupling_Type_RL_Bemf,
    Current_Decoupling_Type_Cross_Bemf,
    Current_Decoupling_Type_RL_Cross_Bemf,
    Current_Decoupling_Type_MAX,
} Current_Decoupling_Type_t;

/**
 * @brief 电机扩展控制配置结构体
 * 
 */
typedef struct
{
    float vel_feedforward_rate;
    float acc_feedforward_rate;
    Motor_Controller_Type_t motor_controller_type;
    int dead_zone_compensation_enable;
    float max_modulation_ratio;
    Current_Decoupling_Type_t current_decoupling_type;
    int mtpa_enable;
    int fw_enable;
    int mtpv_enable;
    Align_Method_t align_method;
    float align_force;
    int align_time;
    float align_threshold;


    int cogging_compensation_enable;
    int friction_compensation_enable;
    int spring_compensation_enable;
    int gravity_compensation_enable;
} Motor_Controller_Config_t;

/**
 * @brief 观测器配置
 * 
 */
typedef struct
{
    Observer_Config_t observser_config;
    NESO_Config_t neso_config;
} Motor_Observer_Config_t;

/**
 * @brief 控制器类型枚举
 * 
 */
typedef enum
{
    Controller_Type_Parallel_PID,
    Controller_Type_Serial_PID,
    Controller_Type_MAX,
    Controller_Type_Impedance = 100,
} Controller_Type_t;

/**
 * @brief 控制器配置结构体
 * 
 */
typedef struct
{
    Controller_Type_t controller_type;
    int auto_tune;
    float controller_band_width;
    float controller_integral_time;
} Controller_Config_t;

/**
 * @brief 力控制器类型枚举
 * 
 */
typedef enum
{
    Force_Controller_Type_Admittance,
    Force_Controller_Type_Impedance,
    Force_Controller_Type_MAX,
} Force_Controller_Type_t;

/**
 * @brief 力控制器配置结构体
 * 
 */
typedef struct
{
    Force_Controller_Type_t controller_type;
    int auto_tune;
    float controller_band_width;
    float Md;
} Force_Controller_Config_t;

/**
 * @brief 环路控制器配置结构体
 * 
 */
typedef struct
{
    Controller_Config_t current_controller_config;
    PID_Config_t id_pid_config;
    PID_Config_t iq_pid_config;
    Controller_Config_t fw_controller_config;
    PID_Config_t fw_pid_config;
    Controller_Config_t speed_controller_config;
    PID_Config_t speed_pid_config;
    Controller_Config_t pos_controller_config;
    PID_Config_t pos_pid_config;
    Force_Controller_Config_t force_controller_config;
    Compliance_Config_t compliance_config;
} Loop_Controller_Config_t;

/**
 * @brief 环路滤波器配置结构体
 * 
 */
typedef struct
{
    Filter_Config_t current_fb_filter_config;
    Filter_Config_t current_ref_filter_config;
    Filter_Config_t speed_out_filter_config;
    Filter_Config_t speed_ref_filter_config;
    Filter_Config_t pos_ref_filter_config;
    Filter_Config_t force_fb_filter_config;
} Loop_Filter_Config_t;

/**
 * @brief 规划电机位置轨迹
 * 
 * @param index 电机编号
 * @param pos 目标位置
 * @param vel 目标速度
 * @param acc 目标加速度
 * @param jerk 目标加加速度
 */
void motor_plan_pos(int index, float pos, float vel, float acc, float jerk);

/**
 * @brief 规划电机速度轨迹
 * 
 * @param index 电机编号
 * @param vel 目标速度
 * @param acc 目标加速度
 * @param jerk 目标加加速度
 */
void motor_plan_vel(int index, float vel, float acc, float jerk);

/**
 * @brief 软着陆复位
 * 
 * @param index 电机编号
 */
void softland_reset(int index);

/**
 * @brief 电机非实时更新
 * 
 * @param index 电机编号
 */
void motor_update_nrt(int index);

/**
 * @brief 电机实时更新
 * 
 * @param index 电机编号
 */
void motor_update_rt(int index);

/**
 * @brief 电机高频实时更新
 * 
 * @param index 电机编号
 */
void motor_update_frt(int index);

/**
 * @brief 根据最近一次PWM写入时刻计算电流环等效延时（含半个电流环周期的零阶保持延时）
 *
 * @param index 电机编号
 * @return 等效控制延时，单位 s；供电流环整定和电角度补偿使用
 */
float current_loop_delay_calculate(int index);

/**
 * @brief 计算 PWM 影子寄存器更新时间
 *
 * @param index 电机编号
 * @return 更新时间延时，单位 us
 */
void pwm_update_delay_record(int index);

/**
 * @brief 设置电机三相占空比
 * 
 * @param index 电机编号
 * @param u u相占空比
 * @param v v相占空比
 * @param w w相占空比
 */
void motor_pwm_set(int index, float u, float v, float w);

/**
 * @brief 初始化电机控制
 * 
 */
void motor_init(void);

/**
 * @brief 复位电机控制器
 * 
 * @param index 电机编号
 */
void motor_controller_reset(int index);

/**
 * @brief 复位电机控制
 * 
 * @param index 电机编号
 */
void motor_reset(int index);

/**
 * @brief 使能电机
 * 
 * @param index 电机编号
 * @return true 成功
 * @return false 失败
 */
bool motor_enable(int index);

/**
 * @brief 失能电机
 * 
 * @param index 电机编号
 * @return true 成功
 * @return false 失败
 */
bool motor_disable(int index);

/**
 * @brief 设置电机错误状态
 * 
 * @param index 电机编号
 * @param code 错误代码
 */
void motor_set_err(int index, uint16_t code);

/**
 * @brief 设置电机扩展错误状态
 * 
 * @param index 电机编号
 * @param ext_code 扩展错误代码
 */
void motor_set_ext_err(int index, uint32_t ext_code);

/**
 * @brief 设置电机警告状态
 * 
 * @param index 电机编号
 * @param warn_code 警告代码
 */
void motor_set_warning(int index, uint32_t warn_code);

/**
 * @brief 复位电机故障
 * 
 * @param index 电机编号
 */
void motor_reset_err(int index);

/**
 * @brief 刷新电机位置信息
 * 
 * @param index 电机编号
 */
void motor_refresh_pos(int index);

/**
 * @brief 设置电机编码器偏移
 * 
 * @param index 电机编号
 * @param offset 编码器偏移
 */
void motor_set_encoder_offset(int index, float offset);

/**
 * @brief 同步电机编码器偏移
 * 
 * @param index 电机编号
 * @param offset 编码器偏移
 */
void motor_sync_encoder_offset(int index, float offset);
