/**
 * @file motion_planner.h
 * @author jinchl (jcl709905683@gmail.com)
 * @brief 
 * @version 0.1
 * @date 2026-04-01
 * 
 * @copyright Copyright (c) 2026
 * 
 */

#pragma once
#include "../math/my_math.h"

typedef enum
{
    Plan_Type_Cycloidal,
    Plan_Type_Trapezoid,
    Plan_Type_None = 0xFF,
} Motion_Plan_Type_t;

typedef enum
{
    Plan_Target_Pos,
    Plan_Target_Vel,
} Motion_Plan_Target_t;

/**
 * @brief 运动控制器结构体。
 *
 * 用于实现平滑的运动控制逻辑，包括速度、加速度的计划和执行。
 */
typedef struct
{
    Motion_Plan_Type_t plan_type;
    Motion_Plan_Target_t plan_target;

    float feedback;      ///< 当前目标反馈值。
    float target;        ///< 目标值。
    float vmax;          ///< 最大速度。
    float accmax;        ///< 加速度和减速度。

    float dt;            ///< 时间增量。
    float tick;          ///< 当前计时器滴答。
    float total_ticks;   ///< 总计时器滴答数。
    float total_t;       ///< 总运动时间。

    float pos_output;    ///< 位置输出。
    float vel_output;    ///< 速度输出。
    float acc_output;    ///< 加速度输出。
    float jerk_output;   ///< 加加速度输出。

    float dir;           ///< 运动方向。
    float v2;            ///< 内部变量，用于运动控制计算。
    float dt1, dt2, dt3; ///< 分段时间间隔。
    float x1, x2;        ///< 内部位置变量。
    float cx1, cx2;      ///< 内部校正变量。

    float a0;
    float v0;
} Motion_Planner_t;

/**
 * @brief 
 * 
 * @param c 轨迹规划器结构体指针
 * @param current_pos 当前位置反馈值
 * @param target_pos 目标位置
 * @param acc_peak 最大加速度
 * @param vel_peak 最大速度
 * @param jerk_peak 最大加加速度
 * @param dt 离散化时间
 * @param type 轨迹类型
 * @return bool 是否开始规划
 */
bool motion_plan(Motion_Planner_t *c, float current_pos,
                 float target_pos, float acc_peak,
                 float vel_peak, float jerk_peak, float dt, int type);
/**
 * @brief 
 * 
 * @param c 轨迹规划器结构体指针
 * @return bool 是否在规划中
 */
bool motion_update(Motion_Planner_t *c);

/**
 * @brief 
 * 
 * @param c 轨迹规划器结构体指针
 * @param current_vel 当前速度反馈值
 * @param target_vel 目标速度
 * @param acc_peak 最大加速度
 * @param jerk_peak 最大加加速度
 * @param dt 离散化时间
 * @param type 轨迹类型
 * @return bool 是否开始规划
 */
bool velocity_plan(Motion_Planner_t *c, float current_vel,
                   float target_vel, float acc_peak,
                   float jerk_peak, float dt, int type);

/**
 * @brief 
 * 
 * @param c 轨迹规划器结构体指针
 * @return bool 是否在规划中
 */
bool velocity_update(Motion_Planner_t *c);

