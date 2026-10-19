/**
 * @file compliance_control.h
 * @author jinchl (jcl709905683@gmail.com)
 * @brief 
 * @version 0.1
 * @date 2026-06-22
 * 
 * @copyright Copyright (c) 2026
 * 
 */


#pragma once

#include <stdbool.h>
#include <string.h>

#include "utils.h"

typedef struct
{
    /**
     * @brief 虚拟质量 Md，单位 kg
     *
     * 作用：
     * Md 越大，导纳控制器对力误差变化越不敏感，加速度变化越慢。
     * Md 越小，响应越快，但更容易受力传感器噪声影响。
     *
     * 对精密下压场景：
     * Md 不宜太小，否则接触力有轻微波动时速度指令会抖动。
     */
    float virtual_mass;

    /**
     * @brief 虚拟阻尼 Bd，单位 N·s/m
     *
     * 作用：
     * Bd 越大，运动越慢、越柔和、越稳定。
     * Bd 越小，运动越快、越灵敏，但更容易振荡。
     *
     * 阻尼项对应：
     * Bd * x_vel
     */
    float virtual_damping;

    /**
     * @brief 虚拟刚度 Kd，单位 N/m
     *
     * 作用：
     * Kd 越大，导纳产生的位移越小，系统表现越“硬”。
     * Kd 越小，同样力误差下允许更大位移，系统表现越“软”。
     *
     * 如果只想做柔顺力控，不希望出现明显虚拟弹簧回弹，可以先设为 0。
     */
    float virtual_stiffness;
} Compliance_Config_t;

typedef struct
{
    /**
     * @brief 最大虚拟位置限制，单位 m
     *
     * 作用：
     * 限制导纳控制器输出给位置环的最大位置。
     *
     * 如果完整导纳输出的是速度指令，这个值仍然有保护意义。
     */
    float max_position;

    /**
     * @brief 最小虚拟位置限制，单位 m
     *
     * 作用：
     * 限制导纳控制器输出给位置环的最小位置。
     *
     * 如果完整导纳输出的是速度指令，这个值仍然有保护意义。
     */
    float min_position;

    /**
     * @brief 最大虚拟速度限制，单位 m/s
     *
     * 作用：
     * 限制导纳控制器输出给速度环的最大速度。
     * 对精密下压非常重要，防止接触过程中运动过快。
     */
    float max_velocity;

    /**
     * @brief 最小虚拟速度限制，单位 m/s
     *
     * 作用：
     * 限制导纳控制器输出给速度环的最小速度。
     * 对精密下压非常重要，防止接触过程中运动过快。
     */
    float min_velocity;

    /**
     * @brief 最大虚拟加速度限制，单位 m/s^2
     *
     * 作用：
     * 限制导纳控制器内部加速度，避免力信号突变导致速度指令突变。
     */
    float max_acceleration;

    /**
     * @brief 最小虚拟加速度限制，单位 m/s^2
     *
     * 作用：
     * 限制导纳控制器内部加速度，避免力信号突变导致速度指令突变。
     */
    float min_acceleration;

    /**
     * @brief 最大推力指令，单位 N
     *
     * 限制阻抗控制器输出推力，防止过大力损坏物品。
     */
    float max_force;

    /**
     * @brief 最小推力指令，单位 N
     *
     * 限制阻抗控制器输出推力，防止过大力损坏物品。
     */
    float min_force;

    /**
     * @brief 最大位置误差限制，单位 m
     *
     * 防止位置误差过大时 Kd * pos_err 产生过大推力。
     */
    float max_pos_err;

    /**
     * @brief 最大速度误差限制，单位 m/s
     *
     * 防止速度误差过大时 Bd * vel_err 产生过大推力。
     */
    float max_vel_err;

    /**
     * @brief 最大加速度误差限制，单位 m/s^2
     */
    float max_acc_err;
} Compliance_Limit_t;

typedef struct
{
    /**
     * @brief 推力前馈，单位 N
     *
     * 可用于：
     * 1. 重力补偿
     * 2. 摩擦补偿
     * 3. 期望恒定下压力
     *
     */
    float force_ff;

    /**
     * @brief 力误差，单位 N
     *
     */
    float force_err;

    /**
     * @brief 位置误差，单位 m
     *
     */
    float pos_err;

    /**
     * @brief 速度误差，单位 m/s
     *
     */
    float vel_err;

    /**
     * @brief 加速度误差，单位 m/s^2
     *
     */
    float acc_err;

    /**
     * @brief 导纳生成的虚拟位移，单位 m
     *
     * 可以理解为在当前接触力误差下，导纳模型希望产生的位置修正量。
     */
    float x_pos;

    /**
     * @brief 导纳控制的初始位移，单位 m
     *
     * 和虚拟位移相加后得到实际位置指令。
     */
    float base_pos;

    /**
     * @brief 导纳生成的虚拟速度，单位 m/s
     *
     * 通常作为速度环的速度指令 v_cmd。
     */
    float x_vel;

    /**
     * @brief 导纳生成的虚拟加速度，单位 m/s^2
     *
     * 由完整 M-B-K 方程计算得到。
     */
    float x_acc;

    /**
     * @brief 位置项输出推力，单位 N
     */
    float force_p;

    /**
     * @brief 速度阻尼项输出推力，单位 N
     */
    float force_d;

    /**
     * @brief 加速度项输出推力，单位 N
     */
    float force_m;

    /**
     * @brief 最终推力指令，单位 N
     */
    float force_cmd;
} Compliance_Status_t;

typedef struct
{
    float dt;
    Compliance_Config_t config;
    Compliance_Limit_t limit;
    Compliance_Status_t status;
} Compliance_Controller_t;

void Compliance_Controller_Init(Compliance_Controller_t *pCtrl, float dt, Compliance_Config_t *pCfg, Compliance_Limit_t *pLimit);
void Compliance_Controller_Tune(Compliance_Controller_t *pCtrl, float Hz, float Md);
void Compliance_Controller_Reset(Compliance_Controller_t *pCtrl);
void Impedance_Update(Compliance_Controller_t *pCtrl, float pos_err, float vel_err, float acc_err, float force_ff, bool use_md);
void Admittance_Update(Compliance_Controller_t *pCtrl, float force_err, bool use_kd);

