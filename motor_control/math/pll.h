/**
 * @file pll.h
 * @author jinchl (jcl709905683@gmail.com)
 * @brief 
 * @version 0.1
 * @date 2026-03-11
 * 
 * @copyright Copyright (c) 2026
 * 
 */


#pragma once
#include "utils.h"
#include "filter.h"

#ifndef PLL_LOCK_TIME
#define PLL_LOCK_TIME (1e-2f)
#endif

#ifndef PLL_LOCK_THRESHOLD
#define PLL_LOCK_THRESHOLD (DEG_TO_RAD(5))
#endif

typedef struct
{
    float kp;
    float ki;
    float fc;
    float we_limit;
    float dt;

    float err;
    float integral;
    float we;   // rads/s
    float we_f; // rads/s
    float hz;
    float hz_f;
    float delta_theta; // rads/ts
    float theta;
    float theta_unlimit;

    bool is_locked;
    float lock_time;

    float alpha;
    Filter_t *p_filter;

} PLL_t;

void PLL_Init(PLL_t *pPLL, float kp, float ki, float fc, float dt, Filter_t *p_filter);
void QPLL_Init(PLL_t *pPLL, float fc, float hz_limit, float dt, Filter_t *p_filter);
void QPLL_Run(PLL_t *pPLL, float sin_value, float cos_value, bool normalize);

typedef struct
{
    float kp;
    float ki;
    float err;
    float dt;
    float vel_estimate;
    float acc_estimate;
} Velocity_Estimator_t;

void Velocity_Estimator_Init(Velocity_Estimator_t *pEstimator, float kp, float ki, float dt);
void Velocity_Estimator_Update(Velocity_Estimator_t *pEstimator, float vel_fb);

typedef struct
{
    float Kp_pos;
    float Ki_pos;
    float err_pos;
    float Kp_vel;
    float Ki_vel;
    float err_vel;
    float dt;
    float pos_estimate;
    float vel_estimate;
    float vel_estimate2;
    float acc_estimate;
} Position_Estimator_t;

void Position_Estimator_Init(Position_Estimator_t *pEstimator, float Kp_pos, float Ki_pos, float Kp_vel, float Ki_vel, float dt);
void Position_Estimator_Update(Position_Estimator_t *pEstimator, float pos_fb);
