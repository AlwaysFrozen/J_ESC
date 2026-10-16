/**
 * @file motion_planner.c
 * @author jinchl (jcl709905683@gmail.com)
 * @brief 
 * @version 0.1
 * @date 2026-04-01
 * 
 * @copyright Copyright (c) 2026
 * 
 */


#include "motion_planner.h"
#include "global.h"

float p2(float t, float x0, float v0, float a)
{
    return x0 + v0 * t + 0.5f * a * t * t;
}

bool trapezoid_prepare(Motion_Planner_t *c, float current_pos,
                       float target_pos, float acc_peak,
                       float vel_peak, float dt)
{
    c->feedback = current_pos;
    c->target   = target_pos;
    c->tick     = 0;
    if (target_pos < current_pos)
    {
        float tmp   = current_pos;
        current_pos = target_pos;
        target_pos  = tmp;
        c->dir      = -1;
    }
    else
    {
        c->dir = 1;
    }

    float v      = vel_peak;
    float v0     = 0.0f;
    float acc    = acc_peak;
    float x0     = 0;
    float x_goal = target_pos - current_pos;

    c->dt1    = (v - v0) / acc;
    float dx1 = p2(c->dt1, 0, v0, acc);
    c->dt3    = v / acc;
    float dx3 = p2(c->dt3, 0, v, -acc);
    c->dt2    = (x_goal - (x0 + dx1 + dx3)) / v;

    float t3 = c->dt1 + c->dt2 + c->dt3;

    if (c->dt2 < 0)
    {
        v      = sqrtf(acc_peak * (x_goal - x0) + 0.5 * v0 * v0);
        t3     = (v - v0) / acc + v / acc;
        c->dt1 = t3 / 2.0f;
        c->dt2 = 0;
    }

    c->v2          = acc * c->dt1;
    c->total_ticks = ceilf(t3 / dt);
    c->total_t     = c->total_ticks * dt;
    c->dt          = dt;
    c->vmax        = v;
    c->accmax      = acc;

    c->cx1 = p2(c->dt1, 0, 0, c->accmax);
    c->cx2 = p2(c->dt2, c->cx1, c->vmax, 0);

    return true;
}

bool trapezoid_control_tick(Motion_Planner_t *c)
{
    if (c->tick > c->total_ticks)
    {
        return false;
    }

    c->tick = c->tick + 1;
    float t = c->tick * c->dt;
    float pos_out;

    if (t <= c->dt1 + c->dt)
    {
        c->x1 = p2(t, 0, 0, c->accmax);
        c->x2 = c->x1;

        pos_out       = c->x1;
        c->vel_output = c->dir * c->accmax * t;
        c->acc_output = c->dir * c->accmax;
    }
    else if (t < (c->dt1 + c->dt2))
    {
        c->x2 = p2(t - c->dt1, c->cx1, c->vmax, 0);

        pos_out       = c->x2;
        c->vel_output = c->dir * c->vmax;
        c->acc_output = 0;
    }
    else
    {
        pos_out = p2(t - (c->dt1 + c->dt2), c->cx2, c->v2, -c->accmax);

        float vel_output = c->vmax - (t - (c->dt1 + c->dt2)) * c->accmax;
        if (vel_output < 0)
            vel_output = 0;
        c->vel_output = c->dir * vel_output;
        c->acc_output = -c->dir * c->accmax;
    }

    c->pos_output  = c->feedback + c->dir * pos_out;
    c->jerk_output = 0;

    return true;
}

bool cycloidal_prepare(Motion_Planner_t *c, float current_pos,
                       float target_pos, float acc_peak,
                       float vel_peak, float jerk_peak, float dt)
{
    c->feedback = current_pos;
    c->target   = target_pos;
    c->tick     = 0;
    c->dt       = dt;

    float distance = fabsf(target_pos - current_pos);

    // 计算满足速度、加速度和jerk限制的最小时间
    float T_v = (2.0f * distance) / vel_peak;
    float T_a = sqrtf((2.0f * _PI * distance) / acc_peak);
    float T_j = cbrtf((4.0f * _PI * _PI * distance) / jerk_peak);

    // 选择最大的时间，以满足所有限制
    float T = fmaxf(fmaxf(T_v, T_a), T_j);

    c->total_ticks = ceilf(T / dt);
    c->total_t     = c->total_ticks * dt;
    c->v0          = (target_pos - current_pos) / c->total_t;
    c->a0          = c->v0 * (2.0f * _PI / c->total_t);
    return true;
}

bool cycloidal_tick(Motion_Planner_t *c)
{
    if (c->tick >= c->total_ticks)
    {
        return false; // 运动完成
    }

    c->tick++;

    float x1              = c->feedback;
    float x2              = c->target;
    float normalized_time = c->tick / c->total_ticks;

    float sin_value = sinf(2.0f * _PI * normalized_time);
    float cos_value = cosf(2.0f * _PI * normalized_time);

    // 位置计算，使用摆线公式
    float cycloidal_factor = normalized_time - (1.0f / (2.0f * _PI)) * sin_value;
    c->pos_output          = x1 + (x2 - x1) * cycloidal_factor;

    // 速度计算
    c->vel_output = c->v0 * (1.0f - cos_value);

    // 加速度计算
    c->acc_output  = c->a0 * sin_value;
    c->jerk_output = c->a0 * (2.0f * _PI / c->total_t) * cos_value;

    return true;
}

bool velocity_trapezoid_prepare(Motion_Planner_t *c, float current_vel,
                                float target_vel, float a_max, float dt)
{
    c->feedback    = current_vel;
    c->target      = target_vel;
    c->tick        = 0;
    c->dt          = dt;
    float dv       = fabsf(target_vel - current_vel);
    c->dir         = (target_vel > current_vel) ? 1 : -1;
    c->accmax      = a_max;
    c->total_t     = dv / a_max;
    c->total_ticks = ceilf(c->total_t / dt);
    c->total_t     = c->total_ticks * dt;
    return true;
}

bool velocity_trapezoid_tick(Motion_Planner_t *c)
{
    if (c->tick >= c->total_ticks)
    {
        return false;
    }
    c->tick++;
    float t = c->tick * c->dt;
    if (t > c->total_t)
        t = c->total_t;
    c->pos_output  = 0;
    c->vel_output  = c->feedback + c->dir * c->accmax * t;
    c->acc_output  = c->dir * c->accmax;
    c->jerk_output = 0;

    return true;
}

bool velocity_cycloidal_prepare(Motion_Planner_t *c, float current_vel,
                                float target_vel, float a_max, float j_max, float dt)
{
    c->feedback    = current_vel;
    c->target      = target_vel;
    c->tick        = 0;
    c->dt          = dt;
    float distance = fabsf(target_vel - current_vel);
    // 计算满足速度、加速度和jerk限制的最小时间
    float T_a      = (2.0f * distance) / a_max;
    float T_j      = sqrtf((2.0f * _PI * distance) / j_max);
    float T        = fmaxf(T_a, T_j);
    c->total_ticks = ceilf(T / dt);
    c->total_t     = c->total_ticks * dt;
    c->v0          = (target_vel - current_vel) / c->total_t;
    c->a0          = c->v0 * (2.0f * _PI / c->total_t);

    return true;
}

bool velocity_cycloidal_tick(Motion_Planner_t *c)
{
    if (c->tick >= c->total_ticks)
    {
        return false;
    }
    c->tick++;
    float normalized_time = (float)c->tick / c->total_ticks;
    if (normalized_time > 1.0f)
        normalized_time = 1.0f;
    float sin_value        = sinf(2.0f * _PI * normalized_time);
    float cos_value        = cosf(2.0f * _PI * normalized_time);
    float cycloidal_factor = normalized_time - (1.0f / (2.0f * _PI)) * sin_value;
    c->vel_output          = c->feedback + (c->target - c->feedback) * cycloidal_factor;
    c->acc_output          = c->v0 * (1.0f - cos_value);
    c->jerk_output         = c->a0 * sin_value;

    return true;
}

bool motion_plan(Motion_Planner_t *c, float current_pos,
                 float target_pos, float acc_peak,
                 float vel_peak, float jerk_peak, float dt, int type)
{
    c->plan_type   = type;
    c->plan_target = Plan_Target_Pos;

    switch (type)
    {
    default:
    case Plan_Type_Cycloidal:
        return cycloidal_prepare(c, current_pos, target_pos, acc_peak, vel_peak, jerk_peak, dt);

    case Plan_Type_Trapezoid:
        return trapezoid_prepare(c, current_pos, target_pos, acc_peak, vel_peak, dt);

    case Plan_Type_None:
        c->target = target_pos;
        return false;
    }
}

bool motion_update(Motion_Planner_t *c)
{
    if (c->plan_target != Plan_Target_Pos)
    {
        return false;
    }

    switch (c->plan_type)
    {
    default:
    case Plan_Type_Cycloidal:
        return cycloidal_tick(c);

    case Plan_Type_Trapezoid:
        return trapezoid_control_tick(c);

    case Plan_Type_None:
        return false;
    }
}

bool velocity_plan(Motion_Planner_t *c, float current_vel,
                   float target_vel, float acc_peak,
                   float jerk_peak, float dt, int type)
{
    c->plan_type   = type;
    c->plan_target = Plan_Target_Vel;

    switch (type)
    {
    default:
    case Plan_Type_Cycloidal:
        return velocity_cycloidal_prepare(c, current_vel, target_vel, acc_peak, jerk_peak, dt);

    case Plan_Type_Trapezoid:
        return velocity_trapezoid_prepare(c, current_vel, target_vel, acc_peak, dt);

    case Plan_Type_None:
        c->target = target_vel;
        return false;
    }
}

bool velocity_update(Motion_Planner_t *c)
{
    if (c->plan_target != Plan_Target_Vel)
    {
        return false;
    }

    switch (c->plan_type)
    {
    default:
    case Plan_Type_Cycloidal:
        return velocity_cycloidal_tick(c);

    case Plan_Type_Trapezoid:
        return velocity_trapezoid_tick(c);

    case Plan_Type_None:
        return false;
    }
}
