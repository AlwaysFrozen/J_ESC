/**
 * @file PLL.c
 * @author jinchl (jcl709905683@gmail.com)
 * @brief 
 * @version 0.1
 * @date 2026-03-11
 * 
 * @copyright Copyright (c) 2026
 * 
 */


#include "pll.h"
#include "angle_utils.h"
#include "filter.h"

// https://zhuanlan.zhihu.com/p/673354158

void PLL_Init(PLL_t *pPLL, float kp, float ki, float fc, float dt, Filter_t *p_filter)
{
    memset(pPLL, 0, sizeof(PLL_t));
    pPLL->kp = kp;
    pPLL->ki = ki;
    pPLL->fc = fc;
    pPLL->dt = dt;

    float t     = 1.0f / (_2PI * fc);
    pPLL->alpha = CONSTRAIN((dt / (dt + t)), 0.0f, 1.0f);

    pPLL->p_filter = p_filter;
}

// https://zhuanlan.zhihu.com/p/665627084
void QPLL_Init(PLL_t *pPLL, float fc, float hz_limit, float dt, Filter_t *p_filter)
{
    memset(pPLL, 0, sizeof(PLL_t));

    float zeta = 0.707f;
    float wn   = _2PI * fc;

    pPLL->kp       = 2 * zeta * wn;
    pPLL->ki       = SQ(wn);
    pPLL->fc       = fc;
    pPLL->we_limit = _2PI * hz_limit;
    pPLL->dt       = dt;

    float t     = 1.0f / (_2PI * fc);
    pPLL->alpha = CONSTRAIN((dt / (dt + t)), 0.0f, 1.0f);

    pPLL->p_filter = p_filter;
}

void QPLL_Run(PLL_t *pPLL, float sin_value, float cos_value, bool normalize)
{
    if (normalize)
    {
        float mag = NORM2_f(sin_value, cos_value);
        if (mag < 1e-6f)
        {
            pPLL->we        = 0;
            pPLL->we_f      = 0;
            pPLL->hz        = 0;
            pPLL->hz_f      = 0;
            pPLL->is_locked = false;
            pPLL->lock_time = 0;
            return;
        }
        sin_value = sin_value / mag;
        cos_value = cos_value / mag;
    }

    pPLL->err            = sin_value * cosf(pPLL->theta) - cos_value * sinf(pPLL->theta);
    pPLL->integral      += pPLL->err * pPLL->ki * pPLL->dt;
    pPLL->integral       = CONSTRAIN(pPLL->integral, -pPLL->we_limit, pPLL->we_limit);
    pPLL->we             = pPLL->err * pPLL->kp + pPLL->integral;
    pPLL->we             = CONSTRAIN(pPLL->we, -pPLL->we_limit, pPLL->we_limit);
    pPLL->hz             = pPLL->we / _2PI;
    pPLL->delta_theta    = pPLL->we * pPLL->dt;
    pPLL->theta_unlimit += pPLL->delta_theta;
    pPLL->theta         += pPLL->delta_theta;
    pPLL->theta          = Normalize_Rad(pPLL->theta);

    if (pPLL->p_filter != NULL)
    {
        pPLL->we_f = Process_Filter(pPLL->p_filter, pPLL->we);
        pPLL->hz_f = pPLL->we_f / _2PI;
    }
    else
    {
        pPLL->we_f += (pPLL->we - pPLL->we_f) * pPLL->alpha;
        pPLL->hz_f  = pPLL->we_f / _2PI;
    }

    if (fabsf(pPLL->err) < PLL_LOCK_THRESHOLD)
    {
        pPLL->lock_time += pPLL->dt;
        if (pPLL->lock_time > PLL_LOCK_TIME)
        {
            pPLL->is_locked = true;
        }
    }
    else
    {
        pPLL->is_locked = false;
        pPLL->lock_time = 0;
    }
}

void Velocity_Estimator_Init(Velocity_Estimator_t *pEstimator, float kp, float ki, float dt)
{
    memset(pEstimator, 0, sizeof(Velocity_Estimator_t));
    pEstimator->kp = kp;
    pEstimator->ki = ki;
    pEstimator->dt = dt;
}

void Velocity_Estimator_Update(Velocity_Estimator_t *pEstimator, float vel_fb)
{
    pEstimator->vel_estimate += pEstimator->acc_estimate * pEstimator->dt;
    pEstimator->err           = vel_fb - pEstimator->vel_estimate;
    pEstimator->vel_estimate += pEstimator->dt * pEstimator->err * pEstimator->kp;
    pEstimator->acc_estimate += pEstimator->dt * pEstimator->err * pEstimator->ki;
}

void Position_Estimator_Init(Position_Estimator_t *pEstimator, float Kp_pos, float Ki_pos, float Kp_vel, float Ki_vel, float dt)
{
    memset(pEstimator, 0, sizeof(Position_Estimator_t));
    pEstimator->Kp_pos = Kp_pos;
    pEstimator->Ki_pos = Ki_pos;
    pEstimator->Kp_vel = Kp_vel;
    pEstimator->Ki_vel = Ki_vel;
    pEstimator->dt     = dt;
}

void Position_Estimator_Update(Position_Estimator_t *pEstimator, float pos_fb)
{
    pEstimator->vel_estimate2 += pEstimator->acc_estimate * pEstimator->dt;
    pEstimator->pos_estimate  += pEstimator->vel_estimate * pEstimator->dt;
    pEstimator->pos_estimate   = Normalize_Rad(pEstimator->pos_estimate);
    float dir                  = Rad_Move_Dir(pos_fb, pEstimator->pos_estimate);
    pEstimator->err_pos        = dir * ABS_Rad_Delta(pos_fb, pEstimator->pos_estimate);
    pEstimator->err_vel        = pEstimator->vel_estimate - pEstimator->vel_estimate2;
    pEstimator->pos_estimate  += pEstimator->dt * pEstimator->err_pos * pEstimator->Kp_pos;
    pEstimator->pos_estimate   = Normalize_Rad(pEstimator->pos_estimate);
    pEstimator->vel_estimate  += pEstimator->dt * pEstimator->err_pos * pEstimator->Ki_pos;
    pEstimator->vel_estimate2 += pEstimator->dt * pEstimator->err_vel * pEstimator->Kp_vel;
    pEstimator->acc_estimate  += pEstimator->dt * pEstimator->err_vel * pEstimator->Ki_vel;
}
