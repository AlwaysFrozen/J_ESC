#include "PID.h"
#include "My_Math.h"

// #define Differential_Enable
// #define Kb_Enable

void Current_Loop_PID_Tune(PID_Position_t *pPID,float Wc,float L,float R)
{
    pPID->Kp = L * Wc;
    pPID->Ki = R * Wc;
    pPID->Kb = pPID->Ki * 3;
}

void PID_Position_Init(PID_Position_t *pPID,float dt,float Kp,float Ki,float Kd,float Kb,float limit_p,float limit_n)
{
    memset(pPID,0,sizeof(PID_Position_t));
    pPID->dt = dt;
    pPID->Kp = Kp;
    pPID->Ki = Ki;
    pPID->Kd = Kd;
    pPID->Kb = Kb;
    pPID->limit_positive = limit_p;
    pPID->limit_negative = limit_n;
}

void PID_Position_Reset(PID_Position_t *pPID)
{
    pPID->out_p = 0;
    pPID->out_i = 0;
    pPID->out_d = 0;
    pPID->out = 0;
    pPID->err = 0;
    pPID->err_last = 0;
    pPID->err_total = 0;
}

float PID_Position_Run(PID_Position_t *pPID,float target,float feedback)
{
    pPID->err = target - feedback;
    pPID->out_p = pPID->Kp * pPID->err;
    pPID->out_i = pPID->Ki * pPID->err_total * pPID->dt;
    #ifdef Differential_Enable
    pPID->out_d = pPID->Kd * (pPID->err - pPID->err_last) / pPID->dt;
    pPID->out_temp = pPID->out_p + pPID->out_i + pPID->out_d;
    #else
    pPID->out_temp = pPID->out_p + pPID->out_i;
    #endif

    if(pPID->out_temp > pPID->limit_positive)
    {
        pPID->out = pPID->limit_positive;
        #ifndef Kb_Enable
        if(pPID->err < 0)
        {
            pPID->err_total += pPID->err;
        }
        #endif
    }
    else if(pPID->out_temp < pPID->limit_negative) 
    {
        pPID->out = pPID->limit_negative;
        #ifndef Kb_Enable
        if(pPID->err > 0)
        {
            pPID->err_total += pPID->err;
        }
        #endif
    }
    else
    {
        pPID->out = pPID->out_temp;
        #ifndef Kb_Enable
        pPID->err_total += pPID->err;
        #endif
    }

    #ifdef Kb_Enable
    pPID->err_total += (pPID->out - pPID->out_temp) * pPID->Kb + pPID->err;
    #endif

    return pPID->out;
}

float FeedForward_PID_Position_Run(PID_Position_t *pPID,float feedforward,float target,float feedback)
{
    pPID->err = target - feedback;
    pPID->out_p = pPID->Kp * pPID->err;
    pPID->out_i = pPID->Ki * pPID->err_total * pPID->dt;
    #ifdef Differential_Enable
    pPID->out_d = pPID->Kd * (pPID->err - pPID->err_last) / pPID->dt;
    pPID->out_temp = feedforward + pPID->out_p + pPID->out_i + pPID->out_d;
    #else
    pPID->out_temp = feedforward + pPID->out_p + pPID->out_i;
    #endif

    if(pPID->out_temp > pPID->limit_positive)
    {
        pPID->out = pPID->limit_positive;
        #ifndef Kb_Enable
        if(pPID->err < 0)
        {
            pPID->err_total += pPID->err;
        }
        #endif
    }
    else if(pPID->out_temp < pPID->limit_negative) 
    {
        pPID->out = pPID->limit_negative;
        #ifndef Kb_Enable
        if(pPID->err > 0)
        {
            pPID->err_total += pPID->err;
        }
        #endif
    }
    else
    {
        pPID->out = pPID->out_temp;
        #ifndef Kb_Enable
        pPID->err_total += pPID->err;
        #endif
    }

    #ifdef Kb_Enable
    pPID->err_total += (pPID->out - pPID->out_temp) * pPID->Kb + pPID->err;
    #endif

    return pPID->out;
}

void PID_Delta_Init(PID_Delta_t *pPID,float dt,float Kp,float Ki,float Kd,float limit_p,float limit_n)
{
    memset(pPID,0,sizeof(PID_Delta_t));
    pPID->dt = dt;
    pPID->Kp = Kp;
    pPID->Ki = Ki;
    pPID->Kd = Kd;
    pPID->limit_positive = limit_p;
    pPID->limit_negative = limit_n;
}

void PID_Delta_Reset(PID_Delta_t *pPID)
{
    pPID->out_p = 0;
    pPID->out_i = 0;
    pPID->out_d = 0;
    pPID->out = 0;
    pPID->err_0 = 0;
    pPID->err_1 = 0;
    pPID->err_2 = 0;
}

float PID_Delta_Run(PID_Delta_t *pPID,float target,float feedback)
{
    pPID->err_0 = target - feedback;
    pPID->out_p = pPID->Kp * (pPID->err_0 - pPID->err_1);
    pPID->out_i = pPID->Ki * pPID->err_0 * pPID->dt;
    #ifdef Differential_Enable
    pPID->out_d = pPID->Kd * (pPID->err_0 - 2 * pPID->err_1 - pPID->err_2) / pPID->dt;
    #endif
    pPID->err_1 = pPID->err_0;
    pPID->err_2 = pPID->err_1;

    if(pPID->out_i > pPID->limit_positive) pPID->out_i = pPID->limit_positive;
    if(pPID->out_i < pPID->limit_negative) pPID->out_i = pPID->limit_negative;

    #ifdef Differential_Enable
    pPID->out = pPID->out_p + pPID->out_i + pPID->out_d;
    #else
    pPID->out = pPID->out_p + pPID->out_i;
    #endif

    if(pPID->out > pPID->limit_positive) pPID->out = pPID->limit_positive;
    if(pPID->out < pPID->limit_negative) pPID->out = pPID->limit_negative;

    return pPID->out;
}
