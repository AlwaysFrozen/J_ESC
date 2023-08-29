#include "PID.h"

// #define USE_Differential

void PID_Position_Init(PID_Position_t *pPID,float Ts,float Kp,float Ki,float Kd,float Kb,float limit_p,float limit_n)
{
    memset(pPID,0,sizeof(PID_Position_t));
    pPID->Ts = Ts;
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
    pPID->out_p = pPID->Kp * pPID->err * pPID->Ts;
    pPID->out_i = pPID->Ki * pPID->err_total * pPID->Ts;
    #ifdef USE_Differential
    pPID->out_d = pPID->Kd * (pPID->err - pPID->err_last) * pPID->Ts;
    pPID->out_temp = pPID->out_p + pPID->out_i + pPID->out_d;
    #else
    pPID->out_temp = pPID->out_p + pPID->out_i;
    #endif

    if(pPID->out_temp > pPID->limit_positive)
    {
        pPID->out = pPID->limit_positive;
    }
    else if(pPID->out_temp < pPID->limit_negative) 
    {
        pPID->out = pPID->limit_negative;
    }
    else
    {
        pPID->out = pPID->out_temp;
    }

    pPID->err_total += ((pPID->out - pPID->out_temp) * pPID->Kb + pPID->err) * pPID->Ts;

    return pPID->out;
}

void PID_Delta_Init(PID_Delta_t *pPID,float Ts,float Kp,float Ki,float Kd,float limit_p,float limit_n)
{
    memset(pPID,0,sizeof(PID_Delta_t));
    pPID->Ts = Ts;
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
    pPID->out_p = pPID->Kp * (pPID->err_0 - pPID->err_1) * pPID->Ts;
    pPID->out_i = pPID->Ki * pPID->err_0 * pPID->Ts;
    #ifdef USE_Differential
    pPID->out_d = pPID->Kd * (pPID->err_0 - 2 * pPID->err_1 - pPID->err_2) * pPID->Ts;
    #endif
    pPID->err_1 = pPID->err_0;
    pPID->err_2 = pPID->err_1;

    if(pPID->out_i > pPID->limit_positive) pPID->out_i = pPID->limit_positive;
    if(pPID->out_i < pPID->limit_negative) pPID->out_i = pPID->limit_negative;

    #ifdef USE_Differential
    pPID->out = pPID->out_p + pPID->out_i + pPID->out_d;
    #else
    pPID->out = pPID->out_p + pPID->out_i;
    #endif

    if(pPID->out > pPID->limit_positive) pPID->out = pPID->limit_positive;
    if(pPID->out < pPID->limit_negative) pPID->out = pPID->limit_negative;

    return pPID->out;
}
