#include "PID.h"
#include "My_Math.h"

// #define Kb_Enable

// https://zhuanlan.zhihu.com/p/146373628
// https://blog.csdn.net/wanrenqi/article/details/131523107
void Current_Loop_Parallel_PID_Tune(PID_t *pPID,float Hz,float L,float R)
{
    float Wc = Hz * _2PI;
    #if 0
    pPID->Kp = L * Wc;
    pPID->Ki = R * Wc;
    #else
    float b = Wc * pPID->dt;
    float a = b * (sqrtf(SQ(sinf(b)) + 1) - sinf(b));
    pPID->Kp = L * a / pPID->dt;
    pPID->Ki = R * a / pPID->dt;
    #endif
    pPID->Kb = pPID->Ki * 3;
}

// https://zhuanlan.zhihu.com/p/454914546
void Current_Loop_Serial_PID_Tune(PID_t *pPID,float Hz,float L,float R)
{
    float Wc = Hz * _2PI;

    #if 1
    pPID->Kp = L * Wc;
    pPID->Ki = R / L;
    #else
    pPID->Kp = L * Wc;
    pPID->Ki = Wc * R / L;
    #endif
}

void PID_Init(PID_t *pPID,float dt,float Kp,float Ki,float Kd,float Kb,float limit_max,float limit_min)
{
    memset(pPID,0,sizeof(PID_t));
    pPID->dt = dt;
    pPID->Kp = Kp;
    pPID->Ki = Ki;
    pPID->Kd = Kd;
    pPID->Kb = Kb;
    pPID->limit_max = limit_max;
    pPID->limit_min = limit_min;
}

void PID_Reset(PID_t *pPID)
{
    pPID->Proportional = 0;
    pPID->Integral = 0;
    pPID->Derivative = 0;
    pPID->out = 0;
    pPID->err_k0 = 0;
    pPID->err_k1 = 0;
    pPID->err_k2 = 0;
}

void PID_Set_Limit(PID_t *pPID,float limit_max,float limit_min)
{
    pPID->limit_max = limit_max;
    pPID->limit_min = limit_min;
}

void PID_Set_Abs_Limit(PID_t *pPID,float limit)
{
    pPID->limit_max = limit;
    pPID->limit_min = -limit;
}

float Parallel_PID_Position_Run(PID_t *pPID,float target,float feedback)
{
    pPID->err_k0 = target - feedback;
    pPID->Proportional = pPID->Kp * pPID->err_k0;
    pPID->Integral += pPID->Ki * pPID->err_k0 * pPID->dt;
    #ifndef Kb_Enable
    pPID->Integral = _constrain(pPID->Integral,pPID->limit_min,pPID->limit_max);
    #endif
    pPID->Derivative = pPID->Kd * (pPID->err_k0 - pPID->err_k1) / pPID->dt;
    pPID->out_temp = pPID->Proportional + pPID->Integral + pPID->Derivative;
    pPID->out = _constrain(pPID->out_temp,pPID->limit_min,pPID->limit_max);
    pPID->saturation = (pPID->out != pPID->out_temp);
    #ifdef Kb_Enable
    pPID->Integral += ((pPID->out - pPID->out_temp) * pPID->Kb + pPID->err_k0) * pPID->dt;
    #endif
    pPID->err_k1 = pPID->err_k0;
    pPID->err_k2 = pPID->err_k1;

    return pPID->out;
}

float FeedForward_Parallel_PID_Position_Run(PID_t *pPID,float feedforward,float target,float feedback)
{
    pPID->err_k0 = target - feedback;
    pPID->Proportional = pPID->Kp * pPID->err_k0;
    pPID->Integral += pPID->Ki * pPID->err_k0 * pPID->dt;
    #ifndef Kb_Enable
    pPID->Integral = _constrain(pPID->Integral,pPID->limit_min,pPID->limit_max);
    #endif
    pPID->Derivative = pPID->Kd * (pPID->err_k0 - pPID->err_k1) / pPID->dt;
    pPID->out_temp = feedforward + pPID->Proportional + pPID->Integral + pPID->Derivative;
    pPID->out = _constrain(pPID->out_temp,pPID->limit_min,pPID->limit_max);
    pPID->saturation = (pPID->out != pPID->out_temp);
    #ifdef Kb_Enable
    pPID->Integral += ((pPID->out - pPID->out_temp) * pPID->Kb + pPID->err_k0) * pPID->dt;
    #endif
    pPID->err_k1 = pPID->err_k0;
    pPID->err_k2 = pPID->err_k1;

    return pPID->out;
}

float Serial_PID_Position_Run(PID_t *pPID,float target,float feedback)
{
    pPID->err_k0 = target - feedback;
    pPID->Proportional = pPID->Kp * pPID->err_k0;
    pPID->Integral += pPID->Ki * pPID->Proportional * pPID->dt;
    pPID->Integral = _constrain(pPID->Integral,pPID->limit_min,pPID->limit_max);
    pPID->Derivative = pPID->Kd * (pPID->err_k0 - pPID->err_k1) / pPID->dt;
    pPID->out_temp = pPID->Proportional + pPID->Integral + pPID->Derivative;
    pPID->out = _constrain(pPID->out_temp,pPID->limit_min,pPID->limit_max);
    pPID->saturation = (pPID->out != pPID->out_temp);
    pPID->err_k1 = pPID->err_k0;
    pPID->err_k2 = pPID->err_k1;

    return pPID->out;
}

float FeedForward_Serial_PID_Position_Run(PID_t *pPID,float feedforward,float target,float feedback)
{
    pPID->err_k0 = target - feedback;
    pPID->Proportional = pPID->Kp * pPID->err_k0;
    pPID->Integral += pPID->Ki * pPID->Proportional * pPID->dt;
    pPID->Derivative = pPID->Kd * (pPID->err_k0 - pPID->err_k1) / pPID->dt;
    pPID->out_temp = feedforward + pPID->Proportional + pPID->Integral + pPID->Derivative;
    pPID->out = _constrain(pPID->out_temp,pPID->limit_min,pPID->limit_max);
    pPID->saturation = (pPID->out != pPID->out_temp);
    pPID->err_k1 = pPID->err_k0;
    pPID->err_k2 = pPID->err_k1;

    return pPID->out;
}

float PID_Delta_Run(PID_t *pPID,float target,float feedback)
{
    pPID->err_k0 = target - feedback;
    pPID->Proportional = pPID->Kp * (pPID->err_k0 - pPID->err_k1);
    pPID->Integral = pPID->Ki * pPID->err_k0 * pPID->dt;
    pPID->Integral = _constrain(pPID->Integral,pPID->limit_min,pPID->limit_max);
    pPID->Derivative = pPID->Kd * (pPID->err_k0 - 2 * pPID->err_k1 + pPID->err_k2) / pPID->dt;
    pPID->err_k1 = pPID->err_k0;
    pPID->err_k2 = pPID->err_k1;
    pPID->out_temp = pPID->Proportional + pPID->Integral + pPID->Derivative;
    pPID->out = _constrain(pPID->out_temp,pPID->limit_min,pPID->limit_max);
    pPID->saturation = (pPID->out != pPID->out_temp);

    return pPID->out;
}

float FeedForward_PID_Delta_Run(PID_t *pPID,float feedforward,float target,float feedback)
{
    pPID->err_k0 = target - feedback;
    pPID->Proportional = pPID->Kp * (pPID->err_k0 - pPID->err_k1);
    pPID->Integral = pPID->Ki * pPID->err_k0 * pPID->dt;
    pPID->Integral = _constrain(pPID->Integral,pPID->limit_min,pPID->limit_max);
    pPID->Derivative = pPID->Kd * (pPID->err_k0 - 2 * pPID->err_k1 + pPID->err_k2) / pPID->dt;
    pPID->err_k1 = pPID->err_k0;
    pPID->err_k2 = pPID->err_k1;
    pPID->out_temp = feedforward + pPID->Proportional + pPID->Integral + pPID->Derivative;
    pPID->out = _constrain(pPID->out_temp,pPID->limit_min,pPID->limit_max);
    pPID->saturation = (pPID->out != pPID->out_temp);

    return pPID->out;
}

float IP_Run(PID_t *pPID,float target,float feedback)
{
    pPID->err_k0 = target - feedback;
    pPID->Proportional = pPID->Kp * feedback;
    // pPID->out_temp = -pPID->Proportional;
    pPID->Integral += pPID->Ki * pPID->err_k0 * pPID->dt;
    pPID->Integral = _constrain(pPID->Integral,pPID->Proportional + pPID->limit_min,pPID->Proportional + pPID->limit_max);
    // pPID->out_temp += _pPID->Integral;
    // pPID->out = _constrain(pPID->out_temp,pPID->limit_min,pPID->limit_max);
    // pPID->saturation = (pPID->out != pPID->out_temp);
    pPID->out = pPID->Integral - pPID->Proportional;
    // pPID->err_k1 = pPID->err_k0;
    // pPID->err_k2 = pPID->err_k1;

    return pPID->out;
}

float FeedForward_IP_Run(PID_t *pPID,float target,float feedback,float feedforward)
{
    pPID->err_k0 = target - feedback;
    pPID->Proportional = pPID->Kp * feedback;
    // pPID->out_temp = -pPID->Proportional;
    pPID->Integral += pPID->Ki * pPID->err_k0 * pPID->dt;
    pPID->Integral = _constrain(pPID->Integral,pPID->Proportional + pPID->limit_min - feedforward,pPID->Proportional + pPID->limit_max - feedforward);
    // pPID->out_temp += _pPID->Integral;
    // pPID->out = _constrain(pPID->out_temp,pPID->limit_min,pPID->limit_max);
    // pPID->saturation = (pPID->out != pPID->out_temp);
    pPID->out = pPID->Integral - pPID->Proportional + feedforward;
    // pPID->err_k1 = pPID->err_k0;
    // pPID->err_k2 = pPID->err_k1;

    return pPID->out;
}

