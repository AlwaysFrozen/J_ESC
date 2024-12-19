#ifndef __PID_H__
#define __PID_H__

#include "math.h"
#include "stdint.h"
#include "stdbool.h"
#include "string.h"

typedef struct
{
    float dt;
    float Kp;
    float Ki;
    float Kd;
    float Kb;

    float Proportional;
    float Integral;
    float Derivative;
    
    float out;
    float out_temp;
        
    float err_k0;
    float err_k1;
    float err_k2;
    
    float limit_max;
    float limit_min;

    bool saturation;
}PID_t;


void Current_Loop_Parallel_PID_Tune(PID_t *pPID,float Hz,float L,float R);
void Current_Loop_Serial_PID_Tune(PID_t *pPID,float Hz,float L,float R);
void PID_Init(PID_t *pPID,float dt,float Kp,float Ki,float Kd,float Kb,float limit_max,float limit_min);
void PID_Reset(PID_t *pPID);
void PID_Set_Limit(PID_t *pPID,float limit_max,float limit_min);
void PID_Set_Abs_Limit(PID_t *pPID,float limit);
float Parallel_PID_Position_Run(PID_t *pPID,float target,float feedback);
float FeedForward_Parallel_PID_Position_Run(PID_t *pPID,float feedforward,float target,float feedback);
float Serial_PID_Position_Run(PID_t *pPID,float target,float feedback);
float FeedForward_Serial_PID_Position_Run(PID_t *pPID,float feedforward,float target,float feedback);
float PID_Delta_Run(PID_t *pPID,float target,float feedback);
float FeedForward_PID_Delta_Run(PID_t *pPID,float feedforward,float target,float feedback);
float IP_Run(PID_t *pPID,float target,float feedback);
float FeedForward_IP_Run(PID_t *pPID,float target,float feedback,float feedforward);

#endif
