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

    float out_p;
    float out_i;
    float out_d;
    float out;
    float out_temp;
        
    float err;
    float err_last;
    float err_total;
    
    float limit_positive;
    float limit_negative;
}PID_Position_t;

typedef struct
{
    float dt;
    float Kp;
    float Ki;
    float Kd;

    float out_p;
    float out_i;
    float out_d;
    float out;

    float err_0;
    float err_1;
    float err_2;

    float limit_positive;
    float limit_negative;
}PID_Delta_t;

void PID_Position_Init(PID_Position_t *pPID,float dt,float Kp,float Ki,float Kd,float Kb,float limit_p,float limit_n);
void PID_Position_Reset(PID_Position_t *pPID);
float PID_Position_Run(PID_Position_t *pPID,float target,float feedback);
void PID_Delta_Init(PID_Delta_t *pPID,float dt,float Kp,float Ki,float Kd,float limit_p,float limit_n);
void PID_Delta_Reset(PID_Delta_t *pPID);
float PID_Delta_Run(PID_Delta_t *pPID,float target,float feedback);

#endif
