#ifndef __RAMP_H__
#define __RAMP_H__

#include "Utils.h"

typedef struct
{
    float input;
    float output;
    float output_max;
    float output_min;
    float rise_time_s;
    float fall_time_s;
    float dt;
    float delta;
    float delta_max;
    float delta_min;
}Ramp_t;

void Ramp_Init(Ramp_t *ramp,float in,float out,float output_max,float output_min,float rise_time_s,float fall_time_s,float dt);
float Ramp_Update(Ramp_t *ramp,float in);


#endif
