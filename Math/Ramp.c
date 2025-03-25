#include "Ramp.h"

void Ramp_Init(Ramp_t *ramp,float in,float out,float output_max,float output_min,float rise_time_s,float fall_time_s,float dt)
{
    ramp->input = in;
    ramp->output = out;
    ramp->output_max = output_max;
    ramp->output_min = output_min;
    ramp->rise_time_s = rise_time_s;
    ramp->fall_time_s = fall_time_s;
    ramp->dt = dt;

    if(ramp->rise_time_s)
    {
        ramp->delta_max = (ramp->output_max - ramp->output_min) / ramp->rise_time_s * ramp->dt;
    }
    else
    {
        ramp->delta_max = ramp->output_max - ramp->output_min;
    }
    if(ramp->fall_time_s)
    {
        ramp->delta_min = (ramp->output_min - ramp->output_max) / ramp->fall_time_s * ramp->dt;
    }
    else
    {
        ramp->delta_min = ramp->output_min - ramp->output_max;
    }
}

float Ramp_Update(Ramp_t *ramp,float in)
{
    ramp->input = in;
    ramp->delta = ramp->input - ramp->output;
    ramp->delta = CONSTRAIN(ramp->delta,ramp->delta_min,ramp->delta_max);
    ramp->output += ramp->delta;
    ramp->output = CONSTRAIN(ramp->output,ramp->output_min,ramp->output_max);

    return ramp->output;
}
