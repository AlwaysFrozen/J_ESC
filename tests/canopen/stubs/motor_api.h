#pragma once
#include <stdbool.h>
#include "global.h"
float axis_units_to_motor_units(int index,float value);
bool motion_plan(Motion_Planner_t *,float,float,float,float,float,float,int);
bool velocity_plan(Motion_Planner_t *,float,float,float,float,float,int);
bool motor_enable(int index);
bool motor_disable(int index);
void motor_reset(int index);
void motor_reset_err(int index);
void motor_config_check(int index);
void motor_init(void);
void encoder_init(void);
bool motor_set_align(int index);
bool motor_set_rs_identify(int index);
bool motor_set_ls_identify(int index);
bool motor_set_pp_identify(int index);
bool motor_set_flux_identify(int index);
