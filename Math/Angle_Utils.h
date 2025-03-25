#ifndef __ANGLE_UTILS_H__
#define __ANGLE_UTILS_H__

#include "Utils.h"

float Normalize_Rad(float rad);
float Normalize_Rad_PI(float rad);
float Normalize_Angle(float deg);
float Normalize_Angle_180(float deg);
float ABS_Rad_Diff(float rad0,float rad1);
float ABS_Angle_Diff(float deg0, float deg1);
int8_t Rad_Diff_Dir(float rad0,float rad1);
int8_t Angle_Diff_Dir(float deg0,float deg1);
float Rad_Average(float rad0,float rad1);
float Rad_Weighted_Average(float rad0, float rad1, float weight);
float Angle_Average(float deg0,float deg1);
float Angle_Weighted_Average(float deg0, float deg1, float weight);
float Rad_Average_Mult(float *rads,uint32_t nums);
float Rad_Weighted_Average_Mult(float *rads, float *weight, uint32_t nums);
float Angle_Average_Mult(float *degs,uint32_t nums);
float Angle_Weighted_Average_Mult(float *degs, float *weight, uint32_t nums);

#endif
