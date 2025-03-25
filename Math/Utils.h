#ifndef __UTILS_H__
#define __UTILS_H__

#include "math.h"
#include "stdbool.h"
#include "stdint.h"
#include "stdlib.h"
#include "string.h"

#define ABS(x)                                  ((x) > 0 ? (x) : -(x))
#define SIGN(a)                                 (((a) < 0) ? (-1) : ((a) > 0))
#define SIGN_F(x)                               (((x) < 0.0f) ? -1.0f : 1.0f)
#define MIN(a, b)                               (((a) < (b)) ? (a) : (b))
#define MAX(a, b)                               (((a) > (b)) ? (a) : (b))
#define ROUND(x)                                ((x) >= 0 ? (long)((x) + 0.5f) : (long)((x)-0.5f))
#define CONSTRAIN(amt, low, high)               ((amt) < (low) ? (low) : ((amt) > (high) ? (high) : (amt)))
#define CONSTRAIN_UNSIGNED(amt, max)            ((amt) > (max) ? (max) : (amt))
#define ISSET(a)                                ((a) != (NOT_SET))
#define SQ(x)                                   ((x) * (x))
#define NORM2_f(x, y)                           (sqrtf(SQ(x) + SQ(y)))
#define IS_INF(x)                               ((x) == (1.0f / 0.0f) || (x) == (-1.0f / 0.0f))
#define IS_NAN(x)                               ((x) != (x))
#define NAN_ZERO(x)                             (x = IS_NAN(x) ? 0.0f : x)

#define DEG_TO_RAD(x)                           (x / 57.295779f)
#define RAD_TO_DEG(x)                           (x * 57.295779f)

#define _1_3                                    (0.33333333333f)
#define _2_3                                    (0.66666666666f)

#define _SQRT3                                  (1.73205080757f)
#define _2_SQRT3                                (1.15470053838f)
#define _1_SQRT3                                (0.57735026919f)
#define _SQRT3_2                                (0.86602540378f)
#define _SQRT2                                  (1.41421356237f)

#define _PI_6                                   (0.52359877559f)
#define _PI_4                                   (0.78539816339f)
#define _PI_3                                   (1.04719755119f)
#define _PI_2                                   (1.57079632679f)
#define _2PI_3                                  (2.09439510239f)
#define _5PI_6                                  (2.61799387799f)
#define _PI                                     (3.14159265359f)
#define _7PI_6                                  (3.66519142918f)
#define _3PI_2                                  (4.71238898038f)
#define _11PI_6                                 (5.75958653158f)
#define _2PI                                    (6.28318530718f)

#define _e                                      (2.71828182845f)

float Max_Abs(float va, float vb);
float Min_Abs(float va, float vb);
void Truncate_Number(float *number, float min, float max);
void Truncate_Number_Abs(float *number, float max);
uint8_t Number_In_Absolute_Range_f32(float num,float base,float range);

#endif
