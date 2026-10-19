/**
 * @file utils.h
 * @author jinchl (jcl709905683@gmail.com)
 * @brief 
 * @version 0.1
 * @date 2026-03-11
 * 
 * @copyright Copyright (c) 2026
 * 
 */


#pragma once
#include <math.h>
#include <stdint.h>

#define BITS_MASK_U32(x) ((uint32_t)((x) >= 32U ? UINT32_MAX : ((1UL << (x)) - 1UL)))
#define BITS_MASK_U16(x) ((uint16_t)((x) >= 16U ? UINT16_MAX : ((1UL << (x)) - 1UL)))
#define BITS_MASK_U8(x)  ((uint8_t)((x) >= 8U ? UINT8_MAX : ((1UL << (x)) - 1UL)))

#define MS_TO_S(ms)  ((ms) / 1e3f)
#define S_TO_MS(s)   ((s) * 1e3f)
#define US_TO_MS(us) ((us) / 1e3f)
#define MS_TO_US(ms) ((ms) * 1e3f)
#define US_TO_S(us)  ((us) / 1e6f)
#define S_TO_US(s)   ((s) * 1e6f)

#define FLOAT_MIN      (1e-8f)
#define FLOAT_EPSILON  (1e-6f)
#define FLOAT_EQ(a, b) (fabsf((a) - (b)) < FLOAT_EPSILON)

#define ABS(x)                       ((x) > 0 ? (x) : -(x))
#define SIGN(a)                      (((a) < 0) ? (-1) : ((a) > 0))
#define SIGN_F(x)                    (((x) < 0.0f) ? -1.0f : 1.0f)
#define MIN(a, b)                    (((a) < (b)) ? (a) : (b))
#define MAX(a, b)                    (((a) > (b)) ? (a) : (b))
#define ROUND(x)                     ((x) >= 0 ? (long)((x) + 0.5f) : (long)((x) - 0.5f))
#define CONSTRAIN(amt, low, high)    ((amt) < (low) ? (low) : ((amt) > (high) ? (high) : (amt)))
#define CONSTRAIN_UNSIGNED(amt, max) ((amt) > (max) ? (max) : (amt))
#define ISSET(a)                     ((a) != (NOT_SET))
#define SQ(x)                        ((x) * (x))
#define NORM2_f(x, y)                (sqrtf(SQ(x) + SQ(y)))
#define IS_INF(x)                    ((x) == (1.0f / 0.0f) || (x) == (-1.0f / 0.0f))
#define IS_NAN(x)                    ((x) != (x))
#define NAN_ZERO(x)                  (x = IS_NAN(x) ? 0.0f : x)

#define DEG_TO_RAD(x)   ((x) / 57.295779f)
#define RAD_TO_DEG(x)   ((x) * 57.295779f)
#define M_TO_MM(x)      ((x) * 1e3f)
#define MM_TO_M(x)      ((x) / 1e3f)
#define RPM_TO_OMEGA(x) ((x) * _2PI / 60.0f)
#define OMEGA_TO_RPM(x) ((x) / _2PI * 60.0f)

#define _1_3 (0.33333333333f)
#define _2_3 (0.66666666666f)
#define _3_2 (1.5f)

#define _SQRT3   (1.73205080757f)
#define _2_SQRT3 (1.15470053838f)
#define _1_SQRT3 (0.57735026919f)
#define _SQRT3_2 (0.86602540378f)
#define _SQRT2   (1.41421356237f)
#define _1_SQRT2 (0.70710678118f)

#define _PI_6   (0.52359877559f)
#define _PI_4   (0.78539816339f)
#define _PI_3   (1.04719755119f)
#define _PI_2   (1.57079632679f)
#define _2PI_3  (2.09439510239f)
#define _5PI_6  (2.61799387799f)
#define _PI     (3.14159265359f)
#define _7PI_6  (3.66519142918f)
#define _4PI_3  (4.18879020478f)
#define _3PI_2  (4.71238898038f)
#define _5PI_3  (5.23598775598f)
#define _11PI_6 (5.75958653158f)
#define _2PI    (6.28318530718f)

#define _e (2.71828182845f)

#define GRAVITY_ACC       (9.80665f)
#define GRAM_TO_KG(g)     ((g) * 1e-3f)
#define KG_TO_GRAM(kg)    ((kg) * 1e3f)
#define GRAM_TO_NEWTON(g) (GRAM_TO_KG(g) * GRAVITY_ACC)
#define NEWTON_TO_GRAM(n) (KG_TO_GRAM((n) / GRAVITY_ACC))
#define KG_TO_NEWTON(kg)  ((kg) * GRAVITY_ACC)
#define NEWTON_TO_KG(n)   ((n) / GRAVITY_ACC)

float Max_Abs(float va, float vb);
float Min_Abs(float va, float vb);
void Truncate_Number(float *number, float min, float max);
void Truncate_Number_Abs(float *number, float max);
uint8_t Number_In_Absolute_Range_f32(float num, float base, float range);
