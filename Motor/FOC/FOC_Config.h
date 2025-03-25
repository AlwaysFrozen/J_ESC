#ifndef __FOC_CONFIG_H__
#define __FOC_CONFIG_H__

#include "Motor.h"
#include "My_Math.h"

/*
    Np = PolePairs
    VPP = 2 * Vdc

    FLUX(wb)    =   (VPP / 2) / (sqrt(3) * We) 
                =   VPP / (2 * sqrt(3) * 2 * pi * Hz)
                =   VPP / (4 * sqrt(3) * pi * Hz)

    KV  = RPM / Vdc
        = (eRPM / Np) / Vdc
        = (Hz * 60 / Np) / Vdc
        = (Hz * 60 * 2) / (Np * VPP)

    FLUX(wb) = 30 / (sqrt(3) * pi * KV * Np)

    Ke = We * FLUX(wb) = 1000 * 2 * pi * Np * FLUX(wb) / 60 = (1000 * Np * VPP) / (120 * sqrt(3) * Hz)
*/

#define FOC_PWM_FREQ                                (20000UL)

#define FOC_CC_LOOP_FREQ                            (FOC_PWM_FREQ)
#define FOC_SC_LOOP_FREQ                            (5000UL)
#define FOC_PC_LOOP_FREQ                            (1000UL)

#define FOC_CC_LOOP_DT                              (1.0f / FOC_CC_LOOP_FREQ)
#define FOC_SC_LOOP_DT                              (1.0f / FOC_SC_LOOP_FREQ)
#define FOC_PC_LOOP_DT                              (1.0f / FOC_PC_LOOP_FREQ)

#define FOC_CURRENT_LOOP_DIV                        (FOC_PWM_FREQ / FOC_CC_LOOP_FREQ)
#define FOC_SPEED_LOOP_DIV                          (FOC_PWM_FREQ / FOC_SC_LOOP_FREQ)
#define FOC_POSITION_LOOP_DIV                       (FOC_PWM_FREQ / FOC_PC_LOOP_FREQ)

#if (FOC_PWM_FREQ % FOC_CC_LOOP_FREQ)
#error FOC_CC_LOOP_FREQ Set Incorrect
#endif

#if (FOC_PWM_FREQ % FOC_SC_LOOP_FREQ)
#error FOC_SC_LOOP_FREQ Set Incorrect
#endif

#if (FOC_PWM_FREQ % FOC_PC_LOOP_FREQ)
#error FOC_PC_LOOP_FREQ Set Incorrect
#endif


#define FOC_MAX_MODULATION_RATIO                    (_SQRT3_2)
// #define FOC_MAX_MODULATION_RATIO                    (1.0f)

#define Decoupling_Mode                             0
#define DT_COMPENSATION_ENABLE                      0
#define MTPA_ENABLE                                 0
#define FW_ENABLE                                   0
#define MTPV_ENABLE                                 0
#define PLL_USE_IIR_FILTER                          0

// sensorless startup config
#define USE_S_CURVE_ACCELERATE                      1
#define MAX_DIFF_ANGLE                              (_PI_6)
// sensorless observer
#define FOC_SMO_ENABLE                              0
#define FOC_NFO_ENABLE                              1

// debug options
#define FOC_DEBUG_ENCODER                           0
#define FOC_DEBUG_HALL                              0

#define FOC_DEBUG_SMO                               0
#define FOC_DEBUG_FLO                               0

#define FOC_DEBUG_SENSORLESS                        (FOC_DEBUG_SMO || FOC_DEBUG_FLO)

#define FOC_DEBUG_OPENLOOP_VF                       0
#define FOC_DEBUG_OPENLOOP_IF                       0

#if ((FOC_DEBUG_ENCODER + FOC_DEBUG_HALL + FOC_DEBUG_SENSORLESS + FOC_DEBUG_OPENLOOP_VF + FOC_DEBUG_OPENLOOP_IF) > 1)
#error FOC Debug Error
#endif

#define MOTOR_14PP_BLDC
// #define MOTOR_7PP_BLDC
// #define MOTOR_2PP_SERVO
// #define MOTOR_11PP_SERVO
// #define MOTOR_1PP_BLDC_FAN
// #define MOTOR_1PP_BLDC_HALL
// #define MOTOR_1PP_BLDC_FAN_SLOW

#ifdef MOTOR_14PP_BLDC
    //Motor Config
    // #define Sensor_Type                                 (SENSOR_LESS)
    // #define Sensor_Type                                 (SENSOR_LESS_VIS)
    #define Sensor_Type                                 (SENSOR_LESS_HFI)
    // #define Sensor_Type                                 (SPI_ENCODER)
    #define Motor_Max_Current                           (50.0f)
    #define Motor_Max_RPM                               (7500.0f)
    #define PolePairs                                   (14)
    #define Ld_H                                        (0.0000487f / 2.0f)
    #define Lq_H                                        (0.0000610f / 2.0f)
    #define Ls_H                                        ((Ld_H + Lq_H) / 2.0f)
    #define Rs_R                                        (0.0855f / 2.0f)
    #define FLUX_Wb                                     (0.00262537f)//(0.0027522f)
    //Control Config
    #define SPEED_LOOP_ENABLE                           (1)
    #define POSITION_LOOP_ENABLE                        (0)
    //SensorLess Config
    #define StartIQMax                                  (5.0f)
    #define StartIQMin                                  (0.5f)
    #define OrderMs                                     (20)
    #define ACCMs                                       (100)
    #define StartUpErpm                                 (3000)
    #define MinErpm                                     (1800)
    //SPEED_PID
    #define SPEED_KP                                    (0.01f)
    #define SPEED_KI                                    (0.2f)
    #define SPEED_KD                                    (0.0f)
    #define SPEED_KB                                    (SPEED_KI * 3)
    //POSITION_PID
    #define POSITION_KP                                 (20.0f)
    #define POSITION_KI                                 (0.0f)
    #define POSITION_KD                                 (0.0f)
    #define POSITION_KB                                 (POSITION_KI * 3)
#endif

#ifdef MOTOR_7PP_BLDC
    //Motor Config
    #define Sensor_Type                                 (SENSOR_LESS)
    // #define Sensor_Type                                 (SENSOR_LESS_VIS)
    // #define Sensor_Type                                 (SENSOR_LESS_HFI)
    // #define Sensor_Type                                 (SPI_ENCODER)
    #define Motor_Max_Current                           (50.0f)
    #define Motor_Max_RPM                               (46250.0f)
    #define PolePairs                                   (7)
    #define Ld_H                                        (0.000001f)//(0.0000203f)
    #define Lq_H                                        (0.000001f)//(0.0000203f)
    #define Ls_H                                        ((Ld_H + Lq_H) / 2.0f)
    #define Rs_R                                        (0.069)
    #define FLUX_Wb                                     (0.0004257365f)
    //Control Config
    #define SPEED_LOOP_ENABLE                           (0)
    #define POSITION_LOOP_ENABLE                        (0)
    //SensorLess Config
    #define StartIQMax                                  (15.0f)
    #define StartIQMin                                  (1.0f)
    #define OrderMs                                     (20)
    #define ACCMs                                       (100)
    #define StartUpErpm                                 (3000)
    #define MinErpm                                     (1800)
    //SPEED_PID
    #define SPEED_KP                                    (0.5f)
    #define SPEED_KI                                    (5.0f)
    #define SPEED_KD                                    (0.0f)
    #define SPEED_KB                                    (SPEED_KI * 3)
    //POSITION_PID
    #define POSITION_KP                                 (20.0f)
    #define POSITION_KI                                 (0.0f)
    #define POSITION_KD                                 (0.0f)
    #define POSITION_KB                                 (POSITION_KI * 3)
#endif

#ifdef MOTOR_2PP_SERVO
    // Motor Config
    // #define Sensor_Type                                 (SENSOR_LESS)
    // #define Sensor_Type                                 (SENSOR_LESS_HFI)
    // #define Sensor_Type                                 (SPI_ENCODER)
    // #define Sensor_Type                                 (HALL_120_SENSOR)
    #define Motor_Max_Current                           (3.0f)
    #define Motor_Max_RPM                               (3000.0f)
    #define PolePairs                                   (2)
    #define Ld_H                                        (0.00180f / 2.0f)
    #define Lq_H                                        (0.00202f / 2.0f)
    #define Ls_H                                        ((Ld_H + Lq_H) / 2.0f)
    #define Rs_R                                        (1.6573f / 2.0f)
    #define FLUX_Wb                                     (0.0147563434f)
    //Control Config
    #define SPEED_LOOP_ENABLE                           (0)
    #define POSITION_LOOP_ENABLE                        (0)
    //SensorLess Config
    #define StartIQMax                                  (3.0f)
    #define StartIQMin                                  (0.15f)
    #define OrderMs                                     (100)
    #define ACCMs                                       (500)
    #define StartUpErpm                                 (1000)
    #define MinErpm                                     (200)
    //SPEED_PID
    #define SPEED_KP                                    (0.01f)
    #define SPEED_KI                                    (0.01f)
    #define SPEED_KD                                    (0.00005f)
    #define SPEED_KB                                    (SPEED_KI * 3)
    //POSITION_PID
    #define POSITION_KP                                 (1000.0f)
    #define POSITION_KI                                 (0.0f)
    #define POSITION_KD                                 (0.0f)
    #define POSITION_KB                                 (POSITION_KI * 3)
#endif

#ifdef MOTOR_11PP_SERVO
    //Motor Config
    #define Sensor_Type                                 (SPI_ENCODER)
    // #define Sensor_Type                                 (SENSOR_LESS)
    #define Motor_Max_Current                           (1.2f)
    #define Motor_Max_RPM                               (2000.0f)
    #define PolePairs                                   (11)
    #define Ld_H                                        (0.001091f / 2.0f)
    #define Lq_H                                        (0.00120f / 2.0f)
    #define Ls_H                                        ((Ld_H + Lq_H) / 2.0f)
    #define Rs_R                                        (7.01f / 2.0f)
    #define FLUX_Wb                                     (0.0067730822f)
    //Control Config
    #define SPEED_LOOP_ENABLE                           (1)
    #define POSITION_LOOP_ENABLE                        (1)
    //SensorLess Config
    #define StartIQMax                                  (0.3f)
    #define StartIQMin                                  (0.15f)
    #define OrderMs                                     (500)
    #define ACCMs                                       (1000)
    #define StartUpErpm                                 (1000)
    #define MinErpm                                     (200)
    //SPEED_PID
    #define SPEED_KP                                    (0.01f)
    #define SPEED_KI                                    (0.01f)
    #define SPEED_KD                                    (0.0f)
    #define SPEED_KB                                    (SPEED_KI * 3)
    //POSITION_PID
    #define POSITION_KP                                 (100.0f)
    #define POSITION_KI                                 (0.0f)
    #define POSITION_KD                                 (0.0f)
    #define POSITION_KB                                 (POSITION_KI * 3)
#endif

#ifdef MOTOR_1PP_BLDC_FAN
    //Motor Config
    #define Sensor_Type                                 (SENSOR_LESS)
    #define Motor_Max_Current                           (5.0f)
    #define Motor_Max_RPM                               (1600.0f)
    #define PolePairs                                   (1)
    #define Ld_H                                        (0.00001372f / 2.0f)
    #define Lq_H                                        (0.00001666f / 2.0f)
    #define Ls_H                                        ((Ld_H + Lq_H) / 2.0f)
    #define Rs_R                                        (0.515f / 2.0f)
    #define FLUX_Wb                                     (0.000663282f)
    //Control Config
    #define SPEED_LOOP_ENABLE                           (0)
    //SensorLess Config
    #define StartIQMax                                  (2.0f)
    #define StartIQMin                                  (0.6f)
    #define OrderMs                                     (500)
    #define ACCMs                                       (2000)
    #define StartUpErpm                                 (4000)
    #define MinErpm                                     (2000)
    //SPEED_PID
    #define SPEED_KP                                    (700.0f)
    #define SPEED_KI                                    (200.0f)
    #define SPEED_KD                                    (0.0f)
    #define SPEED_KB                                    (SPEED_KI * 3)
#endif

#ifdef MOTOR_1PP_BLDC_HALL
    //Motor Config
    // #define Sensor_Type                                 (SENSOR_LESS)
    #define Sensor_Type                                 (HALL_120_SENSOR)
    #define Motor_Max_Current                           (50.0f)
    #define Motor_Max_RPM                               (100000.0f)
    #define PolePairs                                   (1)
    #define Ld_H                                        (0.00000758226f / 2.0f)
    #define Lq_H                                        (0.00000758226f / 2.0f)
    #define Ls_H                                        ((Ld_H + Lq_H) / 2.0f)
    #define Rs_R                                        (0.010674f / 2.0f)
    #define FLUX_Wb                                     (0.0010602479f)
    //Control Config
    #define SPEED_LOOP_ENABLE                           (0)
    #define POSITION_LOOP_ENABLE                        (0)
    //SensorLess Config
    #define StartIQMax                                  (15.0f)
    #define StartIQMin                                  (7.0f)
    #define OrderMs                                     (100)
    #define ACCMs                                       (300)
    #define StartUpErpm                                 (5000)
    #define MinErpm                                     (3000)
    //SPEED_PID
    #define SPEED_KP                                    (700.0f)
    #define SPEED_KI                                    (200.0f)
    #define SPEED_KD                                    (0.0f)
    #define SPEED_KB                                    (SPEED_KI * 3)
    //POSITION_PID
    #define POSITION_KP                                 (20.0f)
    #define POSITION_KI                                 (0.0f)
    #define POSITION_KD                                 (0.0f)
    #define POSITION_KB                                 (POSITION_KI * 3)
#endif

#ifdef MOTOR_1PP_BLDC_FAN_SLOW
    //Motor Config
    #define Sensor_Type                                 (SENSOR_LESS)
    #define Motor_Max_Current                           (6.0f)
    #define Motor_Max_RPM                               (100000.0f)
    #define PolePairs                                   (1)
    #define Ld_H                                        (0.0000241727f / 2.0f)
    #define Lq_H                                        (0.0000241727f / 2.0f)
    #define Ls_H                                        ((Ld_H + Lq_H) / 2.0f)
    #define Rs_R                                        (0.527445f / 2.0f)
    #define FLUX_Wb                                     (0.0008545637f)
    //Control Config
    #define SPEED_LOOP_ENABLE                           (0)
    #define POSITION_LOOP_ENABLE                        (0)
    //SensorLess Config
    #define StartIQMax                                  (5.0f)
    #define StartIQMin                                  (1.0f)
    #define OrderMs                                     (100)
    #define ACCMs                                       (300)
    #define StartUpErpm                                 (5000)
    #define MinErpm                                     (3000)
    //SPEED_PID
    #define SPEED_KP                                    (700.0f)
    #define SPEED_KI                                    (200.0f)
    #define SPEED_KD                                    (0.0f)
    #define SPEED_KB                                    (SPEED_KI * 3)
    //POSITION_PID
    #define POSITION_KP                                 (20.0f)
    #define POSITION_KI                                 (0.0f)
    #define POSITION_KD                                 (0.0f)
    #define POSITION_KB                                 (POSITION_KI * 3)
#endif


#endif
