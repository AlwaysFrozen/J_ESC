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


// #define FOC_MAX_MODULATION_RATIO                    (_SQRT3_2)
#define FOC_MAX_MODULATION_RATIO                    (1.0f)

#define FEED_FORWARD_ENABLE                         1
#define DT_COMPENSATION_ENABLE                      0
#define MTPA_ENABLE                                 0
#define CAL_BY_VOLTAGE                              1


// sensorless startup config
#define USE_S_CURVE_ACCELERATE                      1
#define MAX_DIFF_ANGLE                              _PI_6     /* 360 * 0.083333333 = 30 */
// sensorless observer
#define FOC_SMO_ENABLE                              0
#define FOC_FLO_ENABLE                              1

// debug options
#define FOC_DEBUG_ENCODER                           0
#define FOC_DEBUG_HALL                              0

#define FOC_DEBUG_SMO                               0
#define FOC_DEBUG_FLO                               0

#define FOC_DEBUG_SENSORLESS                        (FOC_DEBUG_SMO || FOC_DEBUG_FLO)


// #define MOTOR_2PP_SERVO
// #define MOTOR_11PP_SERVO
#define MOTOR_14PP_BLDC
// #define MOTOR_1PP_BLDC_FAN
// #define MOTOR_3PP_BLDC
// #define MOTOR_7PP_BLDC

#ifdef MOTOR_2PP_SERVO
    // Motor Config
    // #define Sensor_Type                                 (SENSOR_LESS)
    // #define Sensor_Type                                 (SENSOR_LESS_HFI)
    // #define Sensor_Type                                 (SPI_ENCODER)
    // #define Sensor_Type                                 (HALL_120_SENSOR)
    #define PolePairs                                   (2)
    #define Ld_H                                        (0.00180f / 2.0f)
    #define Lq_H                                        (0.00202f / 2.0f)
    #define Ls_H                                        ((Ld_H + Lq_H) / 2.0f)
    #define Rs_R                                        (1.6573f / 2.0f)
    #define FLUX_Wb                                     (0.0147563434f)
    //Control Config
    #define CURRENT_LOOP_ENABLE                         (1)
    #define SPEED_LOOP_ENABLE                           (0)
    #define POSITION_LOOP_ENABLE                        (0)
    //SensorLess Config
    #define StartIQMax                                  (3.0f)
    #define StartIQMin                                  (0.15f)
    #define OrderMs                                     (100)
    #define ACCMs                                       (500)
    #define StartUpErpm                                 (1000)
    #define MinErpm                                     (200)
    //ID_PID
    #define ID_KP                                       (2.0f)
    #define ID_KI                                       (50.0f)
    #define ID_KD                                       (0.0f)
    #define ID_KB                                       (ID_KI * 3)
    #define ID_LP                                       (FOC_MAX_MODULATION_RATIO)
    #define ID_LN                                       (-FOC_MAX_MODULATION_RATIO)
    //IQ_PID
    #define IQ_KP                                       (2.0f)
    #define IQ_KI                                       (50.0f)
    #define IQ_KD                                       (0.0f)
    #define IQ_KB                                       (IQ_KI * 3)
    #define IQ_LP                                       (FOC_MAX_MODULATION_RATIO)
    #define IQ_LN                                       (-FOC_MAX_MODULATION_RATIO)
    //SPEED_PID
    #define SPEED_KP                                    (0.01f)
    #define SPEED_KI                                    (0.01f)
    #define SPEED_KD                                    (0.00005f)
    #define SPEED_KB                                    (SPEED_KI * 3)
    #define SPEED_LP                                    (MAX_PHASE_CURRENT)
    #define SPEED_LN                                    (-MAX_PHASE_CURRENT)
    //POSITION_PID
    #define POSITION_KP                                 (1000.0f)
    #define POSITION_KI                                 (5000.0f)
    #define POSITION_KD                                 (0.0f)
    #define POSITION_KB                                 (POSITION_KI * 3)
    #define POSITION_LP                                 (6000)
    #define POSITION_LN                                 (-6000)
#endif

#ifdef MOTOR_11PP_SERVO
    //Motor Config
    #define Sensor_Type                                 (SPI_ENCODER)
    // #define Sensor_Type                                 (SENSOR_LESS)
    #define PolePairs                                   (11)
    #define Ld_H                                        (0.001091f / 2.0f)
    #define Lq_H                                        (0.00120f / 2.0f)
    #define Ls_H                                        ((Ld_H + Lq_H) / 2.0f)
    #define Rs_R                                        (7.01f / 2.0f)
    #define FLUX_Wb                                     (0.0147563434f)//ERR incorrect
    //Control Config
    #define CURRENT_LOOP_ENABLE                         (1)
    #define SPEED_LOOP_ENABLE                           (1)
    #define POSITION_LOOP_ENABLE                        (1)
    //SensorLess Config
    #define StartIQMax                                  (0.3f)
    #define StartIQMin                                  (0.15f)
    #define OrderMs                                     (500)
    #define ACCMs                                       (1000)
    #define StartUpErpm                                 (1000)
    #define MinErpm                                     (200)
    //ID_PID
    #define ID_KP                                       (5000.0f)
    #define ID_KI                                       (5000.0f)
    #define ID_KD                                       (0.0f)
    #define ID_KB                                       (ID_KP / ID_KI * 2)
    #define ID_LP                                       (FOC_MAX_MODULATION_RATIO)
    #define ID_LN                                       (-FOC_MAX_MODULATION_RATIO)
    //IQ_PID
    #define IQ_KP                                       (5000.0f)
    #define IQ_KI                                       (50000.0f)
    #define IQ_KD                                       (0.0f)
    #define IQ_KB                                       (IQ_KP / IQ_KI * 2)
    #define IQ_LP                                       (FOC_MAX_MODULATION_RATIO)
    #define IQ_LN                                       (-FOC_MAX_MODULATION_RATIO)
    //SPEED_PID
    #define SPEED_KP                                    (3000.0f)
    #define SPEED_KI                                    (5000.0f)
    #define SPEED_KD                                    (0.0f)
    #define SPEED_KB                                    (SPEED_KI * 3)
    #define SPEED_LP                                    (3000)
    #define SPEED_LN                                    (-3000)
    //POSITION_PID
    #define POSITION_KP                                 (500000.0f)
    #define POSITION_KI                                 (100000.0f)
    #define POSITION_KD                                 (0.0f)
    #define POSITION_KB                                 (POSITION_KI * 3)
    #define POSITION_LP                                 (15000)
    #define POSITION_LN                                 (-15000)
#endif

#ifdef MOTOR_14PP_BLDC
    //Motor Config
    // #define Sensor_Type                                 (SENSOR_LESS)
    // #define Sensor_Type                                 (SENSOR_LESS_VIS)
    #define Sensor_Type                                 (SENSOR_LESS_HFI)
    // #define Sensor_Type                                 (SPI_ENCODER)
    #define PolePairs                                   (14)
    #define Ld_H                                        (0.0000487f / 2.0f)
    #define Lq_H                                        (0.0000610f / 2.0f)
    #define Ls_H                                        ((Ld_H + Lq_H) / 2.0f)
    #define Rs_R                                        (0.0855f / 2.0f)
    #define FLUX_Wb                                     (0.00262537f)//(0.0027522f)
    //Control Config
    #define CURRENT_LOOP_ENABLE                         (1)
    #define SPEED_LOOP_ENABLE                           (0)
    #define POSITION_LOOP_ENABLE                        (0)
    //SensorLess Config
    #define StartIQMax                                  (1.5f)
    #define StartIQMin                                  (0.6f)
    #define OrderMs                                     (200)
    #define ACCMs                                       (500)
    #define StartUpErpm                                 (3000)
    #define MinErpm                                     (1000)
    //ID_PID
    #define ID_KP                                       (0.015f)
    #define ID_KI                                       (10.0f)
    #define ID_KD                                       (0.0f)
    #define ID_KB                                       (ID_KI * 3)
    #define ID_LP                                       (FOC_MAX_MODULATION_RATIO)
    #define ID_LN                                       (-FOC_MAX_MODULATION_RATIO)
    //IQ_PID
    #define IQ_KP                                       (0.015f)
    #define IQ_KI                                       (10.0f)
    #define IQ_KD                                       (0.0f)
    #define IQ_KB                                       (IQ_KI * 3)
    #define IQ_LP                                       (FOC_MAX_MODULATION_RATIO)
    #define IQ_LN                                       (-FOC_MAX_MODULATION_RATIO)
    //SPEED_PID
    #define SPEED_KP                                    (0.004f)
    #define SPEED_KI                                    (0.03f)
    #define SPEED_KD                                    (0.0f)
    #define SPEED_KB                                    (SPEED_KI * 3)
    #define SPEED_LP                                    (MAX_PHASE_CURRENT)
    #define SPEED_LN                                    (-MAX_PHASE_CURRENT)
    //POSITION_PID
    #define POSITION_KP                                 (20.0f)
    #define POSITION_KI                                 (20.0f)
    #define POSITION_KD                                 (0.0f)
    #define POSITION_KB                                 (POSITION_KI * 3)
    #define POSITION_LP                                 (6000)
    #define POSITION_LN                                 (-6000)
#endif

#ifdef MOTOR_1PP_BLDC_FAN
    //Motor Config
    #define Sensor_Type                                 (SENSOR_LESS)
    #define PolePairs                                   (1)
    #define Ld_H                                        (0.00001372f / 2.0f)
    #define Lq_H                                        (0.00001666f / 2.0f)
    #define Ls_H                                        ((Ld_H + Lq_H) / 2.0f)
    #define Rs_R                                        (0.515f / 2.0f)
    #define FLUX_Wb                                     (0.000663282f)
    //Control Config
    #define CURRENT_LOOP_ENABLE                         (1)
    #define SPEED_LOOP_ENABLE                           (0)
    //SensorLess Config
    #define StartIQMax                                  (2.0f)
    #define StartIQMin                                  (0.6f)
    #define OrderMs                                     (500)
    #define ACCMs                                       (2000)
    #define StartUpErpm                                 (4000)
    #define MinErpm                                     (2000)
    //ID_PID
    #define ID_KP                                       (1000.0f)
    #define ID_KI                                       (1000.0f)
    #define ID_KD                                       (0.0f)
    #define ID_KB                                       (ID_KP / ID_KI * 2)
    #define ID_LP                                       (FOC_MAX_MODULATION_RATIO)
    #define ID_LN                                       (-FOC_MAX_MODULATION_RATIO)
    //IQ_PID
    #define IQ_KP                                       (1000.0f)
    #define IQ_KI                                       (1000.0f)
    #define IQ_KD                                       (0.0f)
    #define IQ_KB                                       (IQ_KP / IQ_KI * 2)
    #define IQ_LP                                       (FOC_MAX_MODULATION_RATIO)
    #define IQ_LN                                       (-FOC_MAX_MODULATION_RATIO)
    //SPEED_PID
    #define SPEED_KP                                    (700.0f)
    #define SPEED_KI                                    (200.0f)
    #define SPEED_KD                                    (0.0f)
    #define SPEED_KB                                    (SPEED_KI * 3)
    #define SPEED_LP                                    (MAX_PHASE_CURRENT)
    #define SPEED_LN                                    (-MAX_PHASE_CURRENT)
#endif

#ifdef MOTOR_3PP_BLDC
    //Motor Config
    #define Sensor_Type                                 (SENSOR_LESS)
    #define PolePairs                                   (3)
    #define Ld_H                                        (0.0000086f / 2.0f)
    #define Lq_H                                        (0.0000186f / 2.0f)
    #define Ls_H                                        ((Ld_H + Lq_H) / 2.0f)
    #define Rs_R                                        (0.08f / 2.0f)
    #define FLUX_Wb                                     (5.6369845e-4f)
    //Control Config
    #define CURRENT_LOOP_ENABLE                         (1)
    #define SPEED_LOOP_ENABLE                           (0)
    //SensorLess Config
    #define StartIQMax                                  (6.0f)
    #define StartIQMin                                  (0.3f)
    #define OrderMs                                     (300)
    #define ACCMs                                       (1000)
    #define StartUpErpm                                 (4000)
    #define MinErpm                                     (2000)
    //ID_PID
    #define ID_KP                                       (4000.0f)
    #define ID_KI                                       (4000.0f)
    #define ID_KD                                       (0.0f)
    #define ID_KB                                       (ID_KP / ID_KI * 2)
    #define ID_LP                                       (FOC_MAX_MODULATION_RATIO)
    #define ID_LN                                       (-FOC_MAX_MODULATION_RATIO)
    //IQ_PID
    #define IQ_KP                                       (5000.0f)
    #define IQ_KI                                       (5000.0f)
    #define IQ_KD                                       (0.0f)
    #define IQ_KB                                       (IQ_KP / IQ_KI * 2)
    #define IQ_LP                                       (FOC_MAX_MODULATION_RATIO)
    #define IQ_LN                                       (-FOC_MAX_MODULATION_RATIO)
    //SPEED_PID
    #define SPEED_KP                                    (1200.0f)
    #define SPEED_KI                                    (100.0f)
    #define SPEED_KD                                    (0.0f)
    #define SPEED_KB                                    (SPEED_KI * 3)
    #define SPEED_LP                                    (MAX_PHASE_CURRENT)
    #define SPEED_LN                                    (-MAX_PHASE_CURRENT)
#endif

#ifdef MOTOR_7PP_BLDC
    //Motor Config
    #define Sensor_Type                                 (SENSOR_LESS)
    #define PolePairs                                   (7)
    #define Ld_H                                        (0.0000082f / 2.0f)
    #define Lq_H                                        (0.0000101f / 2.0f)
    #define Ls_H                                        ((Ld_H + Lq_H) / 2.0f)
    #define Rs_R                                        (0.046f / 2.0f)
    #define FLUX_Wb                                     (5.6369845e-4f)
    //Control Config
    #define CURRENT_LOOP_ENABLE                         (1)
    #define SPEED_LOOP_ENABLE                           (0)
    //SensorLess Config
    #define StartIQMax                                  (2.0f)
    #define StartIQMin                                  (0.5f)
    #define OrderMs                                     (30)
    #define ACCMs                                       (150)
    #define StartUpErpm                                 (4000)
    #define MinErpm                                     (2000)
    //ID_PID
    #define ID_KP                                       (200.0f)
    #define ID_KI                                       (200.0f)
    #define ID_KD                                       (0.0f)
    #define ID_KB                                       (ID_KP / ID_KI * 2)
    #define ID_LP                                       (FOC_MAX_MODULATION_RATIO)
    #define ID_LN                                       (-FOC_MAX_MODULATION_RATIO)
    //IQ_PID
    #define IQ_KP                                       (200.0f)
    #define IQ_KI                                       (200.0f)
    #define IQ_KD                                       (0.0f)
    #define IQ_KB                                       (IQ_KP / IQ_KI * 2)
    #define IQ_LP                                       (FOC_MAX_MODULATION_RATIO)
    #define IQ_LN                                       (-FOC_MAX_MODULATION_RATIO)
    //SPEED_PID
    #define SPEED_KP                                    (2000.0f)
    #define SPEED_KI                                    (200.0f)
    #define SPEED_KD                                    (0.0f)
    #define SPEED_KB                                    (SPEED_KI * 3)
    #define SPEED_LP                                    (MAX_PHASE_CURRENT)
    #define SPEED_LN                                    (-MAX_PHASE_CURRENT)
#endif

#endif
