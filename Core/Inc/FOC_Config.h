#ifndef __FOC_CONFIG_H__
#define __FOC_CONFIG_H__

#include "Motor.h"

/*
    FLUX in wb  =   (VPP / 2) / (sqrt(3) * We) 
                =   VPP / (2 * sqrt(3) * 2 * pi * Hz)
                =   VPP / (4 * sqrt(3) * pi * Hz)
*/

#define FOC_PWM_FREQ                                (20000UL)
#define FOC_CC_LOOP_FREQ                            (FOC_PWM_FREQ)
#define FOC_SC_LOOP_FREQ                            (4000UL)

#define FOC_MAX_MODULATION_RATIO                    (_SQRT3_2)
// #define FOC_MAX_MODULATION_RATIO                    (1.0f)

#define FOC_CURRENT_LOOP_DIV                        (FOC_PWM_FREQ / FOC_CC_LOOP_FREQ)
#define FOC_SPEED_LOOP_DIV                          (FOC_PWM_FREQ / FOC_SC_LOOP_FREQ)

#define USE_DT_COMPENSATION                         0
#define USE_MTPA                                    0

// sensorless startup config
#define USE_S_CURVE_ACCELERATE                      1
#define MAX_DIFF_ANGLE                              (6.28318530718f * 0.083333333f)     /* 360 * 0.083333333 = 30 */

#define FOC_DEBUG_ENCODER                           0
#define FOC_DEBUG_ENCODER_USE_ANGLE                 0
#define FOC_DEBUG_ENCODER_USE_SPEED                 0
#define FOC_DEBUG_ENCODER_USE_SPEED_LOOP            0

#define FOC_DEBUG_HALL                              0
#define FOC_DEBUG_HALL_USE_ANGLE                    0
#define FOC_DEBUG_HALL_USE_SPEED                    0
#define FOC_DEBUG_HALL_USE_SPEED_LOOP               0


#define FOC_DEBUG_SENSORLESS                        0

#define FOC_USE_SMO                                 0
#define FOC_DEBUG_SMO                               0
#define FOC_DEBUG_SMO_USE_ANGLE                     0
#define FOC_DEBUG_SMO_USE_SPEED                     0
#define FOC_DEBUG_SMO_USE_SPEED_LOOP                0

#define FOC_USE_FLO                                 1
#define FOC_DEBUG_FLO                               0
#define FOC_DEBUG_FLO_USE_ANGLE                     0
#define FOC_DEBUG_FLO_USE_SPEED                     0
#define FOC_DEBUG_FLO_USE_SPEED_LOOP                0


#define MOTOR_2PP_SERVO
// #define MOTOR_11PP_SERVO
// #define MOTOR_14PP_BLDC
// #define MOTOR_1PP_BLDC_FAN
// #define MOTOR_3PP_BLDC
// #define MOTOR_7PP_BLDC

#ifdef MOTOR_2PP_SERVO
    // Motor Config
    // #define Sensor_Type                                 (SENSOR_LESS)
    // #define Sensor_Type                                 (SPI_ENCODER)
    #define Sensor_Type                                 (HALL_120_SENSOR)
    #define PolePairs                                   (2)
    #define DAxisInd                                    (0.00180f / 2.0f)
    #define QAxisInd                                    (0.00202f / 2.0f)
    #define PhaseInd                                    ((DAxisInd + QAxisInd) / 2.0f)
    #define PhaseRes                                    (1.6573f / 2.0f)
    #define FLUXWb                                      (0.0147563434f)
    //Control Config
    #define CURRENT_LOOP_ENABLE                         (true)
    #define SPEED_LOOP_ENABLE                           (true)
    //SensorLess Config
    #define StartIQMax                                  (3000)
    #define StartIQMin                                  (150)
    #define OrderMs                                     (100)
    #define ACCMs                                       (500)
    #define StartUpErpm                                 (1000)
    #define MinErpm                                     (200)
    //ID_PID
    #define ID_TS                                       (1.0f / FOC_CC_LOOP_FREQ)
    #define ID_KP                                       (10.0f)
    #define ID_KI                                       (200.0f)
    #define ID_KD                                       (0.0f)
    #define ID_KB                                       (ID_KI * 10)
    #define ID_LP                                       (FOC_MAX_MODULATION_RATIO)
    #define ID_LN                                       (-FOC_MAX_MODULATION_RATIO)
    //IQ_PID
    #define IQ_TS                                       (1.0f / FOC_CC_LOOP_FREQ)
    #define IQ_KP                                       (10.0f)
    #define IQ_KI                                       (200.0f)
    #define IQ_KD                                       (0.0f)
    #define IQ_KB                                       (IQ_KI * 10)
    #define IQ_LP                                       (FOC_MAX_MODULATION_RATIO)
    #define IQ_LN                                       (-FOC_MAX_MODULATION_RATIO)
    //SPEED_PID
    #define SPEED_TS                                    (1.0f / FOC_SC_LOOP_FREQ)
    #define SPEED_KP                                    (5000.0f)
    #define SPEED_KI                                    (5000.0f)
    #define SPEED_KD                                    (0.0f)
    #define SPEED_KB                                    (SPEED_KI * 10)
    #define SPEED_LP                                    (MAX_PHASE_CURRENT)
    #define SPEED_LN                                    (-MAX_PHASE_CURRENT)
#endif

#ifdef MOTOR_11PP_SERVO
    //Motor Config
    #define Sensor_Type                                 (SPI_ENCODER)
    // #define Sensor_Type                                 (SENSOR_LESS)
    #define PolePairs                                   (11)
    #define DAxisInd                                    (0.001091f / 2.0f)
    #define QAxisInd                                    (0.00120f / 2.0f)
    #define PhaseInd                                    ((DAxisInd + QAxisInd) / 2.0f)
    #define PhaseRes                                    (7.01f / 2.0f)
    #define FLUXWb                                      (0.0147563434f)//ERR incorrect
    //Control Config
    #define CURRENT_LOOP_ENABLE                         (true)
    #define SPEED_LOOP_ENABLE                           (true)
    //SensorLess Config
    #define StartIQMax                                  (300)
    #define StartIQMin                                  (150)
    #define OrderMs                                     (500)
    #define ACCMs                                       (1000)
    #define StartUpErpm                                 (1000)
    #define MinErpm                                     (200)
    //ID_PID
    #define ID_TS                                       (1.0f / FOC_CC_LOOP_FREQ)
    #define ID_KP                                       (20.0f)
    #define ID_KI                                       (5.0f)
    #define ID_KD                                       (0.0f)
    #define ID_KB                                       (ID_KP / ID_KI * 2)
    #define ID_LP                                       (FOC_MAX_MODULATION_RATIO)
    #define ID_LN                                       (-FOC_MAX_MODULATION_RATIO)
    //IQ_PID
    #define IQ_TS                                       (1.0f / FOC_CC_LOOP_FREQ)
    #define IQ_KP                                       (20.0f)
    #define IQ_KI                                       (5.0f)
    #define IQ_KD                                       (0.0f)
    #define IQ_KB                                       (IQ_KP / IQ_KI * 2)
    #define IQ_LP                                       (FOC_MAX_MODULATION_RATIO)
    #define IQ_LN                                       (-FOC_MAX_MODULATION_RATIO)
    //SPEED_PID
    #define SPEED_TS                                    (1.0f / FOC_SC_LOOP_FREQ)
    #define SPEED_KP                                    (800.0f)
    #define SPEED_KI                                    (100.0f)
    #define SPEED_KD                                    (0.0f)
    #define SPEED_KB                                    (SPEED_KI * 10)
    #define SPEED_LP                                    (MAX_PHASE_CURRENT)
    #define SPEED_LN                                    (-MAX_PHASE_CURRENT)
#endif

#ifdef MOTOR_14PP_BLDC
    //Motor Config
    #define Sensor_Type                                 (SENSOR_LESS)
    // #define Sensor_Type                                 (SENSOR_LESS_HFI)
    // #define Sensor_Type                                 (SPI_ENCODER)
    #define PolePairs                                   (14)
    #define DAxisInd                                    (0.0000487f / 2.0f)
    #define QAxisInd                                    (0.0000610f / 2.0f)
    #define PhaseInd                                    ((DAxisInd + QAxisInd) / 2.0f)
    #define PhaseRes                                    (0.0855f / 2.0f)
    #define FLUXWb                                      (0.0027522f)
    //Control Config
    #define CURRENT_LOOP_ENABLE                         (true)
    #define SPEED_LOOP_ENABLE                           (0)
    //SensorLess Config
    #define StartIQMax                                  (1500)
    #define StartIQMin                                  (600)
    #define OrderMs                                     (200)
    #define ACCMs                                       (1000)
    #define StartUpErpm                                 (3000)
    #define MinErpm                                     (1000)
    //ID_PID
    #define ID_TS                                       (1.0f / FOC_CC_LOOP_FREQ)
    #define ID_KP                                       (1.0f)
    #define ID_KI                                       (50.0f)
    #define ID_KD                                       (0.0f)
    #define ID_KB                                       (ID_KI * 10)
    #define ID_LP                                       (FOC_MAX_MODULATION_RATIO)
    #define ID_LN                                       (-FOC_MAX_MODULATION_RATIO)
    //IQ_PID
    #define IQ_TS                                       (1.0f / FOC_CC_LOOP_FREQ)
    #define IQ_KP                                       (1.0f)
    #define IQ_KI                                       (50.0f)
    #define IQ_KD                                       (0.0f)
    #define IQ_KB                                       (IQ_KI * 10)
    #define IQ_LP                                       (FOC_MAX_MODULATION_RATIO)
    #define IQ_LN                                       (-FOC_MAX_MODULATION_RATIO)
    //SPEED_PID
    #define SPEED_TS                                    (1.0f / FOC_SC_LOOP_FREQ)
    #define SPEED_KP                                    (2500.0f)
    #define SPEED_KI                                    (200.0f)
    #define SPEED_KD                                    (0.0f)
    #define SPEED_KB                                    (SPEED_KI * 10)
    #define SPEED_LP                                    (MAX_PHASE_CURRENT)
    #define SPEED_LN                                    (-MAX_PHASE_CURRENT)
#endif

#ifdef MOTOR_1PP_BLDC_FAN
    //Motor Config
    #define Sensor_Type                                 (SENSOR_LESS)
    #define PolePairs                                   (1)
    #define DAxisInd                                    (0.00001372f / 2.0f)
    #define QAxisInd                                    (0.00001666f / 2.0f)
    #define PhaseInd                                    ((DAxisInd + QAxisInd) / 2.0f)
    #define PhaseRes                                    (0.515f / 2.0f)
    #define FLUXWb                                      (0.000663282f)
    //Control Config
    #define CURRENT_LOOP_ENABLE                         (true)
    #define SPEED_LOOP_ENABLE                           (0)
    //SensorLess Config
    #define StartIQMax                                  (2000)
    #define StartIQMin                                  (600)
    #define OrderMs                                     (500)
    #define ACCMs                                       (2000)
    #define StartUpErpm                                 (4000)
    #define MinErpm                                     (2000)
    //ID_PID
    #define ID_TS                                       (1.0f / FOC_CC_LOOP_FREQ)
    #define ID_KP                                       (1.0f)
    #define ID_KI                                       (1.0f)
    #define ID_KD                                       (0.0f)
    #define ID_KB                                       (ID_KP / ID_KI * 2)
    #define ID_LP                                       (FOC_MAX_MODULATION_RATIO)
    #define ID_LN                                       (-FOC_MAX_MODULATION_RATIO)
    //IQ_PID
    #define IQ_TS                                       (1.0f / FOC_CC_LOOP_FREQ)
    #define IQ_KP                                       (1.0f)
    #define IQ_KI                                       (1.0f)
    #define IQ_KD                                       (0.0f)
    #define IQ_KB                                       (IQ_KP / IQ_KI * 2)
    #define IQ_LP                                       (FOC_MAX_MODULATION_RATIO)
    #define IQ_LN                                       (-FOC_MAX_MODULATION_RATIO)
    //SPEED_PID
    #define SPEED_TS                                    (1.0f / FOC_SC_LOOP_FREQ)
    #define SPEED_KP                                    (700.0f)
    #define SPEED_KI                                    (200.0f)
    #define SPEED_KD                                    (0.0f)
    #define SPEED_KB                                    (SPEED_KI * 10)
    #define SPEED_LP                                    (MAX_PHASE_CURRENT)
    #define SPEED_LN                                    (-MAX_PHASE_CURRENT)
#endif

#ifdef MOTOR_3PP_BLDC
    //Motor Config
    #define Sensor_Type                                 (SENSOR_LESS)
    #define PolePairs                                   (3)
    #define DAxisInd                                    (0.0000086f / 2.0f)
    #define QAxisInd                                    (0.0000186f / 2.0f)
    #define PhaseInd                                    ((DAxisInd + QAxisInd) / 2.0f)
    #define PhaseRes                                    (0.08f / 2.0f)
    #define FLUXWb                                      (5.6369845e-4f)
    //Control Config
    #define CURRENT_LOOP_ENABLE                         (true)
    #define SPEED_LOOP_ENABLE                           (false)
    //SensorLess Config
    #define StartIQMax                                  (6000)
    #define StartIQMin                                  (300)
    #define OrderMs                                     (300)
    #define ACCMs                                       (1000)
    #define StartUpErpm                                 (4000)
    #define MinErpm                                     (2000)
    //ID_PID
    #define ID_TS                                       (1.0f / FOC_CC_LOOP_FREQ)
    #define ID_KP                                       (4.0f)
    #define ID_KI                                       (4.0f)
    #define ID_KD                                       (0.0f)
    #define ID_KB                                       (ID_KP / ID_KI * 2)
    #define ID_LP                                       (FOC_MAX_MODULATION_RATIO)
    #define ID_LN                                       (-FOC_MAX_MODULATION_RATIO)
    //IQ_PID
    #define IQ_TS                                       (1.0f / FOC_CC_LOOP_FREQ)
    #define IQ_KP                                       (5.0f)
    #define IQ_KI                                       (5.0f)
    #define IQ_KD                                       (0.0f)
    #define IQ_KB                                       (IQ_KP / IQ_KI * 2)
    #define IQ_LP                                       (FOC_MAX_MODULATION_RATIO)
    #define IQ_LN                                       (-FOC_MAX_MODULATION_RATIO)
    //SPEED_PID
    #define SPEED_TS                                    (1.0f / FOC_SC_LOOP_FREQ)
    #define SPEED_KP                                    (1200.0f)
    #define SPEED_KI                                    (100.0f)
    #define SPEED_KD                                    (0.0f)
    #define SPEED_KB                                    (SPEED_KI * 10)
    #define SPEED_LP                                    (MAX_PHASE_CURRENT)
    #define SPEED_LN                                    (-MAX_PHASE_CURRENT)
#endif

#ifdef MOTOR_7PP_BLDC
    //Motor Config
    #define Sensor_Type                                 (SENSOR_LESS)
    #define PolePairs                                   (7)
    #define DAxisInd                                    (0.0000082f / 2.0f)
    #define QAxisInd                                    (0.0000101f / 2.0f)
    #define PhaseInd                                    ((DAxisInd + QAxisInd) / 2.0f)
    #define PhaseRes                                    (0.046f / 2.0f)
    #define FLUXWb                                      (5.6369845e-4f)
    //Control Config
    #define CURRENT_LOOP_ENABLE                         (true)
    #define SPEED_LOOP_ENABLE                           (0)
    //SensorLess Config
    #define StartIQMax                                  (2000)
    #define StartIQMin                                  (500)
    #define OrderMs                                     (30)
    #define ACCMs                                       (150)
    #define StartUpErpm                                 (4000)
    #define MinErpm                                     (2000)
    //ID_PID
    #define ID_TS                                       (1.0f / FOC_CC_LOOP_FREQ)
    #define ID_KP                                       (0.2f)
    #define ID_KI                                       (0.2f)
    #define ID_KD                                       (0.0f)
    #define ID_KB                                       (ID_KP / ID_KI * 2)
    #define ID_LP                                       (FOC_MAX_MODULATION_RATIO)
    #define ID_LN                                       (-FOC_MAX_MODULATION_RATIO)
    //IQ_PID
    #define IQ_TS                                       (1.0f / FOC_CC_LOOP_FREQ)
    #define IQ_KP                                       (0.2f)
    #define IQ_KI                                       (0.2f)
    #define IQ_KD                                       (0.0f)
    #define IQ_KB                                       (IQ_KP / IQ_KI * 2)
    #define IQ_LP                                       (FOC_MAX_MODULATION_RATIO)
    #define IQ_LN                                       (-FOC_MAX_MODULATION_RATIO)
    //SPEED_PID
    #define SPEED_TS                                    (1.0f / FOC_SC_LOOP_FREQ)
    #define SPEED_KP                                    (2000.0f)
    #define SPEED_KI                                    (200.0f)
    #define SPEED_KD                                    (0.0f)
    #define SPEED_KB                                    (SPEED_KI * 10)
    #define SPEED_LP                                    (MAX_PHASE_CURRENT)
    #define SPEED_LN                                    (-MAX_PHASE_CURRENT)
#endif

#endif
