#pragma once
#include <stdbool.h>
#include <stdint.h>

/* Deliberately odd-sized payload exercises last-word padding. */
#define BOARD_SUPPORTED_MOTOR_NUMBER 2U
typedef struct { uint8_t bytes[37]; } Global_Configs_t;
typedef enum { MS_Init, MS_Calibrate, MS_IDLE, MS_Running, MS_Err } Motor_State_t;
typedef struct { Motor_State_t motor_state; int is_enabled; } Motor_Status_t;
typedef struct { Motor_Status_t motor_status[BOARD_SUPPORTED_MOTOR_NUMBER]; } Global_Status_t;
extern Global_Configs_t global_config;
extern volatile Global_Status_t global_status;
