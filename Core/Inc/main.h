/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

#include "string.h"
#include <stdio.h>
#include <stdlib.h>

// #define DAC_DEBUG
// #define HALL_TIM_CAPTURE
// #define PWN_INPUT

#define DEBUG_VALUE_CNT   (10)
#define DEBUG_ARR_CNT     (DEBUG_VALUE_CNT + 1)
//Debug Data
extern float debug_arr[DEBUG_ARR_CNT];
void VOFA_Cmd_Analyze(const char *buffer,uint32_t len);


// #define PWM_ADC_DEBUG

#define ADC_SAMPLE_CYCLES ADC_SAMPLETIME_3CYCLES
// #define ADC_SAMPLE_CYCLES ADC_SAMPLETIME_15CYCLES

#define ADC_TRIGER_SOURCE_TIM1_CC4
#define ADC_SAMPLE_HIGH_SIDE

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
