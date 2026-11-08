/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
void DWT_Init(void);
void Debug_Timing_Rebase(void);

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define ENCODER_SPI_CS_Pin GPIO_PIN_4
#define ENCODER_SPI_CS_GPIO_Port GPIOA
#define LED_GREEN_Pin GPIO_PIN_0
#define LED_GREEN_GPIO_Port GPIOB
#define LED_RED_Pin GPIO_PIN_1
#define LED_RED_GPIO_Port GPIOB
#define HALL_A_Pin GPIO_PIN_6
#define HALL_A_GPIO_Port GPIOC
#define HALL_B_Pin GPIO_PIN_7
#define HALL_B_GPIO_Port GPIOC
#define HALL_C_Pin GPIO_PIN_8
#define HALL_C_GPIO_Port GPIOC
#define DRVER_SPI_CS_Pin GPIO_PIN_9
#define DRVER_SPI_CS_GPIO_Port GPIOC
#define DRIVER_EN_Pin GPIO_PIN_5
#define DRIVER_EN_GPIO_Port GPIOB
#define DRIVER_FAULT_Pin GPIO_PIN_7
#define DRIVER_FAULT_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
