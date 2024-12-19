/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file    stm32f4xx_it.c
 * @brief   Interrupt Service Routines.
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

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "My_Math.h"
#include "Motor.h"
#include "BLDC_Motor.h"
#include "FOC_Motor.h"
#include "FOC_Config.h"
#include "Inverter.h"
#include "VIS.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern PCD_HandleTypeDef hpcd_USB_OTG_FS;
extern DMA_HandleTypeDef hdma_adc3;
extern ADC_HandleTypeDef hadc1;
extern ADC_HandleTypeDef hadc2;
extern ADC_HandleTypeDef hadc3;
extern DAC_HandleTypeDef hdac;
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim7;
extern DMA_HandleTypeDef hdma_tim4_ch1;
extern DMA_HandleTypeDef hdma_tim4_ch2;
extern TIM_HandleTypeDef htim3;

/* USER CODE BEGIN EV */

uint32_t PWM_T_Count = 0;
uint32_t PWM_H_Count = 0;
float PWM_Duty = 0;
uint16_t PWM_T_us = 0;
uint16_t PWM_H_us = 0;

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M4 Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
 * @brief This function handles Non maskable interrupt.
 */
void NMI_Handler(void)
{
    /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

    /* USER CODE END NonMaskableInt_IRQn 0 */
    /* USER CODE BEGIN NonMaskableInt_IRQn 1 */
    while (1)
    {
    }
    /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
 * @brief This function handles Hard fault interrupt.
 */
void HardFault_Handler(void)
{
    /* USER CODE BEGIN HardFault_IRQn 0 */

    /* USER CODE END HardFault_IRQn 0 */
    while (1)
    {
        /* USER CODE BEGIN W1_HardFault_IRQn 0 */
        /* USER CODE END W1_HardFault_IRQn 0 */
    }
}

/**
 * @brief This function handles Memory management fault.
 */
void MemManage_Handler(void)
{
    /* USER CODE BEGIN MemoryManagement_IRQn 0 */

    /* USER CODE END MemoryManagement_IRQn 0 */
    while (1)
    {
        /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
        /* USER CODE END W1_MemoryManagement_IRQn 0 */
    }
}

/**
 * @brief This function handles Pre-fetch fault, memory access fault.
 */
void BusFault_Handler(void)
{
    /* USER CODE BEGIN BusFault_IRQn 0 */

    /* USER CODE END BusFault_IRQn 0 */
    while (1)
    {
        /* USER CODE BEGIN W1_BusFault_IRQn 0 */
        /* USER CODE END W1_BusFault_IRQn 0 */
    }
}

/**
 * @brief This function handles Undefined instruction or illegal state.
 */
void UsageFault_Handler(void)
{
    /* USER CODE BEGIN UsageFault_IRQn 0 */

    /* USER CODE END UsageFault_IRQn 0 */
    while (1)
    {
        /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
        /* USER CODE END W1_UsageFault_IRQn 0 */
    }
}

/**
 * @brief This function handles Debug monitor.
 */
void DebugMon_Handler(void)
{
    /* USER CODE BEGIN DebugMonitor_IRQn 0 */

    /* USER CODE END DebugMonitor_IRQn 0 */
    /* USER CODE BEGIN DebugMonitor_IRQn 1 */

    /* USER CODE END DebugMonitor_IRQn 1 */
}

/******************************************************************************/
/* STM32F4xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f4xx.s).                    */
/******************************************************************************/

/**
 * @brief This function handles ADC1, ADC2 and ADC3 global interrupts.
 */
void ADC_IRQHandler(void)
{
    timer_cnt_arr[0] = TIM1->CNT;
    if((TIM1->CR1 & 0x10) == 0x10)
    {
        timer_cnt_arr[0] = TIM1->ARR * 2 - timer_cnt_arr[0];
    }
    // GPIOB->BSRR = GPIO_PIN_0;

    ADC1->SR = 0;
    ADC2->SR = 0;
    ADC3->SR = 0;

    // adc_value[0] = ADC1->JDR1;
    // adc_value[1] = ADC2->JDR1;
    // adc_value[2] = ADC3->JDR1;
    // adc_value[3] = ADC1->JDR2;
    // adc_value[4] = ADC2->JDR2;
    // adc_value[5] = ADC3->JDR2;

    adc_value[0] = ADC1->JDR1;
    adc_value[1] = ADC2->JDR1;
    adc_value[2] = ADC3->JDR1;
    adc_value[3] = (ADC1->JDR2 + ADC1->JDR3 + ADC1->JDR4) / 3;
    adc_value[4] = (ADC2->JDR2 + ADC2->JDR3 + ADC2->JDR4) / 3;
    adc_value[5] = (ADC3->JDR2 + ADC3->JDR3 + ADC3->JDR4) / 3;

    phase_voltage_V.U = adc_value[0] * phase_voltage_V_RATIO;
    phase_voltage_V.V = adc_value[1] * phase_voltage_V_RATIO;
    phase_voltage_V.W = adc_value[2] * phase_voltage_V_RATIO;
    phase_current_A.U = (adc_value[3] - adc_cali.ADC_Cali_Value[3]) * phase_current_A_RATIO;
    phase_current_A.V = (adc_value[4] - adc_cali.ADC_Cali_Value[4]) * phase_current_A_RATIO;
    phase_current_A.W = (adc_value[5] - adc_cali.ADC_Cali_Value[5]) * phase_current_A_RATIO;

    // if(TIM1->CCR1 < (TIM1->ARR - 500) && TIM1->CCR2 < (TIM1->ARR - 500))
    // {
    //     phase_current_A.W = -(phase_current_A.U + phase_current_A.V);
    // }
    // else
    // {
    //     if(TIM1->CCR1 > TIM1->CCR2 && TIM1->CCR1 > TIM1->CCR3)
    //     {
    //         phase_current_A.U = -(phase_current_A.V + phase_current_A.W);
    //     }
    //     else if(TIM1->CCR2 > TIM1->CCR1 && TIM1->CCR2 > TIM1->CCR3)
    //     {
    //         phase_current_A.V = -(phase_current_A.U + phase_current_A.W);
    //     }
    //     else if(TIM1->CCR3 > TIM1->CCR1 && TIM1->CCR3 > TIM1->CCR2)
    //     {
    //         phase_current_A.W = -(phase_current_A.U + phase_current_A.V);
    //     }
    // }

    timer_cnt_arr[1] = TIM1->CNT;
    if ((TIM1->CR1 & 0x10) == 0x10)
    {
        timer_cnt_arr[1] = TIM1->ARR * 2 - timer_cnt_arr[1];
    }

    memcpy(&phase_voltage_V_f,p_phase_voltage_V,sizeof(phase_voltage_V_f));
    memcpy(&phase_current_A_f,p_phase_current_A,sizeof(phase_current_A_f));

    // #if 0
    // FirstOrder_LPF_Cacl(p_phase_voltage_V->U, phase_voltage_V_f.U, Filter_Rate.phase_voltage_filter_rate);
    // FirstOrder_LPF_Cacl(p_phase_voltage_V->V, phase_voltage_V_f.V, Filter_Rate.phase_voltage_filter_rate);
    // FirstOrder_LPF_Cacl(p_phase_voltage_V->W, phase_voltage_V_f.W, Filter_Rate.phase_voltage_filter_rate);
    // FirstOrder_LPF_Cacl(p_phase_current_A->U, phase_current_A_f.U, Filter_Rate.phase_current_filter_rate);
    // FirstOrder_LPF_Cacl(p_phase_current_A->V, phase_current_A_f.V, Filter_Rate.phase_current_filter_rate);
    // FirstOrder_LPF_Cacl(p_phase_current_A->W, phase_current_A_f.W, Filter_Rate.phase_current_filter_rate);
    // #else
    // #if 0
    // float fc = Virtual_Moto.electronic_speed_hz * 2;
    // fc = _constrain(fc,MIN_LPF_FC,MAX_LPF_FC);
    // LPF_UVW_F32(&phase_voltage_V_f,p_phase_voltage_V,MAX_LPF_FC, Virtual_Moto.dt);
    // LPF_UVW_F32(&phase_current_A_f,p_phase_current_A,MAX_LPF_FC, Virtual_Moto.dt);
    // #else
    // LPF_UVW_F32(&phase_voltage_V_f, p_phase_voltage_V, MAX_LPF_FC, Virtual_Moto.dt);
    // LPF_UVW_F32(&phase_current_A_f, p_phase_current_A, MAX_LPF_FC, Virtual_Moto.dt);
    // #endif
    // #endif

    // HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1,DAC_ALIGN_12B_R,_constrain(2048 + phase_current_A.U / 4,0,4095));
    // HAL_DAC_SetValue(&hdac, DAC_CHANNEL_2,DAC_ALIGN_12B_R,_constrain(2048 + phase_current_A_f.U / 4,0,4095));

    Virtual_Moto.V_bus_v = Vbus_ADC * BUS_VOLTAGE_RATIO;
    FirstOrder_LPF_Cacl(Virtual_Moto.V_bus_v, Virtual_Moto.V_bus_v_f, Filter_Rate.bus_voltage_filter_rate);

    timer_cnt_arr[2] = TIM1->CNT;
    if((TIM1->CR1 & 0x10) == 0x10)
    {
        timer_cnt_arr[2] = TIM1->ARR * 2 - timer_cnt_arr[2];
    }
    // GPIOB->BSRR = GPIO_PIN_1;

    if (sensor_calibration_status == Sensor_Calibrated)
    {
        switch (motor_type_now)
        {
            case BLDC:
                BLDC_Process();
                break;

            case FOC:
                FOC_Process();
                break;

            case DC:
                break;

            case Single_Phase_Inverter:
            case Three_Phase_Inverter:
                Inverter_Process();
                break;

            case VIS:
                VIS_Process();
                break;
        }
    }

    timer_cnt_arr[3] = TIM1->CNT;
    if((TIM1->CR1 & 0x10) == 0x10)
    {
        timer_cnt_arr[3] = TIM1->ARR * 2 - timer_cnt_arr[3];
    }
    // GPIOB->BSRR = (uint32_t)GPIO_PIN_1 << 16U;
    // GPIOB->BSRR = (uint32_t)GPIO_PIN_0 << 16U;
}

/**
 * @brief This function handles TIM1 update interrupt and TIM10 global interrupt.
 */
void TIM1_UP_TIM10_IRQHandler(void)
{
    /* USER CODE BEGIN TIM1_UP_TIM10_IRQn 0 */
    GPIOB->BSRR = GPIO_PIN_1;
    /* USER CODE END TIM1_UP_TIM10_IRQn 0 */
    // HAL_TIM_IRQHandler(&htim1);
    TIM1->SR &= ~0x01;
    /* USER CODE BEGIN TIM1_UP_TIM10_IRQn 1 */
    GPIOB->BSRR = (uint32_t)GPIO_PIN_1 << 16U;
    /* USER CODE END TIM1_UP_TIM10_IRQn 1 */
}

/**
 * @brief This function handles TIM1 trigger and commutation interrupts and TIM11 global interrupt.
 */
void TIM1_TRG_COM_TIM11_IRQHandler(void)
{
    /* USER CODE BEGIN TIM1_TRG_COM_TIM11_IRQn 0 */

    /* USER CODE END TIM1_TRG_COM_TIM11_IRQn 0 */
    // HAL_TIM_IRQHandler(&htim1);
    TIM1->SR &= ~0x20;
    /* USER CODE BEGIN TIM1_TRG_COM_TIM11_IRQn 1 */

    /* USER CODE END TIM1_TRG_COM_TIM11_IRQn 1 */
}

/**
 * @brief This function handles TIM3 global interrupt.
 */
void TIM3_IRQHandler(void)
{
    /* USER CODE BEGIN TIM3_IRQn 0 */

    /* USER CODE END TIM3_IRQn 0 */
    // HAL_TIM_IRQHandler(&htim3);
    TIM3->SR = 0;
    /* USER CODE BEGIN TIM3_IRQn 1 */

    /* USER CODE END TIM3_IRQn 1 */
}

/**
 * @brief This function handles TIM4 global interrupt.
 */
void TIM4_IRQHandler(void)
{
    /* USER CODE BEGIN TIM4_IRQn 0 */
    PWM_T_Count = TIM4->CCR1;
    PWM_H_Count = TIM4->CCR2;
    PWM_Duty = (float)PWM_H_Count / PWM_T_Count;
    PWM_T_us = PWM_T_Count / 3;
    PWM_H_us = PWM_H_Count / 3;
    /* USER CODE END TIM4_IRQn 0 */
    // HAL_TIM_IRQHandler(&htim4);
    TIM4->SR = 0;
    /* USER CODE BEGIN TIM4_IRQn 1 */

    /* USER CODE END TIM4_IRQn 1 */
}

/**
 * @brief This function handles TIM7 global interrupt.
 */
void TIM7_IRQHandler(void)
{
    /* USER CODE BEGIN TIM7_IRQn 0 */

    /* USER CODE END TIM7_IRQn 0 */
    // HAL_TIM_IRQHandler(&htim7);
    TIM7->SR = 0;
    HAL_IncTick();
    /* USER CODE BEGIN TIM7_IRQn 1 */

    /* USER CODE END TIM7_IRQn 1 */
}

/**
 * @brief This function handles DMA1 stream0 global interrupt.
 */
void DMA1_Stream0_IRQHandler(void)
{
    /* USER CODE BEGIN DMA1_Stream0_IRQn 0 */

    /* USER CODE END DMA1_Stream0_IRQn 0 */
    HAL_DMA_IRQHandler(&hdma_tim4_ch1);
    /* USER CODE BEGIN DMA1_Stream0_IRQn 1 */

    /* USER CODE END DMA1_Stream0_IRQn 1 */
}

/**
 * @brief This function handles DMA1 stream3 global interrupt.
 */
void DMA1_Stream3_IRQHandler(void)
{
    /* USER CODE BEGIN DMA1_Stream0_IRQn 0 */

    /* USER CODE END DMA1_Stream0_IRQn 0 */
    HAL_DMA_IRQHandler(&hdma_tim4_ch2);
    /* USER CODE BEGIN DMA1_Stream0_IRQn 1 */

    /* USER CODE END DMA1_Stream0_IRQn 1 */
}

/**
 * @brief This function handles DMA2 stream0 global interrupt.
 */
void DMA2_Stream0_IRQHandler(void)
{
    /* USER CODE BEGIN DMA2_Stream0_IRQn 0 */

    /* USER CODE END DMA2_Stream0_IRQn 0 */
    HAL_DMA_IRQHandler(&hdma_adc3);
    /* USER CODE BEGIN DMA2_Stream0_IRQn 1 */

    /* USER CODE END DMA2_Stream0_IRQn 1 */
}

/**
 * @brief This function handles USB On The Go FS global interrupt.
 */
void OTG_FS_IRQHandler(void)
{
    /* USER CODE BEGIN OTG_FS_IRQn 0 */

    /* USER CODE END OTG_FS_IRQn 0 */
    HAL_PCD_IRQHandler(&hpcd_USB_OTG_FS);
    /* USER CODE BEGIN OTG_FS_IRQn 1 */

    /* USER CODE END OTG_FS_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
