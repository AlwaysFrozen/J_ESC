/**
 * @file realtime_callback.c
 * @author jinchl (jcl709905683@gmail.com)
 * @brief 
 * @version 0.1
 * @date 2026-10-16
 * 
 * @copyright Copyright (c) 2026
 * 
 */


#include "realtime.h"
#include "global.h"

#include "stm32f4xx_hal.h"
#include "stm32f4xx_ll_tim.h"
#include "adc.h"
#include "tim.h"

/**
 * @brief 高频实时中断回调
 * 
 */
void high_realtime_callback(void)
{
    volatile Power_Status_t *p_power_status                   = &global_status.power_status;
    volatile Peripheral_Raw_Status_t *p_peripheral_raw_status = &global_status.peripheral_raw_status;


    p_power_status->v_bus     = p_peripheral_raw_status->v_bus_adc_raw * BOARD_BUS_VOLTAGE_CONVERT_COFFCIENT;
    p_power_status->inv_v_bus = p_power_status->v_bus > FLOAT_EPSILON ? 1.0f / p_power_status->v_bus : 0.0f;

    encoder_update_frt(0);

    motor_update_frt(0);
}

/**
 * @brief 实时中断回调
 * 
 */
void realtime_callback(void)
{
    volatile Power_Status_t *p_power_status                   = &global_status.power_status;
    volatile Motor_Basic_Status_t *p_motor_basic_status      = &global_status.motor_basic_status[0];

    p_power_status->core_temp  = 0;
    p_power_status->board_temp = 0;

    p_power_status->i_bus  = p_motor_basic_status->i_bus;
    p_power_status->board_power = p_motor_basic_status->e_power;

    encoder_update_rt(0);

    motor_update_rt(0);
}

/**
 * @brief 非实时中断回调
 * 
 */
void none_realtime_callback(void)
{
    motor_update_nrt(0);
}

/**
 * @brief 初始化高频实时中断
 * 
 */
void high_realtime_init(void)
{
    /* Enable/stabilize both slaves first, then arm the master interrupt.
        In triple mode, InjectedStart on a slave only enables it; ADC1's
        TIM8_CC4 trigger starts all three injected sequences together. */
    if ((HAL_ADCEx_InjectedStart(&hadc2) != HAL_OK) ||
        (HAL_ADCEx_InjectedStart(&hadc3) != HAL_OK) ||
        (HAL_ADCEx_InjectedStart_IT(&hadc1) != HAL_OK))
    {
        Error_Handler();
    }

    /* ADC1 regular scan: board temperature, bus voltage, motor temperature,
        analog input.  The regular results are read from the DMA buffer in the
        injected conversion ISR, so DMA completion interrupts are disabled. */
    if (HAL_ADC_Start_DMA(&hadc1, (uint32_t *)adc1_regular_dma_buffer, 4U) != HAL_OK)
    {
        Error_Handler();
    }
    __HAL_DMA_DISABLE_IT(&hdma_adc1, DMA_IT_TC | DMA_IT_HT | DMA_IT_TE | DMA_IT_DME);
    __HAL_DMA_DISABLE_IT(&hdma_adc1, DMA_IT_FE);
    HAL_NVIC_DisableIRQ(DMA2_Stream4_IRQn);

    HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_4);

    LL_TIM_CC_EnableChannel(TIM1,
        LL_TIM_CHANNEL_CH1  | LL_TIM_CHANNEL_CH1N |
        LL_TIM_CHANNEL_CH2  | LL_TIM_CHANNEL_CH2N |
        LL_TIM_CHANNEL_CH3  | LL_TIM_CHANNEL_CH3N);

    LL_TIM_EnableAllOutputs(TIM1);

    if (HAL_TIM_Base_Start(&htim1) != HAL_OK)
    {
        Error_Handler();
    }
}

/**
 * @brief 初始化实时中断
 * 
 */
void realtime_init(void)
{
    __HAL_TIM_CLEAR_FLAG(&htim2, TIM_FLAG_UPDATE);
    if (HAL_TIM_Base_Start_IT(&htim2) != HAL_OK)
    {
        Error_Handler();
    }
}
