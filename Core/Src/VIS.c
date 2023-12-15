#include "Board_Config.h"
#include "main.h"
#include "Sensor.h"
#include "VoltageCurrentSensor.h"
#include "Motor.h"
#include "VIS.h"
#include "BLDC_Motor.h"
#include "FOC_Motor.h"
#include "FOC.h"
#include "Inverter.h"

#include "Encoder.h"
#include "AS5048a.h"
#include "FOC.h"

// Debug Data
extern float debug_arr[21];

VIS_RUN_t vis_run;
VIS_CONTROL_t  vis_ctrl = 
{
    .PWM_freq = 80000,
    .output_min = 0,
    .output_max = 0.5,
    .output = 0.1,
    .step_us = 200,
    .loop_num = 10,
};

static float current_list[6];
static int32_t index = 0;
static int32_t index_list[10] = {0};


void Vector_0(uint16_t ARR_value,uint16_t CCR_value)
{
    TIM1->ARR = ARR_value;
    TIM1->CCR1 = 0;
    TIM1->CCR2 = 0;
    TIM1->CCR3 = 0;
    TIM1->CCER &= ~0x555;
}

void Vector_1(uint16_t ARR_value,uint16_t CCR_value)
{
    TIM1->ARR = ARR_value;
    TIM1->CCR1 = CCR_value;
    TIM1->CCR2 = 0;
    TIM1->CCR3 = 0;
    TIM1->CCER |= 0x555;
}
 
void Vector_2(uint16_t ARR_value,uint16_t CCR_value)
{
    TIM1->ARR = ARR_value;
    TIM1->CCR1 = CCR_value;
    TIM1->CCR2 = CCR_value;
    TIM1->CCR3 = 0;
    TIM1->CCER |= 0x555;
}

void Vector_3(uint16_t ARR_value,uint16_t CCR_value)
{
    TIM1->ARR = ARR_value;
    TIM1->CCR1 = 0;
    TIM1->CCR2 = CCR_value;
    TIM1->CCR3 = 0;
    TIM1->CCER |= 0x555;
}

void Vector_4(uint16_t ARR_value,uint16_t CCR_value)
{
    TIM1->ARR = ARR_value;
    TIM1->CCR1 = 0;
    TIM1->CCR2 = CCR_value;
    TIM1->CCR3 = CCR_value;
    TIM1->CCER |= 0x555;
}

void Vector_5(uint16_t ARR_value,uint16_t CCR_value)
{
    TIM1->ARR = ARR_value;
    TIM1->CCR1 = 0;
    TIM1->CCR2 = 0;
    TIM1->CCR3 = CCR_value;
    TIM1->CCER |= 0x555;
}

void Vector_6(uint16_t ARR_value,uint16_t CCR_value)
{
    TIM1->ARR = ARR_value;
    TIM1->CCR1 = CCR_value;
    TIM1->CCR2 = 0;
    TIM1->CCR3 = CCR_value;
    TIM1->CCER |= 0x555;
}

void VIS_Process(void)
{
    // AS5048_Read_M_Ang();
    // AS5048_para.e_angle = AS5048_para.m_angle * 2;
    // // AS5048_para.e_angle = AS5048_para.m_angle * 14;
    // AS5048_para.e_angle = Normalize_Angle(AS5048_para.e_angle);

    if(vis_run.run_state == VIS_Run && vis_run.loop_num_cnt)
    {
        if(vis_run.delay_Cnt ++ > vis_run.delay_Cnt_Ref)
        {
            vis_run.delay_Cnt = 0;
            vis_run.delay_Cnt_Ref = vis_run.PWM_freq_now * vis_ctrl.step_us / 1000000;

            vis_run.TIM1_ARR_now = PWM_TIM_BASE_FREQ / 2 / vis_run.PWM_freq_now;
            vis_run.duty = (vis_run.TIM1_ARR_now - 1) * vis_ctrl.output;
            Virtual_Moto.dt = 1.0f / vis_run.PWM_freq_now;

            switch(vis_run.step_index)
            {
                case 0:
                    Vector_0(vis_run.TIM1_ARR_now,vis_run.duty);
                    vis_run.step_index = vis_run.step_index_next;
                    switch(vis_run.step_index_now)
                    {
                        case 1:
                            current_list[0] = fabsf(phase_current_A[0]);
                            break;

                        case 4:
                            current_list[1] = fabsf(phase_current_A[0]);
                            break;

                        case 2:
                            current_list[2] = fabsf(phase_current_A[2]);
                            break;
                        
                        case 5:
                            current_list[3] = fabsf(phase_current_A[2]);
                            break;

                        case 3:
                            current_list[4] = fabsf(phase_current_A[1]);
                            break;
                        
                        case 6:
                            current_list[5] = fabsf(phase_current_A[1]);
                            // index = (uint8_t)(current_list[0] < current_list[1]) + ((uint8_t)(current_list[2] < current_list[3]) << 1) + ((uint8_t)(current_list[4] < current_list[5]) << 2);
                            index = (uint8_t)(current_list[0] > current_list[1]) + ((uint8_t)(current_list[2] > current_list[3]) << 1) + ((uint8_t)(current_list[4] > current_list[5]) << 2);
                            index_list[--vis_run.loop_num_cnt] = index;
                            if(vis_run.loop_num_cnt == 0)
                            {
                                int repeated_index = 0;
                                int cnt = Find_Most_Repeated_Element(index_list,10,&repeated_index);
                                if(cnt > 10 / 2)
                                {
                                    switch(repeated_index)
                                    {
                                        case 0:
                                            vis_run.VIS_credible = 1;
                                            vis_run.VIS_e_angle = _PI_3 * 4;
                                            vis_run.VIS_index = repeated_index;
                                            break;

                                        case 1:
                                            vis_run.VIS_credible = 1;
                                            vis_run.VIS_e_angle = _PI_3 * 5;
                                            vis_run.VIS_index = repeated_index;
                                            break;

                                        case 2:
                                            break;

                                        case 3:
                                            vis_run.VIS_credible = 1;
                                            vis_run.VIS_e_angle = _PI_3 * 0;
                                            vis_run.VIS_index = repeated_index;
                                            break;

                                        case 4:
                                            vis_run.VIS_credible = 1;
                                            vis_run.VIS_e_angle = _PI_3 * 3;
                                            vis_run.VIS_index = repeated_index;
                                            break;

                                        case 5:
                                            break;

                                        case 6:
                                            vis_run.VIS_credible = 1;
                                            vis_run.VIS_e_angle = _PI_3 * 2;
                                            vis_run.VIS_index = repeated_index;
                                            break;

                                        case 7:
                                            vis_run.VIS_credible = 1;
                                            vis_run.VIS_e_angle = _PI_3 * 1;
                                            vis_run.VIS_index = repeated_index;
                                            break;
                                    }

                                    if(vis_run.VIS_credible)
                                    {
                                        Stop_VIS();
                                        motor_type_now = Motor_Config.moto_type;
                                        switch (motor_type_now)
                                        {
                                            case BLDC:
                                                Init_BLDC_Motor();
                                                Start_BLDC_Motor();
                                                break;

                                            case FOC:
                                                Init_FOC_Motor();
                                                Start_FOC_Motor();
                                                break;

                                            case DC:
                                                break;

                                            case Single_Phase_Inverter:
                                            case Three_Phase_Inverter:
                                                Start_Inverter();
                                                break;

                                            default:
                                                break;
                                        }
                                    }
                                    else
                                    {
                                        vis_run.loop_num_cnt = vis_ctrl.loop_num;
                                    }
                                }
                                else
                                {
                                    vis_run.loop_num_cnt = vis_ctrl.loop_num;
                                }
                            }
                            break;
                    }
                    vis_run.step_index_now = 0;
                    break;

                case 1:
                    Vector_1(vis_run.TIM1_ARR_now,vis_run.duty);
                    vis_run.step_index = 0;
                    vis_run.step_index_now = 1;
                    vis_run.step_index_next = 4;
                    break;

                case 4:
                    Vector_4(vis_run.TIM1_ARR_now,vis_run.duty);
                    vis_run.step_index = 0;
                    vis_run.step_index_now = 4;
                    vis_run.step_index_next = 2;
                    break;

                case 2:
                    Vector_2(vis_run.TIM1_ARR_now,vis_run.duty);
                    vis_run.step_index = 0;
                    vis_run.step_index_now = 2;
                    vis_run.step_index_next = 5;
                    break;

                case 5:
                    Vector_5(vis_run.TIM1_ARR_now,vis_run.duty);
                    vis_run.step_index = 0;
                    vis_run.step_index_now = 5;
                    vis_run.step_index_next = 3;
                    break;

                case 3:
                    Vector_3(vis_run.TIM1_ARR_now,vis_run.duty);
                    vis_run.step_index = 0;
                    vis_run.step_index_now = 3;
                    vis_run.step_index_next = 6;
                    break;
                    
                case 6:
                    Vector_6(vis_run.TIM1_ARR_now,vis_run.duty);
                    vis_run.step_index = 0;
                    vis_run.step_index_now = 6;
                    vis_run.step_index_next = 1;
                    break;
            }
        }
    }

    // debug_arr[0] = phase_voltage_V[0];
    // debug_arr[1] = phase_voltage_V[1];
    // debug_arr[2] = phase_voltage_V[2];
    // debug_arr[3] = phase_voltage_V_f[0];
    // debug_arr[4] = phase_voltage_V_f[1];
    // debug_arr[5] = phase_voltage_V_f[2];
    // debug_arr[6] = fabsf(phase_current_A[0]);
    // debug_arr[7] = fabsf(phase_current_A[1]);
    // debug_arr[8] = fabsf(phase_current_A[2]);
    // debug_arr[9] = phase_current_A_f[0];
    // debug_arr[10] = phase_current_A_f[1];
    // debug_arr[11] = phase_current_A_f[2];
    // debug_arr[12] = vis_run.step_index_now;
    // debug_arr[13] = vis_run.step_index_next;
    // debug_arr[14] = vis_run.VIS_index;
    // debug_arr[15] = vis_run.VIS_e_angle;
    // debug_arr[16] = 0;
    // debug_arr[17] = 0;
    // debug_arr[18] = (int)(AS5048_para.e_angle / _PI_3) % 6 * _PI_3;
    // debug_arr[19] = AS5048_para.e_angle;

    // CDC_Transmit_FS((uint8_t *)debug_arr,sizeof(debug_arr));
}

void Clear_Vis(void)
{
    vis_run.VIS_credible = 0;
    vis_run.VIS_e_angle = 0;
    vis_run.VIS_index = 0;
}

void Start_VIS(void)
{
    vis_run.VIS_credible = 0;
    vis_run.step_index = 1;
    vis_run.step_index_next = 0;
    vis_run.loop_num_cnt = vis_ctrl.loop_num;
    vis_run.delay_Cnt_Ref = vis_run.PWM_freq_now * vis_ctrl.step_us / 1000000;
    vis_run.delay_Cnt = vis_run.delay_Cnt_Ref + 1;
    vis_run.run_state = VIS_Run;
}

void Stop_VIS(void)
{
    vis_run.step_index = 1;
    vis_run.step_index_next = 0;
    vis_run.delay_Cnt_Ref = vis_run.PWM_freq_now * vis_ctrl.step_us / 1000000;
    vis_run.delay_Cnt = vis_run.delay_Cnt_Ref + 1;
    vis_run.run_state = VIS_Stop;
    TIM1->CCER &= ~0x555;
}

void Init_VIS(void)
{
    // AS5048_para.m_angle_offset = 0.995028377;
    // AS5048_para.m_angle_offset = 5.67513323;

    vis_run.VIS_credible = 0;
    vis_run.PWM_freq_now = vis_ctrl.PWM_freq;
    vis_run.TIM1_ARR_now = PWM_TIM_BASE_FREQ / 2 / vis_run.PWM_freq_now;
    Virtual_Moto.dt = 1.0f / vis_run.PWM_freq_now;
    TIM1->CCER &= ~0x555;
    // enable preload
    TIM1->CR1 |= 0x80;
    TIM1->CCMR1 |= 0x808;
    TIM1->CCMR2 |= 0x08;
    #ifdef ADC_SAMPLE_HIGH_SIDE
    TIM1->CCR4 = 1;
    #else
    TIM1->CCR4 = TIM1->ARR - 1;
    #endif
}
