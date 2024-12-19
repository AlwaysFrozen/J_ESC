#include "Board_Config.h"
#include "VoltageCurrentSensor.h"
#include "cmsis_os.h"
#include "Motor.h"
#include "BLDC_Motor.h"
#include "FOC.h"
#include "FOC_Motor.h"
#include "FOC_Config.h"

ADC_Cali_t adc_cali = {.ADC_Cali_Value[3] = 2047, .ADC_Cali_Value[4] = 2047, .ADC_Cali_Value[5] = 2047};

uint16_t Vbus_ADC = 0;
float adc_value[6] = {0};

UVW_Axis_t phase_voltage_V;
UVW_Axis_t phase_voltage_V_f;
UVW_Axis_t phase_current_A;
UVW_Axis_t phase_current_A_f;
UVW_Axis_t *p_phase_voltage_V = &phase_voltage_V;
UVW_Axis_t *p_phase_current_A = &phase_current_A;

Sensor_Cali_Err_t Voltage_Current_Sensor_Calibration(void)
{
    Sensor_Cali_Err_t err = Sensor_Cali_Err_NONE;
    ADC_Cali_t adc_cali_temp;

    //adc cali
    MOS_Driver_Disable();
    osDelay(200);
    memset(&adc_cali_temp,0,sizeof(ADC_Cali_t));

    for(adc_cali_temp.ADC_Cali_Cnt = 0;adc_cali_temp.ADC_Cali_Cnt < 64;adc_cali_temp.ADC_Cali_Cnt++)
    {
        for(uint8_t i = 0;i < 6;i++)
        {
            adc_cali_temp.ADC_Cali_Value[i] += adc_value[i];
        }
        osDelay(1);
    }
    for(uint8_t i = 0;i < 6;i++)
    {
        adc_cali_temp.ADC_Cali_Value[i] /= adc_cali_temp.ADC_Cali_Cnt;
        if(i < 3)
        {
            if(adc_cali_temp.ADC_Cali_Value[i] > 100)
            {
                err |= Sensor_Cali_Err_ADC_V_IDLE;
            }
        }
        else
        {
            if(adc_cali_temp.ADC_Cali_Value[i] > 2147 || adc_cali_temp.ADC_Cali_Value[i] < 1947)
            {
                err |= Sensor_Cali_Err_ADC_I_IDLE;
            }
        }
    }
    memcpy(&adc_cali,&adc_cali_temp,sizeof(ADC_Cali_t));
    
#if 0
    uint16_t arr = 0;
    uint16_t ccr = 0;
    float ccr_rate = 0;
    uint16_t current_stable_cnt = 0;

    float phase_current_ratio[3];
    float phase_voltage_sort[3];
    uint16_t phase_voltage_sort_index[3];

    arr = PWM_TIM_BASE_FREQ / 2 / 20000;//20KHz
    ccr = arr * ccr_rate;
    #if 0
    Vector_1(arr,ccr);
    MOS_Driver_Enable();
    while(ccr_rate < 0.2f)
    {
        if(fabsf(phase_current_A[0]) > current_limit || fabsf(phase_current_A[1]) > current_limit || fabsf(phase_current_A[2]) > current_limit)
        {
            if(current_stable_cnt++ > 500)
            {
                break;
            }
        }
        else
        {
            current_stable_cnt = 0;
            ccr = arr * ccr_rate;
            Vector_1(arr,ccr);
            ccr_rate += 0.001f;
        }
        osDelay(2);
    }

    //phase voltage
    memcpy(phase_voltage_sort,phase_voltage_V,sizeof(phase_voltage_sort));
    phase_voltage_sort_index[0] = 0;phase_voltage_sort_index[1] = 1;phase_voltage_sort_index[2] = 2;
    Index_Sort(phase_voltage_sort,3,phase_voltage_sort_index);
    if(phase_voltage_V[phase_voltage_sort_index[0]] < 300 && phase_voltage_V[phase_voltage_sort_index[1]] < 300 && phase_voltage_V[phase_voltage_sort_index[2]] > 300)
    {
        p_phase_voltage_V[0] = &phase_voltage_V[phase_voltage_sort_index[2]];
    }
    else
    {
        err |= Sensor_Cali_Err_PHASE_V_ORDER;
    }
    //phase current
    phase_current_ratio[0] = fabsf(phase_current_A[0] / phase_current_A[1]);//A / B
    phase_current_ratio[1] = fabsf(phase_current_A[0] / phase_current_A[2]);//A / C
    phase_current_ratio[2] = fabsf(phase_current_A[1] / phase_current_A[2]);//B / C
    if(Number_In_Absolute_Range_f32(phase_current_ratio[0],1.0f,0.35f))//A=B
    {
        if(Number_In_Absolute_Range_f32(phase_current_ratio[1],0.5f,0.35f) && Number_In_Absolute_Range_f32(phase_current_ratio[2],0.5f,0.35f))//2A=2B=C
        {
            p_phase_current_A[0] = &phase_current_A[2];
        }
        else
        {
            err |= Sensor_Cali_Err_PHASE_I_ORDER;
        }
    }
    else if(Number_In_Absolute_Range_f32(phase_current_ratio[1],1.0f,0.35f))//A=C
    {
        if(Number_In_Absolute_Range_f32(phase_current_ratio[0],0.5f,0.35f) && Number_In_Absolute_Range_f32(phase_current_ratio[2],2.0f,0.35f))//2A=B=2C
        {
            p_phase_current_A[0] = &phase_current_A[1];
        }
        else
        {
            err |= Sensor_Cali_Err_PHASE_I_ORDER;
        }
    }
    else if(Number_In_Absolute_Range_f32(phase_current_ratio[2],1.0f,0.35f))//B=C
    {
        if(Number_In_Absolute_Range_f32(phase_current_ratio[0],2.0f,0.35f) && Number_In_Absolute_Range_f32(phase_current_ratio[1],2.0f,0.35f))//A=2B=2C
        {
            p_phase_current_A[0] = &phase_current_A[0];
        }
        else
        {
            err |= Sensor_Cali_Err_PHASE_I_ORDER;
        }
    }
    else
    {
        err |= Sensor_Cali_Err_PHASE_I_ORDER;
    }
    Full_Brake();
    osDelay(300);
    Vector_3(arr,ccr);
    osDelay(500);
    //phase voltage
    memcpy(phase_voltage_sort,phase_voltage_V,sizeof(phase_voltage_sort));
    phase_voltage_sort_index[0] = 0;phase_voltage_sort_index[1] = 1;phase_voltage_sort_index[2] = 2;
    Index_Sort(phase_voltage_sort,3,phase_voltage_sort_index);
    if(phase_voltage_V[phase_voltage_sort_index[0]] < 300 && phase_voltage_V[phase_voltage_sort_index[1]] < 300 && phase_voltage_V[phase_voltage_sort_index[2]] > 300)
    {
        p_phase_voltage_V[1] = &phase_voltage_V[phase_voltage_sort_index[2]];
    }
    else
    {
        err |= Sensor_Cali_Err_PHASE_V_ORDER;
    }
    //phase current
    phase_current_ratio[0] = fabsf(phase_current_A[0] / phase_current_A[1]);//A / B
    phase_current_ratio[1] = fabsf(phase_current_A[0] / phase_current_A[2]);//A / C
    phase_current_ratio[2] = fabsf(phase_current_A[1] / phase_current_A[2]);//B / C
    if(Number_In_Absolute_Range_f32(phase_current_ratio[0],1.0f,0.35f))//A=B
    {
        if(Number_In_Absolute_Range_f32(phase_current_ratio[1],0.5f,0.35f) && Number_In_Absolute_Range_f32(phase_current_ratio[2],0.5f,0.35f))//2A=2B=C
        {
            p_phase_current_A[1] = &phase_current_A[2];
        }
        else
        {
            err |= Sensor_Cali_Err_PHASE_I_ORDER;
        }
    }
    else if(Number_In_Absolute_Range_f32(phase_current_ratio[1],1.0f,0.35f))//A=C
    {
        if(Number_In_Absolute_Range_f32(phase_current_ratio[0],0.5f,0.35f) && Number_In_Absolute_Range_f32(phase_current_ratio[2],2.0f,0.35f))//2A=B=2C
        {
            p_phase_current_A[1] = &phase_current_A[1];
        }
        else
        {
            err |= Sensor_Cali_Err_PHASE_I_ORDER;
        }
    }
    else if(Number_In_Absolute_Range_f32(phase_current_ratio[2],1.0f,0.35f))//B=C
    {
        if(Number_In_Absolute_Range_f32(phase_current_ratio[0],2.0f,0.35f) && Number_In_Absolute_Range_f32(phase_current_ratio[1],2.0f,0.35f))//A=2B=2C
        {
            p_phase_current_A[1] = &phase_current_A[0];
        }
        else
        {
            err |= Sensor_Cali_Err_PHASE_I_ORDER;
        }
    }
    else
    {
        err |= Sensor_Cali_Err_PHASE_I_ORDER;
    }
    Full_Brake();
    osDelay(300);
    Vector_5(arr,ccr);
    osDelay(500);
    //phase voltage
    memcpy(phase_voltage_sort,phase_voltage_V,sizeof(phase_voltage_sort));
    phase_voltage_sort_index[0] = 0;phase_voltage_sort_index[1] = 1;phase_voltage_sort_index[2] = 2;
    Index_Sort(phase_voltage_sort,3,phase_voltage_sort_index);
    if(phase_voltage_V[phase_voltage_sort_index[0]] < 300 && phase_voltage_V[phase_voltage_sort_index[1]] < 300 && phase_voltage_V[phase_voltage_sort_index[2]] > 300)
    {
        p_phase_voltage_V[2] = &phase_voltage_V[phase_voltage_sort_index[2]];
    }
    else
    {
        err |= Sensor_Cali_Err_PHASE_V_ORDER;
    }
    //phase current
    phase_current_ratio[0] = fabsf(phase_current_A[0] / phase_current_A[1]);//A / B
    phase_current_ratio[1] = fabsf(phase_current_A[0] / phase_current_A[2]);//A / C
    phase_current_ratio[2] = fabsf(phase_current_A[1] / phase_current_A[2]);//B / C
    if(Number_In_Absolute_Range_f32(phase_current_ratio[0],1.0f,0.35f))//A=B
    {
        if(Number_In_Absolute_Range_f32(phase_current_ratio[1],0.5f,0.35f) && Number_In_Absolute_Range_f32(phase_current_ratio[2],0.5f,0.35f))//2A=2B=C
        {
            p_phase_current_A[2] = &phase_current_A[2];
        }
        else
        {
            err |= Sensor_Cali_Err_PHASE_I_ORDER;
        }
    }
    else if(Number_In_Absolute_Range_f32(phase_current_ratio[1],1.0f,0.35f))//A=C
    {
        if(Number_In_Absolute_Range_f32(phase_current_ratio[0],0.5f,0.35f) && Number_In_Absolute_Range_f32(phase_current_ratio[2],2.0f,0.35f))//2A=B=2C
        {
            p_phase_current_A[2] = &phase_current_A[1];
        }
        else
        {
            err |= Sensor_Cali_Err_PHASE_I_ORDER;
        }
    }
    else if(Number_In_Absolute_Range_f32(phase_current_ratio[2],1.0f,0.35f))//B=C
    {
        if(Number_In_Absolute_Range_f32(phase_current_ratio[0],2.0f,0.35f) && Number_In_Absolute_Range_f32(phase_current_ratio[1],2.0f,0.35f))//A=2B=2C
        {
            p_phase_current_A[2] = &phase_current_A[0];
        }
        else
        {
            err |= Sensor_Cali_Err_PHASE_I_ORDER;
        }
    }
    else
    {
        err |= Sensor_Cali_Err_PHASE_I_ORDER;
    }
    
    //disable output
    Full_Brake();
    MOS_Driver_Disable();
    #endif

    #if 0
    Vector_1(arr,ccr);
    MOS_Driver_Enable();
    while(ccr_rate < 0.2f)
    {
        if(fabsf(phase_current_A[0]) > current_limit || fabsf(phase_current_A[1]) > current_limit || fabsf(phase_current_A[2]) > current_limit)
        {
            if(current_stable_cnt++ > 500)
            {
                break;
            }
        }
        else
        {
            current_stable_cnt = 0;
            ccr = arr * ccr_rate;
            Vector_1(arr,ccr);
            ccr_rate += 0.001f;
        }
        osDelay(2);
    }

    //phase voltage
    memcpy(phase_voltage_sort,phase_voltage_V,sizeof(phase_voltage_sort));
    phase_voltage_sort_index[0] = 0;phase_voltage_sort_index[1] = 1;phase_voltage_sort_index[2] = 2;
    Index_Sort(phase_voltage_sort,3,phase_voltage_sort_index);
    if(phase_voltage_V[phase_voltage_sort_index[0]] < 300 && phase_voltage_V[phase_voltage_sort_index[1]] < 300 && phase_voltage_V[phase_voltage_sort_index[2]] > 300)
    {
        p_phase_voltage_V[0] = &phase_voltage_V[phase_voltage_sort_index[2]];
    }
    else
    {
        err |= Sensor_Cali_Err_PHASE_V_ORDER;
    }
    //phase current
    phase_current_ratio[0] = phase_current_A[0] / phase_current_A[1];//A / B
    phase_current_ratio[1] = phase_current_A[0] / phase_current_A[2];//A / C
    phase_current_ratio[2] = phase_current_A[1] / phase_current_A[2];//B / C
    if(Number_In_Absolute_Range_f32(phase_current_ratio[0],1.0f,0.5f))//A=B
    {
        if(Number_In_Absolute_Range_f32(phase_current_ratio[1],-0.5f,0.5f) && Number_In_Absolute_Range_f32(phase_current_ratio[2],-0.5f,0.5f))//-2A=-2B=C
        {
            p_phase_current_A[0] = &phase_current_A[2];
        }
        else
        {
            err |= Sensor_Cali_Err_PHASE_I_ORDER;
        }
    }
    else if(Number_In_Absolute_Range_f32(phase_current_ratio[1],1.0f,0.5f))//A=C
    {
        if(Number_In_Absolute_Range_f32(phase_current_ratio[0],-0.5f,0.5f) && Number_In_Absolute_Range_f32(phase_current_ratio[2],-2.0f,0.5f))//-2A=B=-2C
        {
            p_phase_current_A[0] = &phase_current_A[1];
        }
        else
        {
            err |= Sensor_Cali_Err_PHASE_I_ORDER;
        }
    }
    else if(Number_In_Absolute_Range_f32(phase_current_ratio[2],1.0f,0.5f))//B=C
    {
        if(Number_In_Absolute_Range_f32(phase_current_ratio[0],-2.0f,0.5f) && Number_In_Absolute_Range_f32(phase_current_ratio[1],-2.0f,0.5f))//A=-2B=-2C
        {
            p_phase_current_A[0] = &phase_current_A[0];
        }
        else
        {
            err |= Sensor_Cali_Err_PHASE_I_ORDER;
        }
    }
    else
    {
        err |= Sensor_Cali_Err_PHASE_I_ORDER;
    }
    Full_Brake();
    osDelay(300);
    Vector_3(arr,ccr);
    osDelay(500);
    //phase voltage
    memcpy(phase_voltage_sort,phase_voltage_V,sizeof(phase_voltage_sort));
    phase_voltage_sort_index[0] = 0;phase_voltage_sort_index[1] = 1;phase_voltage_sort_index[2] = 2;
    Index_Sort(phase_voltage_sort,3,phase_voltage_sort_index);
    if(phase_voltage_V[phase_voltage_sort_index[0]] < 300 && phase_voltage_V[phase_voltage_sort_index[1]] < 300 && phase_voltage_V[phase_voltage_sort_index[2]] > 300)
    {
        p_phase_voltage_V[1] = &phase_voltage_V[phase_voltage_sort_index[2]];
    }
    else
    {
        err |= Sensor_Cali_Err_PHASE_V_ORDER;
    }
    //phase current
    phase_current_ratio[0] = phase_current_A[0] / phase_current_A[1];//A / B
    phase_current_ratio[1] = phase_current_A[0] / phase_current_A[2];//A / C
    phase_current_ratio[2] = phase_current_A[1] / phase_current_A[2];//B / C
    if(Number_In_Absolute_Range_f32(phase_current_ratio[0],1.0f,0.5f))//A=B
    {
        if(Number_In_Absolute_Range_f32(phase_current_ratio[1],-0.5f,0.5f) && Number_In_Absolute_Range_f32(phase_current_ratio[2],-0.5f,0.5f))//-2A=-2B=C
        {
            p_phase_current_A[1] = &phase_current_A[2];
        }
        else
        {
            err |= Sensor_Cali_Err_PHASE_I_ORDER;
        }
    }
    else if(Number_In_Absolute_Range_f32(phase_current_ratio[1],1.0f,0.5f))//A=C
    {
        if(Number_In_Absolute_Range_f32(phase_current_ratio[0],-0.5f,0.5f) && Number_In_Absolute_Range_f32(phase_current_ratio[2],-2.0f,0.5f))//-2A=B=-2C
        {
            p_phase_current_A[1] = &phase_current_A[1];
        }
        else
        {
            err |= Sensor_Cali_Err_PHASE_I_ORDER;
        }
    }
    else if(Number_In_Absolute_Range_f32(phase_current_ratio[2],1.0f,0.5f))//B=C
    {
        if(Number_In_Absolute_Range_f32(phase_current_ratio[0],-2.0f,0.5f) && Number_In_Absolute_Range_f32(phase_current_ratio[1],-2.0f,0.5f))//A=-2B=-2C
        {
            p_phase_current_A[1] = &phase_current_A[0];
        }
        else
        {
            err |= Sensor_Cali_Err_PHASE_I_ORDER;
        }
    }
    else
    {
        err |= Sensor_Cali_Err_PHASE_I_ORDER;
    }
    Full_Brake();
    osDelay(300);
    Vector_5(arr,ccr);
    osDelay(500);
    //phase voltage
    memcpy(phase_voltage_sort,phase_voltage_V,sizeof(phase_voltage_sort));
    phase_voltage_sort_index[0] = 0;phase_voltage_sort_index[1] = 1;phase_voltage_sort_index[2] = 2;
    Index_Sort(phase_voltage_sort,3,phase_voltage_sort_index);
    if(phase_voltage_V[phase_voltage_sort_index[0]] < 300 && phase_voltage_V[phase_voltage_sort_index[1]] < 300 && phase_voltage_V[phase_voltage_sort_index[2]] > 300)
    {
        p_phase_voltage_V[2] = &phase_voltage_V[phase_voltage_sort_index[2]];
    }
    else
    {
        err |= Sensor_Cali_Err_PHASE_V_ORDER;
    }
    //phase current
    phase_current_ratio[0] = phase_current_A[0] / phase_current_A[1];//A / B
    phase_current_ratio[1] = phase_current_A[0] / phase_current_A[2];//A / C
    phase_current_ratio[2] = phase_current_A[1] / phase_current_A[2];//B / C
    if(Number_In_Absolute_Range_f32(phase_current_ratio[0],1.0f,0.5f))//A=B
    {
        if(Number_In_Absolute_Range_f32(phase_current_ratio[1],-0.5f,0.5f) && Number_In_Absolute_Range_f32(phase_current_ratio[2],-0.5f,0.5f))//-2A=-2B=C
        {
            p_phase_current_A[2] = &phase_current_A[2];
        }
        else
        {
            err |= Sensor_Cali_Err_PHASE_I_ORDER;
        }
    }
    else if(Number_In_Absolute_Range_f32(phase_current_ratio[1],1.0f,0.5f))//A=C
    {
        if(Number_In_Absolute_Range_f32(phase_current_ratio[0],-0.5f,0.5f) && Number_In_Absolute_Range_f32(phase_current_ratio[2],-2.0f,0.5f))//-2A=B=-2C
        {
            p_phase_current_A[2] = &phase_current_A[1];
        }
        else
        {
            err |= Sensor_Cali_Err_PHASE_I_ORDER;
        }
    }
    else if(Number_In_Absolute_Range_f32(phase_current_ratio[2],1.0f,0.5f))//B=C
    {
        if(Number_In_Absolute_Range_f32(phase_current_ratio[0],-2.0f,0.5f) && Number_In_Absolute_Range_f32(phase_current_ratio[1],-2.0f,0.5f))//A=-2B=-2C
        {
            p_phase_current_A[2] = &phase_current_A[0];
        }
        else
        {
            err |= Sensor_Cali_Err_PHASE_I_ORDER;
        }
    }
    else
    {
        err |= Sensor_Cali_Err_PHASE_I_ORDER;
    }
    
    //disable output
    Full_Brake();
    MOS_Driver_Disable();
    #endif

    #if 1
    Vector_1(arr,ccr);
    MOS_Driver_Enable();
    while(ccr_rate < 0.2f)
    {
        if(fabsf(phase_current_A[0]) > current_limit || fabsf(phase_current_A[1]) > current_limit || fabsf(phase_current_A[2]) > current_limit)
        {
            if(current_stable_cnt++ > 500)
            {
                break;
            }
        }
        else
        {
            current_stable_cnt = 0;
            ccr = arr * ccr_rate;
            Vector_1(arr,ccr);
            ccr_rate += 0.001f;
        }
        osDelay(2);
    }

    //phase voltage
    memcpy(phase_voltage_sort,phase_voltage_V,sizeof(phase_voltage_sort));
    phase_voltage_sort_index[0] = 0;phase_voltage_sort_index[1] = 1;phase_voltage_sort_index[2] = 2;
    Index_Sort(phase_voltage_sort,3,phase_voltage_sort_index);
    if(phase_voltage_V[phase_voltage_sort_index[0]] < 300 && phase_voltage_V[phase_voltage_sort_index[1]] < 300 && phase_voltage_V[phase_voltage_sort_index[2]] > 300)
    {
        p_phase_voltage_V[0] = &phase_voltage_V[phase_voltage_sort_index[2]];
    }
    else
    {
        err |= Sensor_Cali_Err_PHASE_V_ORDER;
    }
    //phase current
    if(phase_current_A[2] > 0)// A<0 B<0 C>0
    {
        if(phase_current_A[0] < 0 && phase_current_A[1] < 0)//-2A=-2B=C
        {
            p_phase_current_A[0] = &phase_current_A[2];
        }
        else
        {
            err |= Sensor_Cali_Err_PHASE_I_ORDER;
        }
    }
    else if(phase_current_A[1] > 0)// A<0 B>0 C<0
    {
        if(phase_current_A[0] < 0 && phase_current_A[2] < 0)//-2A=B=-2C
        {
            p_phase_current_A[0] = &phase_current_A[1];
        }
        else
        {
            err |= Sensor_Cali_Err_PHASE_I_ORDER;
        }
    }
    else if(phase_current_A[0] > 0)// A>0 B<0 C<0
    {
        if(phase_current_A[1] < 0 && phase_current_A[2] < 0)//A=-2B=-2C
        {
            p_phase_current_A[0] = &phase_current_A[0];
        }
        else
        {
            err |= Sensor_Cali_Err_PHASE_I_ORDER;
        }
    }
    else
    {
        err |= Sensor_Cali_Err_PHASE_I_ORDER;
    }
    Full_Brake();
    osDelay(300);
    Vector_3(arr,ccr);
    osDelay(500);
    //phase voltage
    memcpy(phase_voltage_sort,phase_voltage_V,sizeof(phase_voltage_sort));
    phase_voltage_sort_index[0] = 0;phase_voltage_sort_index[1] = 1;phase_voltage_sort_index[2] = 2;
    Index_Sort(phase_voltage_sort,3,phase_voltage_sort_index);
    if(phase_voltage_V[phase_voltage_sort_index[0]] < 300 && phase_voltage_V[phase_voltage_sort_index[1]] < 300 && phase_voltage_V[phase_voltage_sort_index[2]] > 300)
    {
        p_phase_voltage_V[1] = &phase_voltage_V[phase_voltage_sort_index[2]];
    }
    else
    {
        err |= Sensor_Cali_Err_PHASE_V_ORDER;
    }
    //phase current
    if(phase_current_A[2] > 0)// A<0 B<0 C>0
    {
        if(phase_current_A[0] < 0 && phase_current_A[1] < 0)//-2A=-2B=C
        {
            p_phase_current_A[1] = &phase_current_A[2];
        }
        else
        {
            err |= Sensor_Cali_Err_PHASE_I_ORDER;
        }
    }
    else if(phase_current_A[1] > 0)// A<0 B>0 C<0
    {
        if(phase_current_A[0] < 0 && phase_current_A[2] < 0)//-2A=B=-2C
        {
            p_phase_current_A[1] = &phase_current_A[1];
        }
        else
        {
            err |= Sensor_Cali_Err_PHASE_I_ORDER;
        }
    }
    else if(phase_current_A[0] > 0)// A>0 B<0 C<0
    {
        if(phase_current_A[1] < 0 && phase_current_A[2] < 0)//A=-2B=-2C
        {
            p_phase_current_A[1] = &phase_current_A[0];
        }
        else
        {
            err |= Sensor_Cali_Err_PHASE_I_ORDER;
        }
    }
    else
    {
        err |= Sensor_Cali_Err_PHASE_I_ORDER;
    }
    Full_Brake();
    osDelay(300);
    Vector_5(arr,ccr);
    osDelay(500);
    //phase voltage
    memcpy(phase_voltage_sort,phase_voltage_V,sizeof(phase_voltage_sort));
    phase_voltage_sort_index[0] = 0;phase_voltage_sort_index[1] = 1;phase_voltage_sort_index[2] = 2;
    Index_Sort(phase_voltage_sort,3,phase_voltage_sort_index);
    if(phase_voltage_V[phase_voltage_sort_index[0]] < 300 && phase_voltage_V[phase_voltage_sort_index[1]] < 300 && phase_voltage_V[phase_voltage_sort_index[2]] > 300)
    {
        p_phase_voltage_V[2] = &phase_voltage_V[phase_voltage_sort_index[2]];
    }
    else
    {
        err |= Sensor_Cali_Err_PHASE_V_ORDER;
    }
    //phase current
    phase_current_ratio[0] = phase_current_A[0] / phase_current_A[1];//A / B
    phase_current_ratio[1] = phase_current_A[0] / phase_current_A[2];//A / C
    phase_current_ratio[2] = phase_current_A[1] / phase_current_A[2];//B / C
    if(phase_current_A[2] > 0)// A<0 B<0 C>0
    {
        if(phase_current_A[0] < 0 && phase_current_A[1] < 0)//-2A=-2B=C
        {
            p_phase_current_A[2] = &phase_current_A[2];
        }
        else
        {
            err |= Sensor_Cali_Err_PHASE_I_ORDER;
        }
    }
    else if(phase_current_A[1] > 0)// A<0 B>0 C<0
    {
        if(phase_current_A[0] < 0 && phase_current_A[2] < 0)//-2A=B=-2C
        {
            p_phase_current_A[2] = &phase_current_A[1];
        }
        else
        {
            err |= Sensor_Cali_Err_PHASE_I_ORDER;
        }
    }
    else if(phase_current_A[0] > 0)// A>0 B<0 C<0
    {
        if(phase_current_A[1] < 0 && phase_current_A[2] < 0)//A=-2B=-2C
        {
            p_phase_current_A[2] = &phase_current_A[0];
        }
        else
        {
            err |= Sensor_Cali_Err_PHASE_I_ORDER;
        }
    }
    else
    {
        err |= Sensor_Cali_Err_PHASE_I_ORDER;
    }
    
    //disable output
    Full_Brake();
    MOS_Driver_Disable();
    #endif

    if(err)
    {
        return err;
    }
#endif

    return err;
}

