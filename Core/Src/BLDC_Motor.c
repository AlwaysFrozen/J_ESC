#include "Board_Config.h"
#include "main.h"
#include "Sensor.h"

#include "BLDC_Motor.h"
#include "Motor.h"

#include "AS5048a.h"
#include "FOC_Motor.h"
#include "FOC.h"

#include "cmsis_os.h"


BLDC_RUN_t bldc_run;
BLDC_CONTROL_t  bldc_ctrl = 
{
    /* sensor config */
    // .sensor_type = HALL_120_SENSOR,
    /* motor config */
    .KV = 2500,
    .pole_pairs = 1,
    .delay_angle = 25,
    /* start config */ 
    .start_force = 0.1,
    .start_order_ms = 10,
    .start_first_step_ms = 10,
    .start_time_rate = 0.9,
    .start_force_rate = 0,
    .start_step_min = 6,
    .start_step_max = 60,
    .start_max_mse = 20,
    /* stall protect */ 
    .stall_protect_ms = 100,
    .stall_protect_mse = 200,
    .run_max_mse = 5,
    /* output config */
    .output_min = 0.1,
    .output_max = 1.0,
    /* fliter config */
    .virtual_mid_filter_rate = 0.05,
    /* VVVF config */
    .VVVF_method = BLDC_VVVF_VF,
    .VVVF_ratio = 10,
    .PWM_freq_min = 20 * 1000,
    .PWM_freq_max = 40 * 1000,
};


static const uint8_t step_seq_ccw[6] = {2,6,4,5,1,3};
static const uint8_t step_seq_cw[6] = {2,3,1,5,4,6};
static const uint8_t step_index[7] = {0,5,1,6,3,4,2};
static const uint8_t step_index_seq_ccw[6 * 6] = {1,2,3,4,5,6,2,3,4,5,6,1,3,4,5,6,1,2,4,5,6,1,2,3,5,6,1,2,3,4,6,1,2,3,4,5};
static const uint8_t step_index_seq_cw[6 * 6] = {6,5,4,3,2,1,5,4,3,2,1,6,4,3,2,1,6,5,3,2,1,6,5,4,2,1,6,5,4,3,1,6,5,4,3,2};

static const uint32_t filter_value_list[32] = 
{
    0x00000001,0x00000003,0x00000007,0x0000000f,
    0x0000001f,0x0000003f,0x0000007f,0x000000ff,
    0x000001ff,0x000003ff,0x000007ff,0x00000fff,
    0x00001fff,0x00003fff,0x00007fff,0x0000ffff,
    0x0001ffff,0x0003ffff,0x0007ffff,0x000fffff,
    0x001fffff,0x003fffff,0x007fffff,0x00ffffff,
    0x01ffffff,0x03ffffff,0x07ffffff,0x0fffffff,
    0x1fffffff,0x3fffffff,0x7fffffff,0xffffffff,
};

void MOS_Q15PWM(uint16_t ARR_value,uint16_t CCR_value)
{
    TIM1->ARR = ARR_value;
    TIM1->CCR1 = CCR_value;
    TIM1->CCR2 = 0;
    TIM1->CCR3 = 0;
    TIM1->CCER |= 0x055;
    TIM1->CCER &= ~0x500;
}
 
void MOS_Q16PWM(uint16_t ARR_value,uint16_t CCR_value)
{
    TIM1->ARR = ARR_value;
    TIM1->CCR1 = CCR_value;
    TIM1->CCR2 = 0;
    TIM1->CCR3 = 0;
    TIM1->CCER |= 0x505;
    TIM1->CCER &= ~0x050;
}

void MOS_Q24PWM(uint16_t ARR_value,uint16_t CCR_value)
{
    TIM1->ARR = ARR_value;
    TIM1->CCR1 = 0;
    TIM1->CCR2 = CCR_value;
    TIM1->CCR3 = 0;
    TIM1->CCER |= 0x055;
    TIM1->CCER &= ~0x500;
}

void MOS_Q26PWM(uint16_t ARR_value,uint16_t CCR_value)
{
    TIM1->ARR = ARR_value;
    TIM1->CCR1 = 0;
    TIM1->CCR2 = CCR_value;
    TIM1->CCR3 = 0;
    TIM1->CCER |= 0x550;
    TIM1->CCER &= ~0x005;
}

void MOS_Q34PWM(uint16_t ARR_value,uint16_t CCR_value)
{
    TIM1->ARR = ARR_value;
    TIM1->CCR1 = 0;
    TIM1->CCR2 = 0;
    TIM1->CCR3 = CCR_value;
    TIM1->CCER |= 0x505;
    TIM1->CCER &= ~0x050;
}
 
void MOS_Q35PWM(uint16_t ARR_value,uint16_t CCR_value)
{
    TIM1->ARR = ARR_value;
    TIM1->CCR1 = 0;
    TIM1->CCR2 = 0;
    TIM1->CCR3 = CCR_value;
    TIM1->CCER |= 0x550;
    TIM1->CCER &= ~0x005;
}

void Brake(void)
{
    uint16_t arr = bldc_run.TIM1_ARR_now;
    uint16_t ccr = bldc_run.duty;
    uint32_t temp_ccer = TIM1->CCER;

    TIM1->ARR = arr;
    TIM1->CCR1 = ccr;
    TIM1->CCR2 = ccr;
    TIM1->CCR3 = ccr;
    // TIM1->CCER &= ~0x555;
    // TIM1->CCER |= 0x444;

    temp_ccer &= ~0x555;
    temp_ccer |= 0x444;
    TIM1->CCER = temp_ccer;
}

void Full_Brake(void)
{
    uint16_t arr = PWM_TIM_BASE_FREQ / 2 / bldc_ctrl.PWM_freq_min;
    // uint16_t ccr = 0;
    uint16_t ccr = arr / 2;

    TIM1->ARR = arr;
    TIM1->CCR1 = ccr;
    TIM1->CCR2 = ccr;
    TIM1->CCR3 = ccr;
    TIM1->CCER |= 0x555;
}

void Plug_Brake(void)
{
    uint16_t arr = bldc_run.TIM1_ARR_now;
    uint16_t ccr = bldc_run.duty;
    uint32_t temp_ccer = TIM1->CCER;

    bldc_run.delay_angle_cnt_ref = bldc_run.TIM1_ARR_now * 0.01f;
    bldc_run.delay_Cnt++;
    if(bldc_run.delay_Cnt < bldc_run.delay_angle_cnt_ref)
    {
        TIM1->ARR = arr;
        TIM1->CCR1 = 0;
        TIM1->CCR2 = 0;
        TIM1->CCR3 = 0;
        TIM1->CCER |= 0x555;
    }
    else if(bldc_run.delay_Cnt < bldc_run.delay_angle_cnt_ref * 2)
    {
        TIM1->ARR = arr;
        TIM1->CCR1 = 0;
        TIM1->CCR2 = 0;
        TIM1->CCR3 = 0;
        TIM1->CCER &= ~0x555;
    }
    else if(bldc_run.delay_Cnt == bldc_run.delay_angle_cnt_ref * 2)
    {
        if(phase_voltage_f[0] < 200)
        {
            temp_ccer |= 0x005;
        }
        else
        {
            temp_ccer &= ~0x005;
        }

        if(phase_voltage_f[1] < 200)
        {
            temp_ccer |= 0x050;
        }
        else
        {
            temp_ccer &= ~0x050;
        }

        if(phase_voltage_f[2] < 200)
        {
            temp_ccer |= 0x500;
        }
        else
        {
            temp_ccer &= ~0x500;
        }
        
        TIM1->ARR = arr;
        TIM1->CCR1 = ccr;
        TIM1->CCR2 = ccr;
        TIM1->CCR3 = ccr;
        TIM1->CCER = temp_ccer;
    }
    else if(bldc_run.delay_Cnt == bldc_run.delay_angle_cnt_ref * 3)
    {
        bldc_run.delay_Cnt = 0;
    }
}

#if 0
static void Update_Filter_Value(uint8_t len)
{
    if(len < 1)
    {
        len = 1;
    }

    if(bldc_ctrl.sensor_type == SENSOR_LESS)
    {
        bldc_run.befm_filter_len = len;
        bldc_run.befm_filter_value = 0;
        for(uint8_t i = 0;i < bldc_run.befm_filter_len;i++)
        {
            bldc_run.befm_filter_value |= (1 << i);
        }
    }
    else
    {
        Hall_Sensor.hall_filter_len = len;
        Hall_Sensor.hall_filter_value = 0;
        for(uint8_t i = 0;i < Hall_Sensor.hall_filter_len;i++)
        {
            Hall_Sensor.hall_filter_value |= (1 << i);
        }
    }
}
#endif

static void Update_Filter_Value_Fast(uint8_t len)
{
    if(len < 1)
    {
        len = 1;
    }
    else if(len > 32)
    {
        len = 32;
    }

    bldc_run.befm_filter_len = len;
    bldc_run.befm_filter_value = filter_value_list[len - 1];
    Hall_Sensor.hall_filter_len = len;
    Hall_Sensor.hall_filter_value = filter_value_list[len - 1];
}

static void HallLess_MOS_Switch(uint8_t step)
{
    uint16_t CCR = bldc_run.duty;
    uint16_t ARR = bldc_run.TIM1_ARR_now;

    if(Motor_Config.dir == CCW)
    {
        switch(step)
        {
            case  0x2:
            {
                //B+A-
                MOS_Q24PWM(ARR,CCR);
            }
            break;
            
            case  0x6:
            {
                //C+A-
                MOS_Q34PWM(ARR,CCR);
            }
            break;
            
            case  0x4:
            {
                //C+B-
                MOS_Q35PWM(ARR,CCR);
            }
            break;

            case  0x5:
            {
                //A+B-
                MOS_Q15PWM(ARR,CCR);
            }
            break;
            
            case  0x1:
            {
                //A+C-
                MOS_Q16PWM(ARR,CCR);
            }
            break;
            
            case  0x3:
            {
                //B+C-
                MOS_Q26PWM(ARR,CCR);
            }
            break;
            
            default:
            {
                // Stop_Motor();
                // bldc_run.err = 1;
            }
            break;
        }
    }
    else
    {
        switch(step)
        {
            case  0x2:
            {
                //B+C-
                MOS_Q26PWM(ARR,CCR);
            }
            break;

            case  0x3:
            {
                //A+C-
                MOS_Q16PWM(ARR,CCR);
            }
            break;
            
            case  0x1:
            {
                //A+B-
                MOS_Q15PWM(ARR,CCR);
            }
            break;

            case  0x5:
            {
                //C+B-
                MOS_Q35PWM(ARR,CCR);
            }
            break;

            case  0x4:
            {
                //C+A-
                MOS_Q34PWM(ARR,CCR);
            }
            break;

            case  0x6:
            {
                //B+A-
                MOS_Q24PWM(ARR,CCR);
            }
            break;

            default:
            {
                // Stop_Motor();
                // bldc_run.err = 1;
            }
            break;
        }
    }
}

static void Hall_MOS_Switch(uint8_t step)
{
    uint16_t CCR = bldc_run.duty;
    uint16_t ARR = bldc_run.TIM1_ARR_now;

    if(Motor_Config.dir == CCW)
    {
        switch(step)
        {
            case  2:
            {
                //B+C-
                MOS_Q26PWM(ARR,CCR);
            }
            break;

            case  6:
            {
                //B+A-
                MOS_Q24PWM(ARR,CCR);
            }
            break;

            case  4:
            {
                //C+A-
                MOS_Q34PWM(ARR,CCR);
            }
            break;

            case  5:
            {
                //C+B-
                MOS_Q35PWM(ARR,CCR);
            }
            break;

            case  1:
            {
                //A+B-
                MOS_Q15PWM(ARR,CCR);
            }
            break;

            case  3:
            {
                //A+C-
                MOS_Q16PWM(ARR,CCR);
            }
            break;
            
            default:
            {
                // Stop_Motor();
                // bldc_run.err = 1;
            }
            break;
        }
    }
    else
    {
        switch(step)
        {
            case  2:
            {
                //C+B-
                MOS_Q35PWM(ARR,CCR);
            }
            break;

            case  3:
            {
                //C+A-
                MOS_Q34PWM(ARR,CCR);
            }
            break;

            case  1:
            {
                //B+A-
                MOS_Q24PWM(ARR,CCR);
            }
            break;

            case  5:
            {
                //B+C-
                MOS_Q26PWM(ARR,CCR);
            }
            break;

            case  4:
            {
                //A+C-
                MOS_Q16PWM(ARR,CCR);
            }
            break;

            case  6:
            {
                //A+B-
                MOS_Q15PWM(ARR,CCR);
            }
            break;
            
            default:
            {
                // Stop_Motor();
                // bldc_run.err = 1;
            }
            break;
        }
    }
}

static void VVVF_Process(void)
{
    if(bldc_run.run_state == BLDC_StartUp)
    {
        if(bldc_ctrl.VVVF_method > BLDC_VVVF_None)
        {
            bldc_run.PWM_freq_now = (bldc_ctrl.PWM_freq_max - bldc_ctrl.PWM_freq_min) * bldc_ctrl.start_force + bldc_ctrl.PWM_freq_min;
        }
        else
        {
            bldc_run.PWM_freq_now = bldc_ctrl.PWM_freq_min;
        }
    }
    else //if(bldc_run.run_state == BLDC_Run)
    {
        switch(bldc_ctrl.VVVF_method)
        {
            case BLDC_VVVF_None:
                bldc_run.PWM_freq_now = bldc_ctrl.PWM_freq_min;
                break;

            case BLDC_VVVF_VF:
                bldc_run.PWM_freq_now = (bldc_ctrl.PWM_freq_max - bldc_ctrl.PWM_freq_min) * bldc_ctrl.output + bldc_ctrl.PWM_freq_min;
                break;

            case BLDC_VVVF_FF:
                // bldc_run.PWM_freq_now = abs(bldc_run.eletrical_rpm) / 60 * 6 * bldc_ctrl.VVVF_ratio;
                bldc_run.PWM_freq_now = abs(bldc_run.eletrical_rpm) * bldc_ctrl.VVVF_ratio / 10;
                bldc_run.PWM_freq_now = _constrain(bldc_run.PWM_freq_now, bldc_ctrl.PWM_freq_min, bldc_ctrl.PWM_freq_max);
                break;

            case BLDC_VVVF_VFF:
                // bldc_run.PWM_freq_now = abs(bldc_run.eletrical_rpm) / 60 * 6 * bldc_ctrl.VVVF_ratio;
                bldc_run.PWM_freq_now = abs(bldc_run.eletrical_rpm) * bldc_ctrl.VVVF_ratio / 10;
                bldc_run.PWM_freq_now = _constrain(bldc_run.PWM_freq_now, ((bldc_ctrl.PWM_freq_max - bldc_ctrl.PWM_freq_min) * bldc_ctrl.output + bldc_ctrl.PWM_freq_min), bldc_ctrl.PWM_freq_max);
                break;

            default:
                bldc_run.PWM_freq_now = bldc_ctrl.PWM_freq_max;
                break;
        }

        bldc_run.duty = (bldc_run.TIM1_ARR_now - 1) * bldc_ctrl.output;
    }

    bldc_run.TIM1_ARR_now = PWM_TIM_BASE_FREQ / 2 / bldc_run.PWM_freq_now;
    Virtual_Moto.dt = 1.0f / bldc_run.PWM_freq_now;

    if(bldc_run.duty > (bldc_run.TIM1_ARR_now - 1))
    {
        bldc_run.duty = bldc_run.TIM1_ARR_now - 1;
    }
}

static void HallLess_Current_Process(uint8_t step)
{
    if(Motor_Config.dir == CCW)
    {
        switch(step)
        {
            case  0x2:
            {
                //B+A-
                bldc_run.I_bus_ma = phase_current_f[1];
            }
            break;
            
            case  0x6:
            {
                //C+A-
                bldc_run.I_bus_ma = phase_current_f[2];
            }
            break;
            
            case  0x4:
            {
                //C+B-
                bldc_run.I_bus_ma = phase_current_f[2];
            }
            break;
            
            case  0x5:
            {
                //A+B-
                bldc_run.I_bus_ma = phase_current_f[0];
            }
            break;

            case  0x1:
            {
                //A+C-
                bldc_run.I_bus_ma = phase_current_f[0];
            }
            break;

            case  0x3:
            {
                //B+C-
                bldc_run.I_bus_ma = phase_current_f[1];
            }
            break;

            default:
            break;
        }
    }
    else
    {
        switch(step)
        {
            case  0x2:
            {
                //B+C-
                bldc_run.I_bus_ma = phase_current_f[1];
            }
            break;

            case  0x3:
            {
                //A+C-
                bldc_run.I_bus_ma = phase_current_f[0];
            }
            break;

            case  0x1:
            {
                //A+B-
                bldc_run.I_bus_ma = phase_current_f[0];
            }
            break;

            case  0x5:
            {
                //C+B-
                bldc_run.I_bus_ma = phase_current_f[2];
            }
            break;

            case  0x4:
            {
                //C+A-
                bldc_run.I_bus_ma = phase_current_f[2];
            }
            break;

            case  0x6:
            {
                //B+A-
                bldc_run.I_bus_ma = phase_current_f[1];
            }
            break;
            
            default:
            break;
        }
    }

    FirstOrder_LPF_Cacl(bldc_run.I_bus_ma,bldc_run.I_bus_ma_f,Filter_Rate.bus_current_filter_rate);
}

static void Hall_Current_Process(uint8_t step)
{
    if(Motor_Config.dir == CCW)
    {
        switch(step)
        {
            case  2:     
            {
                //B+C-
                bldc_run.I_bus_ma = phase_current_f[1];
            }
            break;

            case  6:
            {
                //B+A-
                bldc_run.I_bus_ma = phase_current_f[1];
            }
            break;

            case  4:   
            {
                //C+A-
                bldc_run.I_bus_ma = phase_current_f[2];
            }
            break;

            case  5:  
            {
                //C+B-
                bldc_run.I_bus_ma = phase_current_f[2];
            }
            break;

            case  1:    
            {
                //A+B-
                bldc_run.I_bus_ma = phase_current_f[0];
            }
            break;

            case  3:    
            {
                //A+C-
                bldc_run.I_bus_ma = phase_current_f[0];
            }
            break;
            
            default:
            break;
        }
    }
    else
    {
        switch(step)
        {
            case  2:
            {
                //C+B-
                bldc_run.I_bus_ma = phase_current_f[2];
            }
            break;

            case  3:   
            {
                //C+A-
                bldc_run.I_bus_ma = phase_current_f[2];
            }
            break;

            case  1:  
            {
                //B+A-
                bldc_run.I_bus_ma = phase_current_f[1];
            }
            break;

            case  5:    
            {
                //B+C-
                bldc_run.I_bus_ma = phase_current_f[1];
            }
            break;

            case  4:    
            {
                //A+C-
                bldc_run.I_bus_ma = phase_current_f[0];
            }
            break;

            case  6:     
            {
                //A+B-
                bldc_run.I_bus_ma = phase_current_f[0];
            }
            break;
            
            default:
            break;
        }
    }

    FirstOrder_LPF_Cacl(bldc_run.I_bus_ma,bldc_run.I_bus_ma_f,Filter_Rate.bus_current_filter_rate);
}

#ifdef VIRTUAL_MID_HALF_PHASE
static void HallLess_Virtual_Mid_Cal(uint8_t step)
{
    if(Motor_Config.dir == CCW)
    {
        switch(step)
        {
            case  0x2:
            {
                //B+A-
                bldc_run.virtual_mid = phase_voltage_f[1] / 2;
            }
            break;
            
            case  0x6:
            {
                //C+A-
                bldc_run.virtual_mid = phase_voltage_f[2] / 2;
            }
            break;
            
            case  0x4:
            {
                //C+B-
                bldc_run.virtual_mid = phase_voltage_f[2] / 2;
            }
            break;

            case  0x5:
            {
                //A+B-
                bldc_run.virtual_mid = phase_voltage_f[0] / 2;
            }
            break;
            
            case  0x1:
            {
                //A+C-
                bldc_run.virtual_mid = phase_voltage_f[0] / 2;
            }
            break;
            
            case  0x3:
            {
                //B+C-
                bldc_run.virtual_mid = phase_voltage_f[1] / 2;
            }
            break;
            
            default:
            {
            }
            break;
        }
    }
    else
    {
        switch(step)
        {
            case  0x2:
            {
                //B+C-
                bldc_run.virtual_mid = phase_voltage_f[1] / 2;
            }
            break;

            case  0x3:
            {
                //A+C-
                bldc_run.virtual_mid = phase_voltage_f[0] / 2;
            }
            break;
            
            case  0x1:
            {
                //A+B-
                bldc_run.virtual_mid = phase_voltage_f[0] / 2;
            }
            break;

            case  0x5:
            {
                //C+B-
                bldc_run.virtual_mid = phase_voltage_f[2] / 2;
            }
            break;

            case  0x4:
            {
                //C+A-
                bldc_run.virtual_mid = phase_voltage_f[2] / 2;
            }
            break;

            case  0x6:
            {
                //B+A-
                bldc_run.virtual_mid = phase_voltage_f[1] / 2;
            }
            break;

            default:
            {
            }
            break;
        }
    }

    FirstOrder_LPF_Cacl(bldc_run.virtual_mid,bldc_run.virtual_mid_f,bldc_ctrl.virtual_mid_filter_rate);
}
#endif

void BLDC_Process(void)
{
    if(bldc_ctrl.sensor_type == SENSOR_LESS)
    {   
        /* 
            normally the BEFM is sine wave just like sin(x) sin(x + 2pi/3) sin(x - 2pi/3),Their average is 0.
            But the BEFM is in the 0 to Vbus range,so their average is 0.5 * Vbus.
        */
        #if VIRTUAL_MID_3PHASE_AVERAGE
        bldc_run.virtual_mid = (phase_voltage_f[0] + phase_voltage_f[1] + phase_voltage_f[2]) / 3;
        FirstOrder_LPF_Cacl(bldc_run.virtual_mid,bldc_run.virtual_mid_f,bldc_ctrl.virtual_mid_filter_rate);
        #endif
        
        #if VIRTUAL_MID_HALF_MAX_PHASE
        bldc_run.virtual_mid = Virtual_Moto.V_bus_mv_f / 2;
        #endif

        if(bldc_run.run_state > BLDC_StartUp)
        {
            if(bldc_run.eletrical_rpm_abs > 20000 && bldc_run.befm_filter_len != 1)
            {
                Update_Filter_Value_Fast(1);
            }
            else if(bldc_run.eletrical_rpm_abs > 10000 && bldc_run.befm_filter_len != 2)
            {
                Update_Filter_Value_Fast(2);
            }
            else if(bldc_run.eletrical_rpm_abs > 5000 && bldc_run.befm_filter_len != 4)
            {
                Update_Filter_Value_Fast(4);
            }
            else if(bldc_run.befm_filter_len != 8)
            {
                Update_Filter_Value_Fast(8);
            }
        }

        bldc_run.befm_queue[0] = bldc_run.befm_queue[0] << 1;
        bldc_run.befm_queue[1] = bldc_run.befm_queue[1] << 1;
        bldc_run.befm_queue[2] = bldc_run.befm_queue[2] << 1;

        // if((bldc_run.run_state == BLDC_Stop || bldc_run.run_state >= BLDC_Brake) && bldc_run.virtual_mid_f < Virtual_Moto.V_bus_mv_f * 0.1f)
        if(bldc_run.run_state == BLDC_Stop && bldc_run.virtual_mid_f < 100)
        {
            bldc_run.phase_cnt = 0;
            bldc_run.phase_cnt_last = 0;
            bldc_run.phase_cnt_f = 0;
            bldc_run.eletrical_rpm = 0;
            bldc_run.machine_rpm = 0;

            bldc_run.befm_queue[0] = 0; 
            bldc_run.befm_queue[1] = 0;
            bldc_run.befm_queue[2] = 0;
        }
        else
        {
            bldc_run.befm_queue[0] |= phase_voltage_f[0] > bldc_run.virtual_mid_f; 
            bldc_run.befm_queue[1] |= phase_voltage_f[1] > bldc_run.virtual_mid_f;
            bldc_run.befm_queue[2] |= phase_voltage_f[2] > bldc_run.virtual_mid_f;
        }

        bldc_run.befm_filter_res[0] = bldc_run.befm_queue[0] & bldc_run.befm_filter_value;
        if(bldc_run.befm_filter_res[0] == bldc_run.befm_filter_value)
        {
            bldc_run.befm_queue_f[0] = 1;
        }
        else if(bldc_run.befm_filter_res[0] == 0x00)
        {
            bldc_run.befm_queue_f[0] = 0;
        }

        bldc_run.befm_filter_res[1] = bldc_run.befm_queue[1] & bldc_run.befm_filter_value;
        if(bldc_run.befm_filter_res[1] == bldc_run.befm_filter_value)
        {
            bldc_run.befm_queue_f[1] = 1;
        }
        else if(bldc_run.befm_filter_res[1] == 0x00)
        {
            bldc_run.befm_queue_f[1] = 0;
        }

        bldc_run.befm_filter_res[2] = bldc_run.befm_queue[2] & bldc_run.befm_filter_value;
        if(bldc_run.befm_filter_res[2] == bldc_run.befm_filter_value)
        {
            bldc_run.befm_queue_f[2] = 1;
        }
        else if(bldc_run.befm_filter_res[2] == 0x00)
        {
            bldc_run.befm_queue_f[2] = 0;
        }

        bldc_run.befm_index = bldc_run.befm_queue_f[0] + (bldc_run.befm_queue_f[1] << 1) + (bldc_run.befm_queue_f[2] << 2);

        bldc_run.period_cnt++;
        if(bldc_run.befm_index_last != bldc_run.befm_index)
        {
            bldc_run.befm_index_last = bldc_run.befm_index;

            bldc_run.phase_cnt_last = bldc_run.phase_cnt;
            bldc_run.phase_cnt = 0;
            FirstOrder_LPF_Cacl(bldc_run.phase_cnt_last,bldc_run.phase_cnt_f,Filter_Rate.RPM_filter_rate);

            if(bldc_ctrl.delay_angle)
            {
                //bldc_run.delay_angle_cnt_ref = bldc_run.phase_cnt_f * 6 * bldc_ctrl.delay_angle / 360;
                bldc_run.delay_angle_cnt_ref = bldc_run.phase_cnt_f * bldc_ctrl.delay_angle / 60;
            }
            else
            {
                bldc_run.delay_angle_cnt_ref = 0;
            }

            // //bldc_run.eletrical_rpm = bldc_run.PWM_freq_now * 60 / (bldc_run.phase_cnt_f * 6) * bldc_run.dir;
            // bldc_run.eletrical_rpm = bldc_run.PWM_freq_now * 10 / bldc_run.phase_cnt_f * bldc_run.dir;
            // bldc_run.eletrical_rpm_abs = abs(bldc_run.eletrical_rpm);
            // bldc_run.machine_rpm = bldc_run.eletrical_rpm / bldc_ctrl.pole_pairs;
            // bldc_run.machine_rpm_abs = bldc_run.eletrical_rpm_abs / bldc_ctrl.pole_pairs;

            bldc_run.step_list[bldc_run.step_cnt] = step_index[bldc_run.befm_index];
            bldc_run.step_time_cnt[bldc_run.step_cnt] = bldc_run.phase_cnt_last;
            if(++bldc_run.step_cnt >= 6)
            {
                bldc_run.step_cnt = 0;

                // bldc_run.period_cnt += 6 * bldc_run.befm_filter_len;
                bldc_run.eletrical_rpm = bldc_run.PWM_freq_now * 60 / bldc_run.period_cnt * bldc_run.dir;
                bldc_run.eletrical_rpm_abs = abs(bldc_run.eletrical_rpm);
                bldc_run.machine_rpm = bldc_run.eletrical_rpm / bldc_ctrl.pole_pairs;
                bldc_run.machine_rpm_abs = bldc_run.eletrical_rpm_abs / bldc_ctrl.pole_pairs;

                bldc_run.period_cnt = 0;
            }
            
            bldc_run.step_time_sum = bldc_run.step_time_cnt[0] + bldc_run.step_time_cnt[1] + bldc_run.step_time_cnt[2] + bldc_run.step_time_cnt[3] + bldc_run.step_time_cnt[4] + bldc_run.step_time_cnt[5];
            bldc_run.step_time_avg = bldc_run.step_time_sum / 6;
            bldc_run.step_time_mse = SQ(bldc_run.step_time_cnt[0] - bldc_run.step_time_avg) + SQ(bldc_run.step_time_cnt[1] - bldc_run.step_time_avg) + SQ(bldc_run.step_time_cnt[2] - bldc_run.step_time_avg) + SQ(bldc_run.step_time_cnt[3] - bldc_run.step_time_avg) + SQ(bldc_run.step_time_cnt[4] - bldc_run.step_time_avg) + SQ(bldc_run.step_time_cnt[5] - bldc_run.step_time_avg);
            bldc_run.step_time_mse /= 6;
            bldc_run.step_time_mse = sqrt(bldc_run.step_time_mse);

            uint8_t i;
            for(i = 0;i < 6;i++)
            {
                if(memcmp(step_index_seq_ccw + i * 6,bldc_run.step_list,6) == 0)
                {
                    if(bldc_run.run_state == BLDC_StartUp && bldc_run.step_time_mse < bldc_ctrl.start_max_mse && bldc_run.start_step_cnt > bldc_ctrl.start_step_min)
                    {
                        bldc_run.start_seq = BLDC_Start_End;
                        bldc_run.delay_Cnt = 0;
                        bldc_run.delay_Cnt_Ref = 0;
                    }
                    if(bldc_run.step_time_mse < bldc_ctrl.run_max_mse)
                    {
                        bldc_run.dir = CCW;
                    }
                    break;
                }
                else if(memcmp(step_index_seq_cw + i * 6,bldc_run.step_list,6) == 0)
                {
                    if(bldc_run.run_state == BLDC_StartUp && bldc_run.step_time_mse < bldc_ctrl.start_max_mse && bldc_run.start_step_cnt > bldc_ctrl.start_step_min)
                    {
                        bldc_run.start_seq = BLDC_Start_End;
                    }
                    if(bldc_run.step_time_mse < bldc_ctrl.run_max_mse)
                    {
                        bldc_run.dir = CW;
                    }
                    break;
                }
            }

            // if((i == 6 || bldc_run.step_time_mse > bldc_ctrl.stall_protect_mse) && bldc_run.run_state == BLDC_Run)
            // {
            //     Stop_BLDC_Motor();
            // }
        }
        else
        {
            bldc_run.phase_cnt++;
        }

        if(bldc_run.step_index_last != bldc_run.befm_index)
        {
            bldc_run.delay_angle_cnt++;
        }

        // Stall Protect
        if (bldc_run.phase_cnt > bldc_run.PWM_freq_now * bldc_ctrl.stall_protect_ms / 1000)
        {
            bldc_run.phase_cnt = 0;
            bldc_run.phase_cnt_last = 0;
            bldc_run.phase_cnt_f = 0;
            bldc_run.eletrical_rpm = 0;
            bldc_run.machine_rpm = 0;
            if (bldc_run.run_state == BLDC_Run)
            {
                Stop_BLDC_Motor();
            }
        }
    }
    else
    {
        if(bldc_run.eletrical_rpm > 10000 && Hall_Sensor.hall_filter_len > 1)
        {
            Update_Filter_Value_Fast(1);
        }
        else if(Hall_Sensor.hall_filter_len > 2)
        {
            Update_Filter_Value_Fast(2);
        }

        Hall_Sensor.hall_queue[0] = Hall_Sensor.hall_queue[0] << 1;
        Hall_Sensor.hall_queue[1] = Hall_Sensor.hall_queue[1] << 1;
        Hall_Sensor.hall_queue[2] = Hall_Sensor.hall_queue[2] << 1;

        Hall_Sensor.hall_queue[0] |= HALL_A_READ(); 
        Hall_Sensor.hall_queue[1] |= HALL_B_READ();
        Hall_Sensor.hall_queue[2] |= HALL_C_READ();

        Hall_Sensor.hall_filter_res[0] = Hall_Sensor.hall_queue[0] & Hall_Sensor.hall_filter_value;
        if(Hall_Sensor.hall_filter_res[0] == Hall_Sensor.hall_filter_value)
        {
            Hall_Sensor.hall_queue_f[0] = 1;
        }
        else if(Hall_Sensor.hall_filter_res[0] == 0x00)
        {
            Hall_Sensor.hall_queue_f[0] = 0;
        }

        Hall_Sensor.hall_filter_res[1] = Hall_Sensor.hall_queue[1] & Hall_Sensor.hall_filter_value;
        if(Hall_Sensor.hall_filter_res[1] == Hall_Sensor.hall_filter_value)
        {
            Hall_Sensor.hall_queue_f[1] = 1;
        }
        else if(Hall_Sensor.hall_filter_res[1] == 0x00)
        {
            Hall_Sensor.hall_queue_f[1] = 0;
        }

        Hall_Sensor.hall_filter_res[2] = Hall_Sensor.hall_queue[2] & Hall_Sensor.hall_filter_value;
        if(Hall_Sensor.hall_filter_res[2] == Hall_Sensor.hall_filter_value)
        {
            Hall_Sensor.hall_queue_f[2] = 1;
        }
        else if(Hall_Sensor.hall_filter_res[2] == 0x00)
        {
            Hall_Sensor.hall_queue_f[2] = 0;
        }

        Hall_Sensor.hall_index = Hall_Sensor.hall_queue_f[0] + (Hall_Sensor.hall_queue_f[1] << 1) + (Hall_Sensor.hall_queue_f[2] << 2);

        /*Calculated every 60 degrees electrical angle*/
        if(Hall_Sensor.hall_index_last != Hall_Sensor.hall_index)
        {
            Hall_Sensor.hall_index_last = Hall_Sensor.hall_index;
            
            bldc_run.phase_cnt_last = bldc_run.phase_cnt;
            bldc_run.phase_cnt = 0;
            FirstOrder_LPF_Cacl(bldc_run.phase_cnt_last,bldc_run.phase_cnt_f,Filter_Rate.RPM_filter_rate);

            //bldc_run.eletrical_rpm = bldc_run.PWM_freq_now * 60 / (bldc_run.phase_cnt_f * 6) * bldc_run.dir;
            bldc_run.eletrical_rpm = bldc_run.PWM_freq_now * 10 / bldc_run.phase_cnt_f * bldc_run.dir;
            bldc_run.machine_rpm = bldc_run.eletrical_rpm / bldc_ctrl.pole_pairs;
        }
        else
        {
            bldc_run.phase_cnt++;
        }

    }

    Virtual_Moto.electronic_speed_hz = bldc_run.eletrical_rpm_abs / 60.0f;

    switch(bldc_run.run_state)
    {
        case BLDC_Stop:
        {
            TIM1->CCER &= ~0x555;
        }
        break;

        case BLDC_StartUp:
        {
            switch(bldc_run.start_seq)
            {
                case BLDC_Start_Order:
                    bldc_ctrl.output = 0;
                    VVVF_Process();
                    bldc_run.duty = bldc_run.TIM1_ARR_now * bldc_ctrl.start_force;
                    bldc_run.delay_Cnt = 0;
                    bldc_run.delay_Cnt_Ref = bldc_run.PWM_freq_now * bldc_ctrl.start_order_ms / 1000;
                    bldc_run.start_seq = BLDC_Start_Order_Delay;
                    bldc_run.start_index = 0;
                    bldc_run.start_step_cnt = 0;
                    break;

                case BLDC_Start_Order_Delay:
                    bldc_run.delay_Cnt++;
                    if(bldc_run.delay_Cnt > bldc_run.delay_Cnt_Ref)
                    {
                        bldc_run.delay_Cnt = 0;
                        bldc_run.start_seq = BLDC_Start_First_Step;
                    }
                    break;

                case BLDC_Start_First_Step:
                    bldc_run.delay_Cnt_Ref = bldc_run.PWM_freq_now * bldc_ctrl.start_first_step_ms / 1000;
                    bldc_run.start_seq = BLDC_Start_Step_Delay;
                    bldc_run.start_index = 1;
                    break;
                
                case BLDC_Start_Step_Delay:
                    bldc_run.delay_Cnt++;
                    if(bldc_run.delay_Cnt > bldc_run.delay_Cnt_Ref)
                    {
                        bldc_run.delay_Cnt = 0;
                        bldc_run.start_seq = BLDC_Start_Step;
                    }
                    break;

                case BLDC_Start_Step:
                    bldc_run.start_index++;
                    if(bldc_run.start_index > 5)
                    {
                        bldc_run.start_index = 0;
                    }
                    bldc_run.delay_Cnt = 0;
                    bldc_run.start_seq = BLDC_Start_Step_Delay;
                    /*
                    * period per step = ERPM / 60 * 6 
                    * bldc_run.delay_Cnt_Ref > bldc_run.PWM_freq_now / (ERPM / 60 * 6)
                    * ERPM = bldc_ctrl.KV * bldc_ctrl.pole_pairs * bldc_ctrl.start_force * (Virtual_Moto.V_bus_mv_f / 1000)
                    * bldc_run.delay_Cnt_Ref > bldc_run.PWM_freq_now * 10 / ERPM
                    * bldc_run.delay_Cnt_Ref > bldc_run.PWM_freq_now * 10 / (bldc_ctrl.KV * bldc_ctrl.pole_pairs * bldc_ctrl.start_force * (Virtual_Moto.V_bus_mv_f / 1000))
                    * bldc_run.delay_Cnt_Ref > bldc_run.PWM_freq_now * 10000 / (bldc_ctrl.KV * bldc_ctrl.pole_pairs * bldc_ctrl.start_force * Virtual_Moto.V_bus_mv_f)
                    */
                    if(bldc_run.delay_Cnt_Ref > bldc_run.PWM_freq_now * 10 / (bldc_ctrl.KV * bldc_ctrl.pole_pairs * bldc_ctrl.start_force * Virtual_Moto.V_bus_mv_f / 1000))
                    {
                        bldc_run.duty += bldc_run.TIM1_ARR_now * bldc_ctrl.start_force_rate;
                        bldc_run.delay_Cnt_Ref *= bldc_ctrl.start_time_rate;//exponential curve
                        // bldc_run.delay_Cnt_Ref -= bldc_run.PWM_freq_now * bldc_ctrl.start_first_step_ms / 1000 * (1 - bldc_ctrl.start_time_rate);

                        // max output is bldc_ctrl.start_forc * 2
                        // min output is 0.1
                        uint16_t up_lim = _constrain(bldc_run.TIM1_ARR_now * bldc_ctrl.start_force * 2,0,0xFFFF);
                        uint16_t down_lim = bldc_run.TIM1_ARR_now * 0.1f;
                        bldc_run.duty = _constrain(bldc_run.duty,down_lim,up_lim);
                    }
                    else
                    {
                        // bldc_run.start_seq = BLDC_Start_End;
                    }

                    if(bldc_run.start_step_cnt++ > bldc_ctrl.start_step_max)
                    {
                        Stop_BLDC_Motor();
                    }
                    break;
                
                case BLDC_Start_End:
                    bldc_run.run_state = BLDC_Run;
                    bldc_run.start_seq = BLDC_Start_Order;
                    bldc_ctrl.output = bldc_ctrl.output_min;
                    break;

                default://err
                    break;
            }

            if(Motor_Config.dir == CCW)
            {
                bldc_run.start_step_index = step_seq_ccw[bldc_run.start_index];
            }
            else
            {
                bldc_run.start_step_index = step_seq_cw[bldc_run.start_index];
            }
            VVVF_Process();
            HallLess_MOS_Switch(bldc_run.start_step_index);
            #if VIRTUAL_MID_HALF_PHASE
            HallLess_Virtual_Mid_Cal(bldc_run.start_step_index);
            #endif
        }
        break;

        case BLDC_Run:
        {
            if(bldc_ctrl.sensor_type == SENSOR_LESS)
            {
                if(bldc_run.delay_angle_cnt > bldc_run.delay_angle_cnt_ref)
                {
                    bldc_run.delay_angle_cnt = 0;
                    bldc_run.step_index_last = bldc_run.befm_index;
                    bldc_run.step_index = bldc_run.befm_index;
                    VVVF_Process();
                    HallLess_MOS_Switch(bldc_run.step_index);
                    #if VIRTUAL_MID_HALF_PHASE
                    HallLess_Virtual_Mid_Cal(bldc_run.step_index);
                    #endif
                }
                HallLess_Current_Process(bldc_run.step_index);
            }
            else
            {
                bldc_run.run_state = BLDC_Run;
                if(bldc_ctrl.sensor_type == HALL_120_SENSOR)
                {
                    // bldc_run.step_index = Hall_Sensor.hall_index;
                    bldc_run.step_index = hall_seq_calibration_120[Hall_Sensor.hall_index - 1];
                }
                else if(bldc_ctrl.sensor_type == HALL_60_SENSOR)
                {
                    // bldc_run.step_index = Hall_Sensor.hall_index;
                    bldc_run.step_index = hall_seq_calibration_60[Hall_Sensor.hall_index];
                }
                VVVF_Process();
                Hall_Current_Process(bldc_run.step_index);
                Hall_MOS_Switch(bldc_run.step_index);
            }
        }
        break;

        case BLDC_Brake:
        {
            VVVF_Process();
            Brake();
        }
        break;

        case BLDC_FullBrake:
        {
            Full_Brake();
        }
        break;

        case BLDC_PlugBrake:
        {
            VVVF_Process();
            Plug_Brake();
        }
        break;
    }

    debug_arr[0] = phase_voltage[0];
    debug_arr[1] = phase_voltage[1];
    debug_arr[2] = phase_voltage[2];
    debug_arr[3] = bldc_run.virtual_mid;
    debug_arr[4] = phase_voltage_f[0];
    debug_arr[5] = phase_voltage_f[1];
    debug_arr[6] = phase_voltage_f[2];
    debug_arr[7] = bldc_run.virtual_mid_f;
    debug_arr[8] = bldc_run.run_state;
    debug_arr[9] = bldc_run.start_seq;
    debug_arr[10] = bldc_run.start_step_index;
    debug_arr[11] = bldc_run.befm_index;
    debug_arr[12] = bldc_run.step_index;
    debug_arr[13] = bldc_run.phase_cnt_last;
    debug_arr[14] = bldc_run.phase_cnt_f;
    debug_arr[15] = bldc_run.step_time_mse;
    debug_arr[16] = bldc_run.period_cnt;
    debug_arr[17] = bldc_run.I_bus_ma;
    debug_arr[18] = bldc_run.I_bus_ma_f;
    debug_arr[19] = bldc_run.PWM_freq_now;

    CDC_Transmit_FS((uint8_t *)debug_arr,sizeof(debug_arr));
}

void Start_BLDC_Motor(void)
{
    if(bldc_ctrl.sensor_type == SENSOR_LESS)
    {
        bldc_run.start_seq = BLDC_Start_Order;
        if(Motor_Config.dir == CCW)
        {   
            #if ANTI_WIND_STARTUP_ENABLE
            if(bldc_run.eletrical_rpm > 0)
            {
                bldc_run.dir = Motor_Config.dir;
                bldc_run.run_state = BLDC_Run;
                // bldc_ctrl.output = bldc_ctrl.output_min;
                bldc_ctrl.output = (float)(abs(bldc_run.eletrical_rpm)) / bldc_ctrl.KV / bldc_ctrl.pole_pairs / Virtual_Moto.V_bus_mv_f / 1000;
            }
            else if(bldc_run.eletrical_rpm < 0)
            {   
                while(bldc_run.eletrical_rpm != 0)
                {
                    Full_Brake();
                    osDelay(10);
                }
                bldc_run.dir = Motor_Config.dir;
                bldc_run.run_state = BLDC_StartUp;
                bldc_ctrl.output = 0;
            }
            else
            #endif
            {
                bldc_run.dir = Motor_Config.dir;
                bldc_run.start_seq = BLDC_Start_Order;
                bldc_run.run_state = BLDC_StartUp;
                bldc_ctrl.output = 0;
                bldc_run.phase_cnt = 0;
                bldc_run.phase_cnt_last = 0;
                bldc_run.phase_cnt_f = 0;
            }
        }
        else if(Motor_Config.dir == CW)
        {
            #if ANTI_WIND_STARTUP_ENABLE
            if(bldc_run.eletrical_rpm > 0)
            {
                while(bldc_run.eletrical_rpm != 0)
                {
                    Full_Brake();
                    osDelay(10);
                }
                bldc_run.dir = Motor_Config.dir;
                bldc_run.run_state = BLDC_StartUp;
                bldc_ctrl.output = 0;
            }
            else if(bldc_run.eletrical_rpm < 0)
            {
                bldc_run.dir = Motor_Config.dir;
                bldc_run.run_state = BLDC_Run;
                // bldc_ctrl.output = bldc_ctrl.output_min;
                bldc_ctrl.output = (float)(abs(bldc_run.eletrical_rpm)) / bldc_ctrl.KV / bldc_ctrl.pole_pairs / Virtual_Moto.V_bus_mv_f / 1000;
            }
            else
            #endif
            {
                bldc_run.dir = Motor_Config.dir;
                bldc_run.start_seq = BLDC_Start_Order;
                bldc_run.run_state = BLDC_StartUp;
                bldc_ctrl.output = 0;
                bldc_run.phase_cnt = 0;
                bldc_run.phase_cnt_last = 0;
                bldc_run.phase_cnt_f = 0;
            }
        }
    }
    else
    {
        bldc_run.start_seq = BLDC_Start_Order;
        bldc_run.run_state = BLDC_Run;
    }
}

void Stop_BLDC_Motor(void)
{
    bldc_run.run_state = BLDC_Stop;
    bldc_run.start_seq = BLDC_Start_Order;
    bldc_ctrl.output = 0;
}

void Init_BLDC_Motor(void)
{
    Update_Filter_Value_Fast(4);
    bldc_run.PWM_freq_now = bldc_ctrl.PWM_freq_min;
    bldc_run.TIM1_ARR_now = PWM_TIM_BASE_FREQ / 2 / bldc_run.PWM_freq_now;
    Virtual_Moto.dt = 1.0f / bldc_run.PWM_freq_now;
    TIM1->CCER &= ~0x555;
    // disable preload
    TIM1->CR1 &= ~0x80;
    TIM1->CCMR1 &= ~0x808;
    TIM1->CCMR2 &= ~0x08;
    TIM1->ARR = bldc_run.TIM1_ARR_now;
    #ifdef ADC_SAMPLE_HIGH_SIDE
    TIM1->CCR4 = 1;
    #else
    TIM1->CCR4 = TIM1->ARR - 1;
    #endif
}

