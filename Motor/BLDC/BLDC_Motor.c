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
    .pTIM = TIM1,
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
    .start_max_mse = 40,
    /* sensorless run config */ 
    .rapid_demagnetization_enable = 0,
    .befm_detect_delay_angle = 5,
    .befm_filter_angle = 5,
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
    .PWM_freq_min = 7 * 1000,
    .PWM_freq_max = 84 * 1000,
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

// A+C- Vector 30бу
void MOS_Q16PWM(BLDC_CONTROL_t *ctrl,BLDC_RUN_t *run)
{
    ctrl->pTIM->ARR = run->TIM_ARR_now;
    ctrl->pTIM->CCR1 = run->duty;
    ctrl->pTIM->CCR2 = 0;
    ctrl->pTIM->CCR3 = 0;
    ctrl->pTIM->CCER |= 0x505;
    ctrl->pTIM->CCER &= ~0x050;
}

// B+C- Vector 90бу
void MOS_Q26PWM(BLDC_CONTROL_t *ctrl,BLDC_RUN_t *run)
{
    ctrl->pTIM->ARR = run->TIM_ARR_now;
    ctrl->pTIM->CCR1 = 0;
    ctrl->pTIM->CCR2 = run->duty;
    ctrl->pTIM->CCR3 = 0;
    ctrl->pTIM->CCER |= 0x550;
    ctrl->pTIM->CCER &= ~0x005;
}

// B+A- Vector 150бу
void MOS_Q24PWM(BLDC_CONTROL_t *ctrl,BLDC_RUN_t *run)
{
    ctrl->pTIM->ARR = run->TIM_ARR_now;
    ctrl->pTIM->CCR1 = 0;
    ctrl->pTIM->CCR2 = run->duty;
    ctrl->pTIM->CCR3 = 0;
    ctrl->pTIM->CCER |= 0x055;
    ctrl->pTIM->CCER &= ~0x500;
}

// C+A- Vector 210бу
void MOS_Q34PWM(BLDC_CONTROL_t *ctrl,BLDC_RUN_t *run)
{
    ctrl->pTIM->ARR = run->TIM_ARR_now;
    ctrl->pTIM->CCR1 = 0;
    ctrl->pTIM->CCR2 = 0;
    ctrl->pTIM->CCR3 = run->duty;
    ctrl->pTIM->CCER |= 0x505;
    ctrl->pTIM->CCER &= ~0x050;
}

// C+B- Vector 270бу
void MOS_Q35PWM(BLDC_CONTROL_t *ctrl,BLDC_RUN_t *run)
{
    ctrl->pTIM->ARR = run->TIM_ARR_now;
    ctrl->pTIM->CCR1 = 0;
    ctrl->pTIM->CCR2 = 0;
    ctrl->pTIM->CCR3 = run->duty;
    ctrl->pTIM->CCER |= 0x550;
    ctrl->pTIM->CCER &= ~0x005;
}

// A+B- Vector 330бу
void MOS_Q15PWM(BLDC_CONTROL_t *ctrl,BLDC_RUN_t *run)
{
    ctrl->pTIM->ARR = run->TIM_ARR_now;
    ctrl->pTIM->CCR1 = run->duty;
    ctrl->pTIM->CCR2 = 0;
    ctrl->pTIM->CCR3 = 0;
    ctrl->pTIM->CCER |= 0x055;
    ctrl->pTIM->CCER &= ~0x500;
}

// Rapid demagnetization
// A+C- B+
void MOS_Q16PWM_Q2_ON(BLDC_CONTROL_t *ctrl,BLDC_RUN_t *run)
{
    ctrl->pTIM->ARR = run->TIM_ARR_now;
    ctrl->pTIM->CCR1 = run->duty;
    ctrl->pTIM->CCR2 = ctrl->pTIM->ARR;
    ctrl->pTIM->CCR3 = 0;
    ctrl->pTIM->CCER |= 0x555;
}

// B+C- A-
void MOS_Q26PWM_Q4_OFF(BLDC_CONTROL_t *ctrl,BLDC_RUN_t *run)
{
    ctrl->pTIM->ARR = run->TIM_ARR_now;
    ctrl->pTIM->CCR1 = 0;
    ctrl->pTIM->CCR2 = run->duty;
    ctrl->pTIM->CCR3 = 0;
    ctrl->pTIM->CCER |= 0x555;
}

// B+A- C+
void MOS_Q24PWM_Q3_ON(BLDC_CONTROL_t *ctrl,BLDC_RUN_t *run)
{
    ctrl->pTIM->ARR = run->TIM_ARR_now;
    ctrl->pTIM->CCR1 = 0;
    ctrl->pTIM->CCR2 = run->duty;
    ctrl->pTIM->CCR3 = ctrl->pTIM->ARR;
    ctrl->pTIM->CCER |= 0x555;
}

// C+A- B-
void MOS_Q34PWM_Q5_OFF(BLDC_CONTROL_t *ctrl,BLDC_RUN_t *run)
{
    ctrl->pTIM->ARR = run->TIM_ARR_now;
    ctrl->pTIM->CCR1 = 0;
    ctrl->pTIM->CCR2 = 0;
    ctrl->pTIM->CCR3 = run->duty;
    ctrl->pTIM->CCER |= 0x555;
}

// C+B- A+
void MOS_Q35PWM_Q1_ON(BLDC_CONTROL_t *ctrl,BLDC_RUN_t *run)
{
    ctrl->pTIM->ARR = run->TIM_ARR_now;
    ctrl->pTIM->CCR1 = ctrl->pTIM->ARR;
    ctrl->pTIM->CCR2 = 0;
    ctrl->pTIM->CCR3 = run->duty;
    ctrl->pTIM->CCER |= 0x555;
}

// A+B- C-
void MOS_Q15PWM_Q6_OFF(BLDC_CONTROL_t *ctrl,BLDC_RUN_t *run)
{
    ctrl->pTIM->ARR = run->TIM_ARR_now;
    ctrl->pTIM->CCR1 = run->duty;
    ctrl->pTIM->CCR2 = 0;
    ctrl->pTIM->CCR3 = 0;
    ctrl->pTIM->CCER |= 0x555;
}

void Brake(BLDC_CONTROL_t *ctrl,BLDC_RUN_t *run)
{
    uint32_t temp_ccer = ctrl->pTIM->CCER;

    ctrl->pTIM->ARR = run->TIM_ARR_now;
    ctrl->pTIM->CCR1 = run->duty;
    ctrl->pTIM->CCR2 = run->duty;
    ctrl->pTIM->CCR3 = run->duty;
    // ctrl->pTIM->CCER &= ~0x555;
    // ctrl->pTIM->CCER |= 0x444;

    temp_ccer &= ~0x555;
    temp_ccer |= 0x444;
    ctrl->pTIM->CCER = temp_ccer;
}

void Full_Brake(BLDC_CONTROL_t *ctrl,BLDC_RUN_t *run)
{
    uint16_t arr = PWM_TIM_BASE_FREQ / 2 / ctrl->PWM_freq_min;
    // uint16_t ccr = 0;
    uint16_t ccr = arr / 2;

    ctrl->pTIM->ARR = arr;
    ctrl->pTIM->CCR1 = ccr;
    ctrl->pTIM->CCR2 = ccr;
    ctrl->pTIM->CCR3 = ccr;
    ctrl->pTIM->CCER |= 0x555;
}

void Plug_Brake(BLDC_CONTROL_t *ctrl,BLDC_RUN_t *run)
{
    uint16_t arr = run->TIM_ARR_now;
    uint16_t ccr = run->duty;
    uint32_t temp_ccer = ctrl->pTIM->CCER;

    run->delay_angle_cnt_ref = run->TIM_ARR_now * 0.01f;
    run->delay_Cnt++;
    if(run->delay_Cnt < run->delay_angle_cnt_ref)
    {
        ctrl->pTIM->ARR = arr;
        ctrl->pTIM->CCR1 = 0;
        ctrl->pTIM->CCR2 = 0;
        ctrl->pTIM->CCR3 = 0;
        ctrl->pTIM->CCER |= 0x555;
    }
    else if(run->delay_Cnt < run->delay_angle_cnt_ref * 2)
    {
        ctrl->pTIM->ARR = arr;
        ctrl->pTIM->CCR1 = 0;
        ctrl->pTIM->CCR2 = 0;
        ctrl->pTIM->CCR3 = 0;
        ctrl->pTIM->CCER &= ~0x555;
    }
    else if(run->delay_Cnt == run->delay_angle_cnt_ref * 2)
    {
        if(phase_voltage_V_f.U < 200)
        {
            temp_ccer |= 0x005;
        }
        else
        {
            temp_ccer &= ~0x005;
        }

        if(phase_voltage_V_f.V < 200)
        {
            temp_ccer |= 0x050;
        }
        else
        {
            temp_ccer &= ~0x050;
        }

        if(phase_voltage_V_f.W < 200)
        {
            temp_ccer |= 0x500;
        }
        else
        {
            temp_ccer &= ~0x500;
        }
        
        ctrl->pTIM->ARR = arr;
        ctrl->pTIM->CCR1 = ccr;
        ctrl->pTIM->CCR2 = ccr;
        ctrl->pTIM->CCR3 = ccr;
        ctrl->pTIM->CCER = temp_ccer;
    }
    else if(run->delay_Cnt == run->delay_angle_cnt_ref * 3)
    {
        run->delay_Cnt = 0;
    }
}

#if 0
static void Update_Filter_Value(BLDC_CONTROL_t *ctrl,BLDC_RUN_t *run,uint8_t len)
{
    len = _constrain(len,0,31);
    if(ctrl->sensor_type == SENSOR_LESS)
    {
        run->befm_filter_len = len;
        run->befm_filter_value = 0;
        for(uint8_t i = 0;i < run->befm_filter_len;i++)
        {
            run->befm_filter_value |= (1 << i);
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

static void Update_Filter_Value_Fast(BLDC_CONTROL_t *ctrl,BLDC_RUN_t *run,uint8_t len)
{
    len = _constrain(len,0,31);
    run->befm_filter_len = len;
    run->befm_filter_value = filter_value_list[len];
    Hall_Sensor.hall_filter_len = len;
    Hall_Sensor.hall_filter_value = filter_value_list[len];
}

static void HallLess_MOS_Switch(BLDC_CONTROL_t *ctrl,BLDC_RUN_t *run,uint8_t step,uint8_t dir)
{
    if(dir == CCW)
    {
        switch(step)
        {
            case 0x2:
            {
                MOS_Q24PWM(ctrl,run);
            }
            break;
            
            case 0x6:
            {
                MOS_Q34PWM(ctrl,run);
            }
            break;
            
            case 0x4:
            {
                MOS_Q35PWM(ctrl,run);
            }
            break;

            case 0x5:
            {
                MOS_Q15PWM(ctrl,run);
            }
            break;
            
            case 0x1:
            {
                MOS_Q16PWM(ctrl,run);
            }
            break;
            
            case 0x3:
            {
                MOS_Q26PWM(ctrl,run);
            }
            break;
            
            default:
            {
                // Stop_Motor();
                // run->err = 1;
            }
            break;
        }
    }
    else
    {
        switch(step)
        {
            case 0x2:
            {
                MOS_Q26PWM(ctrl,run);
            }
            break;

            case 0x3:
            {
                MOS_Q16PWM(ctrl,run);
            }
            break;
            
            case 0x1:
            {
                MOS_Q15PWM(ctrl,run);
            }
            break;

            case 0x5:
            {
                MOS_Q35PWM(ctrl,run);
            }
            break;

            case 0x4:
            {
                MOS_Q34PWM(ctrl,run);
            }
            break;

            case 0x6:
            {
                MOS_Q24PWM(ctrl,run);
            }
            break;

            default:
            {
                // Stop_Motor();
                // run->err = 1;
            }
            break;
        }
    }
}

static void HallLess_MOS_Switch_Demagnetization(BLDC_CONTROL_t *ctrl,BLDC_RUN_t *run,uint8_t step,uint8_t dir)
{
    if(dir == CCW)
    {
        switch(step)
        {
            case 0x2:
            {
                MOS_Q24PWM_Q3_ON(ctrl,run);
            }
            break;
            
            case 0x6:
            {
                MOS_Q34PWM_Q5_OFF(ctrl,run);
            }
            break;
            
            case 0x4:
            {
                MOS_Q35PWM_Q1_ON(ctrl,run);
            }
            break;

            case 0x5:
            {
                MOS_Q15PWM_Q6_OFF(ctrl,run);
            }
            break;
            
            case 0x1:
            {
                MOS_Q16PWM_Q2_ON(ctrl,run);
            }
            break;
            
            case 0x3:
            {
                MOS_Q26PWM_Q4_OFF(ctrl,run);
            }
            break;
            
            default:
            {
                // Stop_Motor();
                // run->err = 1;
            }
            break;
        }
    }
    else
    {
        switch(step)
        {
            case 0x2:
            {
                MOS_Q26PWM_Q4_OFF(ctrl,run);
            }
            break;

            case 0x3:
            {
                MOS_Q16PWM_Q2_ON(ctrl,run);
            }
            break;
            
            case 0x1:
            {
                MOS_Q15PWM_Q6_OFF(ctrl,run);
            }
            break;

            case 0x5:
            {
                MOS_Q35PWM_Q1_ON(ctrl,run);
            }
            break;

            case 0x4:
            {
                MOS_Q34PWM_Q5_OFF(ctrl,run);
            }
            break;

            case 0x6:
            {
                MOS_Q24PWM_Q3_ON(ctrl,run);
            }
            break;

            default:
            {
                // Stop_Motor();
                // run->err = 1;
            }
            break;
        }
    }
}

static void Hall_MOS_Switch(BLDC_CONTROL_t *ctrl,BLDC_RUN_t *run,uint8_t step,uint8_t dir)
{
    if(dir == CCW)
    {
        switch(step)
        {
            // 30бу + 90бу + 30бу = 150бу
            case 0:
            {
                MOS_Q24PWM(ctrl,run);
            }
            break;
            
            // 90бу + 90бу + 30бу = 210бу
            case 1:
            {
                MOS_Q34PWM(ctrl,run);
            }
            break;
            
            // 150бу + 90бу + 30бу = 270бу
            case 2:
            {
                MOS_Q35PWM(ctrl,run);
            }
            break;
            
            // 210бу + 90бу + 30бу = 330бу
            case 3:
            {
                MOS_Q15PWM(ctrl,run);
            }
            break;
            
            // 270бу + 90бу + 30бу = 30бу
            case 4:
            {
                MOS_Q16PWM(ctrl,run);
            }
            break;
            
            // 330бу + 90бу + 30бу = 900бу
            case 5:
            {
                MOS_Q26PWM(ctrl,run);
            }
            break;
            
            default:
            {
                // Stop_Motor();
                // run->err = 1;
            }
            break;
        }
    }
    else
    {
        switch(step)
        {
            // 30бу - 90бу - 30бу = 270бу
            case 0:
            {
                MOS_Q35PWM(ctrl,run);
            }
            break;

            // 330бу - 90бу - 30бу = 210бу
            case 1:
            {
                MOS_Q15PWM(ctrl,run);
            }
            break;

            // 270бу - 90бу - 30бу = 150бу
            case 2:
            {
                MOS_Q16PWM(ctrl,run);
            }
            break;

            // 210бу - 90бу - 30бу = 90бу
            case 3:
            {
                MOS_Q26PWM(ctrl,run);
            }
            break;

            // 150бу - 90бу - 30бу = 30бу
            case 4:
            {
                MOS_Q24PWM(ctrl,run);
            }
            break;

            // 90бу - 90бу - 30бу = 330бу
            case 5:
            {
                MOS_Q34PWM(ctrl,run);
            }
            break;
            
            default:
            {
                // Stop_Motor();
                // run->err = 1;
            }
            break;
        }
    }
}

static void VVVF_Process(BLDC_CONTROL_t *ctrl,BLDC_RUN_t *run)
{
    switch(ctrl->VVVF_method)
    {
        case BLDC_VVVF_None:
            run->PWM_freq_now = ctrl->PWM_freq_max;
            break;

        case BLDC_VVVF_VF:
            run->PWM_freq_now = (ctrl->PWM_freq_max - ctrl->PWM_freq_min) * ctrl->output + ctrl->PWM_freq_min;
            break;

        case BLDC_VVVF_FF:
            // run->PWM_freq_now = abs(run->eletrical_rpm) / 60 * 6 * ctrl->VVVF_ratio;
            run->PWM_freq_now = abs(run->eletrical_rpm) * ctrl->VVVF_ratio / 10;
            run->PWM_freq_now = _constrain(run->PWM_freq_now, ctrl->PWM_freq_min, ctrl->PWM_freq_max);
            break;

        case BLDC_VVVF_VFF:
            // run->PWM_freq_now = abs(run->eletrical_rpm) / 60 * 6 * ctrl->VVVF_ratio;
            run->PWM_freq_now = abs(run->eletrical_rpm) * ctrl->VVVF_ratio / 10;
            run->PWM_freq_now = _constrain(run->PWM_freq_now, ((ctrl->PWM_freq_max - ctrl->PWM_freq_min) * ctrl->output + ctrl->PWM_freq_min), ctrl->PWM_freq_max);
            break;

        case BLDC_VVVF_AUTO:
            if(bldc_run.phase_cnt_last < 5)
            {
                run->PWM_freq_now += 1000;
            }
            else if(bldc_run.phase_cnt_last > 15)
            {
                run->PWM_freq_now -= 1000;
            }
            run->PWM_freq_now = _constrain(run->PWM_freq_now, ctrl->PWM_freq_min, ctrl->PWM_freq_max);
            break;
            
        default:
            run->PWM_freq_now = ctrl->PWM_freq_max;
            break;
    }

    run->duty = (run->TIM_ARR_now - 1) * ctrl->output;
    run->TIM_ARR_now = PWM_TIM_BASE_FREQ / 2 / run->PWM_freq_now;

    Virtual_Moto.dt = 1.0f / run->PWM_freq_now;
}

static void BLDC_Current_Process(BLDC_CONTROL_t *ctrl,BLDC_RUN_t *run)
{
    switch(ctrl->pTIM->CCER & 0x555)
    {
        case 0x055:
            run->Is = (fabsf(phase_current_A_f.U) + fabsf(phase_current_A_f.V)) / 2;
            break;

        case 0x505:
            run->Is = (fabsf(phase_current_A_f.U) + fabsf(phase_current_A_f.W)) / 2;
            break;
        
        case 0x550:
            run->Is = (fabsf(phase_current_A_f.V) + fabsf(phase_current_A_f.W)) / 2;
            break;
        
        default:
            run->Is = 0;
            break;
    }

    FirstOrder_LPF_Cacl(run->Is,run->Is_f,Filter_Rate.bus_current_filter_rate);
    run->V_bus = Virtual_Moto.V_bus_v_f;
    run->Us = run->V_bus * ctrl->output;
    run->I_bus = run->Us * run->Is_f / run->V_bus;
}

void BLDC_Process(void)
{
    if(bldc_ctrl.sensor_type == SENSOR_LESS)
    {   
        /* 
            normally the BEFM is sine wave just like sin(x) sin(x + 2pi/3) sin(x - 2pi/3),Their average is 0.
            But the BEFM is in the 0 to Vbus range,so their average is 0.5 * Vbus.
        */
        #if VIRTUAL_MID_CAL_BY_3PHASE_AVERAGE
        bldc_run.virtual_mid = (phase_voltage_V_f.U + phase_voltage_V_f.V + phase_voltage_V_f.W) / 3;
        #endif
        
        #if VIRTUAL_MID_CAL_BY_BUS_DUTY
        bldc_run.virtual_mid = Virtual_Moto.V_bus_v_f * bldc_ctrl.output / 2;
        #endif

        #if VIRTUAL_MID_CAL_BY_HALF_PHASE
        switch(bldc_ctrl.pTIM->CCER & 0x555)
        {
            case 0x055:
                bldc_run.virtual_mid = (phase_voltage_V_f.U + phase_voltage_V_f.V) / 2;
                break;

            case 0x505:
                bldc_run.virtual_mid = (phase_voltage_V_f.U + phase_voltage_V_f.W) / 2;
                break;
            
            case 0x550:
                bldc_run.virtual_mid = (phase_voltage_V_f.V + phase_voltage_V_f.W) / 2;
                break;
            
            default:
                bldc_run.virtual_mid = (phase_voltage_V_f.U + phase_voltage_V_f.V + phase_voltage_V_f.W) / 3;
                break;
        }
        #endif

        FirstOrder_LPF_Cacl(bldc_run.virtual_mid,bldc_run.virtual_mid_f,bldc_ctrl.virtual_mid_filter_rate);

        if(bldc_run.run_state == BLDC_Stop && bldc_run.virtual_mid_f < 100)
        {
            bldc_run.phase_cnt = 0;
            bldc_run.phase_cnt_last = 0;
            bldc_run.phase_cnt_f = 0;
            bldc_run.period_cnt = 0;
            bldc_run.period_cnt_last = 0;
            bldc_run.period_cnt_f = 0;
            bldc_run.eletrical_rpm = 0;
            bldc_run.eletrical_rpm_abs = 0;
            bldc_run.machine_rpm = 0;
            bldc_run.machine_rpm_abs = 0;

            bldc_run.befm_queue[0] = 0; 
            bldc_run.befm_queue[1] = 0;
            bldc_run.befm_queue[2] = 0;

            bldc_run.befm_queue_f[0] = 0; 
            bldc_run.befm_queue_f[1] = 0;
            bldc_run.befm_queue_f[2] = 0;
        }
        // else if(++bldc_run.befm_detect_delay_cnt > bldc_run.befm_detect_delay_cnt_ref || bldc_run.run_state == BLDC_StartUp)
        else if((bldc_run.run_state == BLDC_Run && ++bldc_run.befm_detect_delay_cnt > bldc_run.befm_detect_delay_cnt_ref) || bldc_run.run_state != BLDC_Run)
        {
            bldc_run.befm_queue[0] = bldc_run.befm_queue[0] << 1;
            bldc_run.befm_queue[1] = bldc_run.befm_queue[1] << 1;
            bldc_run.befm_queue[2] = bldc_run.befm_queue[2] << 1;
            
            bldc_run.befm_queue[0] |= phase_voltage_V_f.U > bldc_run.virtual_mid_f; 
            bldc_run.befm_queue[1] |= phase_voltage_V_f.V > bldc_run.virtual_mid_f;
            bldc_run.befm_queue[2] |= phase_voltage_V_f.W > bldc_run.virtual_mid_f;

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

            // if(bldc_run.phase_cnt_f)
            // {
            //     //bldc_run.eletrical_rpm = bldc_run.PWM_freq_now * 60 / (bldc_run.phase_cnt_f * 6) * bldc_run.dir;
            //     bldc_run.eletrical_rpm = bldc_run.PWM_freq_now * 10 / bldc_run.phase_cnt_f * bldc_run.dir;
            //     bldc_run.eletrical_rpm_abs = abs(bldc_run.eletrical_rpm);
            //     bldc_run.machine_rpm = bldc_run.eletrical_rpm / bldc_ctrl.pole_pairs;
            //     bldc_run.machine_rpm_abs = bldc_run.eletrical_rpm_abs / bldc_ctrl.pole_pairs;
            // }
            // else
            // {
            //     bldc_run.eletrical_rpm = 0;
            //     bldc_run.eletrical_rpm_abs = 0;
            //     bldc_run.machine_rpm = 0;
            //     bldc_run.machine_rpm_abs = 0;
            // }
            
            bldc_run.step_list[bldc_run.step_cnt] = step_index[bldc_run.befm_index];
            bldc_run.step_time_cnt[bldc_run.step_cnt] = bldc_run.phase_cnt_last;
            if(++bldc_run.step_cnt >= 6)
            {
                bldc_run.step_cnt = 0;

                // bldc_run.period_cnt += 6 * bldc_run.befm_filter_len;
                bldc_run.period_cnt_last = bldc_run.period_cnt;
                bldc_run.period_cnt = 0;
                FirstOrder_LPF_Cacl(bldc_run.period_cnt_last,bldc_run.period_cnt_f,Filter_Rate.RPM_filter_rate);
                
                // if(bldc_ctrl.delay_angle)
                // {
                //     bldc_run.delay_angle_cnt_ref = bldc_run.period_cnt_f * bldc_ctrl.delay_angle / 360;
                // }
                // else
                // {
                //     bldc_run.delay_angle_cnt_ref = 0;
                // }

                bldc_run.befm_filter_len = bldc_run.period_cnt_f * bldc_ctrl.befm_filter_angle / 360;
                bldc_run.befm_filter_len = _constrain(bldc_run.befm_filter_len,0,31);
                bldc_run.befm_filter_value = filter_value_list[bldc_run.befm_filter_len];

                bldc_run.befm_detect_delay_cnt_ref = bldc_run.period_cnt_f * bldc_ctrl.befm_detect_delay_angle / 360;

                if(bldc_run.period_cnt_f)
                {
                    bldc_run.eletrical_rpm = bldc_run.PWM_freq_now * 60 / bldc_run.period_cnt_f * bldc_run.dir;
                    bldc_run.eletrical_rpm_abs = abs(bldc_run.eletrical_rpm);
                    bldc_run.machine_rpm = bldc_run.eletrical_rpm / bldc_ctrl.pole_pairs;
                    bldc_run.machine_rpm_abs = bldc_run.eletrical_rpm_abs / bldc_ctrl.pole_pairs;

                }
                else
                {
                    bldc_run.eletrical_rpm = 0;
                    bldc_run.eletrical_rpm_abs = 0;
                    bldc_run.machine_rpm = 0;
                    bldc_run.machine_rpm_abs = 0;
                }
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

        if(bldc_run.step_index != bldc_run.befm_index)
        {
            bldc_run.delay_angle_cnt++;
        }

        // Stall Protect
        if (bldc_run.phase_cnt > bldc_run.PWM_freq_now * bldc_ctrl.stall_protect_ms / 1000 && bldc_ctrl.stall_protect_ms)
        {
            bldc_run.phase_cnt = 0;
            bldc_run.phase_cnt_last = 0;
            bldc_run.phase_cnt_f = 0;
            bldc_run.period_cnt = 0;
            bldc_run.period_cnt_last = 0;
            bldc_run.period_cnt_f = 0;
            bldc_run.eletrical_rpm = 0;
            bldc_run.eletrical_rpm_abs = 0;
            bldc_run.machine_rpm = 0;
            bldc_run.machine_rpm_abs = 0;
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
            Update_Filter_Value_Fast(&bldc_ctrl,&bldc_run,0);
        }
        else if(Hall_Sensor.hall_filter_len > 2)
        {
            Update_Filter_Value_Fast(&bldc_ctrl,&bldc_run,1);
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
            bldc_ctrl.pTIM->CCER &= ~0x555;
        }
        break;

        case BLDC_StartUp:
        {
            switch(bldc_run.start_seq)
            {
                case BLDC_Start_Order:
                    bldc_ctrl.output = bldc_ctrl.start_force;
                    VVVF_Process(&bldc_ctrl,&bldc_run);
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
                    * ERPM = bldc_ctrl.KV * bldc_ctrl.pole_pairs * bldc_ctrl.start_force * Virtual_Moto.V_bus_v_f
                    * bldc_run.delay_Cnt_Ref > bldc_run.PWM_freq_now * 10 / ERPM
                    * bldc_run.delay_Cnt_Ref > bldc_run.PWM_freq_now * 10 / (bldc_ctrl.KV * bldc_ctrl.pole_pairs * bldc_ctrl.start_force * Virtual_Moto.V_bus_v_f)
                    */
                    if(bldc_run.delay_Cnt_Ref > bldc_run.PWM_freq_now * 10 / (bldc_ctrl.KV * bldc_ctrl.pole_pairs * bldc_ctrl.start_force * Virtual_Moto.V_bus_v_f))
                    {
                        bldc_ctrl.output += bldc_ctrl.start_force_rate;
                        bldc_ctrl.output = _constrain(bldc_ctrl.output,bldc_ctrl.output_min,bldc_ctrl.start_force * 2);

                        bldc_run.delay_Cnt_Ref *= bldc_ctrl.start_time_rate;//exponential curve
                        // bldc_run.delay_Cnt_Ref -= bldc_run.PWM_freq_now * bldc_ctrl.start_first_step_ms / 1000 * (1 - bldc_ctrl.start_time_rate);
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
            bldc_run.befm_detect_delay_cnt = 0;
            VVVF_Process(&bldc_ctrl,&bldc_run);
            HallLess_MOS_Switch(&bldc_ctrl,&bldc_run,bldc_run.start_step_index,Motor_Config.dir);
        }
        break;

        case BLDC_Run:
        {
            if(bldc_ctrl.sensor_type == SENSOR_LESS)
            {
                if(bldc_run.delay_angle_cnt > bldc_run.delay_angle_cnt_ref)
                {
                    bldc_run.delay_angle_cnt = 0;
                    bldc_run.step_index_last = bldc_run.step_index;
                    bldc_run.step_index = bldc_run.befm_index;
                    bldc_run.befm_detect_delay_cnt = 0;
                }
                if(bldc_run.befm_detect_delay_cnt <= bldc_run.befm_detect_delay_cnt_ref && bldc_ctrl.rapid_demagnetization_enable)
                {
                    HallLess_MOS_Switch_Demagnetization(&bldc_ctrl,&bldc_run,bldc_run.step_index,Motor_Config.dir);
                }
                else
                {
                    HallLess_MOS_Switch(&bldc_ctrl,&bldc_run,bldc_run.step_index,Motor_Config.dir);
                }
                VVVF_Process(&bldc_ctrl,&bldc_run);
                BLDC_Current_Process(&bldc_ctrl,&bldc_run);
            }
            else
            {
                bldc_run.run_state = BLDC_Run;
                bldc_run.step_index = hall_index_calibrated[Hall_Sensor.hall_index];
                VVVF_Process(&bldc_ctrl,&bldc_run);
                BLDC_Current_Process(&bldc_ctrl,&bldc_run);
                Hall_MOS_Switch(&bldc_ctrl,&bldc_run,bldc_run.step_index,Motor_Config.dir);
            }
        }
        break;

        case BLDC_Brake:
        {
            VVVF_Process(&bldc_ctrl,&bldc_run);
            Brake(&bldc_ctrl,&bldc_run);
        }
        break;

        case BLDC_FullBrake:
        {
            Full_Brake(&bldc_ctrl,&bldc_run);
        }
        break;

        case BLDC_PlugBrake:
        {
            VVVF_Process(&bldc_ctrl,&bldc_run);
            Plug_Brake(&bldc_ctrl,&bldc_run);
        }
        break;
    }

    debug_arr[0] = phase_voltage_V.U;
    debug_arr[1] = phase_voltage_V.V;
    debug_arr[2] = phase_voltage_V.W;
    debug_arr[3] = bldc_run.virtual_mid;
    debug_arr[4] = phase_voltage_V_f.U;
    debug_arr[5] = phase_voltage_V_f.V;
    debug_arr[6] = phase_voltage_V_f.W;
    debug_arr[7] = bldc_run.virtual_mid_f;
    debug_arr[8] = bldc_run.run_state;
    debug_arr[9] = bldc_run.start_seq;

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
                bldc_ctrl.output = (float)(abs(bldc_run.eletrical_rpm)) / bldc_ctrl.KV / bldc_ctrl.pole_pairs / Virtual_Moto.V_bus_v_f;
            }
            else if(bldc_run.eletrical_rpm < 0)
            {   
                while(bldc_run.eletrical_rpm != 0)
                {
                    Full_Brake(&bldc_ctrl,&bldc_run);
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
                bldc_run.period_cnt = 0;
            }
        }
        else if(Motor_Config.dir == CW)
        {
            #if ANTI_WIND_STARTUP_ENABLE
            if(bldc_run.eletrical_rpm > 0)
            {
                while(bldc_run.eletrical_rpm != 0)
                {
                    Full_Brake(&bldc_ctrl,&bldc_run);
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
                bldc_ctrl.output = (float)(abs(bldc_run.eletrical_rpm)) / bldc_ctrl.KV / bldc_ctrl.pole_pairs / Virtual_Moto.V_bus_v_f;
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
                bldc_run.period_cnt = 0;
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
    Update_Filter_Value_Fast(&bldc_ctrl,&bldc_run,7);
    bldc_run.PWM_freq_now = bldc_ctrl.PWM_freq_min;
    bldc_run.TIM_ARR_now = PWM_TIM_BASE_FREQ / 2 / bldc_run.PWM_freq_now;
    Virtual_Moto.dt = 1.0f / bldc_run.PWM_freq_now;
    bldc_ctrl.pTIM->CCER &= ~0x555;
    // disable preload
    bldc_ctrl.pTIM->CR1 &= ~0x80;
    bldc_ctrl.pTIM->CCMR1 &= ~0x808;
    bldc_ctrl.pTIM->CCMR2 &= ~0x08;
    bldc_ctrl.pTIM->ARR = bldc_run.TIM_ARR_now;
    #ifdef ADC_SAMPLE_HIGH_SIDE
    bldc_ctrl.pTIM->CCR4 = 1;
    #else
    bldc_ctrl.pTIM->CCR4 = bldc_ctrl.pTIM->ARR - 1;
    #endif
}

