#include "Board_Config.h"
#include "HALL.h"
#include "cmsis_os.h"
#include "Motor.h"
#include "BLDC_Motor.h"
#include "FOC.h"
#include "FOC_Motor.h"
#include "FOC_Config.h"

HALL_Sensor_t Hall_Sensor;

// accurate calibration
float hall_index_calibrated[8] = {0,1,2,3,4,5,0,0};
float hall_deg_calibrated[8] = {30,90,150,210,270,330,0,0};
float hall_rad_calibrated[8] = {_PI_6,_PI_2,_5PI_6,_7PI_6,_3PI_2,_11PI_6,0,0};


void Hall_Sensor_Init(void)
{
    memset(&Hall_Sensor, 0, sizeof(Hall_Sensor));
    Update_Hall_Filter_Value(1);
}

void Update_Hall_Filter_Value(uint8_t len)
{
    if (len < 1)
    {
        len = 1;
    }
    Hall_Sensor.hall_filter_len = len;
    Hall_Sensor.hall_filter_value = 0;
    for (uint8_t i = 0; i < Hall_Sensor.hall_filter_len; i++)
    {
        Hall_Sensor.hall_filter_value |= (1 << i);
    }
}

Sensor_Cali_Err_t HALL_Calibration(float current_limit)
{
    Sensor_Cali_Err_t err = Sensor_Cali_Err_NONE;

    DQ_Axis_t V_dq;
    DQ_Axis_t I_dq;
    AB_Axis_t I_alpha_beta;
    TIM_t tim;

    memset(&V_dq,0,sizeof(V_dq));
    memset(&I_dq,0,sizeof(I_dq));
    memset(&I_alpha_beta,0,sizeof(I_alpha_beta));
    memset(&tim,0,sizeof(tim));

    // enable output
    MOS_Driver_Enable();
    TIM1->CCER |= 0x555;
    tim.Reload = TIM1->ARR;
    // set current to 0
    SVPWM_DQ(&V_dq,DEG_TO_RAD(0),&tim);
    TIM1->CCR1 = tim.CMP1;
    TIM1->CCR2 = tim.CMP2;
    TIM1->CCR3 = tim.CMP3;
    while (I_dq.D > 0.01f)
    {
        Clarke_Transmission(&phase_current_A_f, &I_alpha_beta);
        Park_Transmission(&I_alpha_beta, &I_dq, DEG_TO_RAD(0));
        osDelay(1);
    }

    // current set
    while (I_dq.D < current_limit)
    {
        Clarke_Transmission(&phase_current_A_f, &I_alpha_beta);
        Park_Transmission(&I_alpha_beta, &I_dq, DEG_TO_RAD(0));
        SVPWM_DQ(&V_dq,DEG_TO_RAD(0),&tim);
        TIM1->CCR1 = tim.CMP1;
        TIM1->CCR2 = tim.CMP2;
        TIM1->CCR3 = tim.CMP3;

        V_dq.D += 0.001f;
        if (V_dq.D > foc_ctrl.modulation_ratio)
        {
            V_dq.D = foc_ctrl.modulation_ratio;
            break;
        }
        osDelay(1);
    }
    osDelay(1000);

    Hall_Sensor_Init();

    Hall_Sensor.hall_queue_f[0] = HALL_A_READ(); 
    Hall_Sensor.hall_queue_f[1] = HALL_B_READ();
    Hall_Sensor.hall_queue_f[2] = HALL_C_READ();
    Hall_Sensor.hall_index = Hall_Sensor.hall_queue_f[0] + (Hall_Sensor.hall_queue_f[1] << 1) + (Hall_Sensor.hall_queue_f[2] << 2);
    Hall_Sensor.hall_index_last = Hall_Sensor.hall_index;

    uint16_t hall_deg_ccw[2][24] = {0};
    uint16_t hall_deg_cw[2][24] = {0};
    uint16_t edge_cnt[2] = {0};

    int32_t vector_deg = 0;
    uint32_t cnt = 0;
    while(edge_cnt[0] < 24 || cnt < 360 * 4)
    {
        if(cnt++ > 360 * 5)
        {
            err |= Sensor_Cali_Err_HALL_NO_MOVE;
            break;
        }
        SVPWM_DQ(&V_dq,DEG_TO_RAD(vector_deg++),&tim);
        TIM1->CCR1 = tim.CMP1;
        TIM1->CCR2 = tim.CMP2;
        TIM1->CCR3 = tim.CMP3;
        osDelay(2);

        Hall_Sensor.hall_queue_f[0] = HALL_A_READ();
        Hall_Sensor.hall_queue_f[1] = HALL_B_READ();
        Hall_Sensor.hall_queue_f[2] = HALL_C_READ();
        Hall_Sensor.hall_index = Hall_Sensor.hall_queue_f[0] + (Hall_Sensor.hall_queue_f[1] << 1) + (Hall_Sensor.hall_queue_f[2] << 2);
        if (Hall_Sensor.hall_index_last != Hall_Sensor.hall_index)
        {
            Hall_Sensor.hall_index_last = Hall_Sensor.hall_index;
            hall_deg_ccw[0][edge_cnt[0]] = Hall_Sensor.hall_index;
            hall_deg_ccw[1][edge_cnt[0]] = (int32_t)Normalize_Angle(vector_deg) % 360;
            edge_cnt[0]++;
        }
    }
    cnt = 0;
    osDelay(100);
    while(edge_cnt[1] < 24 || cnt < 360 * 4) 
    {
        if(cnt++ > 360 * 5)
        {
            err |= Sensor_Cali_Err_HALL_NO_MOVE;
            break;
        }
        SVPWM_DQ(&V_dq,DEG_TO_RAD(vector_deg--),&tim);
        TIM1->CCR1 = tim.CMP1;
        TIM1->CCR2 = tim.CMP2;
        TIM1->CCR3 = tim.CMP3;
        osDelay(2);

        Hall_Sensor.hall_queue_f[0] = HALL_A_READ();
        Hall_Sensor.hall_queue_f[1] = HALL_B_READ();
        Hall_Sensor.hall_queue_f[2] = HALL_C_READ();
        Hall_Sensor.hall_index = Hall_Sensor.hall_queue_f[0] + (Hall_Sensor.hall_queue_f[1] << 1) + (Hall_Sensor.hall_queue_f[2] << 2);
        if (Hall_Sensor.hall_index_last != Hall_Sensor.hall_index)
        {
            Hall_Sensor.hall_index_last = Hall_Sensor.hall_index;
            hall_deg_cw[0][edge_cnt[1]] = Hall_Sensor.hall_index;
            hall_deg_cw[1][edge_cnt[1]++] = (int32_t)Normalize_Angle(vector_deg) % 360;
        }
    }

    uint16_t hall_same_cnt_ccw[8] = {0};
    uint16_t hall_deg_avg_ccw[8] = {0};
    for(uint8_t i = 0;i < edge_cnt[0];i++)
    {
        hall_same_cnt_ccw[hall_deg_ccw[0][i]]++;
        hall_deg_avg_ccw[hall_deg_ccw[0][i]] += hall_deg_ccw[1][i];
    }
    for(uint8_t i = 0;i < sizeof(hall_same_cnt_ccw) / sizeof(hall_same_cnt_ccw[0]);i++)
    {
        hall_deg_avg_ccw[i] /= hall_same_cnt_ccw[i];
    }
    uint16_t hall_same_cnt_cw[8] = {0};
    uint16_t hall_deg_avg_cw[8] = {0};
    for(uint8_t i = 0;i < edge_cnt[0];i++)
    {
        hall_same_cnt_cw[hall_deg_cw[0][i]]++;
        hall_deg_avg_cw[hall_deg_cw[0][i]] += hall_deg_cw[1][i];
    }
    for(uint8_t i = 0;i < sizeof(hall_same_cnt_cw) / sizeof(hall_same_cnt_cw[0]);i++)
    {
        hall_deg_avg_cw[i] /= hall_same_cnt_cw[i];
    }

    uint16_t hall_same_cnt[8] = {0};
    for(uint8_t i = 0;i < sizeof(hall_same_cnt_cw) / sizeof(hall_same_cnt_cw[0]);i++)
    {
        hall_same_cnt[i] = hall_same_cnt_ccw[i] + hall_same_cnt_cw[i];
    }

    for(uint8_t i = 0;i < 8;i++)
    {
        hall_deg_calibrated[i] = Normalize_Angle(Angle_Average(hall_deg_avg_ccw[i],hall_deg_avg_cw[i]));
    }
    for(uint8_t i = 0;i < 8;i++)
    {
        hall_rad_calibrated[i] = Normalize_Rad(Rad_Average(DEG_TO_RAD(hall_deg_avg_ccw[i]),DEG_TO_RAD(hall_deg_avg_cw[i])));
    }

    uint8_t index_cnt = 0;
    for(uint16_t deg = 0;deg < 360;deg++)
    {
        for(uint8_t i = 0;i < 8;i++)
        {
            if(deg == ((int32_t)hall_deg_calibrated[i]) && hall_same_cnt[i])
            {
                if(index_cnt > 7)
                {
                    err |= Sensor_Cali_Err_HALL_SEQ;
                    break;
                }
                hall_index_calibrated[i] = index_cnt++;
                break;
            }
        }
    }


    // disable output
    TIM1->CCER &= ~0x555;
    MOS_Driver_Disable();

    return err;
}
