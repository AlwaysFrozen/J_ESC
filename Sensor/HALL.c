#include "Board_Config.h"
#include "HALL.h"
#include "cmsis_os.h"
#include "Motor.h"
#include "BLDC_Motor.h"
#include "FOC.h"
#include "FOC_Motor.h"
#include "FOC_Config.h"

HALL_Sensor_t Hall_Sensor;
// rough calibration
uint8_t hall_seq_ideal_120[6] = {2,6,4,5,1,3};
uint8_t hall_seq_ideal_60[6] = {0,4,6,7,3,1};
uint8_t hall_seq_calibration_120[6] = {1,2,3,4,5,6};
uint8_t hall_seq_calibration_60[8] = {0,1,0,3,4,0,6,7};
float hall_deg_seq_ccw[6] = {_7PI_6,_11PI_6,_3PI_2,_PI_2,_5PI_6,_PI_6};
float hall_deg_seq_cw[6] = {_3PI_2,_PI_6,_11PI_6,_5PI_6,_7PI_6,_PI_2};
// accurate calibration
float hall_deg_calibrated[6] = {0};

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

    float ua = 0;
    float ub = 0;
    float ud = 0;
    float uq = 0;

    // enable output
    MOS_Driver_Enable();
    TIM1->CCER |= 0x555;

    // current set
    while (foc_para.Id < current_limit)
    {
        Clarke_Transmission(phase_current_A_f[0], phase_current_A_f[1], phase_current_A_f[2], &foc_para.Ia, &foc_para.Ib);
        Park_Transmission(foc_para.Ia, foc_para.Ib, &foc_para.Id, &foc_para.Iq, DEG_TO_RAD(0));
        Inverse_Park_Transmission(ud, uq, &ua, &ub, DEG_TO_RAD(0));
        SVPWM_Update(ua, ub, ud, uq, DEG_TO_RAD(0));

        ud += 0.001f;
        if (ud > FOC_MAX_MODULATION_RATIO)
        {
            ud = FOC_MAX_MODULATION_RATIO;
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

    uint8_t hall_seq_ccw[12];
    uint8_t hall_seq_cw[12];
    uint8_t hall_seq_ccw_sort[6];
    uint8_t hall_seq_120_sort[6];
    uint8_t hall_seq_60_sort[6];

    for (int16_t i = 0; i < 720; i++) // Rotate 720° CCW
    {
        Inverse_Park_Transmission(ud, uq, &ua, &ub, DEG_TO_RAD(i));
        SVPWM_Update(ua, ub, ud, uq, DEG_TO_RAD(i));
        osDelay(2);

        Hall_Sensor.hall_queue_f[0] = HALL_A_READ();
        Hall_Sensor.hall_queue_f[1] = HALL_B_READ();
        Hall_Sensor.hall_queue_f[2] = HALL_C_READ();
        Hall_Sensor.hall_index = Hall_Sensor.hall_queue_f[0] + (Hall_Sensor.hall_queue_f[1] << 1) + (Hall_Sensor.hall_queue_f[2] << 2);
        if (Hall_Sensor.hall_index_last != Hall_Sensor.hall_index)
        {
            Hall_Sensor.hall_index_last = Hall_Sensor.hall_index;
            hall_deg_ccw[0][edge_cnt[0]] = Hall_Sensor.hall_index;
            hall_deg_ccw[1][edge_cnt[0]++] = i % 360;
        }
        if (i % 60 == 0)
        {
            hall_seq_ccw[i / 60] = Hall_Sensor.hall_index;
        }
    }
    osDelay(100);
    for (int16_t i = 720 - 1; i >= 0; i--) // Rotate 720° CW
    {
        Inverse_Park_Transmission(ud, uq, &ua, &ub, DEG_TO_RAD(i));
        SVPWM_Update(ua, ub, ud, uq, DEG_TO_RAD(i));
        osDelay(2);

        Hall_Sensor.hall_queue_f[0] = HALL_A_READ();
        Hall_Sensor.hall_queue_f[1] = HALL_B_READ();
        Hall_Sensor.hall_queue_f[2] = HALL_C_READ();
        Hall_Sensor.hall_index = Hall_Sensor.hall_queue_f[0] + (Hall_Sensor.hall_queue_f[1] << 1) + (Hall_Sensor.hall_queue_f[2] << 2);
        if (Hall_Sensor.hall_index_last != Hall_Sensor.hall_index)
        {
            Hall_Sensor.hall_index_last = Hall_Sensor.hall_index;
            hall_deg_cw[0][edge_cnt[1]] = Hall_Sensor.hall_index;
            hall_deg_cw[1][edge_cnt[1]++] = i % 360;
        }
        if (i % 60 == 0)
        {
            hall_seq_cw[i / 60] = Hall_Sensor.hall_index;
        }
    }

    // hall_deg_calibrated[0] = DEG_TO_RAD((hall_deg_ccw[1][0] + hall_deg_ccw[1][6] + hall_deg_cw[1][5] + hall_deg_cw[1][11]) / 4);
    // hall_deg_calibrated[1] = DEG_TO_RAD((hall_deg_ccw[1][1] + hall_deg_ccw[1][7] + hall_deg_cw[1][4] + hall_deg_cw[1][10]) / 4);
    // hall_deg_calibrated[2] = DEG_TO_RAD((hall_deg_ccw[1][2] + hall_deg_ccw[1][8] + hall_deg_cw[1][3] + hall_deg_cw[1][9]) / 4);
    // hall_deg_calibrated[3] = DEG_TO_RAD((hall_deg_ccw[1][3] + hall_deg_ccw[1][9] + hall_deg_cw[1][2] + hall_deg_cw[1][8]) / 4);
    // hall_deg_calibrated[4] = DEG_TO_RAD((hall_deg_ccw[1][4] + hall_deg_ccw[1][10] + hall_deg_cw[1][1] + hall_deg_cw[1][7]) / 4);
    // hall_deg_calibrated[5] = DEG_TO_RAD((hall_deg_ccw[1][5] + hall_deg_ccw[1][11] + hall_deg_cw[1][0] + hall_deg_cw[1][6]) / 4);

    // hall_deg_seq_ccw[0] = hall_deg_calibrated[3];
    // hall_deg_seq_ccw[1] = hall_deg_calibrated[5];
    // hall_deg_seq_ccw[2] = hall_deg_calibrated[4];
    // hall_deg_seq_ccw[3] = hall_deg_calibrated[1];
    // hall_deg_seq_ccw[4] = hall_deg_calibrated[2];
    // hall_deg_seq_ccw[5] = hall_deg_calibrated[0];

    // hall_deg_seq_cw[0] = hall_deg_calibrated[4];
    // hall_deg_seq_cw[1] = hall_deg_calibrated[0];
    // hall_deg_seq_cw[2] = hall_deg_calibrated[5];
    // hall_deg_seq_cw[3] = hall_deg_calibrated[2];
    // hall_deg_seq_cw[4] = hall_deg_calibrated[3];
    // hall_deg_seq_cw[5] = hall_deg_calibrated[1];

    for (uint8_t i = 0; i < 5; i++)
    {
        if (hall_seq_ccw[i] != hall_seq_ccw[i + 6])
        {
            err |= Sensor_Cali_Err_HALL_ALIGN;
        }
        if (hall_seq_cw[i] != hall_seq_cw[i + 6])
        {
            err |= Sensor_Cali_Err_HALL_ALIGN;
        }
        if (hall_seq_ccw[i] != hall_seq_cw[i])
        {
            err |= Sensor_Cali_Err_HALL_ALIGN;
        }
    }

    if (err == Sensor_Cali_Err_NONE)
    {
        memcpy(hall_seq_ccw_sort, hall_seq_ccw, sizeof(hall_seq_ccw_sort));
        // quickSort(hall_seq_ccw_sort,0,sizeof(hall_seq_ccw_sort) - 1);
        bubbleSort_u8(hall_seq_ccw_sort, sizeof(hall_seq_ccw_sort));

        memcpy(hall_seq_120_sort, hall_seq_ideal_120, sizeof(hall_seq_ideal_120));
        // quickSort(hall_seq_120_sort,0,sizeof(hall_seq_120_sort) - 1);
        bubbleSort_u8(hall_seq_120_sort, sizeof(hall_seq_120_sort));

        memcpy(hall_seq_60_sort, hall_seq_ideal_60, sizeof(hall_seq_ideal_60));
        // quickSort(hall_seq_60_sort,0,sizeof(hall_seq_60_sort) - 1);
        bubbleSort_u8(hall_seq_60_sort, sizeof(hall_seq_60_sort));

        if (memcmp(hall_seq_ccw_sort, hall_seq_120_sort, sizeof(hall_seq_120_sort) - 1) == 0)
        {
            if (Motor_Config.moto_type == BLDC && bldc_ctrl.sensor_type != HALL_120_SENSOR)
            {
                bldc_ctrl.sensor_type = HALL_120_SENSOR;
                err |= Sensor_Cali_Err_HALL_TYPE;
            }
            #if !FOC_DEBUG_HALL
            else if (Motor_Config.moto_type == FOC && foc_ctrl.sensor_type != HALL_120_SENSOR)
            {
                foc_ctrl.sensor_type = HALL_120_SENSOR;
                err |= Sensor_Cali_Err_HALL_TYPE;
            }
            #endif

            for (uint8_t i = 0; i < 6; i++)
            {
                hall_seq_calibration_120[hall_seq_ccw[i] - 1] = hall_seq_ideal_120[i];
            }
        }
        else if (memcmp(hall_seq_ccw_sort, hall_seq_60_sort, sizeof(hall_seq_60_sort) - 1) == 0)
        {
            if (Motor_Config.moto_type == BLDC && bldc_ctrl.sensor_type != HALL_60_SENSOR)
            {
                bldc_ctrl.sensor_type = HALL_60_SENSOR;
                err |= Sensor_Cali_Err_HALL_TYPE;
            }
            #if !FOC_DEBUG_HALL
            else if (Motor_Config.moto_type == FOC && foc_ctrl.sensor_type != HALL_60_SENSOR)
            {
                foc_ctrl.sensor_type = HALL_60_SENSOR;
                err |= Sensor_Cali_Err_HALL_TYPE;
            }
            #endif

            for (uint8_t i = 0; i < 6; i++)
            {
                hall_seq_calibration_60[hall_seq_ccw[i]] = hall_seq_ideal_60[i];
            }
        }
        else
        {
            err |= Sensor_Cali_Err_HALL_SEQ;
        }
    }

    // disable output
    TIM1->CCER &= ~0x555;
    MOS_Driver_Disable();

    return err;
}
