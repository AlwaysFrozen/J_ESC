#include "Board_Config.h"
#include "Encoder.h"
#include "cmsis_os.h"
#include "Motor.h"
#include "BLDC_Motor.h"
#include "FOC.h"
#include "FOC_Motor.h"
#include "FOC_Config.h"

#include "AS5048a.h"

#define READ_TIMES              32
#define IGNORE_TIMES            4
#define MAX_ERR_DEGREE          5


// AS5048 value cw decrease ccw increase
Encoder_Para_t AS5048_para =
{
    .bits = 14,
};

/* 
    Encoder alignment is divided into five stages
    1. Set the D-axis current and align it to the initial Angle
    2. Rotate counterclockwise
    3. Rotate clockwise -> Return to the initial Angle
    4. Rotate clockwise
    5. Rotate counterclockwise -> Return to the initial Angle
    Take the average of the 3 Angle values in stage 1.3.5 as the final alignment Angle
*/
Sensor_Cali_Err_t Encoder_Calibration(float current_limit)
{
    Sensor_Cali_Err_t err = Sensor_Cali_Err_NONE;

    float ua = 0;
    float ub = 0;
    float ud = 0;
    float uq = 0;
    /* 
        align to the initial Angle
        Considering that the motor is affected by the cogging effect and the rotor stays at an Angle of 30 90 150 210 270 330 in its natural state, the initial alignment Angle should be selected among these values to reduce the error
    */
    int e_degree = 90;
    /* rotation angle -> 1/3 round*/
    int move_e_degree = 120;
    uint16_t raw_data[READ_TIMES];
    float temp = 0;
    /* angle buffer */
    float temp_m_ang[6] = {0};
    /* quadrant buffer */
    uint8_t quadrant[6] = {0};

    // enable output
    MOS_Driver_Enable();
    TIM1->CCER |= 0x555;

    // stage 1
    while (foc_para.Id < current_limit)
    {
        Clarke_Transmission(phase_current_f[0], phase_current_f[1], phase_current_f[2], &foc_para.Ia, &foc_para.Ib);
        Park_Transmission(foc_para.Ia, foc_para.Ib, &foc_para.Id, &foc_para.Iq, DEG_TO_RAD(e_degree));
        Inverse_Park_Transmission(ud, uq, &ua, &ub, DEG_TO_RAD(e_degree));
        SVPWM_Update(ua, ub, ud, uq, DEG_TO_RAD(e_degree));

        ud += 0.001f;
        if (ud > FOC_MAX_MODULATION_RATIO)
        {
            ud = FOC_MAX_MODULATION_RATIO;
            break;
        }
        osDelay(1);
    }

    osDelay(1000);

    for (uint8_t i = 0; i < READ_TIMES; i++)
    {
        raw_data[i] = AS5048a_Read_Raw();
        osDelay(1);
    }
    bubbleSort_u16(raw_data, READ_TIMES);
    for (uint8_t i = IGNORE_TIMES; i < READ_TIMES - IGNORE_TIMES; i++)
    {
        temp += raw_data[i];
    }
    temp /= READ_TIMES - IGNORE_TIMES * 2;
    temp_m_ang[0] = Normalize_Angle(temp / AS5048_MAX_VALUE * _2PI);
    quadrant[0] = temp_m_ang[0] / _PI_2;
    temp = 0;
    // stage 2
    for (int16_t i = 0; i < move_e_degree; i++)
    {
        Inverse_Park_Transmission(ud, uq, &ua, &ub, DEG_TO_RAD(e_degree++));
        SVPWM_Update(ua, ub, ud, uq, DEG_TO_RAD(e_degree));
        osDelay(10);
    }
    osDelay(500);
    for (uint8_t i = 0; i < READ_TIMES; i++)
    {
        raw_data[i] = AS5048a_Read_Raw();
        osDelay(1);
    }
    bubbleSort_u16(raw_data, READ_TIMES);
    for (uint8_t i = IGNORE_TIMES; i < READ_TIMES - IGNORE_TIMES; i++)
    {
        temp += raw_data[i];
    }
    temp /= READ_TIMES - IGNORE_TIMES * 2;
    temp_m_ang[1] = Normalize_Angle(temp / AS5048_MAX_VALUE * _2PI);
    quadrant[1] = temp_m_ang[1] / _PI_2;
    temp = 0;
    // stage 3
    for (int16_t i = move_e_degree; i > 0; i--)
    {
        Inverse_Park_Transmission(ud, uq, &ua, &ub, DEG_TO_RAD(e_degree--));
        SVPWM_Update(ua, ub, ud, uq, DEG_TO_RAD(e_degree));
        osDelay(10);
    }
    osDelay(500);
    for (uint8_t i = 0; i < READ_TIMES; i++)
    {
        raw_data[i] = AS5048a_Read_Raw();
        osDelay(1);
    }
    bubbleSort_u16(raw_data, READ_TIMES);
    for (uint8_t i = IGNORE_TIMES; i < READ_TIMES - IGNORE_TIMES; i++)
    {
        temp += raw_data[i];
    }
    temp /= READ_TIMES - IGNORE_TIMES * 2;
    temp_m_ang[2] = Normalize_Angle(temp / AS5048_MAX_VALUE * _2PI);
    quadrant[2] = temp_m_ang[2] / _PI_2;
    temp = 0;
    // stage 4
    for (int16_t i = move_e_degree; i > 0; i--)
    {
        Inverse_Park_Transmission(ud, uq, &ua, &ub, DEG_TO_RAD(e_degree--));
        SVPWM_Update(ua, ub, ud, uq, DEG_TO_RAD(e_degree));
        osDelay(10);
    }
    osDelay(500);
    for (uint8_t i = 0; i < READ_TIMES; i++)
    {
        raw_data[i] = AS5048a_Read_Raw();
        osDelay(1);
    }
    bubbleSort_u16(raw_data, READ_TIMES);
    for (uint8_t i = IGNORE_TIMES; i < READ_TIMES - IGNORE_TIMES; i++)
    {
        temp += raw_data[i];
    }
    temp /= READ_TIMES - IGNORE_TIMES * 2;
    temp_m_ang[3] = Normalize_Angle(temp / AS5048_MAX_VALUE * _2PI);
    quadrant[3] = temp_m_ang[3] / _PI_2;
    temp = 0;
    // stage 5
    for (int16_t i = 0; i < move_e_degree; i++)
    {
        Inverse_Park_Transmission(ud, uq, &ua, &ub, DEG_TO_RAD(e_degree++));
        SVPWM_Update(ua, ub, ud, uq, DEG_TO_RAD(e_degree));
        osDelay(10);
    }
    osDelay(500);
    for (uint8_t i = 0; i < READ_TIMES; i++)
    {
        raw_data[i] = AS5048a_Read_Raw();
        osDelay(1);
    }
    bubbleSort_u16(raw_data, READ_TIMES);
    for (uint8_t i = IGNORE_TIMES; i < READ_TIMES - IGNORE_TIMES; i++)
    {
        temp += raw_data[i];
    }
    temp /= READ_TIMES - IGNORE_TIMES * 2;
    temp_m_ang[4] = Normalize_Angle(temp / AS5048_MAX_VALUE * _2PI);
    quadrant[4] = temp_m_ang[4] / _PI_2;
    temp = 0;

    temp = DEG_TO_RAD(MAX_ERR_DEGREE);

    if (ABS_Angle_Delta(temp_m_ang[0], temp_m_ang[1]) < temp / 2)
    {
        err |= Sensor_Cali_Err_NO_MOVE;
    }

    if(fabsf(temp_m_ang[1] - temp_m_ang[0]) > _PI)
    {
        if(temp_m_ang[0] < _PI)
        {
            AS5048_para.reverse = 1;
        }
        else
        {
            AS5048_para.reverse = 0;
        }

    }
    else
    {
        if (temp_m_ang[1] > temp_m_ang[0])
        {
            AS5048_para.reverse = 0;
        }
        else
        {
            AS5048_para.reverse = 1;
        }
    }

    if (ABS_Angle_Delta(temp_m_ang[0], temp_m_ang[2]) < temp && ABS_Angle_Delta(temp_m_ang[2], temp_m_ang[4]) < temp && ABS_Angle_Delta(temp_m_ang[0], temp_m_ang[4]) < temp)
    {
        /* take the average of the 3 Angle values */
        temp_m_ang[5] = temp_m_ang[0] + temp_m_ang[2] + temp_m_ang[4];
        /* If the three results fall in the first and fourth quadrants respectively, the result in the first quadrant needs to be added 360° to the average to avoid problems like (1+1+359)/3=120.33 */
        if(quadrant[0] == 3 || quadrant[2] == 3 || quadrant[4] == 3)
        {
            if(quadrant[0] == 0)
            {
                temp_m_ang[5] += _2PI;
            }
            if(quadrant[2] == 0)
            {
                temp_m_ang[5] += _2PI;
            }
            if(quadrant[4] == 0)
            {
                temp_m_ang[5] += _2PI;
            }
        }
        temp_m_ang[5] = Normalize_Angle(temp_m_ang[5] / 3);
        if(AS5048_para.reverse)
        {
            AS5048_para.m_angle_offset = temp_m_ang[5] + DEG_TO_RAD(e_degree) / foc_ctrl.pole_pairs;
        }
        else
        {
            AS5048_para.m_angle_offset = temp_m_ang[5] - DEG_TO_RAD(e_degree) / foc_ctrl.pole_pairs;
        }

        AS5048_Read_M_Ang();
        AS5048_para.e_angle = AS5048_para.m_angle * foc_ctrl.pole_pairs;
        AS5048_para.e_angle = Normalize_Angle(AS5048_para.e_angle);
        foc_para.m_angle_multicycle = AS5048_para.m_angle;
        foc_para.m_cycle = 0;
    }
    else
    {
        err |= Sensor_Cali_Err_ALIGN;
    }

    // disable output
    TIM1->CCER &= ~0x555;
    MOS_Driver_Disable();

    return err;
}
