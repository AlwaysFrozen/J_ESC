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
    编码器对齐分为5个阶段
    1.设定D轴电流并对齐到初始角度
    2.逆时针旋转固定角度
    3.顺时针旋转固定角度->回到初始角度
    4.顺时针旋转固定角度
    5.逆时针旋转固定角度->回到初始角度
    取1.3.5阶段3个角度值的平均值作为最终对齐角度
*/
Sensor_Cali_Err_t Encoder_Calibration(float current_limit)
{
    Sensor_Cali_Err_t err = Sensor_Cali_Err_NONE;

    float ua = 0;
    float ub = 0;
    float ud = 0;
    float uq = 0;
    /* 初始对齐角度 考虑到电机受齿槽效应影响在自然状态下转子停留在30 90 150 210 270 330等角度,初始对齐角度应在这些值中选择以减小误差 */
    int e_degree = 90;
    /* 转动角度 */
    int move_e_degree = 120; // 1/3 round
    uint16_t raw_data[READ_TIMES];
    float temp = 0;
    /* 各阶段所处角度 */
    float temp_m_ang[6] = {0};
    /* 各阶段所处象限 */
    uint8_t quadrant[6] = {0};

    // enable output
    MOS_Driver_Enable();
    TIM1->CCER |= 0x555;

    // current set
    while (foc_para.Id < current_limit)
    {
        Clarke_Transmission(phase_current_A_f[0], phase_current_A_f[1], phase_current_A_f[2], &foc_para.Ia, &foc_para.Ib);
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

    // 0
    for (uint8_t i = 0; i < READ_TIMES;)
    {
        if(AS5048a_Read_Raw(&raw_data[i]))
        {
            i++;
        }
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
    // 1
    for (int16_t i = 0; i < move_e_degree; i++)
    {
        Inverse_Park_Transmission(ud, uq, &ua, &ub, DEG_TO_RAD(e_degree++));
        SVPWM_Update(ua, ub, ud, uq, DEG_TO_RAD(e_degree));
        osDelay(10);
    }
    osDelay(500);
    for (uint8_t i = 0; i < READ_TIMES;)
    {
        if(AS5048a_Read_Raw(&raw_data[i]))
        {
            i++;
        }
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
    // 2
    for (int16_t i = move_e_degree; i > 0; i--)
    {
        Inverse_Park_Transmission(ud, uq, &ua, &ub, DEG_TO_RAD(e_degree--));
        SVPWM_Update(ua, ub, ud, uq, DEG_TO_RAD(e_degree));
        osDelay(10);
    }
    osDelay(500);
    for (uint8_t i = 0; i < READ_TIMES;)
    {
        if(AS5048a_Read_Raw(&raw_data[i]))
        {
            i++;
        }
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
    // 3
    for (int16_t i = move_e_degree; i > 0; i--)
    {
        Inverse_Park_Transmission(ud, uq, &ua, &ub, DEG_TO_RAD(e_degree--));
        SVPWM_Update(ua, ub, ud, uq, DEG_TO_RAD(e_degree));
        osDelay(10);
    }
    osDelay(500);
    for (uint8_t i = 0; i < READ_TIMES;)
    {
        if(AS5048a_Read_Raw(&raw_data[i]))
        {
            i++;
        }
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
    // 4
    for (int16_t i = 0; i < move_e_degree; i++)
    {
        Inverse_Park_Transmission(ud, uq, &ua, &ub, DEG_TO_RAD(e_degree++));
        SVPWM_Update(ua, ub, ud, uq, DEG_TO_RAD(e_degree));
        osDelay(10);
    }
    osDelay(500);
    for (uint8_t i = 0; i < READ_TIMES;)
    {
        if(AS5048a_Read_Raw(&raw_data[i]))
        {
            i++;
        }
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
        /* 三次测量取平均值 */
        temp_m_ang[5] = temp_m_ang[0] + temp_m_ang[2] + temp_m_ang[4];
        /* 如果三次结果分别落在第一和第四象限,取平均值时需要将第一象限的结果加上360°以避免类似(1+1+359)/3=120.33的问题 */
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