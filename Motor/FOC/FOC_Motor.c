#include "Board_Config.h"
#include "main.h"
#include "FOC_Motor.h"
#include "FOC.h"
#include "FOC_Config.h"

#include "PLL.h"

#include "SMO.h"
#include "FluxLinkageObserver.h"

#include "VIS.h"

// Debug Data
extern DAC_HandleTypeDef hdac;
extern float debug_arr[21];

extern uint8_t Dead_Time_Cal(uint32_t TIM_Clock,uint8_t CKD,uint32_t ns);
extern uint16_t Dead_Time_Compensation_Cal(uint32_t TIM_Clock,uint32_t ns,uint8_t centeraligned);

FOC_Para_t foc_para;
FOC_RUN_t foc_run;
FOC_CONTROL_t foc_ctrl =
{
    .sensor_type = Sensor_Type,
    .startup_erpm = StartUpErpm,
    .startup_iq_max = StartIQMax,
    .startup_iq_min = StartIQMin,
    .startup_order_ms = OrderMs,
    .startup_acc_ms = ACCMs,
    .erpm_min = MinErpm,
    .pole_pairs = PolePairs,
    .current_loop_en = CURRENT_LOOP_ENABLE,
    .speed_loop_en = SPEED_LOOP_ENABLE,
    .position_loop_en = POSITION_LOOP_ENABLE,
    .current_loop_div = FOC_CURRENT_LOOP_DIV,
    .speed_loop_div = FOC_SPEED_LOOP_DIV,
    .position_loop_div = FOC_POSITION_LOOP_DIV,
    .modulation_ratio = FOC_MAX_MODULATION_RATIO,
    .PWM_freq = FOC_PWM_FREQ,
};
PID_Position_t ID_PID;
PID_Position_t IQ_PID;
PID_Position_t Speed_PID;
PID_Position_t Position_PID;

HFI_CONTROL_t HFI_ctrl = 
{
    .HFI_Freq = 1000,
    .HFI_Polarity_judgment_ms = 10,
    .HFI_Ud_amplitude = 0.2f,

    .HFI_switch_on_speed = 8.0f,
    .HFI_switch_off_speed = 5.0f,
};

HFI_RUN_t HFI_run = 
{
    .HFI_Ud_sign = 1,
    .Kp = 0.1f,
    .Ki = 0.1f,
};

PLL_t Encoder_PLL;
PLL_t Hall_PLL;

void SVPWM_Update(float Ua_rate,float Ub_rate,float Ud_rate,float Uq_rate,float angle)
{
    //SVPWM Cal
    #if 1
    SVPWM_DQ(Ud_rate,Uq_rate,angle,&foc_para.sector,foc_run.TIM1_ARR_now,&foc_para.CMP1,&foc_para.CMP2,&foc_para.CMP3);
    #else
    SVPWM_AB(Ua_rate, Ub_rate, &foc_para.sector, foc_run.TIM1_ARR_now, &foc_para.CMP1, &foc_para.CMP2, &foc_para.CMP3);
    #endif

    #if DT_COMPENSATION_ENABLE
    foc_para.current_vector_angle = Normalize_Angle(angle + atan2f(foc_para.Iq,foc_para.Id));
    foc_para.current_vector_sector = foc_para.current_vector_angle / _PI_6;
    switch(foc_para.current_vector_sector)
    {
        case 0:
        case 11:
            foc_para.CMP1 += foc_para.dt_compensation_value;
            foc_para.CMP2 -= foc_para.dt_compensation_value;
            foc_para.CMP3 -= foc_para.dt_compensation_value;
            break;

        case 1:
        case 2:
            foc_para.CMP1 += foc_para.dt_compensation_value;
            foc_para.CMP2 += foc_para.dt_compensation_value;
            foc_para.CMP3 -= foc_para.dt_compensation_value;
            break;

        case 3:
        case 4:
            foc_para.CMP1 -= foc_para.dt_compensation_value;
            foc_para.CMP2 += foc_para.dt_compensation_value;
            foc_para.CMP3 -= foc_para.dt_compensation_value;
            break;

        case 5:
        case 6:
            foc_para.CMP1 -= foc_para.dt_compensation_value;
            foc_para.CMP2 += foc_para.dt_compensation_value;
            foc_para.CMP3 += foc_para.dt_compensation_value;
            break;

        case 7:
        case 8:
            foc_para.CMP1 -= foc_para.dt_compensation_value;
            foc_para.CMP2 -= foc_para.dt_compensation_value;
            foc_para.CMP3 += foc_para.dt_compensation_value;
            break;

        case 9:
        case 10:
            foc_para.CMP1 += foc_para.dt_compensation_value;
            foc_para.CMP2 -= foc_para.dt_compensation_value;
            foc_para.CMP3 += foc_para.dt_compensation_value;
            break;

        default:
            break;
    }
    #endif

    foc_para.CMP1 = _constrain(foc_para.CMP1, 0, foc_run.TIM1_ARR_now);
    foc_para.CMP2 = _constrain(foc_para.CMP2, 0, foc_run.TIM1_ARR_now);
    foc_para.CMP3 = _constrain(foc_para.CMP3, 0, foc_run.TIM1_ARR_now);

    //TIM update
    TIM1->CCR1 = foc_para.CMP1;
    TIM1->CCR2 = foc_para.CMP2;
    TIM1->CCR3 = foc_para.CMP3;
}

void SPI_Encoder_Angle_Speed_Cali(void)
{
    // angle
    AS5048_Read_M_Ang();
    PLL_Run(&Encoder_PLL,sinf(AS5048_para.m_angle),cosf(AS5048_para.m_angle));

    #if !FOC_DEBUG_ENCODER
    foc_para.e_angle = Normalize_Angle(Encoder_PLL.theta * foc_ctrl.pole_pairs);
    foc_para.m_angle = Encoder_PLL.theta;
    foc_para.m_angle_multicycle = Encoder_PLL.theta_unlimit;

    foc_run.eletrical_Hz = Encoder_PLL.hz_f * foc_ctrl.pole_pairs;
    foc_run.eletrical_Hz_f = foc_run.eletrical_Hz;
    foc_run.eletrical_rpm = foc_run.eletrical_Hz * 60;
    foc_run.eletrical_rpm_f = foc_run.eletrical_rpm;
    foc_run.machine_rpm = Encoder_PLL.hz_f * 60;
    foc_run.machine_rpm_f = foc_run.machine_rpm;
    #endif
}

void HALL_Angle_Speed_Cali(void)
{
    uint32_t temp0 = 0;
    uint32_t temp1 = 0;

    float up_lim = 0;
    float down_lim = 0;

    // hall
    Hall_Sensor.hall_queue_f[0] = HALL_A_READ();
    Hall_Sensor.hall_queue_f[1] = HALL_B_READ();
    Hall_Sensor.hall_queue_f[2] = HALL_C_READ();
    Hall_Sensor.hall_index = Hall_Sensor.hall_queue_f[0] + (Hall_Sensor.hall_queue_f[1] << 1) + (Hall_Sensor.hall_queue_f[2] << 2);

    if (Hall_Sensor.hall_index_last == Hall_Sensor.hall_index)
    {
        Hall_Sensor.hall_sector_cnt++;
        /* 使用最低转速限制 效果不好 减速阶段响应慢 */
        // if (Hall_Sensor.hall_sector_cnt > foc_run.PWM_freq_now / (1000 / (1000 / (foc_ctrl.erpm_min * 6 / 60))))
        // if (Hall_Sensor.hall_sector_cnt > foc_run.PWM_freq_now * 10 / foc_ctrl.erpm_min)
        /* 使用上一次计数值 减速幅值超过一定百分比则重新计算 */
        if (Hall_Sensor.hall_sector_cnt > Hall_Sensor.hall_sector_cnt_last * 1.05f)
        {
            Hall_Sensor.e_speed = _sign(Hall_Sensor.e_speed) * _PI_3 / (float)Hall_Sensor.hall_sector_cnt; // rads/cycle
            // FirstOrder_LPF_Cacl(Hall_Sensor.e_speed, Hall_Sensor.e_speed_f, Filter_Rate.RPM_filter_rate);

            temp0 = hall_seq_calibration_120[Hall_Sensor.hall_index - 1] - 1;
            /*
                (11*PI/6 + 1*PI/6) / 2 == (5*PI/6 + 7*PI/6) / 2
                当 temp0 == 1 时转子跨越0°电角度,导致估算角度出现180°误差,从而引起电机抖动
            */
            if (temp0 == 1)
            {
                Hall_Sensor.e_angle = 0;
            }
            else
            {
                Hall_Sensor.e_angle = (hall_deg_seq_ccw[temp0] + hall_deg_seq_cw[temp0]) / 2;
            }
            Hall_Sensor.e_angle_observe = Hall_Sensor.e_angle;
        }
        else
        {
            Hall_Sensor.e_angle_observe += Hall_Sensor.e_speed_f;

            up_lim = Hall_Sensor.e_angle + _PI_3;
            down_lim = Hall_Sensor.e_angle - _PI_3;

            Hall_Sensor.e_angle_observe = _constrain(Hall_Sensor.e_angle_observe, down_lim, up_lim);
        }
    }
    else
    {
        Hall_Sensor.hall_sector_cnt_last = Hall_Sensor.hall_sector_cnt;

        // if direction is CCW Hall_Sensor.e_angle_delta will be pi/3 otherwise 5*pi/3
        temp0 = hall_seq_calibration_120[Hall_Sensor.hall_index - 1] - 1;
        temp1 = hall_seq_calibration_120[Hall_Sensor.hall_index_last - 1] - 1;
        Hall_Sensor.e_angle_delta = Normalize_Angle(hall_deg_seq_ccw[temp0] - hall_deg_seq_ccw[temp1]);

        if (Hall_Sensor.e_angle_delta > _PI) // CW
        {
            Hall_Sensor.e_angle = hall_deg_seq_cw[temp0];
            if (Hall_Sensor.hall_sector_cnt)
            {
                if (Hall_Sensor.e_speed >= 0)
                {
                    Hall_Sensor.e_speed = -1e-6; // rads/cycle
                    Hall_Sensor.e_speed_f = -1e-6;
                }
                else
                {
                    Hall_Sensor.e_speed = -_PI_3 / (float)Hall_Sensor.hall_sector_cnt; // rads/cycle
                    // FirstOrder_LPF_Cacl(Hall_Sensor.e_speed, Hall_Sensor.e_speed_f, Filter_Rate.RPM_filter_rate);
                }
            }
            else
            {
                Hall_Sensor.e_speed = 0;
            }
        }
        else // CCW
        {
            Hall_Sensor.e_angle = hall_deg_seq_ccw[temp0];
            if (Hall_Sensor.hall_sector_cnt)
            {
                if (Hall_Sensor.e_speed <= 0)
                {
                    Hall_Sensor.e_speed = 1e-6; // rads/cycle
                    Hall_Sensor.e_speed_f = 1e-6;
                }
                else
                {
                    Hall_Sensor.e_speed = _PI_3 / (float)Hall_Sensor.hall_sector_cnt; // rads/cycle
                    // FirstOrder_LPF_Cacl(Hall_Sensor.e_speed, Hall_Sensor.e_speed_f, Filter_Rate.RPM_filter_rate);
                }
            }
            else
            {
                Hall_Sensor.e_speed = 0;
            }
        }

        Hall_Sensor.hall_index_last = Hall_Sensor.hall_index;
        Hall_Sensor.hall_sector_cnt = 0;

        Hall_Sensor.e_angle_observe = Hall_Sensor.e_angle;
    }

    FirstOrder_LPF_Cacl(Hall_Sensor.e_speed, Hall_Sensor.e_speed_f, Filter_Rate.RPM_filter_rate);
    Hall_Sensor.e_rpm = Hall_Sensor.e_speed_f * foc_run.PWM_freq_now * 60 / _2PI;

    float e_angle = Normalize_Angle(Hall_Sensor.e_angle_observe);
    PLL_Run(&Hall_PLL,sinf(e_angle),cosf(e_angle));

    #if !FOC_DEBUG_HALL
    foc_para.e_angle = Hall_PLL.theta;
    foc_run.eletrical_Hz = Hall_PLL.hz;
    foc_run.eletrical_Hz_f = Hall_PLL.hz_f;
    foc_run.eletrical_rpm = Hall_PLL.hz * 60;
    foc_run.eletrical_rpm_f = Hall_PLL.hz_f * 60;
    foc_run.machine_rpm = foc_run.eletrical_rpm / foc_ctrl.pole_pairs;
    foc_run.machine_rpm_f = foc_run.eletrical_rpm_f / foc_ctrl.pole_pairs;
    #endif
}

// https://zhuanlan.zhihu.com/p/150779067
void HFI_Angle_Speed_Cali(void)
{
    if(HFI_ctrl.HFI_Freq > foc_run.PWM_freq_now / 2)
    {
        HFI_ctrl.HFI_Freq = foc_run.PWM_freq_now / 2;
    }

    if (++HFI_run.HFI_DIV_cnt >= foc_run.PWM_freq_now / HFI_ctrl.HFI_Freq / 2)
    {
        HFI_run.HFI_DIV_cnt = 0;

        foc_para.Ud_rate = HFI_ctrl.HFI_Ud_amplitude * HFI_run.HFI_Ud_sign;
        HFI_run.HFI_Ud_sign *= -1;

        HFI_run.HFI_Ia_last = HFI_run.HFI_Ia_now;
        HFI_run.HFI_Ia_now = foc_para.Ia;
        HFI_run.HFI_Ib_last = HFI_run.HFI_Ib_now;
        HFI_run.HFI_Ib_now = foc_para.Ib;

        HFI_run.HFI_Ia_base = (HFI_run.HFI_Ia_now + HFI_run.HFI_Ia_last) / 2;
        HFI_run.HFI_Ia_delta = (HFI_run.HFI_Ia_now - HFI_run.HFI_Ia_last) / 2;
        HFI_run.HFI_Ib_base = (HFI_run.HFI_Ib_now + HFI_run.HFI_Ib_last) / 2;
        HFI_run.HFI_Ib_delta = (HFI_run.HFI_Ib_now - HFI_run.HFI_Ib_last) / 2;

        if(HFI_run.HFI_Ud_sign > 0)
        {
            HFI_run.HFI_Id_P = fabsf(foc_para.Id);
        }
        else
        {
            HFI_run.HFI_Id_N = fabsf(foc_para.Id);
        }

        // // HFI_ctrl.HFI_e_angle = Normalize_Angle(atan2f(HFI_run.HFI_Ud_sign * HFI_run.HFI_Ib_delta,HFI_run.HFI_Ud_sign * HFI_run.HFI_Ia_delta));
        // HFI_ctrl.HFI_e_angle = Normalize_Angle(atan2f(-HFI_run.HFI_Ud_sign * HFI_run.HFI_Ib_delta,-HFI_run.HFI_Ud_sign * HFI_run.HFI_Ia_delta) - _PI);

        // PLL
        HFI_run.err = HFI_run.HFI_Ud_sign * HFI_run.HFI_Ib_delta * cosf(HFI_run.theta) - HFI_run.HFI_Ud_sign * HFI_run.HFI_Ia_delta * sinf(HFI_run.theta);
        HFI_run.Integral += HFI_run.err * HFI_run.Ki / HFI_ctrl.HFI_Freq;
        HFI_run.Ui = HFI_run.err * HFI_run.Kp + HFI_run.Integral;
        // HFI_ctrl.speed_hz = HFI_ctrl.Ui / (1.0f / HFI_ctrl.HFI_Freq * _2PI);
        HFI_run.speed_hz = HFI_run.Ui * HFI_ctrl.HFI_Freq / _2PI;
        FirstOrder_LPF_Cacl(HFI_run.speed_hz, HFI_run.speed_hz_f, 0.003f);
        HFI_run.theta += HFI_run.Ui;

        if(HFI_run.HFI_Id_P < HFI_run.HFI_Id_N)
        {
            if(HFI_run.HFI_NS_Polarity_judgment_cnt++ > HFI_ctrl.HFI_Freq * HFI_ctrl.HFI_Polarity_judgment_ms / 1000)
            {
                HFI_run.HFI_NS_Polarity_judgment_cnt = 0;
                HFI_run.theta += _PI;
            }
        }
        else
        {
            HFI_run.HFI_NS_Polarity_judgment_cnt = 0;
        }

        HFI_run.theta = Normalize_Angle(HFI_run.theta);
    }
}

void SensorLess_Angle_Speed_Cali(void)
{
    switch(foc_run.run_state)
    {
        case FOC_Order://order in 0 degree
        {
            foc_para.Id_target = 0;
            foc_para.Iq_target += (float)foc_run.start_iq_now / (foc_run.PWM_freq_now * foc_ctrl.startup_order_ms / 2 / 1000);
            if(fabsf(foc_para.Iq_target) > fabsf(foc_run.start_iq_now))
            {
                foc_para.Iq_target = foc_run.start_iq_now;
            }
            foc_para.e_angle = foc_run.start_e_ang;
            
            // if(foc_run.general_cnt < foc_run.PWM_freq_now / (1000.0f / (float)foc_ctrl.startup_order_ms))
            if(foc_run.general_cnt < foc_run.PWM_freq_now * foc_ctrl.startup_order_ms / 1000)
            {
                foc_run.general_cnt++;
            }
            else
            {
                foc_run.start_cnt = 0;
                foc_run.general_cnt = 0;
                foc_run.run_state = FOC_StartUp;
            }
        }
        break;

        case FOC_StartUp:
        {
            #if FOC_SMO_ENABLE && !FOC_DEBUG_SMO
            SMO_Run(&smo_observer,&foc_para);
            foc_run.eletrical_rpm = smo_observer.E_rpm;
            foc_run.eletrical_rpm_f = foc_run.eletrical_rpm;
            foc_run.eletrical_Hz = smo_observer.E_rps;
            foc_run.eletrical_Hz_f = foc_run.eletrical_Hz;
            foc_run.machine_rpm = foc_run.eletrical_rpm_f / foc_ctrl.pole_pairs;
            foc_run.machine_rpm_f = foc_run.machine_rpm;
            #endif

            #if FOC_FLO_ENABLE && !FOC_DEBUG_FLO
            FLO_Run(&flo_observer,&foc_para);
            foc_run.eletrical_rpm = flo_observer.E_rpm;
            foc_run.eletrical_rpm_f = foc_run.eletrical_rpm;
            foc_run.eletrical_Hz = flo_observer.E_rps;
            foc_run.eletrical_Hz_f = foc_run.eletrical_Hz;
            foc_run.machine_rpm = foc_run.eletrical_rpm_f / foc_ctrl.pole_pairs;
            foc_run.machine_rpm_f = foc_run.machine_rpm;
            #endif

            foc_para.Iq_target = foc_run.start_iq_now;

            foc_run.start_cnt++;

            float startup_diff_angle = 0;
            #if FOC_SMO_ENABLE && !FOC_DEBUG_SMO
            startup_diff_angle = ABS_Angle_Delta(smo_observer.E_ang,foc_run.start_e_ang);
            #endif
            #if FOC_FLO_ENABLE && !FOC_DEBUG_FLO
            startup_diff_angle = ABS_Angle_Delta(flo_observer.E_ang,foc_run.start_e_ang);
            #endif
            /* rpp -> rads per period */
            #if USE_S_CURVE_ACCELERATE
            /*
                S型加速曲线
                y = A + B / (1 + e^(-ax + b))
                A 无用设为0
                B 决定曲线最大值
                a 决定曲线上升时间 上升时间大致 = 10 / a
                b 固定为5
            */
            //target rads per PWM period
            float start_target_rpp = foc_ctrl.startup_erpm * _2PI / 60 / foc_run.PWM_freq_now;
            float a = 10.0f / (foc_ctrl.startup_acc_ms * foc_run.PWM_freq_now / 1000);
            foc_run.start_rpp_now = start_target_rpp * 1.1f / (1 + expf(-a * foc_run.start_cnt + 5));
            #else
            /*
                梯形加速曲线
                y = ax
            */
            //target rads per PWM period
            float start_target_rpp = foc_ctrl.startup_erpm * _2PI / 60 / foc_run.PWM_freq_now;
            //acceleration rads per speed loop / speed loop number in 1second * time cal
            float start_rpp_delta = (start_target_rpp / foc_run.PWM_freq_now) * (1000.0f / (float)foc_ctrl.startup_acc_ms);
            if(foc_run.start_rpp_now < start_target_rpp)
            {
                foc_run.start_rpp_now += start_rpp_delta;
            }
            #endif
            #if 1
            if(foc_run.start_rpp_now >= start_target_rpp && startup_diff_angle > MAX_DIFF_ANGLE && fabsf(foc_run.start_iq_now) > foc_ctrl.startup_iq_min && foc_run.eletrical_Hz * foc_run.start_iq_now > 0 &&foc_run.general_cnt == 0)
            {
                // foc_run.start_iq_now += -0.001f * Motor_Config.dir;
                foc_run.start_iq_now *= 0.99995f; // ((20000,0.36787) @ y = 0.99995 ^ x)
            }
            if(startup_diff_angle < MAX_DIFF_ANGLE && fabsf(foc_run.eletrical_rpm) >= foc_ctrl.erpm_min && foc_run.eletrical_Hz * foc_run.start_iq_now > 0)
            #else
            else
            #endif
            {
                if(foc_run.general_cnt++ > foc_run.PWM_freq_now * 10 / 1000)// 10ms
                {
                    if(foc_ctrl.speed_loop_en)
                    {
                        foc_para.Speed_target = foc_run.machine_rpm_f;
                    }
                    else
                    {
                        foc_para.Iq_target = foc_run.start_iq_now;
                    }
                    foc_run.run_state = FOC_CloseLoopRun;
                    foc_run.general_cnt = 0;
                }
            }
            else
            {
                foc_run.general_cnt = 0;
            }

            foc_run.start_e_ang += foc_run.start_rpp_now * Motor_Config.dir;
            foc_run.start_e_ang = Normalize_Angle(foc_run.start_e_ang);
            foc_para.e_angle = foc_run.start_e_ang;
        }
        break;

        case FOC_HFIRun:
        {
            HFI_Angle_Speed_Cali();
        }
        // break;
        case FOC_IDLE:
        case FOC_CloseLoopRun:
        {
            #if FOC_SMO_ENABLE
            SMO_Run(&smo_observer,&foc_para);

            #if !FOC_DEBUG_SMO
            foc_para.e_angle = smo_observer.E_ang;
            foc_run.eletrical_rpm = smo_observer.E_rpm;
            foc_run.eletrical_rpm_f = foc_run.eletrical_rpm;
            foc_run.eletrical_Hz = smo_observer.E_rps;
            foc_run.eletrical_Hz_f = foc_run.eletrical_Hz;
            foc_run.machine_rpm = foc_run.eletrical_rpm_f / foc_ctrl.pole_pairs;
            foc_run.machine_rpm_f = foc_run.machine_rpm;
            #endif
            #endif

            #if FOC_FLO_ENABLE
            FLO_Run(&flo_observer,&foc_para);

            #if !FOC_DEBUG_FLO
            foc_para.e_angle = flo_observer.E_ang;
            foc_run.eletrical_rpm = flo_observer.E_rpm;
            foc_run.eletrical_rpm_f = foc_run.eletrical_rpm;
            foc_run.eletrical_Hz = flo_observer.E_rps;
            foc_run.eletrical_Hz_f = foc_run.eletrical_Hz;
            foc_run.machine_rpm = foc_run.eletrical_rpm_f / foc_ctrl.pole_pairs;
            foc_run.machine_rpm_f = foc_run.machine_rpm;
            #endif
            #endif

            /* stall protection */
            // if(foc_ctrl.sensor_type == SENSOR_LESS && foc_run.run_state == FOC_CloseLoopRun && fabsf(foc_run.eletrical_rpm_f) < foc_ctrl.erpm_min)
            // {
            //     if(foc_run.general_cnt++ > foc_run.PWM_freq_now / 5)
            //     {
            //         extern uint8_t motor_start;
            //         motor_start = 0;
            //         Stop_Motor();
            //     }
            // }
            // else
            // {
            //     foc_run.general_cnt = 0;
            // }

            if(foc_ctrl.sensor_type == SENSOR_LESS_HFI)
            {
                if(foc_run.run_state == FOC_HFIRun && fabsf(HFI_run.speed_hz_f) < HFI_ctrl.HFI_switch_on_speed)
                {
                    foc_para.e_angle = HFI_run.theta;
                    foc_run.eletrical_Hz = HFI_run.speed_hz;
                    foc_run.eletrical_Hz_f = HFI_run.speed_hz_f;
                    foc_run.eletrical_rpm = foc_run.eletrical_Hz * 60;
                    foc_run.eletrical_rpm_f = foc_run.eletrical_Hz_f * 60;
                    foc_run.machine_rpm = foc_run.eletrical_rpm / foc_ctrl.pole_pairs;
                    foc_run.machine_rpm_f = foc_run.eletrical_rpm_f / foc_ctrl.pole_pairs;
                }
                else
                {
                    if(foc_run.run_state == FOC_HFIRun)
                    {
                        foc_run.run_state = FOC_CloseLoopRun;

                        HFI_run.HFI_DIV_cnt = 0;
                        HFI_run.HFI_Ud_sign = 1;
                        HFI_run.HFI_Ia_now = 0;
                        HFI_run.HFI_Ia_last = 0;
                        HFI_run.HFI_Ib_now = 0;
                        HFI_run.HFI_Ib_last = 0;
                        HFI_run.HFI_Ia_base = 0;
                        HFI_run.HFI_Ia_delta = 0;
                        HFI_run.HFI_Ib_base = 0;
                        HFI_run.HFI_Ib_delta = 0;
                        HFI_run.HFI_DIV_cnt = 0;
                        HFI_run.HFI_DIV_cnt = 0;
                        HFI_run.HFI_DIV_cnt = 0;

                        HFI_run.Integral = 0;
                        HFI_run.Ui = 0;
                        HFI_run.err = 0;
                        HFI_run.speed_hz = 0;
                        HFI_run.speed_hz_f = 0;
                    }
                    else if(foc_run.run_state == FOC_CloseLoopRun && fabsf(foc_run.eletrical_Hz_f) < HFI_ctrl.HFI_switch_off_speed)
                    {
                        foc_run.run_state = FOC_HFIRun;
                    }
                }
            }
        }
        break;
    }
}

void FOC_Process(void)
{    
    #if FOC_POSITION_LOOP_DIV > 1
    if (++foc_run.position_loop_cnt >= foc_ctrl.position_loop_div)
    {
        foc_run.position_loop_cnt = 0;
    #endif
        if(foc_ctrl.position_loop_en && foc_run.run_state == FOC_CloseLoopRun)
        {
            foc_para.Speed_target = PID_Position_Run(&Position_PID, foc_para.Position_target, foc_para.m_angle_multicycle);
        }
    #if FOC_POSITION_LOOP_DIV > 1
    }
    #endif 

    #if FOC_SPEED_LOOP_DIV > 1
    if (++foc_run.speed_loop_cnt >= foc_ctrl.speed_loop_div)
    {
        foc_run.speed_loop_cnt = 0;
    #endif
        if(foc_ctrl.speed_loop_en && foc_run.run_state == FOC_CloseLoopRun)
        {
            foc_para.Iq_target = PID_Position_Run(&Speed_PID, foc_para.Speed_target, foc_run.machine_rpm_f);
        }
    #if FOC_SPEED_LOOP_DIV > 1
    }
    #endif


    #if FOC_CURRENT_LOOP_DIV > 1
    if (++foc_run.current_loop_cnt >= foc_ctrl.current_loop_div)
    {
        foc_run.current_loop_cnt = 0;
    #endif

        #if 1
        Clarke_Transmission(phase_voltage_V_f[0], phase_voltage_V_f[1], phase_voltage_V_f[2], &foc_para.Ua, &foc_para.Ub);
        Clarke_Transmission(phase_current_A_f[0], phase_current_A_f[1], phase_current_A_f[2], &foc_para.Ia, &foc_para.Ib);
        #else
        /*
        * Uan = Uag - Ung
        * Ubn = Ubg - Ung
        * Ucn = Ucg - Ung
        * Uan + Ubn + Ucn = 0
        * Uan + Ubn + Ucn = Uag + Ubg + Ucg - 3 * Ung = 0
        * Ung = (Uag + Ubg + Ucg) / 3
        * Uan = 2 / 3 * Uag - 1 / 3 * Ubg - 1 / 3 * Ucg
        * Ubn = -1 / 3 * Uag + 2 / 3 * Ubg - 1 / 3 * Ucg
        * Ucn = -1 / 3 * Uag - 1 / 3 * Ubg + 2 / 3 * Ucg
        */

        foc_para.U_ag = phase_voltage_V_f[0];
        foc_para.U_bg = phase_voltage_V_f[1];
        foc_para.U_cg = phase_voltage_V_f[2];
        
        foc_para.U_an = foc_para.U_ag * _2_3 - foc_para.U_bg * 0.3333333f - foc_para.U_cg * 0.3333333f;
        foc_para.U_bn = -foc_para.U_ag * 0.3333333f + foc_para.U_bg * _2_3 - foc_para.U_cg * 0.3333333f;
        foc_para.U_cn = -foc_para.U_ag * 0.3333333f - foc_para.U_bg * 0.3333333f + foc_para.U_cg * _2_3;
        
        // foc_para.U_an = Virtual_Moto.V_bus_v_f * (foc_para.CMP1 * _2_3 - foc_para.CMP2 * 0.3333333f - foc_para.CMP3 * 0.3333333f) / foc_run.TIM1_ARR_now;
        // foc_para.U_bn = Virtual_Moto.V_bus_v_f * (-foc_para.CMP1 * 0.3333333f + foc_para.CMP2 * _2_3 - foc_para.CMP3 * 0.3333333f) / foc_run.TIM1_ARR_now;
        // foc_para.U_cn = Virtual_Moto.V_bus_v_f * (-foc_para.CMP1 * 0.3333333f - foc_para.CMP2 * 0.3333333f + foc_para.CMP3 * _2_3) / foc_run.TIM1_ARR_now;

        foc_para.I_a = phase_current_A_f[0];
        foc_para.I_b = phase_current_A_f[1];
        foc_para.I_c = phase_current_A_f[2];

        Clarke_Transmission(foc_para.U_an, foc_para.U_bn, foc_para.U_cn, &foc_para.Ua, &foc_para.Ub);
        Clarke_Transmission(foc_para.I_a, foc_para.I_b, foc_para.I_c, &foc_para.Ia, &foc_para.Ib);
        #endif

        #if !FOC_DEBUG_ENCODER
        if (foc_ctrl.sensor_type == SPI_ENCODER)
        #endif
        {
            SPI_Encoder_Angle_Speed_Cali();
        }
            
        #if !FOC_DEBUG_HALL
        if (foc_ctrl.sensor_type == HALL_120_SENSOR || foc_ctrl.sensor_type == HALL_60_SENSOR)
        #endif
        {
            HALL_Angle_Speed_Cali();
        }
            
        #if !FOC_DEBUG_SENSORLESS
        if (foc_ctrl.sensor_type <= SENSOR_LESS_HFI)
        #endif
        {
            SensorLess_Angle_Speed_Cali();
        }


        // switch(foc_ctrl.sensor_type)
        // {
        //     case SENSOR_LESS:
        //     case SENSOR_LESS_VIS:
        //     case SENSOR_LESS_HFI:
        //         SensorLess_Angle_Speed_Cali();
        //         break;
            
        //     case HALL_120_SENSOR:
        //     case HALL_60_SENSOR:
        //         HALL_Angle_Speed_Cali();
        //         break;

        //     case SPI_ENCODER:
        //         SPI_Encoder_Angle_Speed_Cali();
        //         break;
        // }

        Virtual_Moto.electronic_speed_hz = fabsf(foc_run.eletrical_Hz_f);

        Park_Transmission(foc_para.Ia, foc_para.Ib, &foc_para.Id, &foc_para.Iq, foc_para.e_angle);
        Park_Transmission(foc_para.Ua, foc_para.Ub, &foc_para.Ud, &foc_para.Uq, foc_para.e_angle);
        
        if(foc_ctrl.current_loop_en)
        {
            #if MTPA_ENABLE
            MTPA_Cal(&foc_para,FLUX_Wb,Ld_H,Lq_H);
            #endif

            #if FEED_FORWARD_ENABLE
            float we = foc_run.eletrical_Hz_f * _2PI;
            foc_para.Ud_ff_rate = -we * Lq_H * foc_para.Iq / (Virtual_Moto.V_bus_v_f  * _2_3);
            foc_para.Uq_ff_rate = we * (Ld_H * foc_para.Id + FLUX_Wb) / (Virtual_Moto.V_bus_v_f  * _2_3);
            if (foc_run.run_state != FOC_HFIRun)
            {
                foc_para.Ud_rate = FeedForward_PID_Position_Run(&ID_PID, foc_para.Ud_ff_rate, foc_para.Id_target, foc_para.Id);
            }
            foc_para.Uq_rate = FeedForward_PID_Position_Run(&IQ_PID, foc_para.Uq_ff_rate, foc_para.Iq_target, foc_para.Iq);

            #else
            if (foc_run.run_state != FOC_HFIRun)
            {
                foc_para.Ud_rate = PID_Position_Run(&ID_PID, foc_para.Id_target, foc_para.Id);
            }
            foc_para.Uq_rate = PID_Position_Run(&IQ_PID, foc_para.Iq_target, foc_para.Iq);
            #endif


            Inverse_Park_Transmission(foc_para.Ud_rate, foc_para.Uq_rate, &foc_para.Ua_rate, &foc_para.Ub_rate, foc_para.e_angle);

            // modulation ratio limit
            saturate_vector_2d(&foc_para.Ua_rate, &foc_para.Ub_rate, foc_ctrl.modulation_ratio);
            saturate_vector_2d(&foc_para.Ud_rate, &foc_para.Uq_rate, foc_ctrl.modulation_ratio);

            #if 1
            // // Id Iq filter
            // FirstOrder_LPF_Cacl(foc_para.Id, foc_para.Id_f, Filter_Rate.phase_current_filter_rate);
            // FirstOrder_LPF_Cacl(foc_para.Iq, foc_para.Iq_f, Filter_Rate.phase_current_filter_rate);
            // cal voltage vector and current vector
            foc_para.Us = sqrtf(SQ(foc_para.Ua) + SQ(foc_para.Ub));
            FirstOrder_LPF_Cacl(foc_para.Us, foc_para.Us_f, 0.3f);
            foc_para.Us_rate = sqrtf(SQ(foc_para.Ud_rate) + SQ(foc_para.Uq_rate));
            FirstOrder_LPF_Cacl(foc_para.Us_rate, foc_para.Us_rate_f, 0.3f);
            foc_para.Is = sqrtf(SQ(foc_para.Id) + SQ(foc_para.Iq));
            FirstOrder_LPF_Cacl(foc_para.Is, foc_para.Is_f, 0.3f);
            #endif

            // angle compensate
            foc_para.e_angle += foc_run.eletrical_Hz_f / FOC_CC_LOOP_FREQ;
            foc_para.e_angle = Normalize_Angle(foc_para.e_angle);

            SVPWM_Update(foc_para.Ua_rate,foc_para.Ub_rate,foc_para.Ud_rate,foc_para.Uq_rate,foc_para.e_angle);
        }

    #if FOC_CURRENT_LOOP_DIV > 1
    }
    #endif

    // debug_arr[0] = phase_current_A[0];
    // debug_arr[1] = phase_current_A[1];
    // debug_arr[2] = phase_current_A[2];
    // debug_arr[3] = phase_current_A_f[0];
    // debug_arr[4] = phase_current_A_f[1];
    // debug_arr[5] = phase_current_A_f[2];

    debug_arr[0] = foc_para.Id_target;
    debug_arr[1] = foc_para.Id;
    debug_arr[2] = foc_para.Iq_target;
    debug_arr[3] = foc_para.Iq;
    debug_arr[4] = foc_para.Ia;
    debug_arr[5] = foc_para.Ib;
    debug_arr[6] = foc_para.e_angle;
    debug_arr[7] = foc_run.eletrical_Hz_f;

    // debug_arr[8] = foc_para.Speed_target;
    // debug_arr[9] = foc_run.machine_rpm_f;
    // debug_arr[10] = foc_para.Position_target;
    // debug_arr[11] = foc_para.m_angle_multicycle;

    debug_arr[8] = foc_para.Ua;
    debug_arr[9] = foc_para.Ub;
    debug_arr[10] = foc_para.Ua_rate;
    debug_arr[11] = foc_para.Ub_rate;
    debug_arr[12] = foc_para.Ud;
    debug_arr[13] = foc_para.Uq;
    debug_arr[14] = foc_para.Ud_rate;
    debug_arr[15] = foc_para.Uq_rate;
    debug_arr[16] = foc_para.Us;
    debug_arr[17] = foc_para.Us_f;
    debug_arr[18] = foc_para.Us_rate;
    debug_arr[19] = foc_para.Us_rate_f;

    // debug_arr[8] = flo_observer.pll.theta;
    // debug_arr[9] = flo_observer.Theta;
    // debug_arr[10] = flo_observer.R_I_a;
    // debug_arr[11] = flo_observer.R_I_b;
    // debug_arr[12] = flo_observer.flux_a;
    // debug_arr[13] = flo_observer.flux_b;
    // debug_arr[14] = flo_observer.flux_s_a;
    // debug_arr[15] = flo_observer.flux_s_b;
    // debug_arr[16] = flo_observer.flux_r_a;
    // debug_arr[17] = flo_observer.flux_r_b;
    // debug_arr[18] = flo_observer.flux_r_err;
    // debug_arr[19] = flo_observer.pll.speed_hz_f;

    // debug_arr[8] = smo_observer.Theta;
    // debug_arr[9] = smo_observer.EstIalpha;
    // debug_arr[10] = smo_observer.EstIbeta;
    // debug_arr[11] = smo_observer.EalphaFinal;
    // debug_arr[12] = smo_observer.EbetaFinal;
    // debug_arr[13] = smo_observer.Ealpha;
    // debug_arr[14] = smo_observer.Ebeta;
    // debug_arr[15] = smo_observer.Zalpha;
    // debug_arr[16] = smo_observer.Zbeta;
    // debug_arr[17] = smo_observer.IalphaError;
    // debug_arr[18] = smo_observer.IbetaError;

    // debug_arr[8] = HFI_run.HFI_Ia_last;
    // debug_arr[9] = HFI_run.HFI_Ia_now;
    // debug_arr[10] = HFI_run.HFI_Ib_last;
    // debug_arr[11] = HFI_run.HFI_Ib_now;
    // debug_arr[12] = HFI_run.HFI_Ia_base;
    // debug_arr[13] = HFI_run.HFI_Ia_delta;
    // debug_arr[14] = HFI_run.HFI_Ib_base;
    // debug_arr[15] = HFI_run.HFI_Ib_delta;
    // debug_arr[16] = HFI_run.HFI_Id_P;
    // debug_arr[17] = HFI_run.HFI_Id_N;
    // debug_arr[18] = HFI_run.theta;

    // debug_arr[16] = smo_observer.Theta;
    // debug_arr[17] = smo_observer.pll.theta;
    // debug_arr[18] = flo_observer.pll.theta;

    // debug_arr[18] = Hall_PLL.theta;
    // debug_arr[19] = Normalize_Angle(Hall_Sensor.e_angle_observe);

    // debug_arr[18] = Encoder_PLL.theta;
    // debug_arr[19] = AS5048_para.m_angle;

    CDC_Transmit_FS((uint8_t *)debug_arr,sizeof(debug_arr));
}

void Start_FOC_Motor(void)
{
    #if DT_COMPENSATION_ENABLE
    foc_para.dt_compensation_value = Dead_Time_Compensation_Cal(PWM_TIM_BASE_FREQ,200,1);
    #endif

    #if !FOC_DEBUG_ENCODER
    if (foc_ctrl.sensor_type == SPI_ENCODER)
    #endif
    {
        #if !FOC_DEBUG_ENCODER
        foc_run.run_state = FOC_CloseLoopRun;
        #endif
    }

    #if !FOC_DEBUG_HALL
    if (foc_ctrl.sensor_type == HALL_120_SENSOR || foc_ctrl.sensor_type == HALL_60_SENSOR)
    #endif
    {
        #if !FOC_DEBUG_HALL
        foc_run.run_state = FOC_CloseLoopRun;
        #endif
    }

    #if !FOC_DEBUG_SENSORLESS
    if (foc_ctrl.sensor_type <= SENSOR_LESS_HFI)
    #endif
    {
        switch(foc_ctrl.sensor_type)
        {            
            case SENSOR_LESS_HFI:
            {
                foc_run.run_state = FOC_HFIRun;
                foc_para.Ud_rate = 0;
                foc_para.Uq_rate = 0;
                foc_para.Id_target = 0;
                foc_para.Iq_target = 0;
            }
            break;

            case SENSOR_LESS_VIS:
            {
                if(vis_run.VIS_credible == 0)
                {
                    Stop_FOC_Motor();
                    motor_type_now = VIS;
                    Init_VIS();
                    Start_VIS();
                    break;
                }
            }
            case SENSOR_LESS:
            {
                foc_run.general_cnt = 0;
                foc_run.start_iq_now = foc_ctrl.startup_iq_max * Motor_Config.dir;
                foc_run.start_rpp_now = 0;
                foc_run.start_e_ang = Normalize_Angle(0 - _PI_2);
                foc_para.Id_target = 0;
                foc_para.Iq_target = 0;

                if(vis_run.VIS_credible && foc_ctrl.sensor_type == SENSOR_LESS_VIS)
                {
                    foc_run.start_e_ang = Normalize_Angle(vis_run.VIS_e_angle - _PI_2);
                    Clear_Vis();
                }

                if(foc_run.eletrical_rpm_f > foc_ctrl.erpm_min && Motor_Config.dir == CCW)
                {
                    foc_run.run_state = FOC_CloseLoopRun;
                    foc_para.Iq_target = foc_ctrl.startup_iq_min;
                }
                else if(foc_run.eletrical_rpm_f > foc_ctrl.erpm_min && Motor_Config.dir == CCW)
                {
                    foc_run.run_state = FOC_CloseLoopRun;
                    foc_para.Iq_target = -foc_ctrl.startup_iq_min;
                }
                else
                {
                    #if FOC_SMO_ENABLE
                    #if !FOC_DEBUG_SMO
                    foc_run.run_state = FOC_Order;
                    #endif
                    #endif

                    #if FOC_FLO_ENABLE
                    #if !FOC_DEBUG_FLO
                    foc_run.run_state = FOC_Order;

                    // foc_run.run_state = FOC_CloseLoopRun;
                    // foc_para.Iq_target = 1500;
                    #endif
                    #endif
                }

            }
            break;

            default:
            {

            }
            break;
        }
    }

    PID_Position_Reset(&ID_PID);
    PID_Position_Reset(&IQ_PID);
    PID_Position_Reset(&Speed_PID);
    PID_Position_Reset(&Position_PID);
    foc_ctrl.current_loop_en = 1;
    TIM1->CCER |= 0x555;
}

void Stop_FOC_Motor(void)
{
    TIM1->CCER &= ~0x555;
    foc_ctrl.current_loop_en = 0;
    foc_run.run_state = FOC_IDLE;
    memset(&foc_para,0,sizeof(foc_para));
    PID_Position_Reset(&ID_PID);
    PID_Position_Reset(&IQ_PID);
    PID_Position_Reset(&Speed_PID);
    PID_Position_Reset(&Position_PID);
}

void Init_FOC_Motor(void)
{
    foc_ctrl.current_loop_en = 0;
    foc_run.run_state = FOC_IDLE;
    foc_run.PWM_freq_now = foc_ctrl.PWM_freq;
    foc_run.TIM1_ARR_now = PWM_TIM_BASE_FREQ / 2 / foc_run.PWM_freq_now;
    Virtual_Moto.dt = 1.0f / foc_run.PWM_freq_now;
    TIM1->CCER &= ~0x555;
    // enable preload
    TIM1->CR1 |= 0x80;
    TIM1->CCMR1 |= 0x808;
    TIM1->CCMR2 |= 0x08;
    TIM1->ARR = foc_run.TIM1_ARR_now;
    #ifdef ADC_SAMPLE_HIGH_SIDE
    TIM1->CCR4 = 1;
    #else
    TIM1->CCR4 = TIM1->ARR - 1;
    #endif

    #if FOC_SMO_ENABLE
    SMO_Init(&smo_observer,100,1.0f / FOC_CC_LOOP_FREQ);
    #endif

    #if FOC_FLO_ENABLE
    FLO_Init(&flo_observer,100,1.0f / FOC_CC_LOOP_FREQ);
    #endif

    PID_Position_Init(&ID_PID,FOC_CC_LOOP_DT,ID_KP,ID_KI,ID_KD,ID_KB,ID_LP,ID_LN);
    PID_Position_Init(&IQ_PID,FOC_CC_LOOP_DT,IQ_KP,IQ_KI,IQ_KD,IQ_KB,IQ_LP,IQ_LN);

    // Current_Loop_PID_Tune(&ID_PID,FOC_CC_LOOP_FREQ / 10,Ld_H,Rs_R);
    // Current_Loop_PID_Tune(&IQ_PID,FOC_CC_LOOP_FREQ / 10,Lq_H,Rs_R);

    // float t = MIN(Ld_H / Rs_R,Lq_H / Rs_R);
    // float wc = _2PI / t;
    // Current_Loop_PID_Tune(&ID_PID,wc,Ld_H,Rs_R);
    // Current_Loop_PID_Tune(&IQ_PID,wc,Lq_H,Rs_R);

    PID_Position_Init(&Speed_PID,FOC_SC_LOOP_DT,SPEED_KP,SPEED_KI,SPEED_KD,SPEED_KB,SPEED_LP,SPEED_LN);
    PID_Position_Init(&Position_PID,FOC_PC_LOOP_DT,POSITION_KP,POSITION_KI,POSITION_KD,POSITION_KB,POSITION_LP,POSITION_LN);

    PLL_Init(&Encoder_PLL,0.1,0.1,100,1.0f / FOC_CC_LOOP_FREQ);
    PLL_Init(&Hall_PLL,0.1,0.1,100,1.0f / FOC_CC_LOOP_FREQ);
}
