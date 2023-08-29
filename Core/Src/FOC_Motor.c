#include "Board_Config.h"
#include "main.h"
#include "FOC_Motor.h"
#include "FOC.h"
#include "FOC_Config.h"


#include "SMO.h"
#include "FluxLinkageObserver.h"


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
    .current_loop = CURRENT_LOOP_ENABLE,
    .speed_loop = SPEED_LOOP_ENABLE,
    .current_loop_div = FOC_CURRENT_LOOP_DIV,
    .speed_loop_div = FOC_SPEED_LOOP_DIV,
    .PWM_freq = FOC_PWM_FREQ,
};
PID_Position_t ID_PID;
PID_Position_t IQ_PID;
PID_Position_t Speed_PID;

FOC_HFI_Para_t HFI_para;

void SVPWM_Update(float Ua_rate,float Ub_rate,float Ud_rate,float Uq_rate,float angle)
{
    //SVPWM Cal
    #if 1
    SVPWM_DQ(Ud_rate,Uq_rate,angle,&foc_para.sector,foc_run.TIM1_ARR_now,&foc_para.CMP1,&foc_para.CMP2,&foc_para.CMP3);
    #else
    SVPWM_AB(Ua_rate, Ub_rate, &foc_para.sector, foc_run.TIM1_ARR_now, &foc_para.CMP1, &foc_para.CMP2, &foc_para.CMP3);
    #endif

    #if USE_DT_COMPENSATION
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
    AS5048_para.e_angle = AS5048_para.m_angle * foc_ctrl.pole_pairs;
    AS5048_para.e_angle = Normalize_Angle(AS5048_para.e_angle);

    #if (FOC_DEBUG_ENCODER && FOC_DEBUG_ENCODER_USE_SPEED) || (!FOC_DEBUG_ENCODER)
    // speed
    #if FOC_SPEED_LOOP_DIV > 1
    foc_run.speed_loop_cnt++;
    if (foc_run.speed_loop_cnt >= foc_ctrl.speed_loop_div)
    {
        foc_run.speed_loop_cnt = 0;
    #endif
        AS5048_para.m_angle_delta = AS5048_para.m_angle - AS5048_para.m_angle_last;
        if (AS5048_para.m_angle_delta < -_PI)
        {
            AS5048_para.m_angle_delta += _2PI;
        }
        else if (AS5048_para.m_angle_delta > _PI)
        {
            AS5048_para.m_angle_delta -= _2PI;
        }
        AS5048_para.m_rpm = AS5048_para.m_angle_delta / _2PI * foc_run.PWM_freq_now / foc_ctrl.speed_loop_div * 60;
        AS5048_para.m_angle_last = AS5048_para.m_angle;
        FirstOrder_LPF_Cacl(AS5048_para.m_rpm, AS5048_para.m_rpm_f, Filter_Rate.RPM_filter_rate);

        AS5048_para.e_rpm = AS5048_para.m_rpm * foc_ctrl.pole_pairs;
        AS5048_para.e_rpm_f = AS5048_para.m_rpm_f * foc_ctrl.pole_pairs;

        #if (FOC_DEBUG_ENCODER && FOC_DEBUG_ENCODER_USE_SPEED_LOOP) || (!FOC_DEBUG_ENCODER)
        if(foc_ctrl.speed_loop && foc_run.run_state == FOC_CloseLoopRun)
        {
            foc_para.Iq_target = PID_Position_Run(&Speed_PID, foc_para.Speed_target, AS5048_para.e_rpm_f);
        }
        #endif
    #if FOC_SPEED_LOOP_DIV > 1
    }
    #endif
    #endif

    #if (FOC_DEBUG_ENCODER && FOC_DEBUG_ENCODER_USE_ANGLE) || (!FOC_DEBUG_ENCODER)
    foc_para.m_angle = AS5048_para.m_angle;
    foc_para.e_angle = AS5048_para.e_angle;
    foc_run.eletrical_rpm = AS5048_para.e_rpm;
    foc_run.eletrical_rpm_f = AS5048_para.e_rpm_f;
    foc_run.eletrical_Hz = foc_run.eletrical_rpm / 60.0f;
    foc_run.eletrical_Hz_f = foc_run.eletrical_rpm_f / 60.0f;
    foc_run.machine_rpm = AS5048_para.m_rpm;
    foc_run.machine_rpm_f = AS5048_para.m_rpm_f;
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
        // if (Hall_Sensor.hall_sector_cnt > foc_run.PWM_freq_now / (1000 / (1000 / (foc_ctrl.erpm_min * 6 / 60))))
        if (Hall_Sensor.hall_sector_cnt > foc_run.PWM_freq_now * 10 / foc_ctrl.erpm_min)
        {
            Hall_Sensor.e_speed = _sign(Hall_Sensor.e_speed) * _PI_3 / (float)Hall_Sensor.hall_sector_cnt; // rads/cycle
            FirstOrder_LPF_Cacl(Hall_Sensor.e_speed, Hall_Sensor.e_speed_f, Filter_Rate.RPM_filter_rate);

            temp0 = hall_seq_calibration_120[Hall_Sensor.hall_index - 1] - 1;
            /*
                (11*PI/6 + 1*PI/6) / 2 == (5*PI/6 + 7*PI/6) / 2
                When temp0 == 1, the rotor crosses the 0° electrical Angle, resulting in a 180° error in the estimated Angle, resulting in motor jitter
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
                    FirstOrder_LPF_Cacl(Hall_Sensor.e_speed, Hall_Sensor.e_speed_f, Filter_Rate.RPM_filter_rate);
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
                    FirstOrder_LPF_Cacl(Hall_Sensor.e_speed, Hall_Sensor.e_speed_f, Filter_Rate.RPM_filter_rate);
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

    Hall_Sensor.e_rpm = Hall_Sensor.e_speed_f * foc_run.PWM_freq_now * 60 / _2PI;

    #if (FOC_DEBUG_HALL && FOC_DEBUG_HALL_USE_ANGLE) || (!FOC_DEBUG_HALL)
    foc_para.e_angle = Normalize_Angle(Hall_Sensor.e_angle_observe);
    #endif

    #if (FOC_DEBUG_HALL && FOC_DEBUG_HALL_USE_SPEED) || (!FOC_DEBUG_HALL)
    // speed
    #if FOC_SPEED_LOOP_DIV > 1
    foc_run.speed_loop_cnt++;
    if (foc_run.speed_loop_cnt >= foc_ctrl.speed_loop_div)
    {
        foc_run.speed_loop_cnt = 0;
    #endif
        foc_run.eletrical_rpm = Hall_Sensor.e_rpm;
        FirstOrder_LPF_Cacl(foc_run.eletrical_rpm, foc_run.eletrical_rpm_f, Filter_Rate.RPM_filter_rate);
        foc_run.machine_rpm = foc_run.eletrical_rpm / foc_ctrl.pole_pairs;
        foc_run.machine_rpm_f = foc_run.eletrical_rpm_f / foc_ctrl.pole_pairs;
        foc_run.eletrical_Hz = foc_run.eletrical_rpm / 60.0f;
        foc_run.eletrical_Hz_f = foc_run.eletrical_rpm_f / 60.0f;

        #if (FOC_DEBUG_HALL && FOC_DEBUG_HALL_USE_SPEED_LOOP) || (!FOC_DEBUG_HALL)
        if (foc_ctrl.speed_loop && foc_run.run_state == FOC_CloseLoopRun)
        {
            foc_para.Iq_target = PID_Position_Run(&Speed_PID, foc_para.Speed_target, foc_run.eletrical_rpm_f);
        }
        #endif
    #if FOC_SPEED_LOOP_DIV > 1
    }
    #endif
    #endif
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
            #if FOC_USE_SMO && !FOC_DEBUG_SMO
            SMO_Run(&smo_observer,&foc_para);
            foc_run.eletrical_rpm = smo_observer.E_rpm;
            foc_run.eletrical_rpm_f = foc_run.eletrical_rpm;
            foc_run.eletrical_Hz = smo_observer.E_rps;
            foc_run.eletrical_Hz_f = foc_run.eletrical_Hz;
            foc_run.machine_rpm = foc_run.eletrical_rpm_f / foc_ctrl.pole_pairs;
            foc_run.machine_rpm_f = foc_run.machine_rpm;
            #endif

            #if FOC_USE_FLO && !FOC_DEBUG_FLO
            FLO_Run(&flo_observer,&foc_para);
            foc_run.eletrical_rpm = flo_observer.E_rpm;
            foc_run.eletrical_rpm_f = foc_run.eletrical_rpm;
            foc_run.eletrical_Hz = flo_observer.E_rps;
            foc_run.eletrical_Hz_f = foc_run.eletrical_Hz;
            foc_run.machine_rpm = foc_run.eletrical_rpm_f / foc_ctrl.pole_pairs;
            foc_run.machine_rpm_f = foc_run.machine_rpm;
            #endif

            foc_para.Iq_target = foc_run.start_iq_now;

            foc_run.general_cnt++;
            foc_run.start_cnt++;
            if(foc_run.general_cnt >= foc_ctrl.speed_loop_div)
            {
                foc_run.general_cnt = 0;
                #if FOC_USE_SMO && !FOC_DEBUG_SMO
                float startup_diff_angle = fabsf(smo_observer.E_ang - foc_run.start_e_ang);
                #endif
                #if FOC_USE_FLO && !FOC_DEBUG_FLO
                float startup_diff_angle = fabsf(flo_observer.E_ang - foc_run.start_e_ang);
                #endif
                if(startup_diff_angle > _PI)
                {
                    startup_diff_angle = _2PI - startup_diff_angle;
                }
                /* rpp -> rads per period */
                #if USE_S_CURVE_ACCELERATE
                /*
                    S-shaped acceleration curve
                    y = A + B / (1 + e^(-ax + b))
                    A set to 0
                    B Determine maximum
                    a Determine the curve rise time Rise time ,roughly = 10 / a
                    b set to 5
                */
                //target rads per PWM period
                float start_target_rpp = foc_ctrl.startup_erpm * _2PI / 60 / foc_run.PWM_freq_now;
                float a = 10.0f / (foc_ctrl.startup_acc_ms * foc_run.PWM_freq_now / 1000);
                foc_run.start_rpp_now = start_target_rpp * 1.1f / (1 + expf(-a * foc_run.start_cnt + 5));
                #else
                /*
                    Trapezoidal acceleration curve
                    y = ax
                */
                //target rads per PWM period
                float start_target_rpp = foc_ctrl.startup_erpm * _2PI / 60 / foc_run.PWM_freq_now;
                //acceleration rads per speed loop             speed loop number in 1second                                   time cal
                float start_rpp_delta = (start_target_rpp / (foc_run.PWM_freq_now / foc_ctrl.speed_loop_div)) * (1000.0f / (float)foc_ctrl.startup_acc_ms);
                if(foc_run.start_rpp_now < start_target_rpp)
                {
                    foc_run.start_rpp_now += start_rpp_delta;
                }
                #endif
                #if 1
                if(foc_run.start_rpp_now >= start_target_rpp && startup_diff_angle > MAX_DIFF_ANGLE && fabsf(foc_run.start_iq_now) > foc_ctrl.startup_iq_min)
                {
                    foc_run.start_iq_now += -1 * Moto_Config.dir;
                }
                // if(foc_run.start_rpp_now >= start_target_rpp && foc_run.eletrical_rpm >= foc_ctrl.startup_erpm && fabsf(foc_run.start_iq_now) <= foc_ctrl.startup_iq_min)
                if(startup_diff_angle < MAX_DIFF_ANGLE && fabsf(foc_run.eletrical_rpm) >= foc_ctrl.erpm_min)
                #else
                else
                #endif
                {
                    if(foc_ctrl.speed_loop)
                    {
                        foc_para.Speed_target = foc_run.eletrical_rpm_f;
                    }
                    else
                    {
                        foc_para.Iq_target = foc_run.start_iq_now;
                    }
                    foc_run.run_state = FOC_CloseLoopRun;
                    foc_run.general_cnt = 0;
                }
            }

            foc_run.start_e_ang += foc_run.start_rpp_now * Moto_Config.dir;
            foc_run.start_e_ang = Normalize_Angle(foc_run.start_e_ang);
            foc_para.e_angle = foc_run.start_e_ang;
        }
        break;

        case FOC_IDLE:
        case FOC_CloseLoopRun:
        {
            #if FOC_USE_SMO
            SMO_Run(&smo_observer,&foc_para);

            #if (FOC_DEBUG_SMO && FOC_DEBUG_SMO_USE_ANGLE) || (!FOC_DEBUG_SMO)
            foc_para.e_angle = smo_observer.E_ang;
            #endif

            #if (FOC_DEBUG_SMO && FOC_DEBUG_SMO_USE_SPEED) || (!FOC_DEBUG_SMO)
            foc_run.eletrical_rpm = smo_observer.E_rpm;
            foc_run.eletrical_rpm_f = foc_run.eletrical_rpm;
            foc_run.eletrical_Hz = smo_observer.E_rps;
            foc_run.eletrical_Hz_f = foc_run.eletrical_Hz;
            foc_run.machine_rpm = foc_run.eletrical_rpm_f / foc_ctrl.pole_pairs;
            foc_run.machine_rpm_f = foc_run.machine_rpm;
            // if(foc_run.run_state == FOC_CloseLoopRun && fabsf(foc_run.eletrical_rpm_f) < foc_ctrl.erpm_min)
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
            #endif

            #if (FOC_DEBUG_SMO && FOC_DEBUG_SMO_USE_SPEED_LOOP) || (!FOC_DEBUG_SMO)
            // speed
            #if FOC_SPEED_LOOP_DIV > 1
            foc_run.speed_loop_cnt++;
            if (foc_run.speed_loop_cnt >= foc_ctrl.speed_loop_div)
            {
                foc_run.speed_loop_cnt = 0;
            #endif
                if(foc_ctrl.speed_loop && foc_run.run_state == FOC_CloseLoopRun)
                {
                    foc_para.Iq_target = PID_Position_Run(&Speed_PID, foc_para.Speed_target, foc_run.eletrical_rpm_f);
                }
            #if FOC_SPEED_LOOP_DIV > 1
            }
            #endif
            #endif
            #endif

            #if FOC_USE_FLO
            FLO_Run(&flo_observer,&foc_para);

            #if (FOC_DEBUG_FLO && FOC_DEBUG_FLO_USE_ANGLE) || (!FOC_DEBUG_FLO)
            foc_para.e_angle = flo_observer.E_ang;
            #endif

            #if (FOC_DEBUG_FLO && FOC_DEBUG_FLO_USE_SPEED) || (!FOC_DEBUG_FLO)
            foc_run.eletrical_rpm = flo_observer.E_rpm;
            foc_run.eletrical_rpm_f = foc_run.eletrical_rpm;
            foc_run.eletrical_Hz = flo_observer.E_rps;
            foc_run.eletrical_Hz_f = foc_run.eletrical_Hz;
            foc_run.machine_rpm = foc_run.eletrical_rpm_f / foc_ctrl.pole_pairs;
            foc_run.machine_rpm_f = foc_run.machine_rpm;
            // if(foc_run.run_state == FOC_CloseLoopRun && fabsf(foc_run.eletrical_rpm_f) < foc_ctrl.erpm_min)
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
            #endif

            #if (FOC_DEBUG_FLO && FOC_DEBUG_FLO_USE_SPEED_LOOP) || (!FOC_DEBUG_FLO)
            // speed
            #if FOC_SPEED_LOOP_DIV > 1
            foc_run.speed_loop_cnt++;
            if (foc_run.speed_loop_cnt >= foc_ctrl.speed_loop_div)
            {
                foc_run.speed_loop_cnt = 0;
            #endif
                if(foc_ctrl.speed_loop && foc_run.run_state == FOC_CloseLoopRun)
                {
                    foc_para.Iq_target = PID_Position_Run(&Speed_PID, foc_para.Speed_target, foc_run.eletrical_rpm_f);
                }
            #if FOC_SPEED_LOOP_DIV > 1
            }
            #endif
            #endif
            #endif
        }
        break;
    }
}

void HFI_Angle_Speed_Cali(void)
{
    if(foc_run.run_state == FOC_HFIRun)
    {
        HFI_para.HFI_cnt++;
        if (HFI_para.HFI_cnt >= HFI_para.HFI_Div)
        {
            HFI_para.HFI_cnt = 0;

            foc_para.Id_target = HFI_para.HFI_Id_target * HFI_para.HFI_Ud_sign;
            // foc_para.Id_target = foc_para.Id_target + HFI_para.HFI_Id_target * HFI_para.HFI_Ud_sign;
            HFI_para.HFI_Ud_sign *= -1;

            HFI_para.HFI_Ia_last = HFI_para.HFI_Ia_now;
            HFI_para.HFI_Ia_now = foc_para.Ia;
            HFI_para.HFI_Ib_last = HFI_para.HFI_Ib_now;
            HFI_para.HFI_Ib_now = foc_para.Ib;

            HFI_para.HFI_Ia_base = (HFI_para.HFI_Ia_now + HFI_para.HFI_Ia_last) / 2;
            HFI_para.HFI_Ia_delta = (HFI_para.HFI_Ia_now - HFI_para.HFI_Ia_last) / 2;
            HFI_para.HFI_Ib_base = (HFI_para.HFI_Ib_now + HFI_para.HFI_Ib_last) / 2;
            HFI_para.HFI_Ib_delta = (HFI_para.HFI_Ib_now - HFI_para.HFI_Ib_last) / 2;

            HFI_para.HFI_e_angle = Normalize_Angle(atan2f(HFI_para.HFI_Ud_sign * HFI_para.HFI_Ib_delta,HFI_para.HFI_Ud_sign * HFI_para.HFI_Ia_delta));

            foc_para.e_angle = HFI_para.HFI_e_angle;
        }
    }
}

void FOC_Process(void)
{
    #if FOC_CURRENT_LOOP_DIV > 1
    foc_run.current_loop_cnt++;
    if (foc_run.current_loop_cnt >= foc_ctrl.current_loop_div)
    {
        foc_run.current_loop_cnt = 0;
    #endif

        #if 1
        Clarke_Transmission(phase_voltage_f[0], phase_voltage_f[1], phase_voltage_f[2], &foc_para.Ua, &foc_para.Ub);
        Clarke_Transmission(phase_current_f[0], phase_current_f[1], phase_current_f[2], &foc_para.Ia, &foc_para.Ib);
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

        foc_para.U_ag = phase_voltage_f[0];
        foc_para.U_bg = phase_voltage_f[1];
        foc_para.U_cg = phase_voltage_f[2];
        
        foc_para.U_an = foc_para.U_ag * 0.6666666f - foc_para.U_bg * 0.3333333f - foc_para.U_cg * 0.3333333f;
        foc_para.U_bn = -foc_para.U_ag * 0.3333333f + foc_para.U_bg * 0.6666666f - foc_para.U_cg * 0.3333333f;
        foc_para.U_cn = -foc_para.U_ag * 0.3333333f - foc_para.U_bg * 0.3333333f + foc_para.U_cg * 0.6666666f;

        // foc_para.U_an = Virtual_Moto.V_bus_mv_f * (foc_para.CMP1 * 0.6666666f - foc_para.CMP2 * 0.3333333f - foc_para.CMP3 * 0.3333333f) / foc_run.TIM1_ARR_now;
        // foc_para.U_bn = Virtual_Moto.V_bus_mv_f * (-foc_para.CMP1 * 0.3333333f + foc_para.CMP2 * 0.6666666f - foc_para.CMP3 * 0.3333333f) / foc_run.TIM1_ARR_now;
        // foc_para.U_cn = Virtual_Moto.V_bus_mv_f * (-foc_para.CMP1 * 0.3333333f - foc_para.CMP2 * 0.3333333f + foc_para.CMP3 * 0.6666666f) / foc_run.TIM1_ARR_now;

        foc_para.I_a = phase_current_f[0];
        foc_para.I_b = phase_current_f[1];
        foc_para.I_c = phase_current_f[2];

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
        if (foc_ctrl.sensor_type == SENSOR_LESS)
        #endif
        {
            SensorLess_Angle_Speed_Cali();
        }

        if (foc_ctrl.sensor_type == SENSOR_LESS_HFI)
        {
            HFI_Angle_Speed_Cali();
        }

        Virtual_Moto.electronic_speed_hz = fabsf(foc_run.eletrical_Hz_f);

        Park_Transmission(foc_para.Ia, foc_para.Ib, &foc_para.Id, &foc_para.Iq, foc_para.e_angle);
        Park_Transmission(foc_para.Ua, foc_para.Ub, &foc_para.Ud, &foc_para.Uq, foc_para.e_angle);
        #if USE_MTPA
        MTPA_Cal(&foc_para,FLUXWb,DAxisInd,QAxisInd);
        #endif
        if(foc_ctrl.current_loop && foc_run.is_running)
        {
            foc_para.Ud_rate = PID_Position_Run(&ID_PID, foc_para.Id_target, foc_para.Id);
            foc_para.Uq_rate = PID_Position_Run(&IQ_PID, foc_para.Iq_target, foc_para.Iq);
        }
        Inverse_Park_Transmission(foc_para.Ud_rate, foc_para.Uq_rate, &foc_para.Ua_rate, &foc_para.Ub_rate, foc_para.e_angle);

        // modulation ratio limit
        saturate_vector_2d(&foc_para.Ua_rate, &foc_para.Ub_rate, FOC_MAX_MODULATION_RATIO);
        saturate_vector_2d(&foc_para.Ud_rate, &foc_para.Uq_rate, FOC_MAX_MODULATION_RATIO);

        // cal voltage vector and current vector
        foc_para.Us = sqrtf(SQ(foc_para.Ua) + SQ(foc_para.Ub));
        FirstOrder_LPF_Cacl(foc_para.Us, foc_para.Us_f, 0.3f);
        foc_para.Us_rate = sqrtf(SQ(foc_para.Ud_rate) + SQ(foc_para.Uq_rate));
        FirstOrder_LPF_Cacl(foc_para.Us_rate, foc_para.Us_rate_f, 0.3f);
        foc_para.Is = sqrtf(SQ(foc_para.Id) + SQ(foc_para.Iq));
        FirstOrder_LPF_Cacl(foc_para.Is, foc_para.Is_f, 0.3f);

        // angle compensate
        foc_para.e_angle += foc_run.eletrical_Hz_f / FOC_CC_LOOP_FREQ;
        foc_para.e_angle = Normalize_Angle(foc_para.e_angle);

    #if FOC_CURRENT_LOOP_DIV > 1
    }
    #endif

    if (foc_run.is_running)
    {
        SVPWM_Update(foc_para.Ua_rate,foc_para.Ub_rate,foc_para.Ud_rate,foc_para.Uq_rate,foc_para.e_angle);
    }

    debug_arr[0] = foc_para.Id_target;
    debug_arr[1] = foc_para.Id;
    debug_arr[2] = foc_para.Iq_target;
    debug_arr[3] = foc_para.Iq;
    debug_arr[4] = foc_para.Ia;
    debug_arr[5] = foc_para.Ib;
    debug_arr[6] = foc_para.e_angle;
    // debug_arr[6] = foc_run.eletrical_Hz_f;

    // debug_arr[7] = flo_observer.pll.theta;
    // debug_arr[8] = flo_observer.Theta;
    // debug_arr[9] = flo_observer.R_I_a;
    // debug_arr[10] = flo_observer.R_I_b;
    // debug_arr[11] = flo_observer.flux_a;
    // debug_arr[12] = flo_observer.flux_b;
    // debug_arr[13] = flo_observer.flux_s_a;
    // debug_arr[14] = flo_observer.flux_s_b;
    // debug_arr[15] = flo_observer.flux_r_a;
    // debug_arr[16] = flo_observer.flux_r_b;
    // debug_arr[17] = flo_observer.flux_r_err;

    // debug_arr[7] = smo_observer.Theta;
    // debug_arr[8] = smo_observer.EstIalpha;
    // debug_arr[9] = smo_observer.EstIbeta;
    // debug_arr[10] = smo_observer.EalphaFinal;
    // debug_arr[11] = smo_observer.EbetaFinal;
    // debug_arr[12] = smo_observer.Ealpha;
    // debug_arr[13] = smo_observer.Ebeta;
    // debug_arr[14] = smo_observer.Zalpha;
    // debug_arr[15] = smo_observer.Zbeta;
    // debug_arr[16] = smo_observer.IalphaError;
    // debug_arr[17] = smo_observer.IbetaError;

    debug_arr[7] = foc_para.Ua_rate;
    debug_arr[8] = foc_para.Ub_rate;
    debug_arr[9] = foc_para.Ud_rate;
    debug_arr[10] = foc_para.Uq_rate;
    debug_arr[11] = foc_para.Us_rate;
    debug_arr[12] = foc_para.Us_rate_f;
    debug_arr[13] = foc_para.Speed_target;
    debug_arr[14] = foc_run.eletrical_rpm_f;

    // debug_arr[17] = smo_observer.Theta;
    // debug_arr[18] = flo_observer.pll.theta;
    // debug_arr[18] = Normalize_Angle(Hall_Sensor.e_angle_observe);
    debug_arr[19] = AS5048_para.e_angle;
    // debug_arr[19] = Normalize_Angle(Hall_Sensor.e_angle_observe);

    // debug_arr[17] = foc_para.Speed_target;
    // debug_arr[18] = foc_run.eletrical_rpm;
    // debug_arr[19] = foc_run.eletrical_rpm_f;

    CDC_Transmit_FS((uint8_t *)debug_arr,sizeof(debug_arr));
}

void Start_FOC_Motor(void)
{
    TIM1->CCER |= 0x555;
    #if USE_DT_COMPENSATION
    foc_para.dt_compensation_value = Dead_Time_Compensation_Cal(PWM_TIM_BASE_FREQ,100,1);
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
    if (foc_ctrl.sensor_type == SENSOR_LESS)
    #endif
    {
        foc_run.general_cnt = 0;
        foc_run.start_iq_now = foc_ctrl.startup_iq_max * Moto_Config.dir;
        foc_run.start_rpp_now = 0;
        foc_run.start_e_ang = _3PI_2;
        foc_para.Id_target = 0;
        foc_para.Iq_target = 0;

        #if FOC_USE_SMO
        #if !FOC_DEBUG_SMO
        foc_run.run_state = FOC_Order;
        #endif
        #endif

        #if FOC_USE_FLO
        #if !FOC_DEBUG_FLO
        // foc_run.run_state = FOC_Order;
        foc_run.run_state = FOC_CloseLoopRun;
        #endif
        #endif
    }

    if (foc_ctrl.sensor_type == SENSOR_LESS_HFI)
    {
        foc_run.run_state = FOC_HFIRun;
        foc_para.Ud_rate = 0;
        foc_para.Uq_rate = 0;
        foc_para.Id_target = 0;
        foc_para.Iq_target = 0;

        HFI_para.HFI_Div = 10;
        HFI_para.HFI_Id_target = 500;
        HFI_para.HFI_Ud_sign = 1;
    }

    foc_run.is_running = 1;

}

void Stop_FOC_Motor(void)
{
    TIM1->CCER &= ~0x555;
    foc_run.is_running = 0;
    foc_run.run_state = FOC_IDLE;
    memset(&foc_para,0,sizeof(foc_para));
    PID_Position_Reset(&ID_PID);
    PID_Position_Reset(&IQ_PID);
    PID_Position_Reset(&Speed_PID);
}

void Init_FOC_Motor(void)
{
    foc_run.PWM_freq_now = foc_ctrl.PWM_freq;
    foc_run.TIM1_ARR_now = PWM_TIM_BASE_FREQ / 2 / foc_run.PWM_freq_now;
    Virtual_Moto.sample_freq_now = foc_run.PWM_freq_now;
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

    #if FOC_USE_SMO
    SMO_Init(&smo_observer);
    #endif

    #if FOC_USE_FLO
    FLO_Init(&flo_observer);
    #endif

    PID_Position_Init(&ID_PID,ID_TS,ID_KP,ID_KI,ID_KD,ID_KB,ID_LP,ID_LN);
    PID_Position_Init(&IQ_PID,IQ_TS,IQ_KP,IQ_KI,IQ_KD,IQ_KB,IQ_LP,IQ_LN);
    PID_Position_Init(&Speed_PID,SPEED_TS,SPEED_KP,SPEED_KI,SPEED_KD,SPEED_KB,SPEED_LP,SPEED_LN);
}
