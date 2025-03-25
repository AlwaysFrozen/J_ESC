#include "Board_Config.h"
#include "main.h"
#include "FOC_Motor.h"
#include "FOC_Config.h"
#include "FOC.h"
#include "PLL.h"
#include "SMO.h"
#include "NFO.h"
#include "VIS.h"

// Debug Data
extern DAC_HandleTypeDef hdac;
extern float debug_arr[DEBUG_ARR_CNT];

extern uint8_t Dead_Time_Cal(uint32_t TIM_Clock,uint8_t CKD,uint32_t ns);
extern uint16_t Dead_Time_Compensation_Cal(uint32_t TIM_Clock,uint32_t ns,uint8_t centeraligned);

FOC_Para_t foc_para;
FOC_RUN_t foc_run;
FOC_CONTROL_t foc_ctrl =
{
    .pTIM = TIM1,
    .motor_type = SPM,
    .sensor_type = Sensor_Type,
    .pole_pairs = PolePairs,
    .max_current = Motor_Max_Current,
    .max_rpm = Motor_Max_RPM,
    .Rs = Rs_R,
    .Ld = Ld_H,
    .Lq = Lq_H,
    .Ldiff = (Ld_H - Lq_H),
    .Ls = Ls_H,
    .Flux = FLUX_Wb,
    .startup_erpm = StartUpErpm,
    .startup_diff_angle_max = MAX_DIFF_ANGLE,
    .startup_iq_max = StartIQMax,
    .startup_iq_min = StartIQMin,
    .startup_order_ms = OrderMs,
    .startup_acc_ms = ACCMs,
    .startup_stable_ms = 10,
    .erpm_min = MinErpm,
    .svpwm_update_en = 0,
    .speed_loop_en = SPEED_LOOP_ENABLE,
    .position_loop_en = POSITION_LOOP_ENABLE,
    .current_loop_freq = FOC_CC_LOOP_FREQ,
    .speed_loop_freq = FOC_SC_LOOP_FREQ,
    .position_loop_freq = FOC_PC_LOOP_FREQ,
    .current_loop_div = FOC_CURRENT_LOOP_DIV,
    .speed_loop_div = FOC_SPEED_LOOP_DIV,
    .position_loop_div = FOC_POSITION_LOOP_DIV,
    .current_loop_dt = FOC_CC_LOOP_DT,
    .speed_loop_dt = FOC_SC_LOOP_DT,
    .position_loop_dt = FOC_PC_LOOP_DT,
    .modulation_ratio = FOC_MAX_MODULATION_RATIO,
    .PWM_freq = FOC_PWM_FREQ,
    .decoupling_mode = (FOC_Decoupling_Mode_t)Decoupling_Mode,
    .deadtime_compensate_enable = DT_COMPENSATION_ENABLE,
    .MTPA_enable = MTPA_ENABLE,
    .FW_enable = FW_ENABLE,
    .FW_throttle = 0.95f,
    .MTPV_enable = MTPV_ENABLE,
};
PID_t ID_PID;
PID_t IQ_PID;
PID_t Speed_PID;
PID_t Position_PID;

HFI_CONTROL_t HFI_ctrl = 
{
    .HFI_high_freq = 10000,
    .HFI_high_amplitude = 2,
    .HFI_pole_detect_amplitude = 2,
    .HFI_pole_detect_delay_us = 10000,
    .HFI_pole_detect_pluse_us = 500,
    .HFI_pole_detect_pluse_interval_us = 1000,
    .HFI_switch_on_speed = 5.0f,
    .HFI_switch_off_speed = 8.0f,
};

HFI_RUN_t HFI_run = 
{
    .HFI_high_freq_sign = 1,
};

SMO_t SMO_observer;
NFO_t NFO_observer;

PLL_t Encoder_PLL;
PLL_t Hall_PLL;
PLL_t HFI_PLL;

#if PLL_USE_IIR_FILTER
IIR_Filter_t Observer_PLL_IIR;
IIR_Filter_t Encoder_PLL_IIR;
IIR_Filter_t Hall_PLL_IIR;
IIR_Filter_t HFI_PLL_IIR;
#endif

float pid_wc = 0;
uint8_t reset_machine_angle = 0;

void SVPWM_Update(FOC_CONTROL_t *ctrl,FOC_RUN_t *run_parameters,FOC_Para_t *parameters,HFI_RUN_t *hfi_parameters)
{
    if (run_parameters->run_state == FOC_HFIRun)
    {
        // SVPWM_DQ_Voltage(&hfi_parameters->Udq_out,parameters->Uo_max,parameters->e_angle,&parameters->tim);
        // SVPWM_AB_Voltage(&hfi_parameters->Uab_out,parameters->Uo_max,&parameters->tim);
        SPWM_AB_Voltage(&hfi_parameters->Uab_out,parameters->V_bus,&parameters->tim);
    }
    else
    {
        // SVPWM_DQ_Voltage(&parameters->V_dq_out,parameters->Uo_max,parameters->e_angle,&parameters->tim);
        // SVPWM_AB_Voltage(&parameters->V_alpha_beta_out,parameters->Uo_max,&parameters->tim);
        SPWM_AB_Voltage(&parameters->V_alpha_beta_out,parameters->V_bus,&parameters->tim);
    }

    if(ctrl->deadtime_compensate_enable)
    {
        parameters->current_vector_angle = Normalize_Rad(parameters->e_angle + atan2f(parameters->I_dq.Q,parameters->I_dq.D));
        parameters->current_vector_sector = parameters->current_vector_angle / _PI_6;
        switch(parameters->current_vector_sector)
        {
            case 0:
            case 11:
                parameters->tim.CMP1 += parameters->dt_compensation_value;
                parameters->tim.CMP2 -= parameters->dt_compensation_value;
                parameters->tim.CMP3 -= parameters->dt_compensation_value;
                break;

            case 1:
            case 2:
                parameters->tim.CMP1 += parameters->dt_compensation_value;
                parameters->tim.CMP2 += parameters->dt_compensation_value;
                parameters->tim.CMP3 -= parameters->dt_compensation_value;
                break;

            case 3:
            case 4:
                parameters->tim.CMP1 -= parameters->dt_compensation_value;
                parameters->tim.CMP2 += parameters->dt_compensation_value;
                parameters->tim.CMP3 -= parameters->dt_compensation_value;
                break;

            case 5:
            case 6:
                parameters->tim.CMP1 -= parameters->dt_compensation_value;
                parameters->tim.CMP2 += parameters->dt_compensation_value;
                parameters->tim.CMP3 += parameters->dt_compensation_value;
                break;

            case 7:
            case 8:
                parameters->tim.CMP1 -= parameters->dt_compensation_value;
                parameters->tim.CMP2 -= parameters->dt_compensation_value;
                parameters->tim.CMP3 += parameters->dt_compensation_value;
                break;

            case 9:
            case 10:
                parameters->tim.CMP1 += parameters->dt_compensation_value;
                parameters->tim.CMP2 -= parameters->dt_compensation_value;
                parameters->tim.CMP3 += parameters->dt_compensation_value;
                break;

            default:
                break;
        }

        parameters->tim.CMP1 = CONSTRAIN_UNSIGNED(parameters->tim.CMP1, run_parameters->TIM_ARR_now);
        parameters->tim.CMP2 = CONSTRAIN_UNSIGNED(parameters->tim.CMP2, run_parameters->TIM_ARR_now);
        parameters->tim.CMP3 = CONSTRAIN_UNSIGNED(parameters->tim.CMP3, run_parameters->TIM_ARR_now);
    }

    ctrl->pTIM->CCR1 = parameters->tim.CMP1;
    ctrl->pTIM->CCR2 = parameters->tim.CMP2;
    ctrl->pTIM->CCR3 = parameters->tim.CMP3;
}

void Encoder_Angle_Speed_Cal(void)
{
    // switch(foc_ctrl.sensor_type)
    // {
    //     case IIC_ENCODER:
    //         break;

    //     case SPI_ENCODER:
            AS5048_Read_M_Ang();
    //         break;

    //     case ABI_ENCODER:
    //         break;

    //     default:
    //         break;
    // }

    // PLL_Run(&Encoder_PLL,sinf(AS5048_para.m_angle),cosf(AS5048_para.m_angle));
    Normalize_PLL_Run(&Encoder_PLL,sinf(AS5048_para.m_angle),cosf(AS5048_para.m_angle));

    #if !FOC_DEBUG_ENCODER
    if(foc_run.run_state < FOC_VF_OpenLoopRun)
    {
        foc_para.e_angle = Normalize_Rad(Encoder_PLL.theta * foc_ctrl.pole_pairs);
        foc_para.m_angle = Encoder_PLL.theta;
        foc_para.m_angle_multicycle = Encoder_PLL.theta_unlimit;

        foc_run.eletrical_Hz = Encoder_PLL.hz_f * foc_ctrl.pole_pairs;
        foc_run.eletrical_Hz_f = foc_run.eletrical_Hz;
        foc_run.eletrical_rpm = foc_run.eletrical_Hz * 60;
        foc_run.eletrical_rpm_f = foc_run.eletrical_rpm;
        foc_run.machine_rpm = Encoder_PLL.hz_f * 60;
        foc_run.machine_rpm_f = foc_run.machine_rpm;
    }
    #endif
}

void HALL_Angle_Speed_Cal(void)
{
    float up_lim = 0;
    float down_lim = 0;
    int8_t dir = 0;

    // hall
    Hall_Sensor.hall_queue_f[0] = HALL_A_READ();
    Hall_Sensor.hall_queue_f[1] = HALL_B_READ();
    Hall_Sensor.hall_queue_f[2] = HALL_C_READ();
    Hall_Sensor.hall_index = Hall_Sensor.hall_queue_f[0] + (Hall_Sensor.hall_queue_f[1] << 1) + (Hall_Sensor.hall_queue_f[2] << 2);

    if (Hall_Sensor.hall_index_last == Hall_Sensor.hall_index)
    {
        Hall_Sensor.hall_sector_cnt++;
        /* 使用最低转速限制 效果不好 减速阶段响应慢 */
        // if (Hall_Sensor.hall_sector_cnt > foc_ctrl.current_loop_freq / (1000 / (1000 / (foc_ctrl.erpm_min * 6 / 60))))
        // if (Hall_Sensor.hall_sector_cnt > foc_ctrl.current_loop_freq * 10 / foc_ctrl.erpm_min)
        /* 使用上一次计数值 减速幅值超过一定百分比则重新计算 */
        if (Hall_Sensor.hall_sector_cnt > Hall_Sensor.hall_sector_cnt_last * 1.5f)
        {
            Hall_Sensor.e_speed = SIGN_F(Hall_Sensor.e_speed) * _PI_3 / (float)Hall_Sensor.hall_sector_cnt; // rads/cycle
            // FirstOrder_LPF_Cal(Hall_Sensor.e_speed, Hall_Sensor.e_speed_f, Hall_PLL.alpha);

            Hall_Sensor.e_angle = hall_rad_calibrated[Hall_Sensor.hall_index];
            Hall_Sensor.e_angle_observe = Hall_Sensor.e_angle;
        }
        else
        {
            Hall_Sensor.e_angle_observe += Hall_Sensor.e_speed_f;

            up_lim = Hall_Sensor.e_angle + _PI_3;
            down_lim = Hall_Sensor.e_angle - _PI_3;

            Hall_Sensor.e_angle_observe = CONSTRAIN(Hall_Sensor.e_angle_observe, down_lim, up_lim);
        }
    }
    else
    {
        Hall_Sensor.hall_sector_cnt_last = Hall_Sensor.hall_sector_cnt;

        dir = Rad_Diff_Dir(hall_rad_calibrated[Hall_Sensor.hall_index_last],hall_rad_calibrated[Hall_Sensor.hall_index]);
        Hall_Sensor.e_angle_delta = ABS_Rad_Diff(hall_rad_calibrated[Hall_Sensor.hall_index_last],hall_rad_calibrated[Hall_Sensor.hall_index]);
        Hall_Sensor.e_angle = hall_rad_calibrated[Hall_Sensor.hall_index];

        if (Hall_Sensor.hall_sector_cnt)
        {
            if (Hall_Sensor.e_speed * dir >= 0)
            {
                Hall_Sensor.e_speed = dir * _PI_3 / (float)Hall_Sensor.hall_sector_cnt; // rads/cycle
            }
            else
            {
                Hall_Sensor.e_speed = -1e-6; // rads/cycle
                Hall_Sensor.e_speed_f = -1e-6;
            }
        }
        else
        {
            Hall_Sensor.e_speed = 0;
        }


        Hall_Sensor.hall_index_last = Hall_Sensor.hall_index;
        Hall_Sensor.hall_sector_cnt = 0;

        Hall_Sensor.e_angle_observe = Hall_Sensor.e_angle;
    }

    FirstOrder_LPF_Cal(Hall_Sensor.e_speed, Hall_Sensor.e_speed_f, Hall_PLL.alpha);
    Hall_Sensor.e_rpm = Hall_Sensor.e_speed_f * foc_ctrl.current_loop_freq * 60 / _2PI;

    // float e_angle = Hall_Sensor.e_angle + SIGN_F(Hall_Sensor.e_speed) * _PI_6;
    float e_angle = Hall_Sensor.e_angle_observe;
    // PLL_Run(&Hall_PLL,sinf(e_angle),cosf(e_angle));
    Normalize_PLL_Run(&Hall_PLL,sinf(e_angle),cosf(e_angle));

    #if !FOC_DEBUG_HALL
    if(foc_run.run_state < FOC_VF_OpenLoopRun)
    {
        // foc_para.e_angle = e_angle;
        // foc_run.eletrical_rpm = Hall_Sensor.e_rpm;
        // foc_run.eletrical_rpm_f = Hall_Sensor.e_rpm;
        // foc_run.eletrical_Hz = foc_run.eletrical_rpm_f / 60;
        // foc_run.eletrical_Hz_f = foc_run.eletrical_Hz;

        foc_para.e_angle = Hall_PLL.theta;
        foc_run.eletrical_Hz = Hall_PLL.hz;
        foc_run.eletrical_Hz_f = Hall_PLL.hz_f;
        foc_run.eletrical_rpm = Hall_PLL.hz * 60;
        foc_run.eletrical_rpm_f = Hall_PLL.hz_f * 60;

        foc_run.machine_rpm = foc_run.eletrical_rpm / foc_ctrl.pole_pairs;
        foc_run.machine_rpm_f = foc_run.eletrical_rpm_f / foc_ctrl.pole_pairs;
    }
    #endif
}

// https://zhuanlan.zhihu.com/p/150779067
void HFI_Angle_Speed_Cal(void)
{
    if (++HFI_run.HFI_high_freq_cnt >= foc_ctrl.current_loop_freq / HFI_ctrl.HFI_high_freq / 2)
    {
        HFI_run.HFI_high_freq_cnt = 0;

        // HFI_run.HFI_Ud_excitation = HFI_ctrl.HFI_high_amplitude * HFI_run.HFI_high_freq_sign;
        HFI_run.HFI_Ud_excitation = HFI_ctrl.HFI_high_amplitude * HFI_run.HFI_high_freq_sign + HFI_ctrl.HFI_pole_detect_amplitude * HFI_run.HFI_pole_detect_sign;
        
        // voltage
        memcpy(&HFI_run.Uab_k2,&HFI_run.Uab_k1,sizeof(AB_Axis_t));
        memcpy(&HFI_run.Uab_k1,&HFI_run.Uab_k0,sizeof(AB_Axis_t));
        memcpy(&HFI_run.Uab_k0,&foc_para.V_alpha_beta,sizeof(AB_Axis_t));

        // HFI_run.Iab_base.Alpha = (HFI_run.Uab_k0.Alpha + HFI_run.Uab_k1.Alpha) / 2;
        // HFI_run.Iab_base.Beta = (HFI_run.Uab_k0.Beta + HFI_run.Uab_k1.Beta) / 2;
        // HFI_run.Iab_response.Alpha = (HFI_run.Uab_k0.Alpha - HFI_run.Uab_k1.Alpha) / 2;
        // HFI_run.Iab_response.Beta = (HFI_run.Uab_k0.Beta - HFI_run.Uab_k1.Beta) / 2;
        
        HFI_run.Uab_response.Alpha = (HFI_run.Uab_k0.Alpha - 2 * HFI_run.Uab_k1.Alpha + HFI_run.Uab_k2.Alpha) / 4;
        HFI_run.Uab_response.Beta = (HFI_run.Uab_k0.Beta - 2 * HFI_run.Uab_k1.Beta + HFI_run.Uab_k2.Beta) / 4;
        HFI_run.Uab_base.Alpha = (HFI_run.Uab_k0.Alpha + 2 * HFI_run.Uab_k1.Alpha + HFI_run.Uab_k2.Alpha) / 4;
        HFI_run.Uab_base.Beta = (HFI_run.Uab_k0.Beta + 2 * HFI_run.Uab_k1.Beta + HFI_run.Uab_k2.Beta) / 4;

        // current
        memcpy(&HFI_run.Iab_k2,&HFI_run.Iab_k1,sizeof(AB_Axis_t));
        memcpy(&HFI_run.Iab_k1,&HFI_run.Iab_k0,sizeof(AB_Axis_t));
        memcpy(&HFI_run.Iab_k0,&foc_para.I_alpha_beta,sizeof(AB_Axis_t));

        // HFI_run.Iab_base.Alpha = (HFI_run.Iab_k0.Alpha + HFI_run.Iab_k1.Alpha) / 2;
        // HFI_run.Iab_base.Beta = (HFI_run.Iab_k0.Beta + HFI_run.Iab_k1.Beta) / 2;
        // HFI_run.Iab_response.Alpha = (HFI_run.Iab_k0.Alpha - HFI_run.Iab_k1.Alpha) / 2;
        // HFI_run.Iab_response.Beta = (HFI_run.Iab_k0.Beta - HFI_run.Iab_k1.Beta) / 2;
        
        HFI_run.Iab_response.Alpha = (HFI_run.Iab_k0.Alpha - 2 * HFI_run.Iab_k1.Alpha + HFI_run.Iab_k2.Alpha) / 4;
        HFI_run.Iab_response.Beta = (HFI_run.Iab_k0.Beta - 2 * HFI_run.Iab_k1.Beta + HFI_run.Iab_k2.Beta) / 4;
        HFI_run.Iab_base.Alpha = (HFI_run.Iab_k0.Alpha + 2 * HFI_run.Iab_k1.Alpha + HFI_run.Iab_k2.Alpha) / 4;
        HFI_run.Iab_base.Beta = (HFI_run.Iab_k0.Beta + 2 * HFI_run.Iab_k1.Beta + HFI_run.Iab_k2.Beta) / 4;

        // // HFI_ctrl.theta = Normalize_Rad(atan2f(HFI_run.HFI_high_freq_sign * HFI_run.Iab_response.Beta,HFI_run.HFI_high_freq_sign * HFI_run.Iab_response.Alpha));
        // HFI_ctrl.theta = Normalize_Rad(atan2f(-HFI_run.HFI_high_freq_sign * HFI_run.Iab_response.Beta,-HFI_run.HFI_high_freq_sign * HFI_run.Iab_response.Alpha) - _PI);

        // PLL
        // PLL_Run(&HFI_PLL,HFI_run.HFI_high_freq_sign * HFI_run.Iab_response.Beta,HFI_run.HFI_high_freq_sign * HFI_run.Iab_response.Alpha);
        Normalize_PLL_Run(&HFI_PLL,HFI_run.HFI_high_freq_sign * HFI_run.Iab_response.Beta,HFI_run.HFI_high_freq_sign * HFI_run.Iab_response.Alpha);
        HFI_run.theta = HFI_PLL.theta;


        HFI_run.HFI_high_freq_sign *= -1;

        HFI_run.HFI_pole_detect_cnt++;
        switch(HFI_run.HFI_pole_detect_status)
        {
            case HFI_Pole_Detect_Wait_Theta_Stable:
                if(HFI_run.HFI_pole_detect_cnt >= HFI_run.HFI_pole_detect_delay_cnt)
                {
                    HFI_run.HFI_pole_detect_sign = 1;
                    HFI_run.HFI_pole_detect_status++;
                    HFI_run.HFI_pole_detect_cnt = 0;
                }
                break;
            
            case HFI_Pole_Detect_Positive_Pluse:
                if(HFI_run.HFI_pole_detect_cnt >= HFI_run.HFI_pole_detect_pluse_cnt)
                {
                    HFI_run.HFI_pole_detect_sign = 0;
                    HFI_run.HFI_pole_detect_status++;
                    HFI_run.HFI_pole_detect_cnt = 0;
                    // Park_Transmission(&HFI_run.Iab_response, &foc_para.I_dq, HFI_run.theta);
                    Park_Transmission(&foc_para.I_alpha_beta, &foc_para.I_dq, HFI_run.theta);
                    HFI_run.HFI_Id_P = fabsf(foc_para.I_dq.D);
                }
                break;

            case HFI_Pole_Detect_Wait_Current_Stable:
                if(HFI_run.HFI_pole_detect_cnt >= HFI_run.HFI_pole_detect_pluse_interval_cnt)
                {
                    HFI_run.HFI_pole_detect_sign = -1;
                    HFI_run.HFI_pole_detect_status++;
                    HFI_run.HFI_pole_detect_cnt = 0;
                }
                break;

            case HFI_Pole_Detect_Negative_Pluse:
                if(HFI_run.HFI_pole_detect_cnt >= HFI_run.HFI_pole_detect_pluse_cnt)
                {
                    HFI_run.HFI_pole_detect_sign = 0;
                    HFI_run.HFI_pole_detect_cnt = 0;
                    // Park_Transmission(&HFI_run.Iab_response, &foc_para.I_dq, HFI_run.theta);
                    Park_Transmission(&foc_para.I_alpha_beta, &foc_para.I_dq, HFI_run.theta);
                    HFI_run.HFI_Id_N = fabsf(foc_para.I_dq.D);
                    if((fabsf(HFI_run.HFI_Id_P - HFI_run.HFI_Id_N) / ((HFI_run.HFI_Id_P + HFI_run.HFI_Id_N) / 2)) < 0.1f)
                    {
                        HFI_run.HFI_pole_detect_status = HFI_Pole_Detect_Retry;
                    }
                    else
                    {
                        HFI_run.HFI_pole_detect_status = HFI_Pole_Detect_Done;
                        if(HFI_run.HFI_Id_P < HFI_run.HFI_Id_N)
                        {
                            HFI_PLL.theta += _PI;
                            HFI_PLL.theta = Normalize_Rad(HFI_PLL.theta);
                            HFI_run.theta = HFI_PLL.theta;
                        }
                     }
                }
                break;

            case HFI_Pole_Detect_Retry:
                if(HFI_run.HFI_pole_detect_cnt >= HFI_run.HFI_pole_detect_pluse_interval_cnt)
                {
                    HFI_run.HFI_pole_detect_sign = 1;
                    HFI_run.HFI_pole_detect_status = HFI_Pole_Detect_Positive_Pluse;
                    HFI_run.HFI_pole_detect_cnt = 0;
                }
                break;

            default:
                // if((SIGN_F(foc_para.I_dq_target.Q) != SIGN_F(HFI_PLL.hz_f)) && (fabsf(HFI_PLL.hz_f) > HFI_ctrl.HFI_switch_on_speed))
                // {
                //     HFI_run.HFI_pole_detect_sign = 0;
                //     HFI_run.HFI_pole_detect_status = HFI_Pole_Detect_Retry;
                //     HFI_run.HFI_pole_detect_cnt = 0;
                // }
                break;
        }
    }
    
    // memcpy(&foc_para.V_alpha_beta,&foc_para.V_alpha_beta_out,sizeof(AB_Axis_t));
    memcpy(&foc_para.V_alpha_beta,&HFI_run.Uab_base,sizeof(AB_Axis_t));
    memcpy(&foc_para.I_alpha_beta,&HFI_run.Iab_base,sizeof(AB_Axis_t));
}

void SensorLess_Angle_Speed_Cal(void)
{
    #if FOC_SMO_ENABLE
    SMO_Run(&SMO_observer,&foc_para);

    #if !FOC_DEBUG_SMO
    if(foc_run.run_state < FOC_VF_OpenLoopRun)
    {
        foc_para.e_angle = SMO_observer.E_ang;
        foc_run.eletrical_rpm = SMO_observer.E_rpm;
        foc_run.eletrical_rpm_f = foc_run.eletrical_rpm;
        foc_run.eletrical_Hz = SMO_observer.E_rps;
        foc_run.eletrical_Hz_f = foc_run.eletrical_Hz;
        foc_run.machine_rpm = foc_run.eletrical_rpm_f / foc_ctrl.pole_pairs;
        foc_run.machine_rpm_f = foc_run.machine_rpm;
    }
    #endif
    #endif

    #if FOC_NFO_ENABLE
    NFO_Run(&NFO_observer,&foc_para);

    #if !FOC_DEBUG_FLO
    if(foc_run.run_state < FOC_VF_OpenLoopRun)
    {
        foc_para.e_angle = NFO_observer.E_ang;
        foc_run.eletrical_rpm = NFO_observer.E_rpm;
        foc_run.eletrical_rpm_f = foc_run.eletrical_rpm;
        foc_run.eletrical_Hz = NFO_observer.E_rps;
        foc_run.eletrical_Hz_f = foc_run.eletrical_Hz;
        foc_run.machine_rpm = foc_run.eletrical_rpm_f / foc_ctrl.pole_pairs;
        foc_run.machine_rpm_f = foc_run.machine_rpm;
    }
    #endif
    #endif

    switch(foc_run.run_state)
    {
        case FOC_Order://order in 0 degree
        {
            foc_para.I_dq_target.Q += (float)foc_run.start_iq_now / (foc_ctrl.current_loop_freq * foc_ctrl.startup_order_ms / 2 / 1000);
            if(fabsf(foc_para.I_dq_target.Q) > fabsf(foc_run.start_iq_now))
            {
                foc_para.I_dq_target.Q = foc_run.start_iq_now;
            }
            foc_para.e_angle = foc_run.start_e_ang;
            
            // if(foc_run.general_cnt < foc_ctrl.current_loop_freq / (1000.0f / (float)foc_ctrl.startup_order_ms))
            if(foc_run.general_cnt < foc_ctrl.current_loop_freq * foc_ctrl.startup_order_ms / 1000)
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
            foc_para.I_dq_target.Q = foc_run.start_iq_now;

            foc_run.start_cnt++;

            foc_run.startup_diff_angle = ABS_Rad_Diff(foc_para.e_angle,foc_run.start_e_ang);

            #if USE_S_CURVE_ACCELERATE
            /*
                S型加速曲线
                y = A + B / (1 + e^(-ax + b))
                A 无用设为0
                B 决定曲线最大值
                a 决定曲线上升时间 上升时间大致 = 10 / a
                b 固定为5
            */
            float a = 10.0f / (foc_ctrl.startup_acc_ms * foc_ctrl.current_loop_freq / 1000);
            foc_run.startup_erpm_now = Motor_Config.dir * foc_ctrl.startup_erpm / (1 + expf(-a * foc_run.start_cnt + 5));
            #else
            /*
                梯形加速曲线
                y = ax
            */
            foc_run.startup_erpm_now = Motor_Config.dir * foc_ctrl.startup_erpm * foc_run.start_cnt * 1000 / foc_ctrl.current_loop_freq / foc_ctrl.startup_acc_ms;
            #endif

            foc_run.startup_erpm_now = CONSTRAIN(foc_run.startup_erpm_now,-foc_ctrl.startup_erpm,foc_ctrl.startup_erpm);
            foc_run.startup_target_rads_per_cycle = foc_run.startup_erpm_now * _2PI / 60 / foc_ctrl.current_loop_freq;
            
            if(fabsf(foc_run.startup_erpm_now) * 0.9f > fabsf(foc_run.eletrical_rpm_f) || SIGN_F(foc_run.eletrical_rpm_f) != Motor_Config.dir)
            {
                foc_run.start_cnt--;
                if(foc_run.startup_erpm_now >= foc_ctrl.erpm_min)
                {
                    foc_run.start_cnt *= 0.9999f;
                }
                if(fabsf(foc_run.start_iq_now) < foc_ctrl.startup_iq_max)
                {
                    foc_run.start_iq_now *= 1.0001f; // ((20000,7.38) @ y = 1.0001 ^ x)
                }
                foc_run.general_cnt = 0;
            }
            else if(foc_run.startup_diff_angle > foc_ctrl.startup_diff_angle_max)
            {
                if(fabsf(foc_run.startup_erpm_now) >= foc_ctrl.erpm_min && fabsf(foc_run.start_iq_now) > foc_ctrl.startup_iq_min)
                {
                    foc_run.start_iq_now *= 0.999f; // ((20000,0.36787) @ y = 0.99995 ^ x)
                }
                foc_run.general_cnt = 0;
            }
            else if(fabsf(foc_run.eletrical_rpm_f) >= foc_ctrl.erpm_min && SIGN_F(foc_run.eletrical_rpm_f) == Motor_Config.dir)
            {
                if(foc_run.general_cnt++ > foc_ctrl.current_loop_freq * foc_ctrl.startup_stable_ms / 1000)// 10ms
                {
                    if(foc_ctrl.speed_loop_en)
                    {
                        foc_para.Speed_target = foc_run.machine_rpm_f;
                        Speed_PID.Integral = foc_run.start_iq_now;
                        Speed_PID.out = foc_run.start_iq_now;
                    }

                    foc_para.I_dq_target.Q = foc_run.start_iq_now;
                    foc_run.run_state = FOC_CloseLoopRun;
                    foc_run.general_cnt = 0;
                }
            }


            foc_run.start_e_ang += foc_run.startup_target_rads_per_cycle;
            foc_run.start_e_ang = Normalize_Rad(foc_run.start_e_ang);
            foc_para.e_angle = foc_run.start_e_ang;
        }
        break;

        case FOC_HFIRun:
        {
            HFI_Angle_Speed_Cal();
        }
        case FOC_IDLE:
        case FOC_CloseLoopRun:
        {
            /* stall protection */
            // if(foc_ctrl.sensor_type == SENSOR_LESS && foc_run.run_state == FOC_CloseLoopRun && fabsf(foc_run.eletrical_rpm_f) < foc_ctrl.erpm_min)
            // {
            //     if(foc_run.general_cnt++ > foc_ctrl.current_loop_freq / 5)
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
                if(foc_run.run_state == FOC_HFIRun && 
                    ((fabsf(HFI_PLL.hz_f) < HFI_ctrl.HFI_switch_off_speed || fabsf(foc_run.eletrical_Hz_f) < HFI_ctrl.HFI_switch_off_speed || ABS_Rad_Diff(foc_para.e_angle,HFI_run.theta) > foc_ctrl.startup_diff_angle_max) ||
                    foc_para.I_dq_target.Q == 0))
                {
                    foc_para.e_angle = HFI_PLL.theta;
                    foc_run.eletrical_Hz = HFI_PLL.hz;
                    foc_run.eletrical_Hz_f = HFI_PLL.hz_f;
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
                    }
                    else if(foc_run.run_state == FOC_CloseLoopRun && fabsf(foc_run.eletrical_Hz_f) < HFI_ctrl.HFI_switch_on_speed)
                    {
                        foc_run.run_state = FOC_HFIRun;
                        memset(&HFI_run,0,sizeof(HFI_run));
                        HFI_run.HFI_high_freq_sign = 1;
                        HFI_run.HFI_pole_detect_status = HFI_Pole_Detect_Wait_Theta_Stable;
                        HFI_run.HFI_pole_detect_sign = 0;
                        HFI_run.HFI_pole_detect_delay_cnt = HFI_ctrl.HFI_high_freq * HFI_ctrl.HFI_pole_detect_delay_us / 1000000;
                        HFI_run.HFI_pole_detect_pluse_cnt = HFI_ctrl.HFI_high_freq * HFI_ctrl.HFI_pole_detect_pluse_us / 1000000;                    
                        HFI_run.HFI_pole_detect_pluse_interval_cnt = HFI_ctrl.HFI_high_freq * HFI_ctrl.HFI_pole_detect_pluse_interval_us / 1000000;                    
                        HFI_run.HFI_pole_detect_cnt = 0;
                    }
                }
            }
        }
        break;

        default:
            break;
    }
}

void Open_Loop_Angle_Speed_Cal(void)
{
    foc_run.machine_rpm = foc_para.Speed_target;
    foc_run.machine_rpm_f = foc_run.machine_rpm;
    foc_run.eletrical_rpm = foc_run.machine_rpm * foc_ctrl.pole_pairs;
    foc_run.eletrical_rpm_f = foc_run.eletrical_rpm;
    foc_run.eletrical_Hz = foc_run.eletrical_rpm / 60;
    foc_run.eletrical_Hz_f = foc_run.eletrical_Hz;
    foc_para.e_angle += foc_run.eletrical_Hz * _2PI / foc_ctrl.current_loop_freq;
    foc_para.e_angle = Normalize_Rad(foc_para.e_angle);
}

void FOC_Process(void)
{
    timer_cnt_arr[4] = TIM1->CNT;
    if((TIM1->CR1 & 0x10) == 0x10)
    {
        timer_cnt_arr[4] = TIM1->ARR * 2 - timer_cnt_arr[4];
    }

    // if(pid_wc != 0)
    // {
    //     Current_Loop_Parallel_PID_Tune(&ID_PID,pid_wc,foc_ctrl.Ld,foc_ctrl.Rs);
    //     Current_Loop_Parallel_PID_Tune(&IQ_PID,pid_wc,foc_ctrl.Lq,foc_ctrl.Rs);
    //     pid_wc = 0;
    // }

    // if(reset_machine_angle)
    // {
    //     reset_machine_angle = 0;
    //     Reset_Machine_Angle_To_Zero();
    // }
    
    if(foc_run.run_state == FOC_CloseLoopRun || foc_run.run_state == FOC_HFIRun)
    {
        if(foc_ctrl.position_loop_en)
        {
            #if FOC_POSITION_LOOP_DIV > 1
            if (++foc_run.position_loop_cnt >= foc_ctrl.position_loop_div)
            {
                foc_run.position_loop_cnt = 0;
            #endif
                foc_para.Speed_target = Parallel_PID_Position_Run(&Position_PID, foc_para.Position_target, foc_para.m_angle_multicycle);
            #if FOC_POSITION_LOOP_DIV > 1
            }
            #endif 
        }

        if(foc_ctrl.speed_loop_en)
        {
            #if FOC_SPEED_LOOP_DIV > 1
            if (++foc_run.speed_loop_cnt >= foc_ctrl.speed_loop_div)
            {
                foc_run.speed_loop_cnt = 0;
            #endif
                // foc_para.I_dq_target.Q = Parallel_PID_Position_Run(&Speed_PID, foc_para.Speed_target, foc_run.machine_rpm_f);
                foc_para.I_dq_target.Q = IP_Run(&Speed_PID, foc_para.Speed_target, foc_run.machine_rpm_f);
            #if FOC_SPEED_LOOP_DIV > 1
            }
            #endif
        }
    }

    timer_cnt_arr[5] = TIM1->CNT;
    if((TIM1->CR1 & 0x10) == 0x10)
    {
        timer_cnt_arr[5] = TIM1->ARR * 2 - timer_cnt_arr[5];
    }

    #if FOC_CURRENT_LOOP_DIV > 1
    if (++foc_run.current_loop_cnt >= foc_ctrl.current_loop_div)
    {
        foc_run.current_loop_cnt = 0;
    #endif

        // come to the same thing
        #if 1
        Clarke_Transmission(&phase_voltage_V_f, &foc_para.V_alpha_beta);
        Clarke_Transmission(&phase_current_A_f, &foc_para.I_alpha_beta);
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

        memcpy(&foc_para.V_phase_to_ground,&phase_voltage_V_f,sizeof(UVW_Axis_t));
        
        foc_para.V_phase.U = foc_para.V_phase_to_ground.U * _2_3 - foc_para.V_phase_to_ground.V * _1_3 - foc_para.V_phase_to_ground.W * _1_3;
        foc_para.V_phase.V = -foc_para.V_phase_to_ground.U * _1_3 + foc_para.V_phase_to_ground.V * _2_3 - foc_para.V_phase_to_ground.W * _1_3;
        foc_para.V_phase.W = -foc_para.V_phase_to_ground.U * _1_3 - foc_para.V_phase_to_ground.V * _1_3 + foc_para.V_phase_to_ground.W * _2_3;
        
        // foc_para.V_phase.U = Virtual_Moto.V_bus_v_f * (foc_para.tim.CMP1 * _2_3 - foc_para.tim.CMP2 * _1_3 - foc_para.tim.CMP3 * _1_3) / foc_run.TIM_ARR_now;
        // foc_para.V_phase.V = Virtual_Moto.V_bus_v_f * (-foc_para.tim.CMP1 * _1_3 + foc_para.tim.CMP2 * _2_3 - foc_para.tim.CMP3 * _1_3) / foc_run.TIM_ARR_now;
        // foc_para.V_phase.W = Virtual_Moto.V_bus_v_f * (-foc_para.tim.CMP1 * _1_3 - foc_para.tim.CMP2 * _1_3 + foc_para.tim.CMP3 * _2_3) / foc_run.TIM_ARR_now;

        memcpy(&foc_para.I_phase,&phase_current_A_f,sizeof(UVW_Axis_t));

        Clarke_Transmission(&foc_para, &foc_para.V_alpha_beta);
        Clarke_Transmission(&foc_para, &foc_para.I_alpha_beta);
        #endif
        
        #if !FOC_DEBUG_ENCODER
        if (foc_ctrl.sensor_type >= IIC_ENCODER)
        #endif
        {
            Encoder_Angle_Speed_Cal();
        }
            
        #if !FOC_DEBUG_HALL
        if (foc_ctrl.sensor_type == HALL_120_SENSOR || foc_ctrl.sensor_type == HALL_60_SENSOR)
        #endif
        {
            HALL_Angle_Speed_Cal();
        }
            
        #if !FOC_DEBUG_SENSORLESS
        if (foc_ctrl.sensor_type <= SENSOR_LESS_HFI)
        #endif
        {
            SensorLess_Angle_Speed_Cal();
        }

        // switch(foc_ctrl.sensor_type)
        // {
        //     case SENSOR_LESS:
        //     case SENSOR_LESS_VIS:
        //     case SENSOR_LESS_HFI:
        //         SensorLess_Angle_Speed_Cal();
        //         break;
            
        //     case HALL_120_SENSOR:
        //     case HALL_60_SENSOR:
        //         HALL_Angle_Speed_Cal();
        //         break;

        //     case SPI_ENCODER:
        //         Encoder_Angle_Speed_Cal();
        //         break;

        //     default:
        //         break;
        // }
        
        if(foc_run.run_state >= FOC_VF_OpenLoopRun)
        {
            Open_Loop_Angle_Speed_Cal();
        }

        Park_Transmission(&foc_para.V_alpha_beta, &foc_para.V_dq, foc_para.e_angle);
        Park_Transmission(&foc_para.I_alpha_beta, &foc_para.I_dq, foc_para.e_angle);

        timer_cnt_arr[6] = TIM1->CNT;
        if((TIM1->CR1 & 0x10) == 0x10)
        {
            timer_cnt_arr[6] = TIM1->ARR * 2 - timer_cnt_arr[6];
        }

        foc_para.V_bus = Virtual_Moto.V_bus_v;
        foc_para.Uo_max = foc_ctrl.modulation_ratio * foc_para.V_bus * _2_3;
        foc_run.we = foc_run.eletrical_Hz_f * _2PI;

        Virtual_Moto.electronic_speed_hz = fabsf(foc_run.eletrical_Hz_f);
        
        switch (foc_run.run_state)
        {
            case FOC_VF_OpenLoopRun:
            {
                Saturate_Vector_2d(&foc_para.V_dq_out.D, &foc_para.V_dq_out.Q, foc_para.Uo_max);
                Inverse_Park_Transmission(&foc_para.V_dq_out, &foc_para.V_alpha_beta_out, foc_para.e_angle);
            }
            break;

            case FOC_IF_OpenLoopRun:
            {
                Saturate_Vector_2d(&foc_para.I_dq_target.D, &foc_para.I_dq_target.Q, foc_ctrl.max_current);
                foc_para.V_dq_out.D = Parallel_PID_Position_Run(&ID_PID, foc_para.I_dq_target.D, foc_para.I_dq.D);
                foc_para.V_dq_out.Q = Parallel_PID_Position_Run(&IQ_PID, foc_para.I_dq_target.Q, foc_para.I_dq.Q);
                Saturate_Vector_2d(&foc_para.V_dq_out.D, &foc_para.V_dq_out.Q, foc_para.Uo_max);
                Inverse_Park_Transmission(&foc_para.V_dq_out, &foc_para.V_alpha_beta_out, foc_para.e_angle);
            }
            break;

            default:
            {
                switch(foc_para.op_area)
                {
                    case FOC_OP_ID0:
                        foc_para.I_dq_target.D = 0;
                        foc_para.I_dq_target.Q = CONSTRAIN(foc_para.I_dq_target.Q,-foc_ctrl.max_current,foc_ctrl.max_current);
                        if(IQ_PID.saturation && foc_para.V_out_rate > foc_ctrl.FW_throttle && foc_ctrl.FW_enable)
                        {
                            foc_para.op_area = FOC_OP_FW;
                        }
                        break;

                    case FOC_OP_MTPA:
                        MTPA_Cal(&foc_ctrl,&foc_para);
                        Saturate_Vector_2d(&foc_para.I_dq_target.D, &foc_para.I_dq_target.Q, foc_ctrl.max_current);
                        if(IQ_PID.saturation && foc_para.V_out_rate > foc_ctrl.FW_throttle && foc_ctrl.FW_enable)
                        {
                            foc_para.op_area = FOC_OP_FW;
                        }
                        break;

                    case FOC_OP_FW:
                        if(foc_para.V_out_rate > foc_ctrl.FW_throttle)
                        {
                            foc_para.I_dq_target.D = foc_para.I_dq_target.D + 0.001f * (foc_para.Uo_max * foc_ctrl.FW_throttle - foc_para.V_out_f);
                            float id_max = -foc_ctrl.Flux / foc_ctrl.Ld;
                            if(foc_para.I_dq_target.D < id_max)
                            {
                                foc_para.I_dq_target.D = id_max;
                            }
                        }
                        else 
                        {
                            if(foc_para.I_dq_target.D > 1.0f)
                            {
                                foc_para.I_dq_target.D *= 0.99f;
                            }
                            else 
                            {
                                foc_para.I_dq_target.D = 0;
                                if(foc_ctrl.MTPA_enable)
                                {
                                    foc_para.op_area = FOC_OP_MTPA;
                                }
                                else
                                {
                                    foc_para.op_area = FOC_OP_ID0;
                                }
                            }
                        }

                        Saturate_Vector_2d(&foc_para.I_dq_target.D, &foc_para.I_dq_target.Q, foc_ctrl.max_current);
                        break;

                    case FOC_OP_MTPV:
                        break;

                    case FOC_OP_MANUAL:
                        Saturate_Vector_2d(&foc_para.I_dq_target.D, &foc_para.I_dq_target.Q, foc_ctrl.max_current);
                        break;

                    default:
                        break;
                }

                foc_para.V_dq_cross.D = -foc_run.we * foc_ctrl.Lq * foc_para.I_dq.Q;
                foc_para.V_dq_cross.Q = foc_run.we * foc_ctrl.Ld * foc_para.I_dq.D;
                foc_para.V_befm = foc_run.we * foc_ctrl.Flux;

                switch(foc_ctrl.decoupling_mode)
                {
                    case FOC_DECOUPLING_CROSS:
                        foc_para.V_dq_feed_forward.D = foc_para.V_dq_cross.D;
                        foc_para.V_dq_feed_forward.Q = foc_para.V_dq_cross.Q;
                        break;

                    case FOC_DECOUPLING_BEMF:
                        foc_para.V_dq_feed_forward.D = 0;
                        foc_para.V_dq_feed_forward.Q = foc_para.V_befm;
                        break;

                    case FOC_DECOUPLING_CROSS_BEMF:
                        foc_para.V_dq_feed_forward.D = foc_para.V_dq_cross.D;
                        foc_para.V_dq_feed_forward.Q = foc_para.V_dq_cross.Q + foc_para.V_befm;
                        break;

                    default:
                        foc_para.V_dq_feed_forward.D = 0;
                        foc_para.V_dq_feed_forward.Q = 0;
                        break;
                }

                PID_Set_Abs_Limit(&ID_PID,foc_para.Uo_max);
                PID_Set_Abs_Limit(&IQ_PID,foc_para.Uo_max);

                foc_para.V_dq_out.D = FeedForward_Parallel_PID_Position_Run(&ID_PID, foc_para.V_dq_feed_forward.D, foc_para.I_dq_target.D, foc_para.I_dq.D);
                foc_para.V_dq_out.Q = FeedForward_Parallel_PID_Position_Run(&IQ_PID, foc_para.V_dq_feed_forward.Q, foc_para.I_dq_target.Q, foc_para.I_dq.Q);
                
                if (foc_run.run_state == FOC_HFIRun)
                {
                    HFI_run.Udq_out.D = foc_para.V_dq_out.D + HFI_run.HFI_Ud_excitation;
                    HFI_run.Udq_out.Q = foc_para.V_dq_out.Q;
                    Saturate_Vector_2d(&HFI_run.Udq_out.D, &HFI_run.Udq_out.Q, foc_para.Uo_max);
                    Inverse_Park_Transmission(&HFI_run.Udq_out, &HFI_run.Uab_out, foc_para.e_angle);
                    foc_para.V_out = NORM2_f(HFI_run.Udq_out.D,HFI_run.Udq_out.Q);
                }
                else
                {
                    Saturate_Vector_2d(&foc_para.V_dq_out.D, &foc_para.V_dq_out.Q, foc_para.Uo_max);
                    Inverse_Park_Transmission(&foc_para.V_dq_out, &foc_para.V_alpha_beta_out, foc_para.e_angle);
                    foc_para.V_out = NORM2_f(foc_para.V_dq_out.D,foc_para.V_dq_out.Q);
                }
                FirstOrder_LPF_Cal(foc_para.V_out, foc_para.V_out_f, Filter_Rate.bus_voltage_filter_rate);
                foc_para.V_out_rate = foc_para.V_out_f / foc_para.Uo_max;

                // cal voltage vector and current vector
                foc_para.Us = NORM2_f(foc_para.V_alpha_beta.Alpha,foc_para.V_alpha_beta.Beta);
                foc_para.Us_duty = foc_para.Us / foc_para.Uo_max;
                foc_para.Is = NORM2_f(foc_para.I_alpha_beta.Alpha,foc_para.I_alpha_beta.Beta);
                foc_para.Is_duty = foc_para.Is / foc_ctrl.max_current;

                // Why 1.5 and not 1.5 * 1.5
                // see https://zhuanlan.zhihu.com/p/7881303857
                // foc_para.power = foc_para.Us * foc_para.Is * 1.5f;
                foc_para.power = (foc_para.V_alpha_beta.Alpha * foc_para.I_alpha_beta.Alpha + foc_para.V_alpha_beta.Beta * foc_para.I_alpha_beta.Beta) * 1.5f;
                foc_para.I_bus = foc_para.power / foc_para.V_bus;

                // // angle compensate
                // // see https://www.zhihu.com/question/625597876/answer/3258736005
                // foc_para.e_angle += foc_run.we / foc_ctrl.current_loop_freq * 1.5f;
                // foc_para.e_angle = Normalize_Rad(foc_para.e_angle);
            }
            break;
        }
        
        if(foc_ctrl.svpwm_update_en)
        {
            SVPWM_Update(&foc_ctrl,&foc_run,&foc_para,&HFI_run);
        }
    #if FOC_CURRENT_LOOP_DIV > 1
    }
    #endif

    timer_cnt_arr[7] = TIM1->CNT;
    if((TIM1->CR1 & 0x10) == 0x10)
    {
        timer_cnt_arr[7] = TIM1->ARR * 2 - timer_cnt_arr[7];
    }

    // debug_arr[0] = phase_current_A.U;
    // debug_arr[1] = phase_current_A.V;
    // debug_arr[2] = phase_current_A.W;
    // debug_arr[3] = phase_current_A_f.U;
    // debug_arr[4] = phase_current_A_f.V;
    // debug_arr[5] = phase_current_A_f.W;

    // debug_arr[0] = timer_cnt_arr[0];
    // debug_arr[1] = timer_cnt_arr[1];
    // debug_arr[2] = timer_cnt_arr[2];
    // debug_arr[3] = timer_cnt_arr[3];
    // debug_arr[4] = timer_cnt_arr[4];
    // debug_arr[5] = timer_cnt_arr[5];
    // debug_arr[6] = timer_cnt_arr[6];
    // debug_arr[6] = timer_cnt_arr[7];
    // debug_arr[7] = timer_cnt_arr[8];
    // debug_arr[8] = timer_cnt_arr[9];

    debug_arr[0] = foc_para.I_dq_target.D;
    debug_arr[1] = foc_para.I_dq.D;
    debug_arr[2] = foc_para.I_dq_target.Q;
    debug_arr[3] = foc_para.I_dq.Q;
    debug_arr[4] = foc_para.e_angle;
    debug_arr[5] = foc_run.eletrical_Hz_f;

    // debug_arr[4] = NFO_observer.pll.theta;
    // debug_arr[5] = NFO_observer.pll.hz_f;
    // debug_arr[6] = foc_run.start_e_ang;
    // debug_arr[7] = foc_run.startup_erpm_now / 60;
    // debug_arr[8] = foc_run.start_iq_now;
    // debug_arr[9] = foc_run.startup_diff_angle;

    // debug_arr[4] = HFI_run.Iab_response.Alpha;
    // debug_arr[5] = HFI_run.Iab_response.Beta;
    // // debug_arr[6] = HFI_run.Iab_base.Alpha;
    // // debug_arr[7] = HFI_run.Iab_base.Beta;
    // debug_arr[6] = HFI_run.HFI_Id_P;
    // debug_arr[7] = HFI_run.HFI_Id_N;
    // debug_arr[8] = HFI_PLL.theta;
    // debug_arr[9] = HFI_PLL.hz_f;
    
    // debug_arr[6] = foc_para.Us;
    // debug_arr[7] = foc_para.Is;
    // debug_arr[8] = foc_para.Us_duty;
    // debug_arr[9] = foc_para.Is_duty;

    debug_arr[6] = NFO_observer.pll.theta;
    debug_arr[7] = NFO_observer.pll.hz_f;
    debug_arr[8] = HFI_run.HFI_pole_detect_status;
    debug_arr[9] = foc_run.run_state;

    // debug_arr[6] = foc_para.Uo_max;
    // debug_arr[7] = foc_para.V_dq_out.D;
    // debug_arr[8] = foc_para.V_dq_out.Q;
    // debug_arr[9] = foc_para.V_befm;

    // debug_arr[6] = SMO_observer.pll.theta;
    // debug_arr[7] = SMO_observer.pll.hz_f;

    // debug_arr[6] = Hall_Sensor.e_angle_observe;
    // debug_arr[7] = Hall_PLL.theta;

    // debug_arr[6] = foc_para.Speed_target;
    // debug_arr[7] = foc_run.machine_rpm_f;
    // debug_arr[8] = foc_para.Position_target;
    // debug_arr[9] = foc_para.m_angle_multicycle;

    // debug_arr[9] = Hall_Sensor.hall_index;
    // debug_arr[9] = Hall_PLL.theta;
    // debug_arr[9] = Normalize_Rad(Hall_Sensor.e_angle);
    // debug_arr[9] = Normalize_Rad(Hall_Sensor.e_angle_observe);

    // debug_arr[9] = Normalize_Rad(Encoder_PLL.theta * foc_ctrl.pole_pairs);

    CDC_Transmit_FS((uint8_t *)debug_arr,sizeof(debug_arr));

    timer_cnt_arr[8] = TIM1->CNT;
    if((TIM1->CR1 & 0x10) == 0x10)
    {
        timer_cnt_arr[8] = TIM1->ARR * 2 - timer_cnt_arr[8];
    }
}

void Reset_Machine_Angle_To_Zero(void)
{
    foc_para.m_angle_multicycle = Encoder_PLL.theta_unlimit = 0;
}

void Start_FOC_Motor(void)
{
    if(foc_ctrl.deadtime_compensate_enable)
    {
        foc_para.dt_compensation_value = Dead_Time_Compensation_Cal(PWM_TIM_BASE_FREQ,200,1);
    }

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
                memset(&HFI_run,0,sizeof(HFI_run));
                HFI_run.HFI_high_freq_sign = 1;
                HFI_run.HFI_pole_detect_status = HFI_Pole_Detect_Wait_Theta_Stable;
                HFI_run.HFI_pole_detect_sign = 0;
                HFI_run.HFI_pole_detect_delay_cnt = HFI_ctrl.HFI_high_freq * HFI_ctrl.HFI_pole_detect_delay_us / 1000000;
                HFI_run.HFI_pole_detect_pluse_cnt = HFI_ctrl.HFI_high_freq * HFI_ctrl.HFI_pole_detect_pluse_us / 1000000;
                HFI_run.HFI_pole_detect_pluse_interval_cnt = HFI_ctrl.HFI_high_freq * HFI_ctrl.HFI_pole_detect_pluse_interval_us / 1000000;
                HFI_run.HFI_pole_detect_cnt = 0;
                foc_para.I_dq_target.D = 0;
                foc_para.I_dq_target.Q = 0;
                foc_para.Speed_target = 0;
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
                foc_run.startup_target_rads_per_cycle = 0;
                foc_run.startup_erpm_now = 0;
                foc_run.start_e_ang = Normalize_Rad(0 - _PI_2);

                if(vis_run.VIS_credible && foc_ctrl.sensor_type == SENSOR_LESS_VIS)
                {
                    foc_run.start_e_ang = Normalize_Rad(vis_run.VIS_e_angle - _PI_2);
                    Clear_Vis();
                }

                if(fabsf(foc_run.eletrical_rpm_f) > foc_ctrl.erpm_min && Motor_Config.dir == SIGN(foc_run.eletrical_rpm_f))
                {
                    foc_run.run_state = FOC_CloseLoopRun;
                    foc_para.I_dq_target.Q = foc_ctrl.startup_iq_min;
                    if(foc_ctrl.speed_loop_en)
                    {
                        foc_para.Speed_target = foc_run.eletrical_rpm_f / foc_ctrl.pole_pairs;
                    }
                }
                else if(fabsf(foc_run.eletrical_rpm_f) > foc_ctrl.erpm_min && Motor_Config.dir == SIGN(foc_run.eletrical_rpm_f))
                {
                    foc_run.run_state = FOC_CloseLoopRun;
                    foc_para.I_dq_target.Q = -foc_ctrl.startup_iq_min;
                    if(foc_ctrl.speed_loop_en)
                    {
                        foc_para.Speed_target = foc_run.eletrical_rpm_f;
                    }
                }
                else
                {
                    foc_para.I_dq_target.D = 0;
                    foc_para.I_dq_target.Q = 0;
                    foc_para.Speed_target = 0;

                    #if FOC_SMO_ENABLE
                    #if !FOC_DEBUG_SMO
                    foc_run.run_state = FOC_Order;
                    #endif
                    #endif

                    #if FOC_NFO_ENABLE
                    #if !FOC_DEBUG_FLO
                    foc_run.run_state = FOC_Order;
                    // foc_run.run_state = FOC_CloseLoopRun;
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

    #if FOC_DEBUG_OPENLOOP_VF
    foc_run.run_state = FOC_VF_OpenLoopRun;
    #endif

    #if FOC_DEBUG_OPENLOOP_IF
    foc_run.run_state = FOC_IF_OpenLoopRun;
    #endif

    PID_Set_Abs_Limit(&ID_PID,foc_para.Uo_max);
    PID_Set_Abs_Limit(&IQ_PID,foc_para.Uo_max);
    PID_Reset(&ID_PID);
    PID_Reset(&IQ_PID);
    PID_Reset(&Speed_PID);
    PID_Reset(&Position_PID);
    foc_ctrl.svpwm_update_en = 1;
    foc_para.tim.Reload = foc_run.TIM_ARR_now;
    foc_ctrl.pTIM->CCER |= 0x555;
}

void Stop_FOC_Motor(void)
{
    foc_ctrl.pTIM->CCER &= ~0x555;
    foc_ctrl.svpwm_update_en = 0;
    foc_run.run_state = FOC_IDLE;
    memset(&foc_para,0,sizeof(foc_para));
    PID_Reset(&ID_PID);
    PID_Reset(&IQ_PID);
    PID_Reset(&Speed_PID);
    PID_Reset(&Position_PID);
}

void Init_FOC_Motor(void)
{
    foc_run.run_state = FOC_IDLE;
    foc_run.PWM_freq_now = foc_ctrl.PWM_freq;
    foc_run.TIM_ARR_now = PWM_TIM_BASE_FREQ / 2 / foc_run.PWM_freq_now;
    foc_para.tim.Reload = foc_run.TIM_ARR_now;
    Virtual_Moto.dt = 1.0f / foc_run.PWM_freq_now;
    foc_ctrl.pTIM->CCER &= ~0x555;
    // enable preload
    foc_ctrl.pTIM->CR1 |= 0x80;
    foc_ctrl.pTIM->CCMR1 |= 0x808;
    foc_ctrl.pTIM->CCMR2 |= 0x08;
    foc_ctrl.pTIM->ARR = foc_run.TIM_ARR_now;
    #ifdef ADC_SAMPLE_HIGH_SIDE
    foc_ctrl.pTIM->CCR4 = 1;
    #else
    foc_ctrl.pTIM->CCR4 = foc_ctrl.pTIM->ARR - 1;
    #endif

    if(foc_ctrl.Lq / foc_ctrl.Ld > 1.5f)
    {
        foc_ctrl.motor_type = IPM;
    }

    if(foc_ctrl.MTPA_enable)
    {
        foc_para.op_area = FOC_OP_MTPA;
    }
    else
    {
        foc_para.op_area = FOC_OP_ID0;
    }

    #if 1
    // cal motor time constant
    float t_d = foc_ctrl.Ld / foc_ctrl.Rs;
    float t_q = foc_ctrl.Lq / foc_ctrl.Rs;
    float wc_d = 1.0f / (_2PI * t_d) * 3;
    float wc_q = 1.0f / (_2PI * t_q) * 3;
    // PWM cycle should lessthan motor time constant
    if(foc_ctrl.PWM_freq < MAX(wc_d,wc_q))
    {
        foc_run.err |= (1UL << 31);
    }
    #else
    float wc_d =  foc_ctrl.current_loop_freq / 10.0f;
    float wc_q =  foc_ctrl.current_loop_freq / 10.0f;
    #endif

    if(HFI_ctrl.HFI_high_freq > foc_ctrl.current_loop_freq / 2)
    {
        HFI_ctrl.HFI_high_freq = foc_ctrl.current_loop_freq / 2;
    }

    HFI_run.HFI_pole_detect_status = HFI_Pole_Detect_Wait_Theta_Stable;
    HFI_run.HFI_pole_detect_sign = 0;
    HFI_run.HFI_pole_detect_delay_cnt = HFI_ctrl.HFI_high_freq * HFI_ctrl.HFI_pole_detect_delay_us / 1000000;
    HFI_run.HFI_pole_detect_pluse_cnt = HFI_ctrl.HFI_high_freq * HFI_ctrl.HFI_pole_detect_pluse_us / 1000000;
    HFI_run.HFI_pole_detect_pluse_interval_cnt = HFI_ctrl.HFI_high_freq * HFI_ctrl.HFI_pole_detect_pluse_interval_us / 1000000;
    HFI_run.HFI_pole_detect_cnt = 0;

    foc_para.V_bus = Virtual_Moto.V_bus_v;
    foc_para.Uo_max = foc_ctrl.modulation_ratio * foc_para.V_bus * _2_3;
    PID_Init(&ID_PID,foc_ctrl.current_loop_dt,0,0,0,0,foc_para.Uo_max,-foc_para.Uo_max);
    PID_Init(&IQ_PID,foc_ctrl.current_loop_dt,0,0,0,0,foc_para.Uo_max,-foc_para.Uo_max);

    // Maybe the cut-off frequency should be set lower to avoid loop collapse at high speeds
    // wc = foc_ctrl.current_loop_freq / 10;
    Current_Loop_Parallel_PID_Tune(&ID_PID,wc_d,foc_ctrl.Ld,foc_ctrl.Rs);
    Current_Loop_Parallel_PID_Tune(&IQ_PID,wc_q,foc_ctrl.Lq,foc_ctrl.Rs);

    foc_ctrl.max_current = (Motor_Max_Current < BOARD_MAX_PHASE_CURRENT) ? Motor_Max_Current : BOARD_MAX_PHASE_CURRENT;

    PID_Init(&Speed_PID,foc_ctrl.speed_loop_dt,SPEED_KP,SPEED_KI,SPEED_KD,SPEED_KB,foc_ctrl.max_current,-foc_ctrl.max_current);
    PID_Init(&Position_PID,foc_ctrl.position_loop_dt,POSITION_KP,POSITION_KI,POSITION_KD,POSITION_KB,foc_ctrl.max_rpm,-foc_ctrl.max_rpm);

    #if PLL_USE_IIR_FILTER
    // fc = 100hz sample_hz = 20000
    float a[3 + 1] = {1.0,-2.93717073,2.87629972,-0.93909894};
    float b[3 + 1] = {3.75683802e-06,1.12705141e-05,1.12705141e-05,3.75683802e-06};

    // // fc = 300hz sample_hz = 20000
    // float a[3 + 1] = {1.0,-2.81157368,2.64048349,-0.82814628};
    // float b[3 + 1] = {9.54425084e-05,2.86327525e-04,2.86327525e-04,9.54425084e-05};

    Init_IIR_Filter(&Encoder_PLL_IIR,3,foc_ctrl.current_loop_dt,100,a,b);
    Init_IIR_Filter(&Hall_PLL_IIR,3,foc_ctrl.current_loop_dt,100,a,b);
    Init_IIR_Filter(&Observer_PLL_IIR,3,foc_ctrl.current_loop_dt,100,a,b);
    
    // fc = 10hz sample_hz = 10000
    float HFI_a[3 + 1] = {1.0,-2.98743365,2.97494613,-0.98751224};
    float HFI_b[3 + 1] = {3.08123730e-08,9.24371191e-08,9.24371191e-08,3.08123730e-08};

    // // fc = 10hz sample_hz = 10000
    // float HFI_a[3 + 1] = {1.0,-2.96230145,2.92531013,-0.96300212};
    // float HFI_b[3 + 1] = {8.21609743e-07,2.46482923e-06,2.46482923e-06,8.21609743e-07};

    Init_IIR_Filter(&HFI_PLL_IIR,3,foc_ctrl.current_loop_dt,100,HFI_a,HFI_b);

    PLL_Init(&Encoder_PLL,0.3,1.0,100,foc_ctrl.current_loop_dt,&Encoder_PLL_IIR);
    PLL_Init(&Hall_PLL,0.3,1.0,100,foc_ctrl.current_loop_dt,&Hall_PLL_IIR);
    PLL_Init(&HFI_PLL,0.3,1.0,10,2 * 1.0f / HFI_ctrl.HFI_high_freq,&HFI_PLL_IIR);
    #if FOC_SMO_ENABLE
    SMO_Init(&foc_ctrl,&SMO_observer,100,&Observer_PLL_IIR);
    #endif

    #if FOC_NFO_ENABLE
    NFO_Init(&foc_ctrl,&NFO_observer,100,&Observer_PLL_IIR);
    #endif
    #else
    PLL_Init(&Encoder_PLL,0.3,1.0,100,foc_ctrl.current_loop_dt,NULL);
    PLL_Init(&Hall_PLL,0.3,1.0,100,foc_ctrl.current_loop_dt,NULL);
    PLL_Init(&HFI_PLL,0.3,1.0,10,2 * 1.0f / HFI_ctrl.HFI_high_freq,NULL);
    #if FOC_SMO_ENABLE
    SMO_Init(&foc_ctrl,&SMO_observer,100,NULL);
    #endif

    #if FOC_NFO_ENABLE
    NFO_Init(&foc_ctrl,&NFO_observer,100,NULL);
    #endif
    #endif

}
