#include "Board_Config.h"
#include "Inverter.h"
#include "Motor.h"
#include "FOC.h"

#define CARRIER_FREQ            100 * 1000
#define FUNDAMENTAL_FREQ        50


Inverter_RUN_t inverter_run;
Inverter_CONTROL_t inverter_ctrl = 
{
    .output = 0,
    .carrier_freq = CARRIER_FREQ,
    .fundamental_freq = FUNDAMENTAL_FREQ,
};

static void VVVF_Process(void)
{
    inverter_run.PWM_freq_now = inverter_ctrl.carrier_freq;
    inverter_run.TIM1_ARR_now = PWM_TIM_BASE_FREQ / 2 / inverter_ctrl.carrier_freq;

    inverter_run.angle_step = _2PI * inverter_ctrl.fundamental_freq / inverter_run.PWM_freq_now;

    TIM1->ARR = inverter_run.TIM1_ARR_now;
}

static void Sine_Wave_Generate(uint8_t phase_cnt,float out_put,float angle,uint16_t period,uint16_t *CMP1,uint16_t *CMP2,uint16_t *CMP3)
{
    float sine_list[3];

    sine_list[0] = sinf(angle);
    if(phase_cnt == 1)
    {
        sine_list[1] = sinf(Normalize_Angle(angle + _PI));
        *CMP1 = period * sine_list[0] * out_put;
        if(*CMP1 > period)  *CMP1 = period;
        *CMP2 = period * sine_list[1] * out_put;
        if(*CMP2 > period)  *CMP2 = period;
    }
    else if(phase_cnt == 3)
    {
        sine_list[1] = sinf(Normalize_Angle(angle - _2PI_3));
        sine_list[2] = sinf(Normalize_Angle(angle + _2PI_3));
        *CMP1 = period * sine_list[0] * out_put;
        if(*CMP1 > period)  *CMP1 = period;
        *CMP2 = period * sine_list[1] * out_put;
        if(*CMP2 > period)  *CMP2 = period;
        *CMP3 = period * sine_list[2] * out_put;
        if(*CMP3 > period)  *CMP3 = period;
    }

}

void Inverter_Process(void)
{
    if(inverter_run.is_running)
    {
        inverter_run.angle += inverter_run.angle_step;
        inverter_run.angle = Normalize_Angle(inverter_run.angle);

        if(Moto_Config.moto_type == Single_Phase_Inverter)
        {
            Sine_Wave_Generate(1,inverter_ctrl.output,inverter_run.angle,inverter_run.TIM1_ARR_now - 1,&inverter_run.CMP1,&inverter_run.CMP2,&inverter_run.CMP3);
            TIM1->CCR1 = inverter_run.CMP1;
            TIM1->CCR2 = inverter_run.CMP2;
        }
        else if(Moto_Config.moto_type == Three_Phase_Inverter)
        {
            Sine_Wave_Generate(3,inverter_ctrl.output,inverter_run.angle,inverter_run.TIM1_ARR_now - 1,&inverter_run.CMP1,&inverter_run.CMP2,&inverter_run.CMP3);
            TIM1->CCR1 = inverter_run.CMP1;
            TIM1->CCR2 = inverter_run.CMP2;
            TIM1->CCR3 = inverter_run.CMP3;
        }
    }
}

void Start_Inverter(void)
{
    VVVF_Process();
    if(Moto_Config.moto_type == Single_Phase_Inverter)
    {
        TIM1->CCER |= 0x055;
    }
    else if(Moto_Config.moto_type == Three_Phase_Inverter)
    {
        TIM1->CCER |= 0x555;
    }
    inverter_run.is_running = 1;
}

void Stop_Inverter(void)
{
    TIM1->CCER &= ~0x00;
    inverter_run.is_running = 0;
}
