#include "Board_Config.h"
#include "AS5048a.h"
#include "Sensor.h"
#include "Motor.h"
#include "BLDC_Motor.h"
#include "FOC_Motor.h"
#include "FOC.h"
#include "Inverter.h"
#include "VIS.h"

extern TIM_HandleTypeDef htim1;

Virtual_Motor_t Virtual_Moto =
{
    .is_running = 0,
    .state = MS_IDLE,
    .run_quadrant = Quad_0,
    .dt = 1.0f / 200000,
    .electronic_speed_hz = 0,
};

MOTOR_Config_t Motor_Config =
{
    // .moto_type = VIS,
    // .moto_type = BLDC,
    .moto_type = FOC,
    .dir = CCW,
};

MOTOR_Type_t motor_type_now;

Filter_Rate_t Filter_Rate =
{
    .phase_voltage_filter_rate = 0.1,
    .phase_current_filter_rate = 0.1,
    .bus_voltage_filter_rate = 0.03,
    .bus_current_filter_rate = 0.03,
    .RPM_filter_rate = 0.03,
};

void MOS_Driver_Enable(void)
{
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_SET);
}

void MOS_Driver_Disable(void)
{
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET);
}

void Init_Motor(void)
{
    switch (motor_type_now)
    {
        case BLDC:
            Init_BLDC_Motor();
            break;

        case FOC:
            Init_FOC_Motor();
            break;

        case VIS:
            Init_VIS();
            break;

        default:
            break;
    }
}

void Stop_Motor(void)
{
    Virtual_Moto.is_running = 0;

    switch (motor_type_now)
    {
        case BLDC:
            Stop_BLDC_Motor();
            break;

        case FOC:
            Stop_FOC_Motor();
            break;

        case DC:
            break;

        case Single_Phase_Inverter:
        case Three_Phase_Inverter:
            Stop_Inverter();
            break;

        case VIS:
            Stop_VIS();
            break;

        default:
            break;
    }

    MOS_Driver_Disable();
}

void Start_Motor(void)
{
    Virtual_Moto.is_running = 1;

    switch (motor_type_now)
    {
        case BLDC:
            Start_BLDC_Motor();
            break;

        case FOC:
            Start_FOC_Motor();
            break;

        case DC:
            break;

        case Single_Phase_Inverter:
        case Three_Phase_Inverter:
            Start_Inverter();
            break;

        case VIS:
            Start_VIS();
            break;

        default:
            break;
    }

    MOS_Driver_Enable();
}

bool Motor_Is_Running(void)
{
    return Virtual_Moto.is_running;
}

bool Motor_Is_Rotating(void)
{
    switch (motor_type_now)
    {
        case BLDC:
            return bldc_run.eletrical_rpm != 0;

        case FOC:
            return foc_run.eletrical_rpm_f != 0;

        default:
            return 0;
    }
}
