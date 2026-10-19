/**
 * @file encoder.c
 * @author jinchl (jcl709905683@gmail.com)
 * @brief 
 * @version 0.1
 * @date 2026-03-11
 * 
 * @copyright Copyright (c) 2026
 * 
 * 对于直线直流电机配置编码器1mm/round
 * 对于其他电机配置编码器一圈对应机械角度一圈
 */


#include "encoder.h"

#include <math.h>
#include <string.h>

#include "board.h"
#include "global.h"
#include "my_math.h"
#include "motor.h"

#include "tamagawa.h"
#include "hcfa.h"
#include "as5048a.h"

/** 单个高频周期允许的最大编码器位置变化，单位为编码器圈。 */
#define ENCODER_POSITION_JUMP_THRESHOLD_TURNS (0.5f)

/** 累计丢弃的位置异常样本达到该值后上报编码器位置跳变故障。 */
#define ENCODER_POSITION_JUMP_ERROR_LIMIT (10U)

/**
 * @brief 编码器索引二级回调
 * 
 */
static encoder_index_callback index_callback[BOARD_SUPPORTED_MOTOR_NUMBER] = {NULL, NULL};

/**
 * @brief 记录并处理编码器位置无效样本
 *
 * @param index 编码器编号
 */
static inline void encoder_reject_position_sample(int index)
{
    Encoder_Status_t *p_encoder_status = &global_status.encoder_status[index];

    if (p_encoder_status->encoder_position_jump_error_cnt < UINT32_MAX)
    {
        p_encoder_status->encoder_position_jump_error_cnt++;
    }

    if (p_encoder_status->encoder_position_jump_error_cnt >= ENCODER_POSITION_JUMP_ERROR_LIMIT)
    {
        p_encoder_status->encoder_position_jump_error_cnt = 0;
        motor_set_err(index, FAULT_CODE_ENCODER_POSITION_JUMP);
    }
}

/**
 * @brief 编码器索引一级回调
 * 
 * @param index 编码器编号
 */
static inline void encoder_index_handle(int index)
{
    Encoder_Config_t *p_encoder_config = &global_config.encoder_config[index];
    Encoder_Status_t *p_encoder_status = &global_status.encoder_status[index];

    if (index_callback[index] != NULL)
    {
        index_callback[index](index);
    }

    switch (p_encoder_config->encoder_type)
    {
    case Encoder_Type_ABZ:
        p_encoder_status->encoder_index_cnt += SIGN_F(p_encoder_status->encoder_speed);
        break;

    default:
        break;
    }
}

/**
 * @brief m法测速
 * 
 * @param index 编码器编号
 */
static inline void speed_m_method_update(int index)
{
    Encoder_Config_t *p_encoder_config = &global_config.encoder_config[index];
    Encoder_Status_t *p_encoder_status = &global_status.encoder_status[index];

    float pos_delta = p_encoder_status->encoder_pos - p_encoder_status->encoder_pos_last;
    float speed     = pos_delta / BOARD_RT_DT;

    LPF_F32(&p_encoder_status->encoder_speed, speed, p_encoder_config->velocity_solver_band_width, BOARD_RT_DT);
    p_encoder_status->encoder_pos_last = p_encoder_status->encoder_pos;
}

/**
 * @brief m法测加速度
 * 
 * @param index 编码器编号
 */
static inline void acc_m_method_update(int index)
{
    Encoder_Config_t *p_encoder_config = &global_config.encoder_config[index];
    Encoder_Status_t *p_encoder_status = &global_status.encoder_status[index];

    float speed_delta = p_encoder_status->encoder_speed - p_encoder_status->encoder_speed_last;
    float acc         = speed_delta / BOARD_RT_DT;

    LPF_F32(&p_encoder_status->encoder_acc, acc, p_encoder_config->velocity_solver_band_width, BOARD_RT_DT);
    p_encoder_status->encoder_speed_last = p_encoder_status->encoder_speed;
}

/**
 * @brief 速度跟踪器测速和加速度
 * 
 * @param index 编码器编号
 * @param speed_fb 速度反馈
 */
static inline void speed_estimate_update(int index, float speed_fb)
{
    // 二阶观测器设计，极点配置在 -fc (临界阻尼)
    // 传递函数: H(s) = fc^2 / (s^2 + 2*fc*s + fc^2)
    Encoder_Config_t *p_encoder_config = &global_config.encoder_config[index];
    Encoder_Status_t *p_encoder_status = &global_status.encoder_status[index];
    float omega                        = p_encoder_config->velocity_solver_band_width * _2PI;
    float kp                           = 2.0f * omega; // = 2 * zeta * omega_n, zeta = 1
    float ki                           = SQ(omega);    // = omega_n^2

    p_encoder_status->encoder_speed += p_encoder_status->encoder_acc * BOARD_RT_DT;
    double delta_vel                 = speed_fb - p_encoder_status->encoder_speed;
    p_encoder_status->encoder_speed += BOARD_RT_DT * delta_vel * kp;
    p_encoder_status->encoder_acc   += BOARD_RT_DT * delta_vel * ki;
}

/**
 * @brief 位置跟踪器测速和加速度
 * 
 * @param index 编码器编号
 * @param pos_fb 位置反馈
 */
static inline void position_estimate_update(int index, float pos_fb)
{
    // 三阶观测器设计，极点配置在 -fc (临界阻尼)
    // 传递函数: H(s) = fc^3 / (s^3 + 3*fc*s^2 + 3*fc^2*s + fc^3)
    Encoder_Config_t *p_encoder_config = &global_config.encoder_config[index];
    Encoder_Status_t *p_encoder_status = &global_status.encoder_status[index];
    float omega                        = p_encoder_config->velocity_solver_band_width * _2PI;
    float kp                           = 3.0f * omega;
    float ki                           = 3.0f * SQ(omega);
    float kd                           = SQ(omega) * omega;

    p_encoder_status->encoder_pos   += p_encoder_status->encoder_speed * BOARD_RT_DT;
    p_encoder_status->encoder_speed += p_encoder_status->encoder_acc * BOARD_RT_DT;
    double delta_pos                 = pos_fb - p_encoder_status->encoder_pos;
    p_encoder_status->encoder_pos   += BOARD_RT_DT * delta_pos * kp;
    p_encoder_status->encoder_speed += BOARD_RT_DT * delta_pos * ki;
    p_encoder_status->encoder_acc   += BOARD_RT_DT * delta_pos * kd;
}

static inline void encode_speed_update(int index, float pos_fb)
{
    Encoder_Config_t *p_encoder_config = &global_config.encoder_config[index];
    Encoder_Status_t *p_encoder_status = &global_status.encoder_status[index];

    if (p_encoder_config->velocity_solver_band_width < FLOAT_EPSILON)
    {
        motor_set_ext_err(index, EXT_FAULT_CODE_CFG_ENCODER_VELOCITY_SOLVER_BAND_WIDTH);
        return;
    }

    switch (p_encoder_config->velocity_solver_type)
    {
    case Velocity_Solver_M:
        p_encoder_status->encoder_pos = pos_fb;
        speed_m_method_update(index);
        acc_m_method_update(index);

    case Velocity_Solver_Estimator:
        position_estimate_update(index, pos_fb);
        break;

    default:
        motor_set_ext_err(index, EXT_FAULT_CODE_CFG_ENCODER_VELOCITY_SOLVER_TYPE);
        break;
    }
}

static inline bool encoder_get_wdg_error(int index)
{
    Encoder_Config_t *p_encoder_config = &global_config.encoder_config[index];

    switch (p_encoder_config->encoder_type)
    {
    case Encoder_Type_TAMAGAWA:
        return tamagawa_encoder_get_wdg_error(index);

    case Encoder_Type_HCFA:
        return hcfa_encoder_get_wdg_error(index);

    case Encoder_Type_SPI_AS5048A:
        return as5048a_encoder_get_wdg_error(index);
        
    default:
        return true;
    }
}

static inline bool encoder_get_pos_valid(int index)
{
    Encoder_Config_t *p_encoder_config = &global_config.encoder_config[index];

    switch (p_encoder_config->encoder_type)
    {
    case Encoder_Type_TAMAGAWA:
        return tamagawa_encoder_get_pos_valid(index);

    case Encoder_Type_HCFA:
        return hcfa_encoder_get_pos_valid(index);

    case Encoder_Type_SPI_AS5048A:
        return as5048a_encoder_get_pos_valid(index);

    default:
        return false;
    }
}

static inline bool encoder_is_err(int index)
{
    Encoder_Config_t *p_encoder_config = &global_config.encoder_config[index];

    switch (p_encoder_config->encoder_type)
    {
    case Encoder_Type_TAMAGAWA:
        return (tamagawa_encoder_get_almc(index) & TAMAGAWA_ALMC_MASK) != 0;

    case Encoder_Type_HCFA:
        return (hcfa_encoder_get_almc(index) & HCFA_ALMC_MASK) != 0;

    case Encoder_Type_SPI_AS5048A:
        return as5048a_encoder_get_error(index);

    default:
        return false;
    }
}

static inline bool encoder_is_batt_err(int index)
{
    Encoder_Config_t *p_encoder_config = &global_config.encoder_config[index];

    switch (p_encoder_config->encoder_type)
    {
    case Encoder_Type_TAMAGAWA:
        return (tamagawa_encoder_get_almc(index) & TAMAGAWA_BATT_MASK) != 0;

    case Encoder_Type_HCFA:
        return (hcfa_encoder_get_almc(index) & HCFA_BATT_MASK) != 0;

    default:
        return false;
    }
}

static inline bool encoder_get_error_reset_done(int index)
{
    Encoder_Config_t *p_encoder_config = &global_config.encoder_config[index];

    switch (p_encoder_config->encoder_type)
    {
    case Encoder_Type_TAMAGAWA:
        return tamagawa_encoder_get_error_reset_done(index);

    case Encoder_Type_HCFA:
        return hcfa_encoder_get_error_reset_done(index);

    case Encoder_Type_SPI_AS5048A:
        return as5048a_encoder_get_error_reset_done(index);

    default:
        return false;
    }
}

static inline void encoder_clear_error_reset_done(int index)
{
    Encoder_Config_t *p_encoder_config = &global_config.encoder_config[index];

    switch (p_encoder_config->encoder_type)
    {
    case Encoder_Type_TAMAGAWA:
        tamagawa_encoder_clear_error_reset_done(index);
        break;

    case Encoder_Type_HCFA:
        hcfa_encoder_clear_error_reset_done(index);
        break;

    case Encoder_Type_SPI_AS5048A:
        as5048a_encoder_clear_error_reset_done(index);
        break;

    default:
        break;
    }
}

static inline void encoder_reset_error(int index)
{
    Encoder_Config_t *p_encoder_config = &global_config.encoder_config[index];

    switch (p_encoder_config->encoder_type)
    {
    case Encoder_Type_TAMAGAWA:
        tamagawa_encoder_reset_error(index);
        break;

    case Encoder_Type_HCFA:
        hcfa_encoder_reset_error(index);
        break;

    case Encoder_Type_SPI_AS5048A:
        as5048a_encoder_reset_error(index);
        break;

    default:
        break;
    }
}

static inline bool encoder_get_abs_reset_done(int index)
{
    Encoder_Config_t *p_encoder_config = &global_config.encoder_config[index];

    switch (p_encoder_config->encoder_type)
    {
    case Encoder_Type_TAMAGAWA:
        return tamagawa_encoder_get_abs_reset_done(index);

    case Encoder_Type_HCFA:
        return hcfa_encoder_get_abs_reset_done(index);

    default:
        return false;
    }
}

static inline void encoder_clear_abs_reset_done(int index)
{
    Encoder_Config_t *p_encoder_config = &global_config.encoder_config[index];

    switch (p_encoder_config->encoder_type)
    {
    case Encoder_Type_TAMAGAWA:
        tamagawa_encoder_clear_abs_reset_done(index);
        break;

    case Encoder_Type_HCFA:
        hcfa_encoder_clear_abs_reset_done(index);
        break;

    default:
        break;
    }
}

static inline void encoder_reset_abs(int index)
{
    Encoder_Config_t *p_encoder_config = &global_config.encoder_config[index];

    switch (p_encoder_config->encoder_type)
    {
    case Encoder_Type_TAMAGAWA:
        tamagawa_encoder_reset_abs(index);
        break;

    case Encoder_Type_HCFA:
        hcfa_encoder_reset_abs(index);
        break;

    default:
        break;
    }
}

static inline bool encoder_get_abm_reset_done(int index)
{
    Encoder_Config_t *p_encoder_config = &global_config.encoder_config[index];

    switch (p_encoder_config->encoder_type)
    {
    case Encoder_Type_TAMAGAWA:
        return tamagawa_encoder_get_abm_reset_done(index);

    case Encoder_Type_HCFA:
        return hcfa_encoder_get_abm_reset_done(index);

    default:
        return false;
    }
}

static inline void encoder_clear_abm_reset_done(int index)
{
    Encoder_Config_t *p_encoder_config = &global_config.encoder_config[index];

    switch (p_encoder_config->encoder_type)
    {
    case Encoder_Type_TAMAGAWA:
        tamagawa_encoder_clear_abm_reset_done(index);
        break;

    case Encoder_Type_HCFA:
        hcfa_encoder_clear_abm_reset_done(index);
        break;

    default:
        break;
    }
}

static inline void encoder_reset_abm(int index)
{
    Encoder_Config_t *p_encoder_config = &global_config.encoder_config[index];

    switch (p_encoder_config->encoder_type)
    {
    case Encoder_Type_TAMAGAWA:
        tamagawa_encoder_reset_abm(index);
        break;

    case Encoder_Type_HCFA:
        hcfa_encoder_reset_abm(index);
        break;

    default:
        break;
    }
}

static inline void encoder_read_noraml(int index)
{
    Encoder_Config_t *p_encoder_config = &global_config.encoder_config[index];

    switch (p_encoder_config->encoder_type)
    {

    case Encoder_Type_TAMAGAWA:
        tamagawa_encoder_read_all(index);
        break;

    case Encoder_Type_HCFA:
        hcfa_encoder_read_all(index);
        break;

    case Encoder_Type_SPI_AS5048A:
        as5048a_encoder_read_single_turn(index);
        break;

    default:
        break;
    }
}

static inline uint32_t encoder_get_abs(int index)
{
    Encoder_Config_t *p_encoder_config = &global_config.encoder_config[index];

    switch (p_encoder_config->encoder_type)
    {

    case Encoder_Type_TAMAGAWA:
        return tamagawa_encoder_get_abs(index);

    case Encoder_Type_HCFA:
        return hcfa_encoder_get_abs(index);

    case Encoder_Type_SPI_AS5048A:
        return as5048a_encoder_get_abs(index);

    default:
        return 0;
    }
}

static inline int32_t encoder_get_abm(int index)
{
    Encoder_Config_t *p_encoder_config = &global_config.encoder_config[index];

    switch (p_encoder_config->encoder_type)
    {
    case Encoder_Type_TAMAGAWA:
        return tamagawa_encoder_get_abm(index);

    case Encoder_Type_HCFA:
        return hcfa_encoder_get_abm(index);

    default:
        return 0;
    }
}
/**
 * @brief 编码器实时更新
 * 
 * @param index 编码器编号
 */
void encoder_update_rt(int index)
{
    Encoder_Config_t *p_encoder_config = &global_config.encoder_config[index];
    Encoder_Status_t *p_encoder_status = &global_status.encoder_status[index];

    switch (p_encoder_config->encoder_type)
    {
    case Encoder_Type_ABZ:
        encode_speed_update(index, p_encoder_status->encoder_delta_pos * p_encoder_config->encoder_delta_dir);
        break;

    case Encoder_Type_SPI_AS5048A:
        encode_speed_update(index, p_encoder_status->encoder_abs_pos * p_encoder_config->encoder_abs_dir);
        break;

    case Encoder_Type_TAMAGAWA:
    case Encoder_Type_HCFA: {
        switch (p_encoder_status->encoder_special_function_flag)
        {
        case ENCODER_SPECIAL_FUNCTION_RESET_ERROR:
            if (encoder_get_error_reset_done(index))
            {
                p_encoder_status->encoder_special_function_flag = 0;
                encoder_clear_error_reset_done(index);
                encoder_read_noraml(index);
            }
            else
            {
                if (encoder_get_wdg_error(index))
                {
                    motor_set_err(index, FAULT_CODE_ENCODER_LOST);
                }

                encoder_reset_error(index);
            }
            break;

        case ENCODER_SPECIAL_FUNCTION_RESET_ABS:
            if (encoder_get_abs_reset_done(index))
            {
                p_encoder_status->delay_time += BOARD_RT_DT;
                if (p_encoder_status->delay_time >= MS_TO_S(20))
                {
                    p_encoder_status->encoder_special_function_flag = 0;
                    encoder_clear_abs_reset_done(index);
                    encoder_read_noraml(index);
                }
            }
            else
            {
                if (encoder_get_wdg_error(index))
                {
                    motor_set_err(index, FAULT_CODE_ENCODER_LOST);
                }

                encoder_reset_abs(index);
                p_encoder_status->delay_time = 0;
            }
            break;

        case ENCODER_SPECIAL_FUNCTION_RESET_ABM:
            if (encoder_get_abm_reset_done(index))
            {
                p_encoder_status->delay_time += BOARD_RT_DT;
                if (p_encoder_status->delay_time >= MS_TO_S(20))
                {
                    p_encoder_status->encoder_special_function_flag = 0;
                    encoder_clear_abm_reset_done(index);
                    encoder_read_noraml(index);
                }
            }
            else
            {
                if (encoder_get_wdg_error(index))
                {
                    motor_set_err(index, FAULT_CODE_ENCODER_LOST);
                }

                encoder_reset_abm(index);
                p_encoder_status->delay_time = 0;
            }
            break;

        default: {
            if (encoder_get_pos_valid(index))
            {
                bool first_read          = false;
                int32_t encoder_abs_turn = p_encoder_status->encoder_abs_turn;
                uint32_t encoder_abs_cnt;
                float encoder_abs_fractional = 0;
                float encoder_abs_pos        = 0;
                if (!p_encoder_status->encoder_abs_position_valid)
                {
                    first_read                             = true;
                    encoder_abs_cnt                        = encoder_get_abs(index);
                    p_encoder_status->encoder_abs_cnt_last = encoder_abs_cnt;
                    p_encoder_status->encoder_speed        = 0;
                    p_encoder_status->encoder_speed_last   = 0;
                    p_encoder_status->encoder_acc          = 0;
                    p_encoder_status->encoder_acc_last     = 0;
                }
                else
                {
                    p_encoder_status->encoder_abs_cnt_last = p_encoder_status->encoder_abs_cnt;
                    encoder_abs_cnt                        = encoder_get_abs(index);
                }

                if (p_encoder_config->encoder_struct == Encoder_Struct_Linear)
                {
                    float pos              = encoder_abs_cnt * p_encoder_status->encoder_abs_cnt_inv;
                    encoder_abs_fractional = pos - (int)pos;
                    encoder_abs_turn       = (int)pos;
                    encoder_abs_pos        = pos - p_encoder_status->encoder_abs_zero_offset - p_encoder_status->encoder_abs_home_offset * p_encoder_config->encoder_abs_dir;
                }
                else if (p_encoder_config->encoder_struct == Encoder_Struct_Rotary)
                {
                    if (encoder_abs_cnt >= p_encoder_status->encoder_abs_max_cnt)
                    {
                        motor_set_err(index, FAULT_CODE_ENCODER_OVERFLOW);
                    }

                    if (p_encoder_config->encoder_mult_turn_bits == 0)
                    {
                        int32_t delta        = encoder_abs_cnt - p_encoder_status->encoder_abs_cnt_last;
                        int32_t half_max_cnt = p_encoder_status->encoder_abs_max_cnt * 0.5f;
                        if (delta > half_max_cnt)
                        {
                            encoder_abs_turn--;
                        }
                        else if (delta < -half_max_cnt)
                        {
                            encoder_abs_turn++;
                        }
                    }
                    else
                    {
                        encoder_abs_turn = encoder_get_abm(index);
                    }

                    encoder_abs_fractional = (float)encoder_abs_cnt * p_encoder_status->encoder_abs_cnt_inv;
                    encoder_abs_pos        = encoder_abs_fractional + (float)encoder_abs_turn - p_encoder_status->encoder_abs_zero_offset - p_encoder_status->encoder_abs_home_offset * p_encoder_config->encoder_abs_dir;
                }

                if (p_encoder_status->encoder_abs_position_valid && fabsf(encoder_abs_pos - p_encoder_status->encoder_abs_pos) > ENCODER_POSITION_JUMP_THRESHOLD_TURNS)
                {
                    encoder_reject_position_sample(index);
                    p_encoder_status->encoder_abs_pos = encoder_abs_pos;
                }
                else
                {
                    p_encoder_status->encoder_abs_cnt        = encoder_abs_cnt;
                    p_encoder_status->encoder_abs_turn       = encoder_abs_turn;
                    p_encoder_status->encoder_abs_fractional = encoder_abs_fractional;
                    if (p_encoder_config->encoder_abs_dir == 1)
                    {
                        p_encoder_status->encoder_pos_fractional = encoder_abs_fractional;
                    }
                    else
                    {
                        p_encoder_status->encoder_pos_fractional = 1.0f - encoder_abs_fractional;
                    }
                    p_encoder_status->encoder_abs_pos = encoder_abs_pos;

                    if (first_read)
                    {
                        p_encoder_status->encoder_pos                = p_encoder_status->encoder_abs_pos * p_encoder_config->encoder_abs_dir;
                        p_encoder_status->encoder_pos_last           = p_encoder_status->encoder_pos;
                        p_encoder_status->encoder_speed_last         = p_encoder_status->encoder_speed;
                        p_encoder_status->encoder_abs_position_valid = true;
                    }
                }

                encode_speed_update(index, p_encoder_status->encoder_abs_pos * p_encoder_config->encoder_abs_dir);
            }

            if (encoder_get_wdg_error(index))
            {
                p_encoder_status->encoder_abs_position_valid = false;
                motor_set_err(index, FAULT_CODE_ENCODER_LOST);
            }
            else if (encoder_is_err(index))
            {
                p_encoder_status->encoder_abs_position_valid = false;
                motor_set_err(index, FAULT_CODE_ENCODER_FAULT);
            }
            else if (encoder_is_batt_err(index))
            {
                motor_set_warning(index, WARN_CODE_ENCODER_BATTERY);
            }

            encoder_read_noraml(index);
        }
        break;
        }
    }
    break;

    default:
        motor_set_ext_err(index, EXT_FAULT_CODE_CFG_ENCODER_TYPE);
        break;
    }
}

/**
 * @brief 编码器高频实时更新
 * 
 * @param index 编码器编号
 */
void encoder_update_frt(int index)
{
    Encoder_Config_t *p_encoder_config = &global_config.encoder_config[index];
    Encoder_Status_t *p_encoder_status = &global_status.encoder_status[index];

    switch (p_encoder_config->encoder_type)
    {
    case Encoder_Type_ABZ:{
        p_encoder_status->encoder_delta_cnt_last = p_encoder_status->encoder_delta_cnt;
        p_encoder_status->encoder_delta_cnt      = 0; // todo
        int32_t delta                            = p_encoder_status->encoder_delta_cnt - p_encoder_status->encoder_delta_cnt_last;
        int32_t half_max_cnt                     = p_encoder_status->encoder_delta_max_cnt * 0.5f;
        if (delta > half_max_cnt)
        {
            p_encoder_status->encoder_delta_turn--;
        }
        else if (delta < -half_max_cnt)
        {
            p_encoder_status->encoder_delta_turn++;
        }

        p_encoder_status->encoder_delta_fractional = (float)p_encoder_status->encoder_delta_cnt * p_encoder_status->encoder_delta_cnt_inv;
        if (p_encoder_config->encoder_delta_dir == 1)
        {
            p_encoder_status->encoder_pos_fractional = p_encoder_status->encoder_delta_fractional;
        }
        else
        {
            p_encoder_status->encoder_pos_fractional = 1.0f - p_encoder_status->encoder_delta_fractional;
        }

        float encoder_delta_pos = p_encoder_status->encoder_delta_fractional + (float)p_encoder_status->encoder_delta_turn - p_encoder_status->encoder_delta_home_offset * p_encoder_config->encoder_delta_dir;

        if (p_encoder_status->encoder_delta_position_valid && fabsf(encoder_delta_pos - p_encoder_status->encoder_delta_pos) > ENCODER_POSITION_JUMP_THRESHOLD_TURNS)
        {
            encoder_reject_position_sample(index);
            break;
        }

        p_encoder_status->encoder_delta_pos            = encoder_delta_pos;
        p_encoder_status->encoder_delta_position_valid = true;
    }
    break;

    case Encoder_Type_TAMAGAWA:
    case Encoder_Type_HCFA:
        break;

    case Encoder_Type_SPI_AS5048A:{
        p_encoder_status->encoder_abs_cnt_last = p_encoder_status->encoder_abs_cnt;
        p_encoder_status->encoder_abs_cnt      = as5048a_encoder_get_abs(index);
        int32_t delta                            = p_encoder_status->encoder_abs_cnt - p_encoder_status->encoder_abs_cnt_last;
        int32_t half_max_cnt                     = p_encoder_status->encoder_abs_max_cnt * 0.5f;
        if (delta > half_max_cnt)
        {
            p_encoder_status->encoder_abs_turn--;
        }
        else if (delta < -half_max_cnt)
        {
            p_encoder_status->encoder_abs_turn++;
        }

        p_encoder_status->encoder_abs_fractional = (float)p_encoder_status->encoder_abs_cnt * p_encoder_status->encoder_abs_cnt_inv;
        if (p_encoder_config->encoder_abs_dir == 1)
        {
            p_encoder_status->encoder_pos_fractional = p_encoder_status->encoder_abs_fractional;
        }
        else
        {
            p_encoder_status->encoder_pos_fractional = 1.0f - p_encoder_status->encoder_abs_fractional;
        }

        float encoder_abs_pos = p_encoder_status->encoder_abs_fractional + (float)p_encoder_status->encoder_abs_turn - p_encoder_status->encoder_abs_home_offset * p_encoder_config->encoder_abs_dir;

        if (p_encoder_status->encoder_abs_position_valid && fabsf(encoder_abs_pos - p_encoder_status->encoder_abs_pos) > ENCODER_POSITION_JUMP_THRESHOLD_TURNS)
        {
            encoder_reject_position_sample(index);
            break;
        }

        p_encoder_status->encoder_abs_pos            = encoder_abs_pos;
        p_encoder_status->encoder_abs_position_valid = true;
    }
    break;

    default:
        motor_set_ext_err(index, EXT_FAULT_CODE_CFG_ENCODER_TYPE);
        break;
    }
}

/**
 * @brief 初始化编码器
 * 
 */
void encoder_init(void)
{
    Encoder_Config_t *p_encoder_config = NULL;
    Motor_Config_t *p_motor_config     = NULL;
    Encoder_Status_t *p_encoder_status = NULL;

    for (uint8_t i = 0; i < BOARD_SUPPORTED_MOTOR_NUMBER; i++)
    {
        p_encoder_config = &global_config.encoder_config[i];
        p_motor_config   = &global_config.motor_config[i];
        p_encoder_status = &global_status.encoder_status[i];
        bool have_delta  = p_encoder_config->encoder_type == Encoder_Type_ABZ;
        bool have_abs    = p_encoder_config->encoder_type >= Encoder_Type_ABZ;

        memset(p_encoder_status, 0, sizeof(Encoder_Status_t));

        if (p_encoder_config->encoder_type >= Encoder_Type_MAX)
        {
            motor_set_ext_err(i, EXT_FAULT_CODE_CFG_ENCODER_TYPE);
            continue;
        }
        if ((int)p_encoder_config->encoder_struct != (int)p_motor_config->motor_struct)
        {
            motor_set_ext_err(i, EXT_FAULT_CODE_CFG_ENCODER_STRUCT);
            continue;
        }
        if (have_delta)
        {
            if (p_encoder_config->encoder_delta_cpr <= 0)
            {
                motor_set_ext_err(i, EXT_FAULT_CODE_CFG_ENCODER_DELTA_CPR);
                continue;
            }
            if (p_encoder_config->encoder_delta_dir != 1 && p_encoder_config->encoder_delta_dir != -1)
            {
                motor_set_ext_err(i, EXT_FAULT_CODE_CFG_ENCODER_DELTA_DIR);
                continue;
            }

            p_encoder_status->encoder_delta_max_cnt = p_encoder_config->encoder_delta_cpr;
            p_encoder_status->encoder_delta_cnt_inv = 1.0f / (float)p_encoder_status->encoder_delta_max_cnt;
        }
        if (have_abs)
        {
            if (p_encoder_config->encoder_abs_cpr <= 0)
            {
                motor_set_ext_err(i, EXT_FAULT_CODE_CFG_ENCODER_ABS_CPR);
                continue;
            }
            if (p_encoder_config->encoder_baudrate <= 0)
            {
                motor_set_ext_err(i, EXT_FAULT_CODE_CFG_ENCODER_BAUDRATE);
                continue;
            }
            if (p_encoder_config->encoder_single_turn_bits <= 0 || p_encoder_config->encoder_single_turn_bits > 32)
            {
                motor_set_ext_err(i, EXT_FAULT_CODE_CFG_ENCODER_SINGLE_TURN_BITS);
                continue;
            }
            if (p_encoder_config->encoder_mult_turn_bits < 0 || p_encoder_config->encoder_mult_turn_bits > 24)
            {
                motor_set_ext_err(i, EXT_FAULT_CODE_CFG_ENCODER_MULT_TURN_BITS);
                continue;
            }
            if (p_encoder_config->encoder_abs_dir != 1 && p_encoder_config->encoder_abs_dir != -1)
            {
                motor_set_ext_err(i, EXT_FAULT_CODE_CFG_ENCODER_ABS_DIR);
                continue;
            }

            p_encoder_status->encoder_abs_max_cnt = p_encoder_config->encoder_abs_cpr;
            p_encoder_status->encoder_abs_cnt_inv = 1.0f / (float)p_encoder_status->encoder_abs_max_cnt;
        }

        switch (p_encoder_config->encoder_type)
        {
        case Encoder_Type_ABZ:
            // todo
            break;

        case Encoder_Type_TAMAGAWA:
        case Encoder_Type_HCFA:
            if (p_encoder_config->encoder_type == Encoder_Type_TAMAGAWA)
            {
                if (!tamagawa_encoder_init(i, p_encoder_config->encoder_baudrate, p_encoder_config->encoder_single_turn_bits, p_encoder_config->encoder_mult_turn_bits))
                {
                    motor_set_err(i, FAULT_CODE_ENCODER_INIT);
                }
            }
            else if (p_encoder_config->encoder_type == Encoder_Type_HCFA)
            {
                if (!hcfa_encoder_init(i, p_encoder_config->encoder_baudrate, p_encoder_config->encoder_single_turn_bits, p_encoder_config->encoder_mult_turn_bits))
                {
                    motor_set_err(i, FAULT_CODE_ENCODER_INIT);
                }
            }
            break;

        case Encoder_Type_SPI_AS5048A:
            as5048a_encoder_init(i);
            break;

        default:
            motor_set_ext_err(i, EXT_FAULT_CODE_CFG_ENCODER_TYPE);
            break;
        }
    }
}

/**
 * @brief 复位编码器
 * 
 * @param index 编码器编号
 */
void encoder_reset(int index)
{
    Encoder_Config_t *p_encoder_config                   = &global_config.encoder_config[index];
    Encoder_Status_t *p_encoder_status                   = &global_status.encoder_status[index];
    Motor_Config_t *p_motor_config                       = &global_config.motor_config[index];
    Motor_Controller_Config_t *p_motor_controller_config = &global_config.motor_controller_config[index];

    p_encoder_status->encoder_position_jump_error_cnt = 0;
    p_encoder_status->encoder_delta_position_valid    = false;
    p_encoder_status->encoder_abs_position_valid      = false;
    p_encoder_status->encoder_pos                     = 0;
    p_encoder_status->encoder_pos_fractional          = 0;
    p_encoder_status->encoder_pos_last                = 0;
    p_encoder_status->encoder_speed                   = 0;
    p_encoder_status->encoder_speed_last              = 0;
    p_encoder_status->encoder_acc                     = 0;
    p_encoder_status->encoder_acc_last                = 0;
    p_encoder_status->encoder_pos_second              = 0;
    p_encoder_status->encoder_pos_fractional_second   = 0;

    switch (p_encoder_config->encoder_type)
    {
    case Encoder_Type_ABZ:
        switch (p_motor_config->motor_type)
        {
        case Motor_Type_BLDC_3_Phase:
            switch (p_motor_controller_config->motor_controller_type)
            {
            case Motor_Controller_Type_FOC:
                // 修正电气角度偏移
                foc_encoder_reset(index);
                break;

            default:
                break;
            }
            break;

        default:
            break;
        }
        p_encoder_status->encoder_delta_cnt         = 0;
        p_encoder_status->encoder_delta_cnt_last    = 0;
        p_encoder_status->encoder_delta_turn        = 0;
        p_encoder_status->encoder_delta_fractional  = 0;
        p_encoder_status->encoder_delta_home_offset = 0;
        p_encoder_status->encoder_delta_pos         = 0;
        p_encoder_status->encoder_delta_pos_last    = 0;
        p_encoder_status->encoder_index_cnt         = 0;
        break;

    case Encoder_Type_TAMAGAWA:
    case Encoder_Type_HCFA:
    case Encoder_Type_SPI_AS5048A:
        p_encoder_status->encoder_abs_zero_offset       = p_encoder_status->encoder_abs_fractional + p_encoder_status->encoder_abs_turn;
        p_encoder_status->encoder_special_function_flag = 0;
        break;

    default:
        break;
    }
}

/**
 * @brief 复位编码器错误
 * 
 * @param index 编码器编号
 */
void encoder_set_reset_err(int index)
{
    Encoder_Config_t *p_encoder_config = &global_config.encoder_config[index];
    Encoder_Status_t *p_encoder_status = &global_status.encoder_status[index];

    switch (p_encoder_config->encoder_type)
    {
    case Encoder_Type_TAMAGAWA:
    case Encoder_Type_HCFA:
    case Encoder_Type_SPI_AS5048A:
        if (encoder_is_err(index))
        {
            encoder_clear_error_reset_done(index);
            p_encoder_status->encoder_special_function_flag = ENCODER_SPECIAL_FUNCTION_RESET_ERROR;
        }
        break;

    default:
        break;
    }
}

/**
 * @brief 复位编码器ABS
 * 
 * @param index 编码器编号
 */
void encoder_set_reset_abs(int index)
{
    Encoder_Config_t *p_encoder_config = &global_config.encoder_config[index];
    Encoder_Status_t *p_encoder_status = &global_status.encoder_status[index];

    switch (p_encoder_config->encoder_type)
    {
    case Encoder_Type_TAMAGAWA:
    case Encoder_Type_HCFA:
        if (!encoder_is_err(index))
        {
            encoder_clear_abs_reset_done(index);
            p_encoder_status->encoder_special_function_flag = ENCODER_SPECIAL_FUNCTION_RESET_ABS;
        }
        break;

    default:
        break;
    }
}

/**
 * @brief 复位编码器ABM
 * 
 * @param index 编码器编号
 */
void encoder_set_reset_abm(int index)
{
    Encoder_Config_t *p_encoder_config = &global_config.encoder_config[index];
    Encoder_Status_t *p_encoder_status = &global_status.encoder_status[index];

    switch (p_encoder_config->encoder_type)
    {
    case Encoder_Type_TAMAGAWA:
    case Encoder_Type_HCFA:
        if (!encoder_is_err(index))
        {
            encoder_clear_abm_reset_done(index);
            p_encoder_status->encoder_special_function_flag = ENCODER_SPECIAL_FUNCTION_RESET_ABM;
        }
        break;

    default:
        break;
    }
}

/**
 * @brief 注册编码器二级回调
 * 
 * @param index 编码器编号
 * @param handler 回调函数
 */
void encoder_register_index_handle(int index, encoder_index_callback handler)
{
    index_callback[index] = handler;
}

/**
 * @brief 取消注册编码器二级回调
 * 
 * @param index 编码器编号
 */
void encoder_unregister_index_handle(int index)
{
    index_callback[index] = NULL;
}

/**
 * @brief 设置编码器零点偏置
 * 
 * @param index 编码器编号
 * @param offset 零点偏置
 */
void encoder_set_zero_offset(int index, float offset)
{
    Encoder_Config_t *p_encoder_config = &global_config.encoder_config[index];
    Encoder_Status_t *p_encoder_status = &global_status.encoder_status[index];

    switch (p_encoder_config->encoder_type)
    {
    case Encoder_Type_TAMAGAWA:
    case Encoder_Type_HCFA:
    case Encoder_Type_SPI_AS5048A:
        p_encoder_status->encoder_abs_zero_offset    = offset;
        p_encoder_status->encoder_abs_position_valid = false;
        break;

    default:
        break;
    }
}

/**
 * @brief 获取编码器零点偏置
 * 
 * @param index 编码器编号
 * @return float 零点偏置
 */
float encoder_get_zero_offset(int index)
{
    Encoder_Config_t *p_encoder_config = &global_config.encoder_config[index];
    Encoder_Status_t *p_encoder_status = &global_status.encoder_status[index];

    switch (p_encoder_config->encoder_type)
    {
    case Encoder_Type_ABZ:
        return 0;

    case Encoder_Type_TAMAGAWA:
    case Encoder_Type_HCFA:
    case Encoder_Type_SPI_AS5048A:
        if (p_encoder_config->encoder_mult_turn_bits != 0)
        {
            return p_encoder_status->encoder_abs_zero_offset;
        }
        else
        {
            return p_encoder_status->encoder_abs_zero_offset - (int)p_encoder_status->encoder_abs_zero_offset;
        }

    default:
        break;
    }
}

/**
 * @brief 获取编码器回零偏置
 * 
 * @param index 编码器编号
 * @param offset 位置偏置
 */
void encoder_set_home_offset(int index, float offset)
{
    Encoder_Config_t *p_encoder_config = &global_config.encoder_config[index];
    Encoder_Status_t *p_encoder_status = &global_status.encoder_status[index];

    switch (p_encoder_config->encoder_type)
    {
    case Encoder_Type_ABZ:
        p_encoder_status->encoder_delta_home_offset    = offset;
        p_encoder_status->encoder_delta_position_valid = false;
        break;

    case Encoder_Type_TAMAGAWA:
    case Encoder_Type_HCFA:
    case Encoder_Type_SPI_AS5048A:
        p_encoder_status->encoder_abs_home_offset    = offset;
        p_encoder_status->encoder_abs_position_valid = false;
        break;

    default:
        break;
    }
}

/**
 * @brief 设置编码器回零偏置
 * 
 * @param index 编码器编号
 * @return float 位置偏置
 */
float encoder_get_home_offset(int index)
{
    Encoder_Config_t *p_encoder_config = &global_config.encoder_config[index];
    Encoder_Status_t *p_encoder_status = &global_status.encoder_status[index];

    switch (p_encoder_config->encoder_type)
    {
    case Encoder_Type_ABZ:
        return p_encoder_status->encoder_delta_home_offset;

    case Encoder_Type_TAMAGAWA:
    case Encoder_Type_HCFA:
    case Encoder_Type_SPI_AS5048A:
        return p_encoder_status->encoder_abs_home_offset;

    default:
        break;
    }
}

/**
 * @brief 刷新编码器位置信息
 * 
 * @param index 编码器编号
 */
void encoder_refresh_pos(int index)
{
    Encoder_Config_t *p_encoder_config = &global_config.encoder_config[index];
    Encoder_Status_t *p_encoder_status = &global_status.encoder_status[index];

    switch (p_encoder_config->encoder_type)
    {
    case Encoder_Type_ABZ: {
        p_encoder_status->encoder_delta_fractional = (float)p_encoder_status->encoder_delta_cnt * p_encoder_status->encoder_delta_cnt_inv;
        if (p_encoder_config->encoder_delta_dir == 1)
        {
            p_encoder_status->encoder_pos_fractional = p_encoder_status->encoder_delta_fractional;
        }
        else
        {
            p_encoder_status->encoder_pos_fractional = 1.0f - p_encoder_status->encoder_delta_fractional;
        }


        float encoder_delta_pos = p_encoder_status->encoder_delta_fractional + (float)p_encoder_status->encoder_delta_turn - p_encoder_status->encoder_delta_home_offset * p_encoder_config->encoder_delta_dir;

        p_encoder_status->encoder_delta_pos      = encoder_delta_pos;
        p_encoder_status->encoder_delta_pos_last = encoder_delta_pos;
        p_encoder_status->encoder_pos            = encoder_delta_pos * p_encoder_config->encoder_delta_dir;
    }
    break;

    case Encoder_Type_TAMAGAWA: 
    case Encoder_Type_HCFA: 
    case Encoder_Type_SPI_AS5048A:
    {
        int32_t encoder_abs_turn = p_encoder_status->encoder_abs_turn;
        uint32_t encoder_abs_cnt;
        p_encoder_status->encoder_abs_cnt_last = p_encoder_status->encoder_abs_cnt;
        encoder_abs_cnt                        = encoder_get_abs(index);

        if (p_encoder_config->encoder_mult_turn_bits != 0)
        {
            encoder_abs_turn = encoder_get_abm(index);
        }

        float encoder_abs_fractional = (float)encoder_abs_cnt * p_encoder_status->encoder_abs_cnt_inv;
        float encoder_abs_pos        = encoder_abs_fractional + (float)encoder_abs_turn - p_encoder_status->encoder_abs_zero_offset - p_encoder_status->encoder_abs_home_offset * p_encoder_config->encoder_abs_dir;

        p_encoder_status->encoder_abs_cnt        = encoder_abs_cnt;
        p_encoder_status->encoder_abs_turn       = encoder_abs_turn;
        p_encoder_status->encoder_abs_fractional = encoder_abs_fractional;
        if (p_encoder_config->encoder_abs_dir == 1)
        {
            p_encoder_status->encoder_pos_fractional = encoder_abs_fractional;
        }
        else
        {
            p_encoder_status->encoder_pos_fractional = 1.0f - encoder_abs_fractional;
        }
        p_encoder_status->encoder_abs_pos      = encoder_abs_pos;
        p_encoder_status->encoder_abs_pos_last = encoder_abs_pos;
        p_encoder_status->encoder_pos          = encoder_abs_pos * p_encoder_config->encoder_abs_dir;
    }
    break;

    default:
        motor_set_ext_err(index, EXT_FAULT_CODE_CFG_ENCODER_TYPE);
        break;
    }
}

/**
 * @brief 将编码器位置向上或向下取整
 * 
 * @param index 编码器编号
 * @param dir 取整方向
 * @param home_offset 回零偏置
 */
void encoder_round_pos(int index, int dir, float home_offset)
{
    Encoder_Config_t *p_encoder_config = &global_config.encoder_config[index];
    Encoder_Status_t *p_encoder_status = &global_status.encoder_status[index];

    switch (p_encoder_config->encoder_type)
    {
    case Encoder_Type_TAMAGAWA:
    case Encoder_Type_HCFA:
    case Encoder_Type_SPI_AS5048A:
    {
        float zero_offset = p_encoder_status->encoder_abs_fractional + p_encoder_status->encoder_abs_turn;
        if (dir == 1)
        {
            zero_offset = ceilf(zero_offset);
        }
        else
        {
            zero_offset = floorf(zero_offset);
        }
        p_encoder_status->encoder_abs_zero_offset      = zero_offset;
        p_encoder_status->encoder_abs_home_offset      = home_offset;
        p_encoder_status->encoder_delta_position_valid = false;
        p_encoder_status->encoder_abs_position_valid   = false;
    }
    break;

    default:
        motor_set_ext_err(index, EXT_FAULT_CODE_CFG_ENCODER_TYPE);
        break;
    }
}

/**
 * @brief 同步编码器的绝对位置和增量位置
 * 
 * @param index 编码器编号
 */
void encoder_sync_pos(int index)
{
    Encoder_Config_t *p_encoder_config = &global_config.encoder_config[index];
    Encoder_Status_t *p_encoder_status = &global_status.encoder_status[index];

    switch (p_encoder_config->encoder_type)
    {
    default:
        break;
    }
}
