/**
 * @file hfi.c
 * @author jinchl (jcl709905683@gmail.com)
 * @brief 
 * @version 0.1
 * @date 2026-08-07
 * 
 * @copyright Copyright (c) 2026
 * 
 */


#include "hfi.h"

#include <math.h>
#include <string.h>

#include "board.h"
#include "global.h"

#include "my_math.h"
#include "motor.h"
#include "foc.h"

static volatile uint32_t hfi_response_sequence[BOARD_SUPPORTED_MOTOR_NUMBER] = {0};

/**
 * @brief HFI初始化
 * 
 * @param index 电机编号
 * @param enable_pole_identify 是否进行极性辨识.true:用于无感零速启动 false:用于无感低速启动,实际角度与观测器同步无需极性辨识
 */
void hfi_init(int index, int enable_pole_identify)
{
    HFI_Config_t *p_hfi_config = &global_config.hfi_config[index];
    FOC_Status_t *p_foc_status = &global_status.foc_status[index];
    HFI_Status_t *p_hfi_status = &global_status.hfi_status[index];
    PLL_t *p_hfi_pll           = &hfi_pll[index];

    memset(p_hfi_status, 0, sizeof(HFI_Status_t));
    hfi_response_sequence[index] = 0;

    memcpy(&p_hfi_status->uab_k1, &p_foc_status->u_ab, sizeof(AB_Axis_t));
    memcpy(&p_hfi_status->uab_k0, &p_foc_status->u_ab, sizeof(AB_Axis_t));

    memcpy(&p_hfi_status->iab_k1, &p_foc_status->i_ab, sizeof(AB_Axis_t));
    memcpy(&p_hfi_status->iab_k0, &p_foc_status->i_ab, sizeof(AB_Axis_t));

    if (p_hfi_config->div < 1)
    {
        p_hfi_config->div = 1;
    }

    float hfi_update_freq = (float)BOARD_HRT_FREQ / (float)p_hfi_config->div;

    p_hfi_status->enable_pole_identify       = enable_pole_identify;
    p_hfi_status->sign                       = 1;
    p_hfi_status->sign_last                  = 0;
    p_hfi_status->pole_identify_sign         = 0;
    p_hfi_status->pole_identify_status       = HFI_Pole_Identify_Init;
    p_hfi_status->pole_identify_stable_cnt   = US_TO_HRT_TICKS(p_hfi_config->pole_identify_stable_us);
    p_hfi_status->pole_identify_pulse_cnt    = US_TO_HRT_TICKS(p_hfi_config->pole_identify_pulse_us);
    p_hfi_status->pole_identify_interval_cnt = US_TO_HRT_TICKS(p_hfi_config->pole_identify_interval_us);

    QPLL_Init(p_hfi_pll, p_hfi_config->pll_fc, p_hfi_config->pll_fc, 1.0f / hfi_update_freq, NULL);
}

/**
 * @brief HFI更新
 * 
 * @param index 电机编号
 */
void hfi_update(int index)
{
    HFI_Config_t *p_hfi_config = &global_config.hfi_config[index];
    FOC_Status_t *p_foc_status = &global_status.foc_status[index];
    HFI_Status_t *p_hfi_status = &global_status.hfi_status[index];
    PLL_t *p_hfi_pll           = &hfi_pll[index];

    if (++p_hfi_status->div_cnt >= p_hfi_config->div)
    {
        p_hfi_status->div_cnt = 0;

        if (p_hfi_status->is_open_loop)
        {
            p_hfi_status->udq_excitation.D = p_hfi_status->vf_amp * p_hfi_status->sign;
        }
        else
        {
            p_hfi_status->udq_excitation.D = p_hfi_config->amp * p_hfi_status->sign;
        }

        // voltage
        memcpy(&p_hfi_status->uab_k1, &p_hfi_status->uab_k0, sizeof(AB_Axis_t));
        memcpy(&p_hfi_status->uab_k0, &p_foc_status->u_ab, sizeof(AB_Axis_t));

        p_hfi_status->uab_response.Alpha = (p_hfi_status->uab_k0.Alpha - p_hfi_status->uab_k1.Alpha) * 0.5f;
        p_hfi_status->uab_response.Beta  = (p_hfi_status->uab_k0.Beta - p_hfi_status->uab_k1.Beta) * 0.5f;
        p_hfi_status->uab_base.Alpha     = (p_hfi_status->uab_k0.Alpha + p_hfi_status->uab_k1.Alpha) * 0.5f;
        p_hfi_status->uab_base.Beta      = (p_hfi_status->uab_k0.Beta + p_hfi_status->uab_k1.Beta) * 0.5f;

        // current
        memcpy(&p_hfi_status->iab_k1, &p_hfi_status->iab_k0, sizeof(AB_Axis_t));
        memcpy(&p_hfi_status->iab_k0, &p_foc_status->i_ab, sizeof(AB_Axis_t));

        p_hfi_status->iab_response.Alpha = (p_hfi_status->iab_k0.Alpha - p_hfi_status->iab_k1.Alpha) * 0.5f;
        p_hfi_status->iab_response.Beta  = (p_hfi_status->iab_k0.Beta - p_hfi_status->iab_k1.Beta) * 0.5f;
        p_hfi_status->iab_base.Alpha     = (p_hfi_status->iab_k0.Alpha + p_hfi_status->iab_k1.Alpha) * 0.5f;
        p_hfi_status->iab_base.Beta      = (p_hfi_status->iab_k0.Beta + p_hfi_status->iab_k1.Beta) * 0.5f;

        /*
         * 同步解调： 
         *
         *   iab_response ≈ sign_last * I(theta_rotor)
         *   iab_input    ≈ I(theta_rotor)
         *
         * 本次采样得到的响应通常对应上一次更新施加的注入电压，因此这里
         * 使用 `sign_last`，而不是当前的 `sign`。该乘法只消除注入电压正
         * 负交替造成的极性变化，不会把 alpha/beta 分量变成非负值；分量
         * 的正负号仍用于保留响应矢量的象限信息。
         */
        p_hfi_status->iab_input.Alpha = p_hfi_status->sign_last * p_hfi_status->iab_response.Alpha;
        p_hfi_status->iab_input.Beta  = p_hfi_status->sign_last * p_hfi_status->iab_response.Beta;

        hfi_response_sequence[index]++;

        /* 保存刚刚处理的响应所对应的注入极性，然后为下一次高频注入翻转极性。 */
        p_hfi_status->sign_last  = p_hfi_status->sign;
        p_hfi_status->sign      *= -1;

        /*
         * `iab_input` 是 alpha-beta 坐标系下的响应矢量。虽然可以使用
         * atan2(Beta, Alpha) 计算瞬时相位，但该方法对噪声非常敏感，且当
         * 响应矢量幅值较小时角度会变得不可靠。因此这里使用 QPLL 对相位
         * 进行连续跟踪。
         */
        // p_hfi_status->theta = Normalize_Rad(atan2f(p_hfi_status->iab_input.Beta, p_hfi_status->iab_input.Alpha));

        /*
         * PLL 相位检测器的输入为：
         *
         *   sin_value = iab_input.Beta
         *   cos_value = iab_input.Alpha
         *
         * QPLL 直接使内部估计角度跟踪该电流响应矢量的相位，它不能直接
         * 读取未知的真实转子角度。但是，响应相位同时取决于真实转子角度
         * 和当前注入角度：由于 Ld != Lq，注入角度误差会改变 d/q 电流比例，
         * 从而改变响应矢量方向。在稳定的闭环平衡点，响应相位与估计角度
         * 重合，通常对应注入方向与转子 D 轴对齐的情况。
         *
         * PLL 先计算相位误差，再通过 PI 控制器得到估计角速度，并对角速度
         * 积分更新 theta。由于响应可能存在 pi 极性歧义，下面的极性识别
         * 状态机会通过正、负 D 轴电流脉冲来消除该歧义。
         */
        QPLL_Run(p_hfi_pll, p_hfi_status->iab_input.Beta, p_hfi_status->iab_input.Alpha, true);
        p_hfi_status->theta = p_hfi_pll->theta;

        p_foc_status->theta_hfi = p_hfi_pll->theta;
        p_foc_status->we_hfi    = p_hfi_pll->we_f;
    }
}

/**
 * @brief HFI极性辨识更新
 * 
 * @param index 电机编号
 */
void hfi_pole_identify_update(int index)
{
    HFI_Config_t *p_hfi_config = &global_config.hfi_config[index];
    FOC_Status_t *p_foc_status = &global_status.foc_status[index];
    HFI_Status_t *p_hfi_status = &global_status.hfi_status[index];
    PLL_t *p_hfi_pll           = &hfi_pll[index];

    p_hfi_status->pole_identify_cnt++;
    switch (p_hfi_status->pole_identify_status)
    {
    case HFI_Pole_Identify_Init:
        p_hfi_status->pole_identify_sign   = 0;
        p_hfi_status->pole_identify_status = HFI_Pole_Identify_Wait_Stable_Zero;
        p_hfi_status->pole_identify_cnt    = 0;
        p_hfi_status->id_positive_sum      = 0;
        p_hfi_status->id_negative_sum      = 0;
        p_hfi_status->id_positive          = 0;
        p_hfi_status->id_negative          = 0;
        break;

    case HFI_Pole_Identify_Wait_Stable_Zero:
        if (p_hfi_status->pole_identify_cnt >= p_hfi_status->pole_identify_interval_cnt)
        {
            p_hfi_status->pole_identify_sign   = 1;
            p_hfi_status->pole_identify_status = HFI_Pole_Identify_Positive_pulse;
            p_hfi_status->pole_identify_cnt    = 0;
        }
        break;

    /*
    * 高频凸极响应通常具有 PI 周期性，因此仅依靠 PLL 可能锁定在
    * theta_rotor 或 theta_rotor + PI。这里分别施加正、负 D 轴电流脉冲，
    * 比较测得的 D 轴电流幅值，从而选择实际的磁极方向。
    */
    case HFI_Pole_Identify_Positive_pulse:
        if (p_hfi_status->pole_identify_cnt >= p_hfi_status->pole_identify_pulse_cnt)
        {
            p_hfi_status->id_positive          += fabsf(p_foc_status->i_dq.D);
            p_hfi_status->pole_identify_sign    = 0;
            p_hfi_status->pole_identify_cnt     = 0;
            p_hfi_status->pole_identify_status  = HFI_Pole_Identify_Wait_Stable_Positive;
        }
        break;

    case HFI_Pole_Identify_Wait_Stable_Positive:
        if (p_hfi_status->pole_identify_cnt >= p_hfi_status->pole_identify_interval_cnt)
        {
            p_hfi_status->pole_identify_sign   = -1;
            p_hfi_status->pole_identify_cnt    = 0;
            p_hfi_status->pole_identify_status = HFI_Pole_Identify_Negative_pulse;
        }
        break;

    case HFI_Pole_Identify_Negative_pulse:
        if (p_hfi_status->pole_identify_cnt >= p_hfi_status->pole_identify_pulse_cnt)
        {
            p_hfi_status->id_negative          += fabsf(p_foc_status->i_dq.D);
            p_hfi_status->pole_identify_sign    = 0;
            p_hfi_status->pole_identify_cnt     = 0;
            p_hfi_status->pole_identify_status  = HFI_Pole_Identify_Wait_Stable_Negative;
        }
        break;

    case HFI_Pole_Identify_Wait_Stable_Negative:
        if (p_hfi_status->pole_identify_cnt >= p_hfi_status->pole_identify_interval_cnt)
        {
            float diff = p_hfi_status->id_positive - p_hfi_status->id_negative;
            float avg  = 0.5f * (p_hfi_status->id_positive + p_hfi_status->id_negative);
            if (fabsf(diff) / avg < 0.01f || avg < 0.05f)
            {
                p_hfi_status->pole_identify_status = HFI_Pole_Identify_Retry;
            }
            else
            {
                p_hfi_status->pole_identify_status = HFI_Pole_Identify_Done;
                if (p_hfi_status->id_positive < p_hfi_status->id_negative)
                {
                    p_hfi_pll->theta        += _PI;
                    p_hfi_pll->theta         = Normalize_Rad(p_hfi_pll->theta);
                    p_hfi_status->theta      = p_hfi_pll->theta;
                    p_foc_status->theta_hfi  = p_hfi_pll->theta;
                }
            }
        }
        break;

    case HFI_Pole_Identify_Retry:
        if (p_hfi_status->pole_identify_cnt >= p_hfi_status->pole_identify_interval_cnt)
        {
            p_hfi_status->pole_identify_sign   = 1;
            p_hfi_status->pole_identify_status = HFI_Pole_Identify_Positive_pulse;
            p_hfi_status->pole_identify_cnt    = 0;
            p_hfi_status->id_positive_sum      = 0;
            p_hfi_status->id_negative_sum      = 0;
            p_hfi_status->id_positive          = 0;
            p_hfi_status->id_negative          = 0;
            if (++p_hfi_status->pole_identify_retry_cnt >= POLE_IDENTIFY_MAX_RETRY_TIMES)
            {
                motor_set_err(index, FAULT_CODE_HFI);
            }
        }
        break;

    case HFI_Pole_Identify_Sync_With_External: {
        float rad_diff = ABS_Rad_Delta(p_foc_status->theta, p_hfi_pll->theta);

        if (rad_diff > _PI_2)
        {
            // if(++p_hfi_status->pole_identify_cnt > p_hfi_status->pole_identify_pulse_cnt)
            {
                p_hfi_status->pole_identify_cnt  = 0;
                p_hfi_pll->theta                += _PI;
                p_hfi_pll->theta                 = Normalize_Rad(p_hfi_pll->theta);
                p_hfi_status->theta              = p_hfi_pll->theta;
            }
            // else
            // {
            //     p_hfi_status->pole_identify_cnt = 0;
            // }
        }
        else
        {
            if (p_hfi_status->pole_identify_cnt > p_hfi_status->pole_identify_pulse_cnt)
            {
                p_hfi_status->pole_identify_cnt    = 0;
                p_hfi_status->pole_identify_status = HFI_Pole_Identify_Done;
            }
        }
    }
    break;


    case HFI_Pole_Identify_Done:
        p_foc_status->i_dq_ref.D = 0;
        break;

    default:
        break;
    }

    p_hfi_status->udq_excitation.D = p_hfi_config->pole_identify_amp * p_hfi_status->pole_identify_sign;
}

/**
 * @brief 获取最新高频响应的序号
 * 每当 hfi_update() 完成一次新的电压/电流响应解调时递增，用于避免低频任务对同一份高频响应重复处理。
 * @param index 电机编号
 * @return uint32_t 响应序号
 */
uint32_t hfi_get_response_sequence(int index)
{
    return hfi_response_sequence[index];
}

/**
 * @brief 判断HFI结果是否有效
 * 
 * @param index 电机编号
 * @return true 结果有效
 * @return false 结果无效
 */
bool is_hfi_valid(int index)
{
    HFI_Status_t *p_hfi_status = &global_status.hfi_status[index];

    return p_hfi_status->pole_identify_status == HFI_Pole_Identify_Done;
}

/**
 * @brief 判断HFI是否已经在低速下建立稳定跟踪
 * 
 * @param index 电机编号
 * @return true 已经稳定跟踪
 * @return false 没有稳定跟踪
 */
bool is_hfi_locked(int index)
{
    PLL_t *p_hfi_pll = &hfi_pll[index];

    return p_hfi_pll->is_locked && fabsf(p_hfi_pll->we_f) < HFI_LOW_SPEED_THRESHOLD;
}
