/**
 * @file foc.c
 * @author jinchl (jcl709905683@gmail.com)
 * @brief 
 * @version 0.1
 * @date 2026-03-30
 * 
 * @copyright Copyright (c) 2026
 * 
 */


#include "foc.h"

#include <math.h>
#include <string.h>

#include "board.h"
#include "global.h"

#include "my_math.h"
#include "motor.h"
#include "encoder.h"
#include "hfi.h"
#include "inverter.h"

#if defined(HFI_DEBUG) && defined(FAST_CURRENT_LOOP_ENABLE)
#error "HFI_DEBUG cannot be used with FAST_CURRENT_LOOP_ENABLE"
#endif

#ifndef FOC_LS_HFI_SAMPLE_TO_PWM_DELAY_S
/*
 * ADC电流采样到新HFI电压在PWM reload生效的延迟。
 *
 * ADC时钟为50MHz。单通道采样保持时间为20个周期，即0.40us；
 * 16位转换时间为21个周期，即0.42us；每通道合计0.82us。
 * 四通道的采样保持完成时刻依次约为0.40、1.22、2.04、2.86us，
 * 全部转换完成约为3.28us。
 *
 * PWM=100kHz、HRT=50kHz时，PWM每10us reload一次：
 *   延迟约为7.14~9.60us；若错过10us reload，则为17.14~19.60us。
 *
 * PWM=50kHz、HRT=50kHz时，PWM每20us reload一次：
 *   延迟约为17.14~19.60us。
 */
// #define FOC_LS_HFI_SAMPLE_TO_PWM_DELAY_S (8.37e-6f)
#define FOC_LS_HFI_SAMPLE_TO_PWM_DELAY_S (1.8e-5f)
#endif

#define FOC_LS_SCAN_BIN_COUNT      (72U)
#define FOC_LS_EXTREME_BIN_COUNT   (5U)
#define FOC_LS_MIN_SAMPLES_PER_BIN (4U)

typedef struct
{
    float inductance_sum[FOC_LS_SCAN_BIN_COUNT];
    uint16_t sample_count[FOC_LS_SCAN_BIN_COUNT];
    uint32_t last_response_sequence;
} FOC_Ls_Scan_t;

static FOC_Ls_Scan_t foc_ls_scan[BOARD_SUPPORTED_MOTOR_NUMBER] = {0};

#define FLUX_IDENTIFY_WE         (_2PI * 30.0f)
#define FLUX_IDENTIFY_MAX_WE_ERR (_2PI * 5.0f)
#define FLUX_IDENTIFY_KP         (0.001f)
#define FLUX_IDENTIFY_KI         (0.01f)

static inline bool is_MTPA_allowed(int index)
{
#ifndef FAST_CURRENT_LOOP_ENABLE
    Motor_Config_t *p_motor_config                       = &global_config.motor_config[index];
    Motor_Controller_Config_t *p_motor_controller_config = &global_config.motor_controller_config[index];

    return p_motor_controller_config->mtpa_enable && p_motor_config->saliency_ratio > FOC_MTPA_SALIENCY_RATE;
#else
    return false;
#endif
}

static inline bool is_FW_allowed(int index)
{
#ifndef FAST_CURRENT_LOOP_ENABLE
    Motor_Controller_Config_t *p_motor_controller_config = &global_config.motor_controller_config[index];
    FOC_Status_t *p_foc_status                           = &global_status.foc_status[index];

    return p_motor_controller_config->fw_enable && p_foc_status->us_out_rate_raw > FOC_FW_VOLTAGE_RATE;
#else
    return false;
#endif
}

/**
 * @brief MTPA计算
 * 
 * @param p_motor_config 电机本体配置结构体指针
 * @param p_foc_status foc运行反馈结构体指针
 */
void MTPA_Cal(Motor_Config_t *p_motor_config, FOC_Status_t *p_foc_status)
{
    float l_diff = p_motor_config->L_diff;

    if (fabsf(l_diff) < FLOAT_EPSILON)
    {
        p_foc_status->i_dq_ref.D = 0;
        p_foc_status->i_dq_ref.Q = CONSTRAIN(p_foc_status->current_ref, -p_motor_config->peak_current, p_motor_config->peak_current);
        return;
    }

    /*
        方法一：低电流近似公式
        Id = ΔL × Iq² / ψ
        https://zhuanlan.zhihu.com/p/558759346
        它是方法二在 |ΔL·Iq| << ψ 条件下的泰勒展开。
        优点：
        计算量最低，不需要开平方；
        低电流、弱凸极电机上结果接近精确公式；
        输出连续，适合算力很紧张的控制器。
        缺点：
        高电流下误差迅速增大；
        |Id| 随 Iq² 增长，通常会给出过大的负 d 轴电流；
        增加铜耗，并可能增加退磁风险；
        Flux 接近零时会除零，不适用于同步磁阻电机；
        电机饱和后 Ld/Lq/Flux 变化，误差更明显。
        适合：低电流区域、凸极率不高、只需要简单近似的场合。
     */
    // p_foc_status->i_dq_ref.D = SQ(p_foc_status->current_ref) * l_diff / p_motor_config->Flux;

    /*
        方法二：以 Iq 为自变量的精确公式
        Id = [sqrt(ψ² + 4ΔL²Iq²) - ψ] / (2ΔL)
        https://zhuanlan.zhihu.com/p/624474437
        优点：
        对给定 Iq 的 MTPA 关系是精确的；
        可以直接使用现有 i_dq_ref.Q；
        不需要改变现有 q 轴电流指令的语义；
        相比方法一，高电流下不会产生过大的 Id。
        缺点：
        添加 Id 后，总电流幅值会大于原来的 |Iq|；
        受峰值电流圆限制后，Id/Iq 可能被同时缩放；
        如果转矩指令仍简单使用 Iq=T/kt，增加的磁阻转矩会使实际转矩高于目标；
        使用 sqrt(x)-ψ 会在低电流时发生浮点消减误差。
        推荐使用数值更稳定的等价形式：
        Id = 2ΔL·Iq² / [sqrt(ψ² + 4ΔL²Iq²) + ψ]
     */
    // p_foc_status->i_dq_ref.D = (2 * l_diff * SQ(p_foc_status->current_ref)) / (sqrtf(SQ(p_motor_config->Flux) + 4 * SQ(l_diff) * SQ(p_foc_status->current_ref)) + p_motor_config->Flux);

    /*
        方法三：以定子电流矢量幅值 Is 为自变量的精确公式（参考 VESC）
        Id = [sqrt(psi^2 + 8*dL^2*Is^2) - psi] / (4*dL)
        Iq = sign(T) * sqrt(Is^2 - Id^2)

        该方法在 Id^2 + Iq^2 = Is^2 的固定电流圆上求最大转矩。这里将进入
        上层未经 MTPA 变换的 current_ref 作为带转矩方向的 Is 指令，计算 Id 后
        必须同步重算 Iq；不能只增加 Id 而保留原 Iq，否则总电流会超过 Is 指令。
        Is 通过独立参数传入，避免高频循环把上一次重算后的 Iq 再次当作 Is，
        导致电流指令在连续调用中递归衰减。

        优点：
        能够在给定总电流限制下获得最大转矩，充分利用逆变器电流能力；
        高电流和强凸极 IPMSM 上通常比低电流近似公式更准确；
        生成的 Id/Iq 天然位于同一个电流圆上，不依赖后续限幅修正比例。
        缺点：
        会改变原 q 轴指令的语义，调用方应把它视为总电流幅值指令；
        直接转矩模式若仍使用 Iq = T/kt，需进一步补偿磁阻转矩造成的转矩偏差；
        固定 Ld/Lq/Flux 未考虑磁饱和，实机最优点仍可能与理论值存在偏差。

        为避免低电流时 sqrt(x) - psi 的浮点消减误差，实际使用等价形式：
        Id = 2*dL*Is^2 / [sqrt(psi^2 + 8*dL^2*Is^2) + psi]
     */
    float is_ref  = fabsf(p_foc_status->current_ref);
    float iq_sign = p_foc_status->current_ref < 0.0f ? -1.0f : 1.0f;

    if (is_ref < FLOAT_EPSILON)
    {
        p_foc_status->i_dq_ref.D = 0.0f;
        p_foc_status->i_dq_ref.Q = 0.0f;
        return;
    }

    float mtpa_root = sqrtf(SQ(p_motor_config->Flux) + 8.0f * SQ(l_diff * is_ref));
    float id_ref    = 2.0f * l_diff * SQ(is_ref) / (mtpa_root + p_motor_config->Flux);
    float iq_ref    = sqrtf(fmaxf(0.0f, SQ(is_ref) - SQ(id_ref)));

    p_foc_status->i_dq_ref.D = id_ref;
    p_foc_status->i_dq_ref.Q = iq_sign * iq_ref;
}

/**
 * @brief 根据目标扭矩计算MTPA总电流参考值
 *
 * @param motor 电机本体配置结构体指针
 * @param torque_ref 扭矩参考
 * @return 带扭矩方向的定子电流矢量幅值参考
 */
float IPM_MTPA_From_Torque(const Motor_Config_t *motor, float torque_ref)
{
    const float torque_abs = fabsf(torque_ref);
    const float is_sign    = torque_ref < 0.0f ? -1.0f : 1.0f;

    const float flux               = motor->Flux;
    const float l_diff             = motor->Ld - motor->Lq;
    const float torque_coefficient = 1.5f * motor->pp;

    if (torque_abs < FLOAT_EPSILON)
    {
        return 0.0f;
    }

    /*
     * 无凸极效应或凸极效应很弱时退化为表贴式 PMSM：
     * T = 1.5 * pp * Flux * Iq
     */
    if (fabsf(l_diff) < FLOAT_EPSILON)
    {
        float denominator = torque_coefficient * flux;

        if (fabsf(denominator) > FLOAT_EPSILON)
        {
            return CONSTRAIN(torque_ref / denominator,
                             -motor->peak_current,
                             motor->peak_current);
        }
        return 0.0f;
    }

    float is_low  = 0.0f;
    float is_high = motor->peak_current;

    /*
     * 检查峰值电流下能够产生的最大 MTPA 扭矩。
     */
    float root_high =
        sqrtf(SQ(flux) + 8.0f * SQ(l_diff * is_high));

    float id_high =
        2.0f * l_diff * SQ(is_high) / (root_high + flux);

    float iq_high =
        sqrtf(fmaxf(0.0f, SQ(is_high) - SQ(id_high)));

    float torque_max =
        torque_coefficient
        * iq_high
        * (flux + l_diff * id_high);

    /*
     * 需求扭矩超过能力时，输出带方向的峰值总电流。
     */
    if (torque_abs >= torque_max)
    {
        return is_sign * is_high;
    }

    /*
     * T(Is) 在正常 IPM MTPA 区域内单调递增，
     * 使用固定次数二分，执行时间确定。
     */
    for (int i = 0; i < 16; i++)
    {
        float is_mid = 0.5f * (is_low + is_high);

        float root =
            sqrtf(SQ(flux) + 8.0f * SQ(l_diff * is_mid));

        float id =
            2.0f * l_diff * SQ(is_mid) / (root + flux);

        float iq =
            sqrtf(fmaxf(0.0f, SQ(is_mid) - SQ(id)));

        float torque =
            torque_coefficient
            * iq
            * (flux + l_diff * id);

        if (torque < torque_abs)
        {
            is_low = is_mid;
        }
        else
        {
            is_high = is_mid;
        }
    }

    return is_sign * 0.5f * (is_low + is_high);
}

/**
 * @brief MTPV计算
 * 
 * @param p_motor_config 电机本体配置结构体指针
 * @param p_foc_status foc运行反馈结构体指针
 */
void MTPV_Cal(Motor_Config_t *p_motor_config, FOC_Status_t *p_foc_status)
{
    // float l_diff = p_motor_config->L_diff;

    // https://zhuanlan.zhihu.com/p/716381269
    /*
        2 * SQ(p_motor_config->Ld) * l_diff * SQ(p_foc_status->i_dq.D) +
        (4 * SQ(p_motor_config->Ld) * p_motor_config->Flux - 2 * p_motor_config->Ld * p_motor_config->Lq * p_motor_config->Flux) * p_foc_status->i_dq.D +
        2 * p_motor_config->Ld * SQ(p_motor_config->Flux) -
        2 * SQ(p_motor_config->Lq) * l_diff * SQ(p_foc_status->i_dq.Q) = 0
    */
}

static inline bool foc_send_align_result(int index)
{
    Encoder_Config_t *p_encoder_config = &global_config.encoder_config[index];
    FOC_Status_t *p_foc_status         = &global_status.foc_status[index];

    switch (p_encoder_config->encoder_type)
    {
        default:
        return true;
    }
}

/**
 * @brief foc编码器更新
 * 
 * @param index 电机编号
 */
static inline void foc_encoder_update(int index)
{
    Motor_Config_t *p_motor_config     = &global_config.motor_config[index];
    FOC_Status_t *p_foc_status         = &global_status.foc_status[index];
    Encoder_Status_t *p_encoder_status = &global_status.encoder_status[index];

    float rads_encoder = p_encoder_status->encoder_pos_fractional * _2PI;

    p_foc_status->rads_encoder = rads_encoder;

    switch (p_motor_config->motor_struct)
    {
    case Motor_Struct_Linear:
        p_foc_status->theta_encoder = Normalize_Rad(rads_encoder - p_foc_status->rads_encoder_offset);
#ifndef FAST_CURRENT_LOOP_ENABLE
        p_foc_status->we_encoder = p_encoder_status->encoder_speed * _2PI;
#endif
        break;

    case Motor_Struct_Rotary:
        p_foc_status->theta_encoder = Normalize_Rad((rads_encoder - p_foc_status->rads_encoder_offset) * p_motor_config->pp);
#ifndef FAST_CURRENT_LOOP_ENABLE
        p_foc_status->we_encoder = p_encoder_status->encoder_speed * p_motor_config->pp * _2PI;
#endif
        break;

    default:
        break;
    }
}

/**
 * @brief foc编码器应用
 * 
 * @param index 电机编号
 */
static inline void foc_encoder_apply(int index)
{
    FOC_Status_t *p_foc_status = &global_status.foc_status[index];

    p_foc_status->theta = p_foc_status->theta_encoder;
    p_foc_status->we    = p_foc_status->we_encoder;

#ifdef FAST_CURRENT_LOOP_ENABLE
    p_foc_status->theta_comp = p_foc_status->theta;
#else
    // 本周期尚未写入PWM，使用上一周期该轴的实测写入时刻预测电压生效的等效延时。
    p_foc_status->theta_comp = p_foc_status->theta + p_foc_status->we * current_loop_delay_calculate(index);
    p_foc_status->theta_comp = Normalize_Rad(p_foc_status->theta_comp);
#endif
}

/**
 * @brief foc hfi 应用
 * 
 * @param index 电机编号
 */
static inline void foc_hfi_apply(int index)
{
    FOC_Status_t *p_foc_status = &global_status.foc_status[index];
    HFI_Status_t *p_hfi_status = &global_status.hfi_status[index];

    p_foc_status->theta = p_foc_status->theta_hfi;
    p_foc_status->we    = p_foc_status->we_hfi;

    // hfi应用在低速状态不需要进行电角度延迟补偿
    p_foc_status->theta_comp = p_foc_status->theta_hfi;

    // 利用基波响应恢复采样值,完成电流闭环
    memcpy(&p_foc_status->u_ab, &p_hfi_status->uab_base, sizeof(AB_Axis_t));
    memcpy(&p_foc_status->i_ab, &p_hfi_status->iab_base, sizeof(AB_Axis_t));
}

/**
 * @brief foc 克拉克变换更新
 * 
 * @param index 电机编号
 */
static inline void foc_clark_update(int index)
{
    Motor_Basic_Status_t *p_motor_basic_status = &global_status.motor_basic_status[index];
    FOC_Status_t *p_foc_status                 = &global_status.foc_status[index];

#ifndef FAST_CURRENT_LOOP_ENABLE
    Clarke_Transmission(&p_motor_basic_status->phase_voltage, &p_foc_status->u_ab);
#else
    memcpy(&p_foc_status->u_ab, &p_foc_status->u_ab_out, sizeof(AB_Axis_t));
#endif
    Clarke_Transmission(&p_motor_basic_status->phase_current, &p_foc_status->i_ab);
}

/**
 * @brief foc 帕克变换更新
 * 
 * @param index 电机编号
 */
static inline void foc_park_update(int index)
{
    FOC_Status_t *p_foc_status = &global_status.foc_status[index];

#ifndef FAST_CURRENT_LOOP_ENABLE
    Park_Transmission_Fast(&p_foc_status->u_ab, &p_foc_status->u_dq, p_foc_status->sin_cos);
#else
    memcpy(&p_foc_status->u_dq, &p_foc_status->u_dq_out, sizeof(AB_Axis_t));
#endif
    Park_Transmission_Fast(&p_foc_status->i_ab, &p_foc_status->i_dq, p_foc_status->sin_cos);
}

/**
 * @brief foc 电压极限圆更新
 * 
 * @param index 电机编号
 */
static inline void foc_us_limit_update(int index)
{
    Motor_Controller_Config_t *p_motor_controller_config = &global_config.motor_controller_config[index];
    Motor_Basic_Status_t *p_motor_basic_status           = &global_status.motor_basic_status[index];
    FOC_Status_t *p_foc_status                           = &global_status.foc_status[index];

    /*
        us_max 为能够调制出的最大电压矢量
        uo_max 为在当前调制比下能够调制出的最大电压矢量
    */
    p_foc_status->us_max = p_motor_basic_status->v_bus * _1_SQRT3;
    p_foc_status->uo_max = p_motor_controller_config->max_modulation_ratio * p_foc_status->us_max;
}

/**
 * @brief foc电压开环模式更新
 * 
 * @param index 电机编号
 */
static inline void foc_vf_update(int index)
{
    FOC_Status_t *p_foc_status = &global_status.foc_status[index];

    Saturate_Vector_2d_Fast(&p_foc_status->u_dq_ref.D, &p_foc_status->u_dq_ref.Q, p_foc_status->uo_max);
    Inverse_Park_Transmission(&p_foc_status->u_dq_ref, &p_foc_status->u_ab_out, p_foc_status->theta_openloop);
    memcpy(&p_foc_status->u_dq_out, &p_foc_status->u_dq_ref, sizeof(DQ_Axis_t));
}

/**
 * @brief foc电流开环模式更新
 * 
 * @param index 电机编号
 */
static inline void foc_if_update(int index)
{
    Motor_Config_t *p_motor_config                     = &global_config.motor_config[index];
    Loop_Controller_Config_t *p_loop_controller_config = &global_config.loop_controller_config[index];
    Loop_Controller_t *p_loop_controller_status        = &global_status.loop_controller_status[index];
    FOC_Status_t *p_foc_status                         = &global_status.foc_status[index];

    PID_Set_Abs_Limit(&p_loop_controller_status->id_pid, p_foc_status->uo_max);
    PID_Set_Abs_Limit(&p_loop_controller_status->iq_pid, p_foc_status->uo_max);

    Saturate_Vector_2d_Fast(&p_foc_status->i_dq_ref.D, &p_foc_status->i_dq_ref.Q, p_motor_config->peak_current);
    switch (p_loop_controller_config->current_controller_config.controller_type)
    {
    case Controller_Type_Parallel_PID:
        p_foc_status->u_dq_out.D = Parallel_PID_Position_Update(&p_loop_controller_status->id_pid, 0, p_foc_status->i_dq_ref.D, p_foc_status->i_dq.D);
        p_foc_status->u_dq_out.Q = Parallel_PID_Position_Update(&p_loop_controller_status->iq_pid, 0, p_foc_status->i_dq_ref.Q, p_foc_status->i_dq.Q);
        break;

    case Controller_Type_Serial_PID:
        p_foc_status->u_dq_out.D = Serial_PID_Position_Update(&p_loop_controller_status->id_pid, 0, p_foc_status->i_dq_ref.D, p_foc_status->i_dq.D);
        p_foc_status->u_dq_out.Q = Serial_PID_Position_Update(&p_loop_controller_status->iq_pid, 0, p_foc_status->i_dq_ref.Q, p_foc_status->i_dq.Q);
        break;

    default:
        break;
    }
    Saturate_Vector_2d_Fast(&p_foc_status->u_dq_out.D, &p_foc_status->u_dq_out.Q, p_foc_status->uo_max);
    Inverse_Park_Transmission(&p_foc_status->u_dq_out, &p_foc_status->u_ab_out, p_foc_status->theta_openloop);
}

/**
 * @brief 使用进入弱磁时的实时工作点自动整定弱磁 PI
 *
 * 高速区近似有 @f$|U_s|\approx |\omega_e|(\psi_f+L_d I_d)@f$，因此
 * 电压对 d 轴电流的小信号增益为 @f$K_{fw}\approx|\omega_e|L_d@f$。
 * 最小对象增益按照当前可用电压和峰值电流自适应缩放，避免速度反馈过小或
 * 异常电压饱和时计算出过大的控制器增益。
 *
 * @param[in] index 电机编号
 */
static void foc_fw_loop_auto_tune(int index)
{
    Motor_Config_t *p_motor_config                     = &global_config.motor_config[index];
    Loop_Controller_Config_t *p_loop_controller_config = &global_config.loop_controller_config[index];
    Loop_Controller_t *p_loop_controller_status        = &global_status.loop_controller_status[index];
    FOC_Status_t *p_foc_status                         = &global_status.foc_status[index];

    if (!p_loop_controller_config->fw_controller_config.auto_tune)
    {
        return;
    }

    float ld           = p_motor_config->Ld;
    float peak_current = p_motor_config->peak_current;
    float bandwidth    = p_loop_controller_config->fw_controller_config.controller_band_width;
    float ti           = MS_TO_S(p_loop_controller_config->fw_controller_config.controller_integral_time);

    if (ld <= FLOAT_MIN || peak_current <= FLOAT_EPSILON || bandwidth <= FLOAT_EPSILON || ti <= FLOAT_EPSILON)
    {
        p_loop_controller_status->fw_pid.config.kp = 0.0f;
        p_loop_controller_status->fw_pid.config.ki = 0.0f;
        return;
    }

    float min_plant_gain = p_foc_status->uo_max / peak_current * FOC_FW_MIN_PLANT_GAIN_RATE;
    min_plant_gain       = MAX(min_plant_gain, FLOAT_EPSILON);
    float plant_gain     = MAX(fabsf(p_foc_status->we) * ld, min_plant_gain);

    switch (p_loop_controller_config->fw_controller_config.controller_type)
    {
    case Controller_Type_Parallel_PID:
        Flux_Weakening_Loop_Parallel_PID_Tune(&p_loop_controller_status->fw_pid,
                                              bandwidth, ti, plant_gain);
        break;

    case Controller_Type_Serial_PID:
        Flux_Weakening_Loop_Serial_PID_Tune(&p_loop_controller_status->fw_pid,
                                            bandwidth, ti, plant_gain);
        break;

    default:
        p_loop_controller_status->fw_pid.config.kp = 0.0f;
        p_loop_controller_status->fw_pid.config.ki = 0.0f;
        break;
    }
}

/**
 * @brief 更新 FOC 运行区域并执行弱磁电压外环
 *
 * 弱磁环工作流程：
 * 1. ID0/MTPA 区根据未限幅电压矢量利用率判断是否达到弱磁进入阈值；
 * 2. 进入弱磁区后，以可用电压的 FOC_FW_VOLTAGE_RATE 为目标；
 * 3. PI 输出负 d 轴电流给定，通过减小等效永磁体磁链降低反电动势；
 * 4. d/q 轴电流矢量最终受峰值电流圆限制；
 * 5. 电压利用率低于退出阈值后返回 ID0 或 MTPA 区，并清除弱磁积分状态。
 *
 * @param[in] index 电机编号
 */
static inline void foc_op_area_update(int index)
{
    Motor_Config_t *p_motor_config                       = &global_config.motor_config[index];
    Motor_Controller_Config_t *p_motor_controller_config = &global_config.motor_controller_config[index];
    Loop_Controller_Config_t *p_loop_controller_config   = &global_config.loop_controller_config[index];
    Loop_Controller_t *p_loop_controller_status          = &global_status.loop_controller_status[index];
    FOC_Status_t *p_foc_status                           = &global_status.foc_status[index];

    switch (p_foc_status->op_area)
    {
    case FOC_OP_ID0:
        p_foc_status->i_dq_ref.D = 0;
        p_foc_status->i_dq_ref.Q = CONSTRAIN(p_foc_status->current_ref, -p_motor_config->peak_current, p_motor_config->peak_current);
        if (is_FW_allowed(index))
        {
            PID_Reset(&p_loop_controller_status->fw_pid);
            foc_fw_loop_auto_tune(index);
            p_foc_status->op_area = FOC_OP_FW;
        }
        break;

    case FOC_OP_MTPA:
        MTPA_Cal(p_motor_config, p_foc_status);
        Saturate_Vector_2d_Fast(&p_foc_status->i_dq_ref.D, &p_foc_status->i_dq_ref.Q, p_motor_config->peak_current);
        if (is_FW_allowed(index))
        {
            PID_Reset(&p_loop_controller_status->fw_pid);
            foc_fw_loop_auto_tune(index);
            p_foc_status->op_area = FOC_OP_FW;
        }
        break;

    case FOC_OP_FW:
        if (p_motor_controller_config->fw_enable && p_foc_status->us_out_rate_raw > FOC_FW_EXIT_VOLTAGE_RATE)
        {
            switch (p_loop_controller_config->fw_controller_config.controller_type)
            {
            case Controller_Type_Parallel_PID:
                p_foc_status->i_dq_ref.D = Parallel_PID_Position_Update(&p_loop_controller_status->fw_pid, 0, p_foc_status->uo_max * FOC_FW_VOLTAGE_RATE, p_foc_status->us_out_raw_f);
                break;

            case Controller_Type_Serial_PID:
                p_foc_status->i_dq_ref.D = Serial_PID_Position_Update(&p_loop_controller_status->fw_pid, 0, p_foc_status->uo_max * FOC_FW_VOLTAGE_RATE, p_foc_status->us_out_raw_f);
                break;

            default:
                break;
            }

            if (p_loop_controller_status->fw_pid.status.saturation)
            {
                motor_set_warning(index, WARN_CODE_FW_LOOP_SATURATION);
            }
        }
        else
        {
            PID_Reset(&p_loop_controller_status->fw_pid);
            if (is_MTPA_allowed(index))
            {
                p_foc_status->op_area = FOC_OP_MTPA;
            }
            else
            {
                p_foc_status->op_area = FOC_OP_ID0;
            }
        }

        Saturate_Vector_2d_Fast(&p_foc_status->i_dq_ref.D, &p_foc_status->i_dq_ref.Q, p_motor_config->peak_current);
        break;

    case FOC_OP_MTPV:
        break;

    case FOC_OP_MANUAL:
        Saturate_Vector_2d_Fast(&p_foc_status->i_dq_ref.D, &p_foc_status->i_dq_ref.Q, p_motor_config->peak_current);
        break;

    default:
        break;
    }
}

/**
 * @brief foc电流环前馈解耦更新
 * 
 * @param index 电机编号
 */
static inline void foc_decoupling_update(int index)
{
    Motor_Config_t *p_motor_config                       = &global_config.motor_config[index];
    Motor_Controller_Config_t *p_motor_controller_config = &global_config.motor_controller_config[index];
    FOC_Status_t *p_foc_status                           = &global_status.foc_status[index];

    if (p_motor_controller_config->current_decoupling_type > Current_Decoupling_Type_None || p_foc_status->overide_decoupling_update)
    {
        p_foc_status->i_dq_delta.D = p_foc_status->i_dq.D - p_foc_status->i_dq_last.D;
        p_foc_status->i_dq_delta.Q = p_foc_status->i_dq.Q - p_foc_status->i_dq_last.Q;
        FirstOrder_LPF_Cal(p_foc_status->i_dq_delta.D, p_foc_status->i_dq_delta_f.D, 0.1f);
        FirstOrder_LPF_Cal(p_foc_status->i_dq_delta.Q, p_foc_status->i_dq_delta_f.Q, 0.1f);
        p_foc_status->u_dq_rl.D    = p_foc_status->i_dq.D * p_motor_config->Rs + p_motor_config->Ld * p_foc_status->i_dq_delta_f.D / BOARD_HRT_DT;
        p_foc_status->u_dq_rl.Q    = p_foc_status->i_dq.Q * p_motor_config->Rs + p_motor_config->Lq * p_foc_status->i_dq_delta_f.Q / BOARD_HRT_DT;
        p_foc_status->u_dq_cross.D = -p_foc_status->we * p_motor_config->Lq * p_foc_status->i_dq.Q;
        p_foc_status->u_dq_cross.Q = p_foc_status->we * p_motor_config->Ld * p_foc_status->i_dq.D;
        p_foc_status->u_bemf       = p_foc_status->we * p_motor_config->Flux;

        switch (p_motor_controller_config->current_decoupling_type)
        {
        case Current_Decoupling_Type_RL:
            p_foc_status->u_dq_feed_forward.D = p_foc_status->u_dq_rl.D;
            p_foc_status->u_dq_feed_forward.Q = p_foc_status->u_dq_rl.Q;
            break;

        case Current_Decoupling_Type_Cross:
            p_foc_status->u_dq_feed_forward.D = p_foc_status->u_dq_cross.D;
            p_foc_status->u_dq_feed_forward.Q = p_foc_status->u_dq_cross.Q;
            break;

        case Current_Decoupling_Type_RL_Cross:
            p_foc_status->u_dq_feed_forward.D = p_foc_status->u_dq_rl.D + p_foc_status->u_dq_cross.D;
            p_foc_status->u_dq_feed_forward.Q = p_foc_status->u_dq_rl.Q + p_foc_status->u_dq_cross.Q;
            break;

        case Current_Decoupling_Type_Bemf:
            p_foc_status->u_dq_feed_forward.D = 0;
            p_foc_status->u_dq_feed_forward.Q = p_foc_status->u_bemf;
            break;

        case Current_Decoupling_Type_RL_Bemf:
            p_foc_status->u_dq_feed_forward.D = p_foc_status->u_dq_rl.D;
            p_foc_status->u_dq_feed_forward.Q = p_foc_status->u_dq_rl.Q + p_foc_status->u_bemf;
            break;

        case Current_Decoupling_Type_Cross_Bemf:
            p_foc_status->u_dq_feed_forward.D = p_foc_status->u_dq_cross.D;
            p_foc_status->u_dq_feed_forward.Q = p_foc_status->u_dq_cross.Q + p_foc_status->u_bemf;
            break;

        case Current_Decoupling_Type_RL_Cross_Bemf:
            p_foc_status->u_dq_feed_forward.D = p_foc_status->u_dq_rl.D + p_foc_status->u_dq_cross.D;
            p_foc_status->u_dq_feed_forward.Q = p_foc_status->u_dq_rl.Q + p_foc_status->u_dq_cross.Q + p_foc_status->u_bemf;
            break;

        default:
            p_foc_status->u_dq_feed_forward.D = 0;
            p_foc_status->u_dq_feed_forward.Q = 0;
            break;
        }
    }
    else
    {
        p_foc_status->u_dq_feed_forward.D = 0;
        p_foc_status->u_dq_feed_forward.Q = 0;
    }
}

/**
 * @brief foc电流环更新
 * 
 * @param index 电机编号
 */
static inline void foc_current_loop_update(int index)
{
    Loop_Controller_Config_t *p_loop_controller_config = &global_config.loop_controller_config[index];
    Loop_Controller_t *p_loop_controller_status        = &global_status.loop_controller_status[index];
    FOC_Status_t *p_foc_status                         = &global_status.foc_status[index];

    float id_err = p_foc_status->i_dq_ref.D - p_foc_status->i_dq.D;
    float iq_err = p_foc_status->i_dq_ref.Q - p_foc_status->i_dq.Q;
    int is_current_follow_err =
        FOC_Current_Follow_Err_Exceeded(p_foc_status->i_dq_ref.D, id_err) || FOC_Current_Follow_Err_Exceeded(p_foc_status->i_dq_ref.Q, iq_err);

    if (is_current_follow_err)
    {
        if (++p_foc_status->current_err_cnt > MS_TO_HRT_TICKS(CURRENT_ERR_MS))
        {
            motor_set_err(index, FAULT_CODE_CURRENT_FOLLOW_ERR);
        }
    }
    else
    {
        p_foc_status->current_err_cnt = 0;
    }

    PID_Set_Abs_Limit(&p_loop_controller_status->id_pid, p_foc_status->uo_max);
    PID_Set_Abs_Limit(&p_loop_controller_status->iq_pid, p_foc_status->uo_max);

    switch (p_loop_controller_config->current_controller_config.controller_type)
    {
    case Controller_Type_Parallel_PID:
        p_foc_status->u_dq_out_raw.D = Parallel_PI_Update(&p_loop_controller_status->id_pid, p_foc_status->u_dq_feed_forward.D, id_err);
        p_foc_status->u_dq_out_raw.Q = Parallel_PI_Update(&p_loop_controller_status->iq_pid, p_foc_status->u_dq_feed_forward.Q, iq_err);
        break;

    case Controller_Type_Serial_PID:
        p_foc_status->u_dq_out_raw.D = Serial_PI_Update(&p_loop_controller_status->id_pid, p_foc_status->u_dq_feed_forward.D, id_err);
        p_foc_status->u_dq_out_raw.Q = Serial_PI_Update(&p_loop_controller_status->iq_pid, p_foc_status->u_dq_feed_forward.Q, iq_err);
        break;

    default:
        break;
    }

    if (p_loop_controller_status->id_pid.status.saturation || p_loop_controller_status->iq_pid.status.saturation)
    {
        motor_set_warning(index, WARN_CODE_CURRENT_LOOP_SATURATION);
    }
}

/**
 * @brief foc电压极限圆限制更新
 * 
 * @param index 电机编号
 */
static inline void foc_limit_circle_update(int index)
{
    FOC_Status_t *p_foc_status = &global_status.foc_status[index];
    HFI_Status_t *p_hfi_status = &global_status.hfi_status[index];

#ifndef FAST_CURRENT_LOOP_ENABLE
    memcpy(&p_foc_status->u_dq_out, &p_foc_status->u_dq_out_raw, sizeof(DQ_Axis_t));
    Inverse_Park_Transmission_Fast(&p_foc_status->u_dq_out_raw, &p_foc_status->u_ab_out_raw, p_foc_status->sin_cos);
    p_foc_status->us_out_raw = NORM2_f(p_foc_status->u_dq_out_raw.D, p_foc_status->u_dq_out_raw.Q);
    FirstOrder_LPF_Cal(p_foc_status->us_out_raw, p_foc_status->us_out_raw_f, 0.1f);
    p_foc_status->us_out_rate_raw = p_foc_status->uo_max > FLOAT_EPSILON ? p_foc_status->us_out_raw_f / p_foc_status->uo_max : 0.0f;
#else
    p_foc_status->u_dq_out.D = p_foc_status->u_dq_out_raw.D;
    p_foc_status->u_dq_out.Q = p_foc_status->u_dq_out_raw.Q;
#endif

#ifndef FAST_CURRENT_LOOP_ENABLE
    if ((p_foc_status->run_state & HFI_RUNNING_MASK) != 0)
    {
        p_foc_status->u_dq_out.D = p_foc_status->u_dq_out.D + p_hfi_status->udq_excitation.D;
        Saturate_Vector_2d_Fast(&p_foc_status->u_dq_out.D, &p_foc_status->u_dq_out.Q, p_foc_status->uo_max);
        // 使用theta_comp进行逆park变换会导致注入电压方向是在观测角度基础上补偿后的角度，而非HFI观测角度，按理来说这应该有问题,但实际测试效果并不差。
        // 而使用theta_hfi进行逆park变换可以保证注入电压方向始终与HFI观测角度一致，但会导致初始阶段电流突变且极性判断失效
        Inverse_Park_Transmission_Fast(&p_foc_status->u_dq_out, &p_foc_status->u_ab_out, p_foc_status->sin_cos);
        // Inverse_Park_Transmission(&p_foc_status->u_dq_out, &p_foc_status->u_ab_out, p_foc_status->theta_hfi);
    }
    else
#endif
    {
        Saturate_Vector_2d_Fast(&p_foc_status->u_dq_out.D, &p_foc_status->u_dq_out.Q, p_foc_status->uo_max);
        Inverse_Park_Transmission_Fast(&p_foc_status->u_dq_out, &p_foc_status->u_ab_out, p_foc_status->sin_cos);
    }

#ifndef FAST_CURRENT_LOOP_ENABLE
    p_foc_status->us_out = NORM2_f(p_foc_status->u_dq_out.D, p_foc_status->u_dq_out.Q);
    FirstOrder_LPF_Cal(p_foc_status->us_out, p_foc_status->us_out_f, 0.1f);
    p_foc_status->us_out_rate = p_foc_status->us_out_f / p_foc_status->uo_max;
#endif
}

/**
 * @brief foc功率参数更新
 * 
 * @param index 电机编号
 */
static inline void foc_power_update(int index)
{
    Motor_Config_t *p_motor_config             = &global_config.motor_config[index];
    Motor_Basic_Status_t *p_motor_basic_status = &global_status.motor_basic_status[index];
    FOC_Status_t *p_foc_status                 = &global_status.foc_status[index];

#ifndef FAST_CURRENT_LOOP_ENABLE
    p_foc_status->us              = NORM2_f(p_foc_status->u_ab.Alpha, p_foc_status->u_ab.Beta);
    p_foc_status->us_duty         = p_foc_status->us / p_foc_status->uo_max;
    p_foc_status->is              = NORM2_f(p_foc_status->i_ab.Alpha, p_foc_status->i_ab.Beta);
    p_foc_status->is_duty         = p_foc_status->is / p_motor_config->peak_current;
    p_motor_basic_status->e_power = (p_foc_status->u_ab.Alpha * p_foc_status->i_ab.Alpha + p_foc_status->u_ab.Beta * p_foc_status->i_ab.Beta) * 1.5f;
    p_foc_status->current_fb      = p_foc_status->is * SIGN(p_foc_status->i_dq.Q);
#else
    p_motor_basic_status->e_power = (p_foc_status->u_ab_out.Alpha * p_foc_status->i_ab.Alpha + p_foc_status->u_ab_out.Beta * p_foc_status->i_ab.Beta) * 1.5f;
    p_foc_status->current_fb      = p_foc_status->i_dq.Q;
#endif

    if (p_motor_config->saliency_ratio > FOC_MTPA_SALIENCY_RATE && p_motor_config->Flux > FLOAT_MIN)
    {
        p_foc_status->torque_fb = 1.5f * p_motor_config->pp * (p_motor_config->Flux * p_foc_status->i_dq.Q + p_motor_config->L_diff * p_foc_status->i_dq.D * p_foc_status->i_dq.Q);
    }
    else if (p_motor_config->kt > FLOAT_MIN)
    {
        p_foc_status->torque_fb = p_motor_config->kt * p_foc_status->i_dq.Q;
    }
    else
    {
        p_foc_status->torque_fb = 1.5f * p_motor_config->pp * p_motor_config->Flux * p_foc_status->i_dq.Q;
    }

    p_motor_basic_status->i_bus = p_motor_basic_status->e_power * p_motor_basic_status->inv_v_bus;
}

/**
 * @brief foc pwm更新
 * 
 * @param index 电机编号
 */
static inline void foc_pwm_update(int index)
{
    Motor_Controller_Config_t *p_motor_controller_config = &global_config.motor_controller_config[index];
    Core_Status_t *pstatus                              = &global_status.core_status;
    Motor_Basic_Status_t *p_motor_basic_status           = &global_status.motor_basic_status[index];
    FOC_Status_t *p_foc_status                           = &global_status.foc_status[index];

    SPWM_AB_Voltage(&p_foc_status->u_ab_out, p_motor_basic_status->inv_v_bus, &p_motor_basic_status->pwm_duty_raw);

    if (p_motor_controller_config->dead_zone_compensation_enable)
    {
        p_foc_status->dt_compensation_value = PWM_DEAD_ZONE_COMPENSATION_DUTY;

        PWM_Dead_Zone_Compensation(&p_motor_basic_status->phase_current,
                                   p_foc_status->dt_compensation_value,
                                   PWM_DEAD_ZONE_TRANSITION_CURRENT,
                                   &p_motor_basic_status->pwm_duty_raw,
                                   &p_motor_basic_status->pwm_duty_comp,
                                   &p_motor_basic_status->pwm_duty_out);
    }
    else
    {
        p_foc_status->dt_compensation_value = 0.0f;
        memset(&p_motor_basic_status->pwm_duty_comp, 0, sizeof(UVW_Axis_t));
        memcpy(&p_motor_basic_status->pwm_duty_out, &p_motor_basic_status->pwm_duty_raw, sizeof(UVW_Axis_t));
    }

    p_motor_basic_status->phase_voltage_out.U = p_motor_basic_status->pwm_duty_raw.U * p_motor_basic_status->v_bus;
    p_motor_basic_status->phase_voltage_out.V = p_motor_basic_status->pwm_duty_raw.V * p_motor_basic_status->v_bus;
    p_motor_basic_status->phase_voltage_out.W = p_motor_basic_status->pwm_duty_raw.W * p_motor_basic_status->v_bus;

    motor_pwm_set(index, p_motor_basic_status->pwm_duty_out.U, p_motor_basic_status->pwm_duty_out.V, p_motor_basic_status->pwm_duty_out.W);
    pstatus->pwm_update_time[index] = BOARD_MOTO_CNT_TO_US(get_pwm_counter(index));
}

/**
 * @brief foc快速寻相实时更新
 * 
 * @param index 电机编号
 */
static inline void foc_align_fast_update_rt(int index)
{
    Motor_Config_t *p_motor_config                       = &global_config.motor_config[index];
    Encoder_Config_t *p_encoder_config                   = &global_config.encoder_config[index];
    Motor_Controller_Config_t *p_motor_controller_config = &global_config.motor_controller_config[index];
    Motor_Status_t *p_motor_status                       = &global_status.motor_status[index];
    FOC_Status_t *p_foc_status                           = &global_status.foc_status[index];
    Encoder_Status_t *p_encoder_status                   = &global_status.encoder_status[index];

    float force  = CONSTRAIN(p_motor_controller_config->align_force, 0.005f, 1.0f);
    float id_ref = force * p_motor_config->peak_current;

    switch (p_foc_status->align_state)
    {
    case 0: // 电流开环强拖到-PI/2弧度
        p_foc_status->run_state           = FOC_IF_OpenLoop;
        p_foc_status->i_dq_ref.D          = id_ref;
        p_foc_status->i_dq_ref.Q          = 0;
        p_foc_status->theta_openloop      = -_PI_2;
        p_foc_status->align_tick          = 0;
        p_foc_status->align_state         = 1;
        p_foc_status->rads_encoder_offset = 0;
        break;

    case 1: // 等待位置稳定
        if (++p_foc_status->align_tick > MS_TO_RT_TICKS(p_motor_controller_config->align_time / 2))
        {
            p_foc_status->align_tick  = 0;
            p_foc_status->align_state = 2;
        }
        break;

    case 2: // 正向开环运行PI/2弧度
        if (++p_foc_status->align_tick > MS_TO_RT_TICKS(p_motor_controller_config->align_time))
        {
            p_foc_status->align_tick  = 0;
            p_foc_status->align_state = 3;
        }
        else
        {
            p_foc_status->theta_openloop = Normalize_Rad(-_PI_2 + (float)p_foc_status->align_tick / MS_TO_RT_TICKS(p_motor_controller_config->align_time) * _PI_2);
        }
        break;

    case 3: // 等待位置稳定
        if (++p_foc_status->align_tick > MS_TO_RT_TICKS(p_motor_controller_config->align_time / 2) && p_foc_status->align_tick > MS_TO_RT_TICKS(30))
        {
            p_foc_status->align_tick  = 0;
            p_foc_status->align_state = 4;
        }
        break;

    case 4: // 寻相完成，保存机械角度偏移
        p_foc_status->rads_encoder_offset = p_foc_status->rads_encoder;

        if (foc_send_align_result(index))
        {
            p_motor_status->is_aligned   = 1;
            p_foc_status->i_dq_ref.D     = 0;
            p_foc_status->i_dq_ref.Q     = 0;
            p_foc_status->theta_openloop = 0;
            p_foc_status->align_state    = 5;
        }
        break;

    default:
        break;
    }
}

/**
 * @brief foc慢速寻相实时更新
 * 
 * @param index 电机编号
 */
static inline void foc_align_slow_update_rt(int index)
{
    Motor_Config_t *p_motor_config                       = &global_config.motor_config[index];
    Encoder_Config_t *p_encoder_config                   = &global_config.encoder_config[index];
    Motor_Controller_Config_t *p_motor_controller_config = &global_config.motor_controller_config[index];
    Motor_Status_t *p_motor_status                       = &global_status.motor_status[index];
    FOC_Status_t *p_foc_status                           = &global_status.foc_status[index];
    Encoder_Status_t *p_encoder_status                   = &global_status.encoder_status[index];

    const float step_deg            = 5.0f;
    const int max_retry_cnt         = 144; // 144 * 5° = 720°
    float deg_err_threshold         = p_motor_controller_config->align_threshold;
    float delta_theta               = 0;
    float abs_delta_theta           = 0;
    float delta_theta_second        = 0;
    float abs_delta_theta_second    = 0;
    const float encoder_angle_ratio = (p_motor_config->motor_struct == Motor_Struct_Rotary) ? p_motor_config->pp : 1.0f;

    float force  = CONSTRAIN(p_motor_controller_config->align_force, 0.005f, 1.0f);
    float id_ref = force * p_motor_config->peak_current;

    switch (p_foc_status->align_state)
    {
    case 0: // 电流开环强拖到0弧度
        p_foc_status->run_state              = FOC_IF_OpenLoop;
        p_foc_status->i_dq_ref.D             = id_ref;
        p_foc_status->i_dq_ref.Q             = 0;
        p_foc_status->theta_openloop         = 0;
        p_foc_status->align_tick             = 0;
        p_foc_status->align_state            = 1;
        p_foc_status->align_initial          = 0;
        p_foc_status->align_offset_candidate = 0;
        p_foc_status->align_retry_count      = 0;
        p_foc_status->align_dir_detected     = 0;
        p_foc_status->rads_encoder_offset    = 0;

        p_foc_status->align_initial_second       = 0;
        p_foc_status->align_dir_detected_second  = 0;
        p_foc_status->rads_encoder_offset_second = 0;
        break;

    case 1: // 等待位置稳定
        if (++p_foc_status->align_tick > MS_TO_RT_TICKS(p_motor_controller_config->align_time / 2))
        {
            p_foc_status->align_tick    = 0;
            p_foc_status->align_state   = 2;
            p_foc_status->align_initial = p_foc_status->rads_encoder;

            p_foc_status->align_initial_second = p_encoder_status->encoder_pos_fractional_second * _2PI;
        }
        break;

    case 2: // 正向开环运行5°
        if (++p_foc_status->align_tick > MS_TO_RT_TICKS(p_motor_controller_config->align_time / 18))
        {
            p_foc_status->align_tick             = 0;
            p_foc_status->align_state            = 3;
            p_foc_status->align_offset_candidate = Normalize_Rad(p_foc_status->align_offset_candidate + DEG_TO_RAD(step_deg));
            p_foc_status->theta_openloop         = p_foc_status->align_offset_candidate;
        }
        else
        {
            p_foc_status->theta_openloop = Normalize_Rad(p_foc_status->align_offset_candidate + (float)p_foc_status->align_tick / MS_TO_RT_TICKS(p_motor_controller_config->align_time / 18) * DEG_TO_RAD(step_deg));
        }
        break;

    case 3: // 判断运行距离误差
        delta_theta     = Rad_Delta(p_foc_status->rads_encoder, p_foc_status->align_initial);
        abs_delta_theta = fabsf(delta_theta) * encoder_angle_ratio;
        if (abs_delta_theta > DEG_TO_RAD(step_deg - deg_err_threshold) && abs_delta_theta < DEG_TO_RAD(step_deg + deg_err_threshold))
        {
            // 第一次位移判断：记录运动方向（正向驱动下delta_theta应为正）
            if (p_foc_status->align_dir_detected == 0)
            {
                p_foc_status->align_dir_detected = (delta_theta > 0) ? 1 : -1;
            }

            p_foc_status->align_initial = p_foc_status->rads_encoder;
            p_foc_status->align_tick    = 0;
            p_foc_status->align_state   = 4;
        }
        else
        {
            p_foc_status->align_initial      = p_foc_status->rads_encoder;
            p_foc_status->align_tick         = 0;
            p_foc_status->align_state        = 2;
            p_foc_status->align_dir_detected = 0; // 重试时重置方向检测

            if (++p_foc_status->align_retry_count > max_retry_cnt)
            {
                motor_set_err(index, FAULT_CODE_ALIGN_NO_CONVERGENCE);
            }
        }
        break;

    case 4: // 正向开环运行5°
        if (++p_foc_status->align_tick > MS_TO_RT_TICKS(p_motor_controller_config->align_time / 18))
        {
            p_foc_status->align_tick             = 0;
            p_foc_status->align_state            = 5;
            p_foc_status->align_offset_candidate = Normalize_Rad(p_foc_status->align_offset_candidate + DEG_TO_RAD(step_deg));
            p_foc_status->theta_openloop         = p_foc_status->align_offset_candidate;
        }
        else
        {
            p_foc_status->theta_openloop = Normalize_Rad(p_foc_status->align_offset_candidate + (float)p_foc_status->align_tick / MS_TO_RT_TICKS(p_motor_controller_config->align_time / 18) * DEG_TO_RAD(step_deg));
        }
        break;

    case 5: // 判断运行距离误差
        delta_theta     = Rad_Delta(p_foc_status->rads_encoder, p_foc_status->align_initial);
        abs_delta_theta = fabsf(delta_theta) * encoder_angle_ratio;
        if (abs_delta_theta > DEG_TO_RAD(step_deg - deg_err_threshold) && abs_delta_theta < DEG_TO_RAD(step_deg + deg_err_threshold))
        {
            // 第二次位移判断：与第一次结果对比，一致时确认方向检测有效
            int dir_step = (delta_theta > 0) ? 1 : -1;

            if (p_foc_status->align_dir_detected == dir_step)
            {
                p_foc_status->align_dir_detected = dir_step * 2; // 两次一致，确认: 2=正确, -2=需修正
            }
            else
            {
                p_foc_status->align_dir_detected = 0; // 两次不一致，放弃修正
            }

            p_foc_status->align_tick  = 0;
            p_foc_status->align_state = 6;
        }
        else
        {
            p_foc_status->align_initial      = p_foc_status->rads_encoder;
            p_foc_status->align_tick         = 0;
            p_foc_status->align_state        = 2;
            p_foc_status->align_dir_detected = 0; // 重试时重置方向检测

            if (++p_foc_status->align_retry_count > max_retry_cnt)
            {
                motor_set_err(index, FAULT_CODE_ALIGN_NO_CONVERGENCE);
            }
        }
        break;

    case 6: // 判断寻相结果
        // 若两次位移判断均检测到电机方向与配置不符，自动修正motor_dir并通知core0，然后重新寻相
        if (p_foc_status->align_dir_detected == -2)
        {
            int corrected_dir = -p_motor_config->motor_dir;

            p_motor_config->motor_dir = corrected_dir;

            // 使用修正后的方向重新寻相
            p_foc_status->align_state        = 0;
            p_foc_status->align_dir_detected = 0;
            break;
        }

        // 两次方向判断不一致，放弃本次寻相结果，回到state 2重试
        if (p_foc_status->align_dir_detected == 0)
        {
            p_foc_status->align_initial = p_foc_status->rads_encoder;
            p_foc_status->align_state   = 2;
            break;
        }

        p_foc_status->align_tick  = 0;
        p_foc_status->align_state = 7;
        break;

    case 7: // 等待位置稳定
        if (++p_foc_status->align_tick > MS_TO_RT_TICKS(10))
        {
            p_foc_status->align_tick  = 0;
            p_foc_status->align_state = 8;
        }
        break;

    case 8: // 应用寻相结果
        switch (p_motor_config->motor_struct)
        {
        case Motor_Struct_Linear:
            p_foc_status->rads_encoder_offset = Normalize_Rad(p_foc_status->rads_encoder - p_foc_status->theta_openloop);
            break;

        case Motor_Struct_Rotary:
            p_foc_status->rads_encoder_offset = Normalize_Rad(p_foc_status->rads_encoder - p_foc_status->theta_openloop / p_motor_config->pp);
            break;

        default:
            break;
        }

        if (foc_send_align_result(index))
        {
            p_motor_status->is_aligned   = 1;
            p_foc_status->i_dq_ref.D     = 0;
            p_foc_status->i_dq_ref.Q     = 0;
            p_foc_status->theta_openloop = 0;
            p_foc_status->align_state    = 9;
        }
        break;

    default:
        break;
    }
}

/**
 * @brief foc二分寻相实时更新
 * 
 * @param index 电机编号
 */
static inline void foc_align_bisection_update_rt(int index)
{
    Motor_Config_t *p_motor_config                       = &global_config.motor_config[index];
    Encoder_Config_t *p_encoder_config                   = &global_config.encoder_config[index];
    Motor_Controller_Config_t *p_motor_controller_config = &global_config.motor_controller_config[index];
    Motor_Basic_Status_t *p_motor_basic_status           = &global_status.motor_basic_status[index];
    Motor_Status_t *p_motor_status                       = &global_status.motor_status[index];
    FOC_Status_t *p_foc_status                           = &global_status.foc_status[index];
    Encoder_Status_t *p_encoder_status                   = &global_status.encoder_status[index];

    p_foc_status->us_max = p_motor_basic_status->v_bus * _1_SQRT3;
    p_foc_status->uo_max = p_motor_controller_config->max_modulation_ratio * p_foc_status->us_max;

    float force  = CONSTRAIN(p_motor_controller_config->align_force, 0.005f, 1.0f);
    float ud_ref = force * p_foc_status->uo_max;

    const float initial_correction = _PI_2; // 初始修正量为90度
    const float min_correction     = DEG_TO_RAD(0.1f);
    float theta_threshold          = DEG_TO_RAD(p_motor_controller_config->align_threshold);
    uint32_t rand                  = 0;

    float delta_theta     = 0;
    float abs_delta_theta = 0;

    switch (p_foc_status->align_state)
    {
    case 0: // 起始角度随机获取
        p_foc_status->run_state              = FOC_VF_OpenLoop;
        p_foc_status->u_dq_ref.D             = 0;
        p_foc_status->u_dq_ref.Q             = 0;
        p_foc_status->theta_openloop         = 0;
        p_foc_status->align_tick             = 0;
        p_foc_status->align_state            = 1;
        p_foc_status->align_initial          = p_foc_status->rads_encoder;
        p_foc_status->align_offset_candidate = (float)rand / (float)UINT32_MAX * _2PI;
        p_foc_status->align_correction       = initial_correction;
        p_foc_status->rads_encoder_offset    = 0;
        break;

    case 1: // 设置初始位置反馈
        p_foc_status->u_dq_ref.D    = 0;
        p_foc_status->align_initial = p_foc_status->rads_encoder;
        p_foc_status->align_tick    = 0;
        p_foc_status->align_state   = 2;
        break;

    case 2: // 增加电压并检查位置反馈
        if (p_foc_status->u_dq_ref.D < ud_ref)
        {
            p_foc_status->u_dq_ref.D += ud_ref / MS_TO_S((float)p_motor_controller_config->align_time) * BOARD_RT_DT;
        }

        p_foc_status->theta_openloop = Normalize_Rad(p_foc_status->align_offset_candidate);

        delta_theta     = Rad_Delta(p_foc_status->rads_encoder, p_foc_status->align_initial);
        abs_delta_theta = fabsf(delta_theta);
        if (abs_delta_theta > theta_threshold)
        {
            p_foc_status->align_offset_candidate += (delta_theta > 0) ? -p_foc_status->align_correction : p_foc_status->align_correction;
            // 开始时二分修正
            if (p_foc_status->align_correction > min_correction)
            {
                p_foc_status->align_correction /= 2.0f;
            }
            else
            {
                motor_set_err(index, FAULT_CODE_ALIGN_NO_CONVERGENCE);
            }
            p_foc_status->align_tick  = 0;
            p_foc_status->align_state = 3;
        }
        else if (p_foc_status->u_dq_ref.D >= ud_ref)
        {
            p_foc_status->align_tick++;
        }

        if (p_foc_status->align_tick > MS_TO_RT_TICKS(20))
        {
            p_foc_status->align_tick  = 0;
            p_foc_status->align_state = 4;
        }
        break;

    case 3: // 延时释放运动惯量
        p_foc_status->u_dq_ref.D     = 0;
        p_foc_status->theta_openloop = 0;
        if (++p_foc_status->align_tick > MS_TO_RT_TICKS(50))
        {
            p_foc_status->align_tick  = 0;
            p_foc_status->align_state = 1;
        }
        break;

    case 4: // 延时释放运动惯量
        p_foc_status->u_dq_ref.D     = 0;
        p_foc_status->theta_openloop = 0;
        if (++p_foc_status->align_tick > MS_TO_RT_TICKS(50))
        {
            p_foc_status->align_tick  = 0;
            p_foc_status->align_state = 5;
        }
        break;

    case 5: // 测试相位角是否正确
        p_foc_status->align_tick    = 0;
        p_foc_status->u_dq_ref.D    = 0.0f;
        p_foc_status->align_initial = p_foc_status->rads_encoder;
        p_foc_status->align_state   = 6;
        break;

    case 6:
        if (p_foc_status->u_dq_ref.D < ud_ref)
        {
            p_foc_status->u_dq_ref.D += ud_ref / MS_TO_S((float)p_motor_controller_config->align_time) * BOARD_RT_DT;
        }

        p_foc_status->theta_openloop = Normalize_Rad(p_foc_status->align_offset_candidate + _PI_2);

        delta_theta     = Rad_Delta(p_foc_status->rads_encoder, p_foc_status->align_initial);
        abs_delta_theta = fabsf(delta_theta);
        if (abs_delta_theta >= theta_threshold)
        {
            if (delta_theta < 0)
            {
                // 先不报错，处理成 + 180度再进行测试
                p_foc_status->align_offset_candidate += _PI;
                p_foc_status->align_tick              = 0;
                p_foc_status->align_state             = 5;
            }
            else
            {
                p_foc_status->align_tick  = 0;
                p_foc_status->align_state = 7;
            }
        }
        break;

    case 7:
        if (p_foc_status->u_dq_ref.D < ud_ref)
        {
            p_foc_status->u_dq_ref.D += ud_ref / MS_TO_S((float)p_motor_controller_config->align_time) * BOARD_RT_DT;
        }

        p_foc_status->theta_openloop = Normalize_Rad(p_foc_status->align_offset_candidate);

        if (++p_foc_status->align_tick > MS_TO_RT_TICKS(30))
        {
            p_foc_status->align_tick  = 0;
            p_foc_status->align_state = 8;
        }
        break;

    case 8:
        switch (p_motor_config->motor_struct)
        {
        case Motor_Struct_Linear:
            p_foc_status->rads_encoder_offset = Normalize_Rad(p_foc_status->rads_encoder - p_foc_status->theta_openloop);
            break;

        case Motor_Struct_Rotary:
            p_foc_status->rads_encoder_offset = Normalize_Rad(p_foc_status->rads_encoder - p_foc_status->theta_openloop / p_motor_config->pp);
            break;

        default:
            break;
        }

        if (foc_send_align_result(index))
        {
            p_motor_status->is_aligned   = 1;
            p_foc_status->u_dq_ref.D     = 0;
            p_foc_status->u_dq_ref.Q     = 0;
            p_foc_status->theta_openloop = 0;
            p_foc_status->align_state    = 9;
        }
        break;

    default:
        break;
    }
}

/**
 * @brief foc HFI 寻相实时更新
 * 
 * @param index 电机编号
 */
static inline void foc_align_hfi_update_rt(int index)
{
    Motor_Config_t *p_motor_config     = &global_config.motor_config[index];
    Encoder_Config_t *p_encoder_config = &global_config.encoder_config[index];
    Motor_Status_t *p_motor_status     = &global_status.motor_status[index];
    FOC_Status_t *p_foc_status         = &global_status.foc_status[index];
    Encoder_Status_t *p_encoder_status = &global_status.encoder_status[index];

    switch (p_foc_status->align_state)
    {
    case 0: // 进入HFI模式
        p_foc_status->run_state   = FOC_HFI_Init;
        p_foc_status->i_dq_ref.D  = 0;
        p_foc_status->i_dq_ref.Q  = 0;
        p_foc_status->align_state = 1;
        break;

    case 1: // 等待极性辨识完成
        if (p_foc_status->run_state == FOC_HFI_CloseLoop)
        {
            p_foc_status->align_state = 2;
        }
        break;

    case 2: // 等待位置稳定
        if (++p_foc_status->align_tick > MS_TO_RT_TICKS(30))
        {
            p_foc_status->align_tick  = 0;
            p_foc_status->align_state = 3;
        }
        break;

    case 3: // 寻相完成，保存机械角度偏移
        switch (p_motor_config->motor_struct)
        {
        case Motor_Struct_Linear:
            p_foc_status->rads_encoder_offset = Normalize_Rad(p_foc_status->rads_encoder - p_foc_status->theta_hfi);
            break;

        case Motor_Struct_Rotary:
            p_foc_status->rads_encoder_offset = Normalize_Rad(p_foc_status->rads_encoder - p_foc_status->theta_hfi / p_motor_config->pp);
            break;

        default:
            break;
        }

        if (foc_send_align_result(index))
        {
            p_foc_status->run_state    = FOC_CloseLoop;
            p_motor_status->is_aligned = 1;
            p_foc_status->align_state  = 4;
        }
        break;

    default:
        break;
    }
}

/**
 * @brief foc寻相实时更新
 * 
 * @param index 电机编号
 */
void foc_align_update_rt(int index)
{
    Motor_Controller_Config_t *p_motor_controller_config = &global_config.motor_controller_config[index];

    switch (p_motor_controller_config->align_method)
    {
    case Align_Method_Fast:
        foc_align_fast_update_rt(index);
        break;

    case Align_Method_Slow:
        foc_align_slow_update_rt(index);
        break;

    case Align_Method_Bisection:
        foc_align_bisection_update_rt(index);
        break;

#ifndef FAST_CURRENT_LOOP_ENABLE
    case Align_Method_HFI:
        foc_align_hfi_update_rt(index);
        break;
#endif

    default:
        break;
    }
}

/**
 * @brief foc相电阻辨识实时更新
 * 
 * @param index 电机编号
 */
void foc_rs_identify_update_rt(int index)
{
    Motor_Config_t *p_motor_config     = &global_config.motor_config[index];
    Motor_Status_t *p_motor_status     = &global_status.motor_status[index];
    FOC_Status_t *p_foc_status         = &global_status.foc_status[index];
    Encoder_Status_t *p_encoder_status = &global_status.encoder_status[index];

    switch (p_foc_status->identify_state)
    {
    case 0: // 进入电压开环模式
        p_foc_status->run_state      = FOC_VF_OpenLoop;
        p_foc_status->u_dq_ref.D     = 0;
        p_foc_status->u_dq_ref.Q     = 0;
        p_foc_status->theta_openloop = 0;
        p_foc_status->identify_state = 1;
        p_foc_status->identify_tick  = 0;
        memset(&p_foc_status->u_dq_cache, 0, sizeof(DQ_Axis_t) * 2);
        memset(&p_foc_status->i_dq_cache, 0, sizeof(DQ_Axis_t) * 2);
        break;

    case 1: // 缓慢增加D轴电压,直至电流达到额定电流的一半或电压达到最大电压的一半,并等待电流稳定
        if (p_foc_status->i_dq.D < p_motor_config->rated_current * 0.5f && p_foc_status->u_dq_ref.D < p_foc_status->uo_max * 0.5f)
        {
            p_foc_status->u_dq_ref.D    += 0.001f;
            p_foc_status->identify_tick  = 0;
        }
        else
        {
            if (p_foc_status->i_dq.D > FLOAT_EPSILON && fabsf(p_foc_status->i_dq.Q) < 0.1f * p_motor_config->rated_current && fabsf(p_encoder_status->encoder_speed) < 0.05f && fabsf((p_foc_status->i_dq.D - p_foc_status->i_dq_last.D) / p_foc_status->i_dq.D) < 0.05f)
            {
                LPF_F32(&p_foc_status->u_dq_cache[0].D, p_foc_status->u_dq.D, 10.0f, BOARD_RT_DT);
                LPF_F32(&p_foc_status->i_dq_cache[0].D, p_foc_status->i_dq.D, 10.0f, BOARD_RT_DT);
                p_foc_status->identify_tick++;
                if (p_foc_status->identify_tick > MS_TO_RT_TICKS(100))
                {
                    p_foc_status->identify_state = 2;
                    p_foc_status->identify_tick  = 0;
                }
            }
            else
            {
                p_foc_status->identify_tick   = 0;
                p_foc_status->u_dq_cache[0].D = p_foc_status->u_dq.D;
                p_foc_status->i_dq_cache[0].D = p_foc_status->i_dq.D;
            }
        }
        break;

    case 2: // 缓慢增加D轴电压,直至电流达到额定电流或电压达到最大电压,并等待电流稳定
        if (p_foc_status->i_dq.D < p_motor_config->rated_current && p_foc_status->u_dq_ref.D < p_foc_status->uo_max)
        {
            p_foc_status->u_dq_ref.D    += 0.001f;
            p_foc_status->identify_tick  = 0;
        }
        else
        {
            if (p_foc_status->i_dq.D > FLOAT_EPSILON && fabsf(p_foc_status->i_dq.Q) < 0.1f * p_motor_config->rated_current && fabsf(p_encoder_status->encoder_speed) < 0.05f && fabsf((p_foc_status->i_dq.D - p_foc_status->i_dq_last.D) / p_foc_status->i_dq.D) < 0.05f)
            {
                LPF_F32(&p_foc_status->u_dq_cache[1].D, p_foc_status->u_dq.D, 10.0f, BOARD_RT_DT);
                LPF_F32(&p_foc_status->i_dq_cache[1].D, p_foc_status->i_dq.D, 10.0f, BOARD_RT_DT);
                p_foc_status->identify_tick++;
                if (p_foc_status->identify_tick > MS_TO_RT_TICKS(100))
                {
                    p_foc_status->identify_state = 3;
                    p_foc_status->identify_tick  = 0;
                }
            }
            else
            {
                p_foc_status->identify_tick   = 0;
                p_foc_status->u_dq_cache[1].D = p_foc_status->u_dq.D;
                p_foc_status->i_dq_cache[1].D = p_foc_status->i_dq.D;
            }
        }
        break;

    case 3: // 计算辨识结果
    {
        p_foc_status->u_dq_ref.D = 0;
        p_foc_status->u_dq_ref.Q = 0;
        float delta_u            = p_foc_status->u_dq_cache[1].D - p_foc_status->u_dq_cache[0].D;
        float delta_i            = p_foc_status->i_dq_cache[1].D - p_foc_status->i_dq_cache[0].D;
        float rs_identify        = delta_u / delta_i;
        if (delta_u < FLOAT_EPSILON || delta_i < FLOAT_EPSILON || IS_NAN(rs_identify) || IS_INF(rs_identify) || rs_identify < FLOAT_EPSILON)
        {
            motor_set_err(index, FAULT_CODE_RS_IDENTIFY);
            p_foc_status->identify_state = 5;
            break;
        }
        p_foc_status->rs_identify = rs_identify;
        p_foc_status->identify_state = 4;
        p_motor_config->Rs = p_foc_status->rs_identify;
    }
    break;

    case 4:
        // 相电阻辨识完成
        p_motor_status->identify_done = 1;
        p_foc_status->identify_state  = 5;
        break;

    default:

        break;
    }
}

/**
 * @brief 计算存在采样到 PWM 生效延迟时的对称方波 RL 响应
 *
 * 令 x=Rs*T/L、d=delta/T。一次相邻采样区间内，旧极性电压继续作用 delta，
 * 新极性电压作用 T-delta。周期稳态下归一化电流响应为：
 *
 *     k(x) = (1 - 2*exp(-(1-d)*x) + exp(-x)) / (1 + exp(-x))
 *
 * 当 delta=0 时，该式退化为 tanh(x/2)。在 0<=delta<T/2 时，k(x)
 * 随 x 从 0 单调增加到 1，可使用二分法稳定反解。
 */
static float foc_ls_delayed_response(float x, float delay_ratio)
{
    float exp_t   = expf(-x);
    float exp_new = expf(-(1.0f - delay_ratio) * x);
    return (1.0f - 2.0f * exp_new + exp_t) / (1.0f + exp_t);
}

/**
 * @brief 根据带固定 PWM 生效延迟的双极性方波响应反解相电感
 *
 * HFI 的 is_response=(i[k]-i[k-1])/2。由于新注入电压在 ADC 采样后
 * 还需等待 PWM reload 才生效，相邻采样之间并非完整施加同一极性的电压，
 * 因此不能直接使用 L=Rs*T/(2*atanh(k))。这里先数值求解 x=Rs*T/L，
 * 再由 L=Rs*T/x 得到电感。
 *
 * @param us_response 高频注入指令电压幅值 U，单位 V
 * @param is_response 高频电流半差响应幅值 I，单位 A
 * @param Rs          已辨识的相电阻，单位 ohm
 * @param period      相邻 HFI 响应采样间隔 T，单位 s
 * @param delay       电流采样到新 HFI 电压实际生效的延迟 delta，单位 s
 * @return float      相电感，单位 H；参数或响应无效时返回 0
 */
static float foc_ls_rl_solve(float us_response, float is_response, float Rs, float period, float delay)
{
    if (us_response <= FLOAT_EPSILON || is_response <= FLOAT_EPSILON || Rs <= FLOAT_EPSILON || period <= 0.0f || delay < 0.0f || 2.0f * delay >= period)
    {
        return 0.0f;
    }

    float k = is_response * Rs / us_response;
    if (k >= 0.6f || k <= 0.1f)
    {
        return 0.0f;
    }

    float delay_ratio = delay / period;
    float x_low       = 0.0f;
    float x_high      = 1.0f;

    /* 扩大上界，直到模型响应覆盖实测 k。 */
    while (x_high < 32.0f && foc_ls_delayed_response(x_high, delay_ratio) < k)
    {
        x_high *= 2.0f;
    }

    if (foc_ls_delayed_response(x_high, delay_ratio) < k)
    {
        return 0.0f;
    }

    /* 20 次二分在当前 k 范围内可提供远高于采样精度的求解分辨率。 */
    for (int i = 0; i < 20; i++)
    {
        float x_mid = 0.5f * (x_low + x_high);
        if (foc_ls_delayed_response(x_mid, delay_ratio) < k)
        {
            x_low = x_mid;
        }
        else
        {
            x_high = x_mid;
        }
    }

    float x = 0.5f * (x_low + x_high);
    float l = Rs * period / x;

    if (IS_NAN(l) || IS_INF(l) || l <= FLOAT_EPSILON)
    {
        return 0.0f;
    }

    return l;
}

static void foc_ls_scan_reset(int index)
{
    memset(&foc_ls_scan[index], 0, sizeof(foc_ls_scan[index]));
    foc_ls_scan[index].last_response_sequence = hfi_get_response_sequence(index);
}

/**
 * @brief 将一个新的 HFI 响应按注入角度累计到扫描桶
 *
 * 注入电压只有当前开环坐标系的 D 轴分量，因此先将同步解调后的 alpha-beta
 * 电流响应变换到该坐标系，只使用与注入电压同向的 D 轴分量反解电感。
 * Q 轴分量是凸极性导致的交叉响应，若直接对 alpha-beta 矢量取模，会系统性
 * 放大电流响应并改变角度相关曲线。
 */
static void foc_ls_scan_accumulate(int index, float us_response, float Rs, float period, float theta)
{
    FOC_Ls_Scan_t *scan        = &foc_ls_scan[index];
    HFI_Status_t *p_hfi_status = &global_status.hfi_status[index];
    uint32_t response_sequence = hfi_get_response_sequence(index);

    if (response_sequence == scan->last_response_sequence)
    {
        return;
    }
    scan->last_response_sequence = response_sequence;

    DQ_Axis_t i_dq_response;
    Park_Transmission(&p_hfi_status->iab_input, &i_dq_response, theta);
    float is_response = fabsf(i_dq_response.D);
    float l           = foc_ls_rl_solve(us_response, is_response, Rs, period, FOC_LS_HFI_SAMPLE_TO_PWM_DELAY_S);
    if (l <= FLOAT_MIN)
    {
        return;
    }

    float angle  = Normalize_Rad(theta);
    uint32_t bin = (uint32_t)(angle * (float)FOC_LS_SCAN_BIN_COUNT / _2PI);
    if (bin >= FOC_LS_SCAN_BIN_COUNT)
    {
        bin = FOC_LS_SCAN_BIN_COUNT - 1U;
    }

    if (scan->sample_count[bin] < UINT16_MAX)
    {
        scan->inductance_sum[bin] += l;
        scan->sample_count[bin]++;
    }
}

/**
 * @brief 由角度桶均值提取 D/Q 轴电感
 *
 * 先对每个角度桶内的多个样本求均值，再对最低和最高若干桶分别求均值。
 * 相比扫描期间直接记录单点最小/最大值，这能抑制电流噪声、瞬态和偶发
 * 无效响应对 Ld/Lq，尤其是 Lq 最大值的偏置。
 */
static bool foc_ls_scan_finish(int index, DQ_Axis_t *l_identify)
{
    FOC_Ls_Scan_t *scan = &foc_ls_scan[index];
    float bin_mean[FOC_LS_SCAN_BIN_COUNT];
    uint32_t valid_bin_count = 0;

    for (uint32_t i = 0; i < FOC_LS_SCAN_BIN_COUNT; i++)
    {
        if (scan->sample_count[i] >= FOC_LS_MIN_SAMPLES_PER_BIN)
        {
            bin_mean[valid_bin_count++] = scan->inductance_sum[i] / (float)scan->sample_count[i];
        }
    }

    if (valid_bin_count < 2U * FOC_LS_EXTREME_BIN_COUNT)
    {
        return false;
    }

    /* 扫描结束时仅执行一次插入排序，避免在实时扫描过程中维护噪声敏感极值。 */
    for (uint32_t i = 1; i < valid_bin_count; i++)
    {
        float value = bin_mean[i];
        uint32_t j  = i;
        while (j > 0U && bin_mean[j - 1U] > value)
        {
            bin_mean[j] = bin_mean[j - 1U];
            j--;
        }
        bin_mean[j] = value;
    }

    float ld_sum = 0.0f;
    float lq_sum = 0.0f;
    for (uint32_t i = 0; i < FOC_LS_EXTREME_BIN_COUNT; i++)
    {
        ld_sum += bin_mean[i];
        lq_sum += bin_mean[valid_bin_count - 1U - i];
    }

    l_identify->D = ld_sum / (float)FOC_LS_EXTREME_BIN_COUNT;
    l_identify->Q = lq_sum / (float)FOC_LS_EXTREME_BIN_COUNT;
    return true;
}

/**
 * @brief foc相电感辨识实时更新
 *
 * @param index 电机编号
 */
void foc_ls_identify_update_rt(int index)
{
    Motor_Config_t *p_motor_config = &global_config.motor_config[index];
    HFI_Config_t *p_hfi_config     = &global_config.hfi_config[index];
    Motor_Status_t *p_motor_status = &global_status.motor_status[index];
    FOC_Status_t *p_foc_status     = &global_status.foc_status[index];
    HFI_Status_t *p_hfi_status     = &global_status.hfi_status[index];
    float us_response              = p_hfi_status->vf_amp;
    DQ_Axis_t i_dq_response;
    Park_Transmission(&p_hfi_status->iab_input, &i_dq_response, p_foc_status->theta_openloop);
    float is_response        = fabsf(i_dq_response.D);
    float dt                 = BOARD_HRT_DT * p_hfi_config->div;
    const float identify_deg = 360.0f;
    const int identify_tick  = MS_TO_RT_TICKS(1000);

    switch (p_foc_status->identify_state)
    {
    case 0: // 进入HFI开环模式
        p_foc_status->run_state      = FOC_HFI_OpenLoop;
        p_hfi_status->is_open_loop   = true;
        p_hfi_status->vf_amp         = 0.0f;
        p_foc_status->theta_openloop = 0;
        p_foc_status->identify_state = 1;
        p_foc_status->identify_tick  = 0;
        memset(&p_foc_status->l_identify, 0, sizeof(DQ_Axis_t));
        p_foc_status->rs_identify = 0;
        foc_ls_scan_reset(index);
        break;

    case 1: // 缓慢增加高频注入电压,直至高频电流响应达到额定电流的25%或电压达到最大电压的25%,并等待电流稳定
        if (fabsf(is_response) < p_motor_config->rated_current * 0.25f && p_hfi_status->vf_amp < p_foc_status->uo_max * 0.25f)
        {
            p_hfi_status->vf_amp        += 0.001f;
            p_foc_status->identify_tick  = 0;
        }
        else
        {
            if (++p_foc_status->identify_tick > MS_TO_RT_TICKS(20) && us_response > FLOAT_EPSILON && is_response > FLOAT_EPSILON)
            {
                float l = foc_ls_rl_solve(us_response, is_response, p_motor_config->Rs, dt, FOC_LS_HFI_SAMPLE_TO_PWM_DELAY_S);
                if (l <= FLOAT_MIN)
                {
                    motor_set_err(index, FAULT_CODE_LS_IDENTIFY);
                    p_foc_status->identify_state = 4;
                    break;
                }

                p_foc_status->identify_state = 2;
                p_foc_status->identify_tick  = 0;
                p_foc_status->l_identify.D   = l;
                p_foc_status->l_identify.Q   = l;
                p_foc_status->rs_identify    = l;
                foc_ls_scan_reset(index);
            }
        }
        break;

    case 2: // 开环旋转360°,扫描l的最大值和最小值
        if (++p_foc_status->identify_tick > identify_tick)
        {
            p_foc_status->identify_tick = 0;
            if (foc_ls_scan_finish(index, &p_foc_status->l_identify))
            {
                p_foc_status->identify_state = 3;
            }
            else
            {
                motor_set_err(index, FAULT_CODE_LS_IDENTIFY);
                p_foc_status->identify_state = 4;
            }
        }
        else
        {
            foc_ls_scan_accumulate(index, us_response, p_motor_config->Rs, dt, p_foc_status->theta_openloop);

            p_foc_status->theta_openloop = Normalize_Rad(p_foc_status->theta_openloop + DEG_TO_RAD(identify_deg) / identify_tick);
        }
        break;

    case 3: // 校验辨识结果
        if (p_foc_status->l_identify.D < FLOAT_MIN || p_foc_status->l_identify.Q < FLOAT_MIN || p_foc_status->l_identify.Q / p_foc_status->l_identify.D > 10.0f)
        {
            motor_set_err(index, FAULT_CODE_LS_IDENTIFY);
        }
        else
        {
            p_motor_status->identify_done = 1;
            p_foc_status->identify_state  = 4;
            p_motor_config->Ld = p_foc_status->l_identify.D;
            p_motor_config->Lq = p_foc_status->l_identify.Q;
        }
        break;

    default:
        break;
    }
}

/**
 * @brief foc磁链辨识实时更新
 * 
 * @param index 电机编号
 */
void foc_flux_identify_update_rt(int index)
{
    Motor_Config_t *p_motor_config = &global_config.motor_config[index];
    Motor_Status_t *p_motor_status = &global_status.motor_status[index];
    FOC_Status_t *p_foc_status     = &global_status.foc_status[index];

    if (p_motor_config->motor_struct != Motor_Struct_Rotary)
    {
        motor_set_err(index, FAULT_CODE_MOTOR_TYPE_IDENTIFY);
        return;
    }

    switch (p_foc_status->identify_state)
    {
    case 0: // 进入闭环模式
        p_foc_status->run_state                 = FOC_CloseLoop;
        p_foc_status->op_area                   = FOC_OP_MANUAL;
        p_foc_status->overide_decoupling_update = true;
        p_foc_status->i_dq_ref.D                = 0;
        p_foc_status->i_dq_ref.Q                = 0;
        p_foc_status->current_ref               = 0;
        p_foc_status->identify_state            = 1;
        p_foc_status->identify_tick             = 0;
        p_foc_status->encoder_pos_cache         = 0;
        p_foc_status->rs_identify               = 0;
        break;

    case 1: // 借用rs_identify作为速度规划,借用encoder_pos_cache作为积分值构建一个简易过阻尼速度环.等待速度稳定后再计算辨识结果
    {
        if (p_foc_status->rs_identify < FLUX_IDENTIFY_WE)
        {
            p_foc_status->rs_identify += FLUX_IDENTIFY_WE * BOARD_RT_DT; // 加速时间1S
        }
        float we                         = p_foc_status->we;
        float we_abs                     = fabsf(we);
        float we_err                     = p_foc_status->rs_identify - we;
        float we_err_abs                 = fabsf(we_err);
        p_foc_status->encoder_pos_cache += FLUX_IDENTIFY_KI * we_err * BOARD_RT_DT;
        p_foc_status->encoder_pos_cache  = CONSTRAIN(p_foc_status->encoder_pos_cache, -p_motor_config->peak_current, p_motor_config->peak_current);
        p_foc_status->current_ref        = FLUX_IDENTIFY_KP * we_err + p_foc_status->encoder_pos_cache;
        p_foc_status->i_dq_ref.Q         = CONSTRAIN(p_foc_status->current_ref, -p_motor_config->peak_current, p_motor_config->peak_current);
        float eq                         = p_foc_status->u_dq.Q - p_foc_status->u_dq_rl.Q - p_foc_status->u_dq_cross.Q;
        float flux                       = eq / we;
        if (we_abs < FLOAT_EPSILON)
        {
            flux = 0;
        }
        LPF_F32(&p_foc_status->flux_identify, flux, 100.0f, BOARD_RT_DT);

        if (p_foc_status->rs_identify >= FLUX_IDENTIFY_WE && we_err_abs < FLUX_IDENTIFY_MAX_WE_ERR && p_foc_status->flux_identify > FLOAT_EPSILON)
        {
            p_foc_status->identify_tick++;
            if (p_foc_status->identify_tick > MS_TO_RT_TICKS(1000))
            {
                p_foc_status->identify_state = 2;
                p_foc_status->identify_tick  = 0;
            }
        }
        else
        {
            p_foc_status->identify_tick = 0;
        }
    }
    break;

    case 2: // 应用辨识结果
        p_foc_status->identify_state = 3;
        p_motor_config->Flux = p_foc_status->flux_identify;
        break;

    case 3: // 磁链辨识完成
        p_foc_status->overide_decoupling_update = false;
        p_motor_status->identify_done           = 1;
        p_foc_status->identify_state            = 5;
        p_foc_status->i_dq_ref.D                = 0;
        p_foc_status->i_dq_ref.Q                = 0;
        p_foc_status->current_ref               = 0;
        if (is_MTPA_allowed(index))
        {
            p_foc_status->op_area = FOC_OP_MTPA;
        }
        else
        {
            p_foc_status->op_area = FOC_OP_ID0;
        }
        break;

    default:

        break;
    }
}

/**
 * @brief foc极对数辨识实时更新
 * 
 * @param index 电机编号
 */
void foc_pp_identify_update_rt(int index)
{
    Motor_Config_t *p_motor_config     = &global_config.motor_config[index];
    Motor_Status_t *p_motor_status     = &global_status.motor_status[index];
    FOC_Status_t *p_foc_status         = &global_status.foc_status[index];
    Encoder_Status_t *p_encoder_status = &global_status.encoder_status[index];

    const float identify_deg = 360.0f;
    const int identify_tick  = MS_TO_RT_TICKS(1000);
    const float min_deg      = 9.0f;   // 40 polepairs
    const float max_deg      = 400.0f; // 40 polepairs
    const int max_retry_cnt  = 6;
    float delta_rads         = 0;
    float abs_delta_rads     = 0;

    if (p_motor_config->motor_struct != Motor_Struct_Rotary)
    {
        motor_set_err(index, FAULT_CODE_MOTOR_TYPE_IDENTIFY);
        return;
    }

    switch (p_foc_status->identify_state)
    {
    case 0: // 进入电压开环模式
        p_foc_status->run_state          = FOC_VF_OpenLoop;
        p_foc_status->u_dq_ref.D         = 0;
        p_foc_status->u_dq_ref.Q         = 0;
        p_foc_status->theta_openloop     = 0;
        p_foc_status->identify_state     = 1;
        p_foc_status->identify_tick      = 0;
        p_foc_status->encoder_pos_cache  = 0;
        p_foc_status->align_dir_detected = 0;
        p_foc_status->align_retry_count  = 0;
        break;

    case 1: // 缓慢增加D轴电压,直至电流达到额定电流或电压达到最大电压,并等待电流稳定
        if (p_foc_status->i_dq.D < p_motor_config->rated_current && p_foc_status->u_dq_ref.D < p_foc_status->uo_max)
        {
            p_foc_status->u_dq_ref.D    += 0.001f;
            p_foc_status->identify_tick  = 0;
        }
        else
        {
            if (p_foc_status->i_dq.D > FLOAT_EPSILON && fabsf(p_foc_status->i_dq.Q) < 0.1f * p_motor_config->rated_current && fabsf(p_encoder_status->encoder_speed) < 0.05f && fabsf((p_foc_status->i_dq.D - p_foc_status->i_dq_last.D) / p_foc_status->i_dq.D) < 0.05f)
            {
                p_foc_status->identify_tick++;
                if (p_foc_status->identify_tick > MS_TO_RT_TICKS(100))
                {
                    p_foc_status->encoder_pos_cache = p_encoder_status->encoder_pos * _2PI;
                    p_foc_status->identify_state    = 2;
                    p_foc_status->identify_tick     = 0;
                }
            }
            else
            {
                p_foc_status->identify_tick = 0;
            }
        }
        break;

    case 2: // 正向开环运行360°
        if (++p_foc_status->identify_tick > identify_tick)
        {
            p_foc_status->identify_tick  = 0;
            p_foc_status->identify_state = 3;
        }
        else
        {
            p_foc_status->theta_openloop = Normalize_Rad(p_foc_status->theta_openloop + DEG_TO_RAD(identify_deg) / identify_tick);
        }
        break;

    case 3: // 判断运行距离误差
        delta_rads     = p_encoder_status->encoder_pos * _2PI - p_foc_status->encoder_pos_cache;
        abs_delta_rads = fabsf(delta_rads);
        if (abs_delta_rads >= DEG_TO_RAD(min_deg) && abs_delta_rads < DEG_TO_RAD(max_deg))
        {
            // 第一次位移判断：记录运动方向（正向驱动下delta_theta应为正）
            if (p_foc_status->align_dir_detected == 0)
            {
                p_foc_status->align_dir_detected = (delta_rads > 0) ? 1 : -1;
            }
            float magnification             = DEG_TO_RAD(identify_deg) / abs_delta_rads;
            p_foc_status->pp_identify       = ROUND(magnification);
            p_foc_status->encoder_pos_cache = p_encoder_status->encoder_pos * _2PI;
            p_foc_status->identify_tick     = 0;
            p_foc_status->identify_state    = 4;
        }
        else
        {
            p_foc_status->encoder_pos_cache  = p_encoder_status->encoder_pos * _2PI;
            p_foc_status->identify_tick      = 0;
            p_foc_status->identify_state     = 2;
            p_foc_status->align_dir_detected = 0; // 重试时重置方向检测

            if (++p_foc_status->align_retry_count > max_retry_cnt)
            {
                motor_set_err(index, FAULT_CODE_PP_IDENTIFY);
            }
        }
        break;

    case 4: // 正向开环运行360°
        if (++p_foc_status->identify_tick > identify_tick)
        {
            p_foc_status->identify_tick  = 0;
            p_foc_status->identify_state = 5;
        }
        else
        {
            p_foc_status->theta_openloop = Normalize_Rad(p_foc_status->theta_openloop + DEG_TO_RAD(identify_deg) / identify_tick);
        }
        break;

    case 5: // 判断运行距离误差
        delta_rads     = p_encoder_status->encoder_pos * _2PI - p_foc_status->encoder_pos_cache;
        abs_delta_rads = fabsf(delta_rads);
        if (abs_delta_rads >= DEG_TO_RAD(min_deg) && abs_delta_rads < DEG_TO_RAD(max_deg))
        {
            // 第二次位移判断：与第一次结果对比，一致时确认方向检测有效
            int dir_step = (delta_rads > 0) ? 1 : -1;

            if (p_foc_status->align_dir_detected == dir_step)
            {
                p_foc_status->align_dir_detected = dir_step * 2; // 两次一致，确认: 2=正确, -2=需修正
            }
            else
            {
                p_foc_status->align_dir_detected = 0; // 两次不一致，放弃修正
            }

            float magnification = DEG_TO_RAD(identify_deg) / abs_delta_rads;
            if (p_foc_status->pp_identify != round(magnification))
            {
                p_foc_status->pp_identify = 0;
            }
            p_foc_status->encoder_pos_cache = p_encoder_status->encoder_pos * _2PI;
            p_foc_status->identify_tick     = 0;
            p_foc_status->identify_state    = 6;
        }
        else
        {
            p_foc_status->encoder_pos_cache  = p_encoder_status->encoder_pos * _2PI;
            p_foc_status->identify_tick      = 0;
            p_foc_status->identify_state     = 2;
            p_foc_status->align_dir_detected = 0; // 重试时重置方向检测

            if (++p_foc_status->align_retry_count > max_retry_cnt)
            {
                motor_set_err(index, FAULT_CODE_PP_IDENTIFY);
            }
        }
        break;

    case 6: // 应用方向辨识结果
        // 两次方向判断不一致或极对数判断不一致，放弃本次结果，回到state 2重试
        if (p_foc_status->align_dir_detected == 0 || p_foc_status->pp_identify == 0)
        {
            p_foc_status->encoder_pos_cache = p_encoder_status->encoder_pos * _2PI;
            p_foc_status->identify_state    = 2;
            if (++p_foc_status->align_retry_count > max_retry_cnt)
            {
                motor_set_err(index, FAULT_CODE_PP_IDENTIFY);
            }
            break;
        }

        p_foc_status->u_dq_ref.D     = 0;
        p_foc_status->u_dq_ref.Q     = 0;
        p_foc_status->theta_openloop = 0;

        int corrected_dir = p_motor_config->motor_dir;
        if (p_foc_status->align_dir_detected == -2)
        {
            corrected_dir = -corrected_dir;
        }

        p_foc_status->identify_state = 7;
        p_motor_config->motor_dir = corrected_dir;
        break;

    case 7: // 应用极对数辨识结果
        if (++p_foc_status->identify_tick > MS_TO_RT_TICKS(1))
        {
            p_motor_status->identify_done = 1;
            p_foc_status->identify_tick   = 0;
            p_foc_status->identify_state  = 8;
            p_motor_config->pp = p_foc_status->pp_identify;
        }
        break;

    default:

        break;
    }
}

/**
 * @brief foc实时更新
 * 
 * @param index 电机编号
 */
void foc_update_rt(int index)
{
}

/**
 * @brief foc高频实时更新
 * 
 * @param index 电机编号
 */
void foc_update_frt(int index)
{
    Motor_Config_t *p_motor_config = &global_config.motor_config[index];
    FOC_Status_t *p_foc_status     = &global_status.foc_status[index];
    HFI_Status_t *p_hfi_status     = &global_status.hfi_status[index];

    PLL_t *p_hfi_pll = &hfi_pll[index];

    foc_encoder_update(index);
    foc_us_limit_update(index);
    foc_clark_update(index);

    switch (p_foc_status->run_state)
    {
    case FOC_VF_OpenLoop:
        Sin_Cos_Cal(p_foc_status->theta_openloop, &p_foc_status->sin_cos);
        foc_park_update(index);
        foc_vf_update(index);
        foc_pwm_update(index);
        break;

    case FOC_IF_OpenLoop:
        Sin_Cos_Cal(p_foc_status->theta_openloop, &p_foc_status->sin_cos);
        foc_park_update(index);
        foc_if_update(index);
        foc_pwm_update(index);
        break;

#ifndef FAST_CURRENT_LOOP_ENABLE
    case FOC_HFI_OpenLoop:
        Sin_Cos_Cal(p_foc_status->theta_openloop, &p_foc_status->sin_cos);
        foc_park_update(index);
        hfi_update(index);
        // // 借用idq来显示解调后的dq轴电流高频响应
        // Park_Transmission_Fast(&p_hfi_status->iab_input, &p_foc_status->i_dq, p_foc_status->sin_cos);
        p_foc_status->u_dq_out.D = p_hfi_status->udq_excitation.D;
        p_foc_status->u_dq_out.Q = 0;
        Saturate_Vector_2d_Fast(&p_foc_status->u_dq_out.D, &p_foc_status->u_dq_out.Q, p_foc_status->uo_max);
        Inverse_Park_Transmission(&p_foc_status->u_dq_out, &p_foc_status->u_ab_out, p_foc_status->theta_openloop);
        foc_pwm_update(index);
        break;

    case FOC_HFI_Init:
        hfi_update(index);
        foc_hfi_apply(index);
        Sin_Cos_Cal(p_foc_status->theta, &p_foc_status->sin_cos);
        foc_park_update(index);
        p_foc_status->i_dq_ref.D = 0;
        p_foc_status->i_dq_ref.Q = 0;
        foc_current_loop_update(index);
        foc_limit_circle_update(index);
        foc_power_update(index);
        foc_pwm_update(index);
        if (is_hfi_locked(index))
        {
            // HFI已经在低速下建立稳定跟踪,但此时还无法区分跟踪的是N极还是S极,切换到极性辨识流程
            p_foc_status->run_state = FOC_HFI_Identify;
        }
        break;

    case FOC_HFI_Identify:
        foc_park_update(index);
        hfi_pole_identify_update(index);
        p_foc_status->u_dq_out.D = p_hfi_status->udq_excitation.D;
        p_foc_status->u_dq_out.Q = 0;
        Saturate_Vector_2d_Fast(&p_foc_status->u_dq_out.D, &p_foc_status->u_dq_out.Q, p_foc_status->uo_max);
        Inverse_Park_Transmission(&p_foc_status->u_dq_out, &p_foc_status->u_ab_out, p_foc_status->theta);
        foc_pwm_update(index);
        if (is_hfi_valid(index))
        {
            p_foc_status->run_state = FOC_HFI_CloseLoop;
        }
        break;

    case FOC_HFI_CloseLoop:
        hfi_update(index);
        foc_hfi_apply(index);
        Sin_Cos_Cal(p_foc_status->theta, &p_foc_status->sin_cos);
        foc_park_update(index);
        p_foc_status->i_dq_ref.D = 0;
        p_foc_status->i_dq_ref.Q = CONSTRAIN(p_foc_status->current_ref, -p_motor_config->peak_current, p_motor_config->peak_current);
        foc_current_loop_update(index);
        foc_limit_circle_update(index);
        foc_power_update(index);
        foc_pwm_update(index);
        break;
#endif

#ifndef HFI_POLARITY_PULSE_TEST
    case FOC_CloseLoop:
#ifndef FAST_CURRENT_LOOP_ENABLE
        foc_encoder_apply(index);
#else
        p_foc_status->theta      = p_foc_status->theta_encoder;
        p_foc_status->theta_comp = p_foc_status->theta_encoder;
#endif
        Sin_Cos_Cal(p_foc_status->theta, &p_foc_status->sin_cos);
        foc_park_update(index);
#ifndef FAST_CURRENT_LOOP_ENABLE
        foc_op_area_update(index);
        foc_decoupling_update(index);
        Sin_Cos_Cal(p_foc_status->theta_comp, &p_foc_status->sin_cos);
#else
        p_foc_status->i_dq_ref.D = 0;
        p_foc_status->i_dq_ref.Q = CONSTRAIN(p_foc_status->current_ref, -p_motor_config->peak_current, p_motor_config->peak_current);
#endif
        foc_current_loop_update(index);
        foc_limit_circle_update(index);
        foc_power_update(index);
        foc_pwm_update(index);
        break;
#else
    case FOC_CloseLoop: {
        HFI_Config_t *p_hfi_config = &global_config.hfi_config[index];
        const int pulse_ticks      = US_TO_HRT_TICKS(p_hfi_config->pole_identify_pulse_us);
        const int interval_ticks   = US_TO_HRT_TICKS(p_hfi_config->pole_identify_interval_us);
        const int half_cycle_ticks = pulse_ticks + interval_ticks;
        const int cycle_ticks      = 2 * half_cycle_ticks;
        p_foc_status->theta        = p_foc_status->theta_encoder;
        p_foc_status->theta_comp   = p_foc_status->theta_encoder;
        Sin_Cos_Cal(p_foc_status->theta, &p_foc_status->sin_cos);
        foc_park_update(index);
        // 使用从0开始的相位计数，先输出再递增，保证正负脉冲及间隔等长。
        if (p_foc_status->identify_tick < 0 || p_foc_status->identify_tick >= cycle_ticks)
        {
            p_foc_status->identify_tick = 0;
        }
        if (p_foc_status->identify_tick < pulse_ticks)
        {
            p_foc_status->u_dq_out.D = p_hfi_config->pole_identify_amp;
        }
        else if (p_foc_status->identify_tick < half_cycle_ticks)
        {
            p_foc_status->u_dq_out.D = 0;
        }
        else if (p_foc_status->identify_tick < half_cycle_ticks + pulse_ticks)
        {
            p_foc_status->u_dq_out.D = -p_hfi_config->pole_identify_amp;
        }
        else
        {
            p_foc_status->u_dq_out.D = 0;
        }
        if (++p_foc_status->identify_tick >= cycle_ticks)
        {
            p_foc_status->identify_tick = 0;
        }
        p_foc_status->u_dq_out.Q = 0;
        Saturate_Vector_2d_Fast(&p_foc_status->u_dq_out.D, &p_foc_status->u_dq_out.Q, p_foc_status->uo_max);
        Inverse_Park_Transmission(&p_foc_status->u_dq_out, &p_foc_status->u_ab_out, p_foc_status->theta);
        foc_power_update(index);
        foc_pwm_update(index);
    }
    break;
#endif

    default:
        break;
    }

#ifndef FAST_CURRENT_LOOP_ENABLE
    memcpy(&p_foc_status->u_ab_last, &p_foc_status->u_ab, sizeof(AB_Axis_t));
    memcpy(&p_foc_status->i_ab_last, &p_foc_status->i_ab, sizeof(AB_Axis_t));
    memcpy(&p_foc_status->u_dq_last, &p_foc_status->u_dq, sizeof(DQ_Axis_t));
    memcpy(&p_foc_status->i_dq_last, &p_foc_status->i_dq, sizeof(DQ_Axis_t));
#endif
}

/**
 * @brief foc编码器复位
 * 
 * @param index 电机编号
 */
void foc_encoder_reset(int index)
{
    FOC_Status_t *p_foc_status = &global_status.foc_status[index];

    p_foc_status->rads_encoder_offset = Normalize_Rad(p_foc_status->rads_encoder_offset - p_foc_status->rads_encoder);
}

/**
 * @brief 复位FOC电机控制器
 * 
 * @param index 电机编号
 */
void foc_controller_reset(int index)
{
    FOC_Status_t *p_foc_status = &global_status.foc_status[index];

    p_foc_status->run_state = FOC_IDLE;
    if (is_MTPA_allowed(index))
    {
        p_foc_status->op_area = FOC_OP_MTPA;
    }
    else
    {
        p_foc_status->op_area = FOC_OP_ID0;
    }

    p_foc_status->align_state     = 0;
    p_foc_status->align_tick      = 0;
    p_foc_status->current_ref     = 0;
    p_foc_status->torque_ref      = 0;
    p_foc_status->u_dq_ref.D      = 0;
    p_foc_status->u_dq_ref.Q      = 0;
    p_foc_status->i_dq_ref.D      = 0;
    p_foc_status->i_dq_ref.Q      = 0;
    p_foc_status->theta_openloop  = 0;
    p_foc_status->current_err_cnt = 0;

    p_foc_status->identify_state = 0;
    p_foc_status->identify_tick  = 0;
    memset(&p_foc_status->u_dq_cache, 0, sizeof(DQ_Axis_t) * 2);
    memset(&p_foc_status->i_dq_cache, 0, sizeof(DQ_Axis_t) * 2);
    p_foc_status->encoder_pos_cache = 0;
    p_foc_status->rs_identify       = 0;
    memset(&p_foc_status->l_identify, 0, sizeof(DQ_Axis_t));
    p_foc_status->pp_identify               = 0;
    p_foc_status->flux_identify             = 0;
    p_foc_status->overide_decoupling_update = false;

#ifndef FAST_CURRENT_LOOP_ENABLE
    hfi_init(index, 1);
#endif
}

/**
 * @brief 计算指定定子电流幅值下的FOC最大可用扭矩
 *
 * ID0使用配置的kt；kt无效时由永磁转矩系数1.5*pp*Flux计算。
 * MTPA使用固定电流圆上的最优Id/Iq，同时计入永磁转矩和磁阻转矩。
 *
 * @param index 电机编号
 * @param current 定子电流幅值
 * @return float 对应的正向扭矩能力
 */
static float foc_torque_from_current(int index, float current)
{
    Motor_Config_t *p_motor_config = &global_config.motor_config[index];
    float is                       = fabsf(current);

    if (is <= FLOAT_EPSILON)
    {
        return 0.0f;
    }

    if (is_MTPA_allowed(index))
    {
        float l_diff  = p_motor_config->L_diff;
        float flux    = p_motor_config->Flux;
        float root    = sqrtf(SQ(flux) + 8.0f * SQ(l_diff * is));
        float divisor = root + flux;

        if (fabsf(divisor) > FLOAT_EPSILON)
        {
            float id = 2.0f * l_diff * SQ(is) / divisor;
            float iq = sqrtf(fmaxf(0.0f, SQ(is) - SQ(id)));

            return fabsf(1.5f * p_motor_config->pp * iq * (flux + l_diff * id));
        }
    }

    if (fabsf(p_motor_config->kt) > FLOAT_EPSILON)
    {
        return is * fabsf(p_motor_config->kt);
    }

    return is * fabsf(1.5f * p_motor_config->pp * p_motor_config->Flux);
}

/**
 * @brief 根据FOC控制方式计算并缓存额定扭矩和峰值扭矩
 *
 * @param index 电机编号
 */
void foc_calculate_torque_limits(int index)
{
    Motor_Config_t *p_motor_config = &global_config.motor_config[index];
    Motor_Status_t *p_motor_status = &global_status.motor_status[index];

    p_motor_status->rated_torque = foc_torque_from_current(index, p_motor_config->rated_current);
    p_motor_status->peak_torque  = foc_torque_from_current(index, p_motor_config->peak_current);
}

/**
 * @brief 设置FOC控制器为手动模式
 * 
 * @param index 电机编号
 * @param id_ref d轴电流参考
 * @param iq_ref q轴电流参考
 */
void foc_controller_set_manual_mode(int index, float id_ref, float iq_ref)
{
    Motor_Config_t *p_motor_config = &global_config.motor_config[index];
    FOC_Status_t *p_foc_status     = &global_status.foc_status[index];

    if ((p_foc_status->run_state & HFI_RUNNING_MASK) == 0)
    {
        p_foc_status->run_state = FOC_CloseLoop;
    }
    p_foc_status->op_area = FOC_OP_MANUAL;

    p_foc_status->i_dq_ref.D = CONSTRAIN(id_ref, -p_motor_config->peak_current, p_motor_config->peak_current);
    p_foc_status->i_dq_ref.Q = CONSTRAIN(iq_ref, -p_motor_config->peak_current, p_motor_config->peak_current);
}

/**
 * @brief 设置FOC控制器为自动模式
 * 
 * @param index 电机编号
 */
void foc_controller_set_auto_mode(int index)
{
    FOC_Status_t *p_foc_status = &global_status.foc_status[index];

    if ((p_foc_status->run_state & HFI_RUNNING_MASK) == 0)
    {
        p_foc_status->run_state = FOC_CloseLoop;
    }
    if (is_MTPA_allowed(index))
    {
        p_foc_status->op_area = FOC_OP_MTPA;
    }
    else
    {
        p_foc_status->op_area = FOC_OP_ID0;
    }
}

/**
 * @brief 设置FOC电流参考
 * 
 * @param index 电机编号
 * @param current_ref 电流参考
 */
void foc_set_current_ref(int index, float current_ref)
{
    FOC_Status_t *p_foc_status = &global_status.foc_status[index];

    if ((p_foc_status->run_state & HFI_RUNNING_MASK) == 0)
    {
#ifndef HFI_DEBUG
        p_foc_status->run_state = FOC_CloseLoop;
#else
        p_foc_status->run_state = FOC_HFI_Init;
#endif
    }

    p_foc_status->current_ref = current_ref;

    if (is_MTPA_allowed(index))
    {
        p_foc_status->op_area = FOC_OP_MTPA;
    }
    else
    {
        p_foc_status->op_area = FOC_OP_ID0;
    }
}

/**
 * @brief 获取FOC电流反馈
 * 
 * @param index 电机编号
 * @return float 电流反馈
 */
float foc_get_current_fb(int index)
{
    FOC_Status_t *p_foc_status = &global_status.foc_status[index];

    return p_foc_status->current_fb;
}

/**
 * @brief 设置FOC扭矩参考
 * 
 * @param index 电机编号
 * @param torque_ref 设置FOC扭矩参考
 */
void foc_set_torque_ref(int index, float torque_ref)
{
    Motor_Config_t *p_motor_config = &global_config.motor_config[index];
    Motor_Status_t *p_motor_status = &global_status.motor_status[index];
    FOC_Status_t *p_foc_status     = &global_status.foc_status[index];

    if ((p_foc_status->run_state & HFI_RUNNING_MASK) == 0)
    {
        p_foc_status->run_state = FOC_CloseLoop;
    }

    p_foc_status->torque_ref = torque_ref;

    if (is_MTPA_allowed(index))
    {
        p_foc_status->op_area = FOC_OP_MTPA;
    }
    else
    {
        p_foc_status->op_area = FOC_OP_ID0;
    }

    if (is_MTPA_allowed(index))
    {
        p_foc_status->current_ref = IPM_MTPA_From_Torque(p_motor_config, torque_ref);
    }
    else if (p_motor_config->kt > FLOAT_MIN)
    {
        p_foc_status->current_ref = torque_ref / p_motor_config->kt;
    }
    else
    {
        /* ID0 时 T = 1.5 * pp * Flux * Iq。 */
        float torque_coefficient = 1.5f * p_motor_config->pp * p_motor_config->Flux;

        if (torque_coefficient > FLOAT_MIN)
        {
            p_foc_status->current_ref = torque_ref / torque_coefficient;
        }
        else
        {
            p_foc_status->current_ref = 0.0f;
            motor_set_ext_err(index, EXT_FAULT_CODE_CFG_MOTOR_FLUX);
        }
    }

    p_motor_status->current_ref = p_foc_status->current_ref;
}

/**
 * @brief 获取FOC扭矩反馈
 * 
 * @param index 电机编号
 * @return float 扭矩反馈
 */
float foc_get_torque_fb(int index)
{
    FOC_Status_t *p_foc_status = &global_status.foc_status[index];

    return p_foc_status->torque_fb;
}

/**
 * @brief 设置FOC开环电压
 * 
 * @param index 电机编号
 * @param ud d轴电压
 * @param uq q轴电压
 * @param p_theta 开环电角度
 * @param p_we 开环电角速度
 */
void foc_set_vf_voltage(int index, float ud, float uq, float *p_theta, float *p_we)
{
    FOC_Status_t *p_foc_status = &global_status.foc_status[index];

    p_foc_status->run_state  = FOC_VF_OpenLoop;
    p_foc_status->u_dq_ref.D = ud;
    p_foc_status->u_dq_ref.Q = uq;

    if (p_theta)
    {
        p_foc_status->theta_openloop = Normalize_Rad(*p_theta);
    }
    else if (p_we)
    {
        p_foc_status->theta_openloop = Normalize_Rad(p_foc_status->theta_openloop + *p_we * BOARD_RT_DT);
    }
}

/**
 * @brief 设置FOC开环电流
 * 
 * @param index 电机编号
 * @param id d轴电流
 * @param iq q轴电流
 * @param p_theta 开环电角度
 * @param p_we 开环电角速度
 */
void foc_set_if_current(int index, float id, float iq, float *p_theta, float *p_we)
{
    FOC_Status_t *p_foc_status = &global_status.foc_status[index];

    p_foc_status->run_state  = FOC_IF_OpenLoop;
    p_foc_status->i_dq_ref.D = id;
    p_foc_status->i_dq_ref.Q = iq;

    if (p_theta)
    {
        p_foc_status->theta_openloop = Normalize_Rad(*p_theta);
    }
    else if (p_we)
    {
        p_foc_status->theta_openloop = Normalize_Rad(p_foc_status->theta_openloop + *p_we * BOARD_RT_DT);
    }
}

/**
 * @brief 设置FOC HFI开环电压
 * 
 * @param index 电机编号
 * @param ud d轴电压
 * @param p_theta 开环电角度
 * @param p_we 开环电角速度
 */
void foc_set_hfi_vf_voltage(int index, float ud, float *p_theta, float *p_we)
{
    FOC_Status_t *p_foc_status = &global_status.foc_status[index];
    HFI_Status_t *p_hfi_status = &global_status.hfi_status[index];

    p_foc_status->run_state    = FOC_HFI_OpenLoop;
    p_hfi_status->is_open_loop = true;
    p_hfi_status->vf_amp       = ud;

    if (p_we && *p_we != 0)
    {
        p_foc_status->theta_openloop = Normalize_Rad(p_foc_status->theta_openloop + *p_we * BOARD_RT_DT);
    }
    else if (p_theta && *p_theta != 0)
    {
        p_foc_status->theta_openloop = Normalize_Rad(*p_theta);
    }
    else
    {
        p_foc_status->theta_openloop = 0;
    }
}

/**
 * @brief 设置FOC编码器偏移
 * 
 * @param index 电机编号
 * @param offset 编码器偏移
 */
void foc_set_encoder_offset(int index, float offset)
{
    FOC_Status_t *p_foc_status = &global_status.foc_status[index];

    p_foc_status->rads_encoder_offset = offset;
}

/**
 * @brief 获取FOC编码器偏移
 * 
 * @param index 电机编号
 * @return float 编码器偏移
 */
float foc_get_encoder_offset(int index)
{
    FOC_Status_t *p_foc_status = &global_status.foc_status[index];

    return p_foc_status->rads_encoder_offset;
}
