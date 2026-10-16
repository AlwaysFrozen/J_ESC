/**
 * @file Foc_Utils.c
 * @author jinchl (jcl709905683@gmail.com)
 * @brief 
 * @version 0.1
 * @date 2026-03-30
 * 
 * @copyright Copyright (c) 2026
 * 
 */

#include "foc_utils.h"

/**
 * @brief 基于αβ轴输出比例的空间矢量PWM调制
 * 
 * @param v_ab_rate αβ轴电压比例
 * @param duty 占空比
 * @return uint8_t 当前扇区
 */
uint8_t SVPWM_AB(AB_Axis_t *v_ab_rate, UVW_Axis_t *duty)
{
    uint8_t sector;

    if (v_ab_rate->Beta >= 0.0f)
    {
        if (v_ab_rate->Alpha > 0.0f)
        {
            if (v_ab_rate->Beta / v_ab_rate->Alpha >= _SQRT3)
            {
                sector = 2;
            }
            else
            {
                sector = 1;
            }
        }
        else if (v_ab_rate->Alpha < 0.0f)
        {
            if (v_ab_rate->Beta / v_ab_rate->Alpha >= -_SQRT3)
            {
                sector = 3;
            }
            else
            {
                sector = 2;
            }
        }
        else
        {
            sector = 2;
        }
    }
    else
    {
        if (v_ab_rate->Alpha > 0.0f)
        {
            if (v_ab_rate->Beta / v_ab_rate->Alpha >= -_SQRT3)
            {
                sector = 6;
            }
            else
            {
                sector = 5;
            }
        }
        else if (v_ab_rate->Alpha < 0.0f)
        {
            if (v_ab_rate->Beta / v_ab_rate->Alpha >= _SQRT3)
            {
                sector = 5;
            }
            else
            {
                sector = 4;
            }
        }
        else
        {
            sector = 5;
        }
    }

    float tA, tB, tC;

    switch (sector)
    {
    // sector 1-2
    case 1: {
        // Vector on-times
        float t1 = (v_ab_rate->Alpha - _1_SQRT3 * v_ab_rate->Beta);
        float t2 = (_2_SQRT3 * v_ab_rate->Beta);

        tC = (1.0f - t1 - t2) / 2;
        tB = tC + t2;
        tA = tB + t1;
        break;
    }

    // sector 2-3
    case 2: {
        // Vector on-times
        float t2 = (v_ab_rate->Alpha + _1_SQRT3 * v_ab_rate->Beta);
        float t3 = (-v_ab_rate->Alpha + _1_SQRT3 * v_ab_rate->Beta);

        tC = (1.0f - t2 - t3) / 2;
        tA = tC + t2;
        tB = tA + t3;
        break;
    }

    // sector 3-4
    case 3: {
        // Vector on-times
        float t3 = (_2_SQRT3 * v_ab_rate->Beta);
        float t4 = (-v_ab_rate->Alpha - _1_SQRT3 * v_ab_rate->Beta);

        tA = (1.0f - t3 - t4) / 2;
        tC = tA + t4;
        tB = tC + t3;
        break;
    }

    // sector 4-5
    case 4: {
        // Vector on-times
        float t4 = (-v_ab_rate->Alpha + _1_SQRT3 * v_ab_rate->Beta);
        float t5 = (-_2_SQRT3 * v_ab_rate->Beta);

        tA = (1.0f - t4 - t5) / 2;
        tB = tA + t4;
        tC = tB + t5;
        break;
    }

    // sector 5-6
    case 5: {
        // Vector on-times
        float t5 = (-v_ab_rate->Alpha - _1_SQRT3 * v_ab_rate->Beta);
        float t6 = (v_ab_rate->Alpha - _1_SQRT3 * v_ab_rate->Beta);

        tB = (1.0f - t5 - t6) / 2;
        tA = tB + t6;
        tC = tA + t5;
        break;
    }

    // sector 6-1
    case 6: {
        // Vector on-times
        float t6 = (-_2_SQRT3 * v_ab_rate->Beta);
        float t1 = (v_ab_rate->Alpha + _1_SQRT3 * v_ab_rate->Beta);

        tB = (1.0f - t6 - t1) / 2;
        tC = tB + t6;
        tA = tC + t1;
        break;
    }
    }

    tA = CONSTRAIN(tA, 0, 1.0f);
    tB = CONSTRAIN(tB, 0, 1.0f);
    tC = CONSTRAIN(tC, 0, 1.0f);

    duty->U = tA;
    duty->V = tB;
    duty->W = tC;

    return sector;
}

/**
 * @brief 基于αβ轴输出电压的空间矢量PWM调制
 * 
 * @param v_ab αβ轴电压
 * @param us_max 最大电压矢量幅值
 * @param duty 占空比
 * @return uint8_t 
 */
uint8_t SVPWM_AB_Voltage(AB_Axis_t *v_ab, float us_max, UVW_Axis_t *duty)
{
    AB_Axis_t v_ab_rate;

    v_ab_rate.Alpha = v_ab->Alpha / us_max;
    v_ab_rate.Beta  = v_ab->Beta / us_max;

    return SVPWM_AB(&v_ab_rate, duty);
}

/**
 * @brief 基于dq轴输出比例的空间矢量PWM调制
 * 
 * @param u_dq_rate dq轴电压比例
 * @param angle 电气角度
 * @param duty 占空比
 * @return uint8_t 当前扇区
 */
uint8_t SVPWM_DQ(DQ_Axis_t *u_dq_rate, float angle, UVW_Axis_t *duty)
{
    uint8_t sector;
    float angle_out;
    float angle_offset;
    float Uout;

    if (u_dq_rate->D)
    {
        Uout      = NORM2_f(u_dq_rate->D, u_dq_rate->Q);
        angle_out = Normalize_Rad(angle + atan2f(u_dq_rate->Q, u_dq_rate->D));
    }
    else
    {
        Uout      = u_dq_rate->Q;
        angle_out = Normalize_Rad(angle + _PI_2);
    }

    sector        = floorf(angle_out / _PI_3);
    angle_offset  = angle_out - sector * _PI_3;
    sector       += 1;

    float st, ct;
    st = sinf(angle_offset);
    ct = cosf(angle_offset);

    /*
    t0~t7为三相全桥对应的8个矢量的作用时间
    T1为当前扇区的1号有效矢量的作用时间
    T2为当前扇区的2号有效矢量的作用时间
    T1、T2对应不同扇区t1~t6的作用时间
    T0为当前扇区的0矢量作用时间之和，又因为t0与t7矢量作用时长相等，所以为方便计算将T0除二即t0矢量的作用时长用作占空比计算
    */
    float T1 = Uout * (ct - _1_SQRT3 * st);
    float T2 = _2_SQRT3 * Uout * st;
    float T0 = (1.0f - T1 - T2) * 0.5f; // actually this T0 is (t0 + t7) / 2,because t0 == t7

    float Ta, Tb, Tc;
    switch (sector)
    {
    case 1:
        Ta = T1 + T2 + T0;
        Tb = T2 + T0;
        Tc = T0;
        break;

    case 2:
        Ta = T1 + T0;
        Tb = T1 + T2 + T0;
        Tc = T0;
        break;

    case 3:
        Ta = T0;
        Tb = T1 + T2 + T0;
        Tc = T2 + T0;
        break;

    case 4:
        Ta = T0;
        Tb = T1 + T0;
        Tc = T1 + T2 + T0;
        break;

    case 5:
        Ta = T2 + T0;
        Tb = T0;
        Tc = T1 + T2 + T0;
        break;

    case 6:
        Ta = T1 + T2 + T0;
        Tb = T0;
        Tc = T1 + T0;
        break;

    default:
        // possible error state
        Ta = 0;
        Tb = 0;
        Tc = 0;
        break;
    }

    Ta = CONSTRAIN(Ta, 0.0f, 1.0f);
    Tb = CONSTRAIN(Tb, 0.0f, 1.0f);
    Tc = CONSTRAIN(Tc, 0.0f, 1.0f);

    duty->U = Ta;
    duty->V = Tb;
    duty->W = Tc;

    return sector;
}

/**
 * @brief 基于dq轴输出电压的空间矢量PWM调制
 * 
 * @param u_dq dq轴电压比例
 * @param us_max 最大电压矢量幅值
 * @param angle 电气角度
 * @param duty 占空比
 * @return uint8_t 当前扇区
 */
uint8_t SVPWM_DQ_Voltage(DQ_Axis_t *u_dq, float us_max, float angle, UVW_Axis_t *duty)
{
    DQ_Axis_t u_dq_rate;

    u_dq_rate.D = u_dq->D / us_max;
    u_dq_rate.Q = u_dq->Q / us_max;

    return SVPWM_DQ(&u_dq_rate, angle, duty);
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
