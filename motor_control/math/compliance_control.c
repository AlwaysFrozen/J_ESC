/**
 * @file Compliance_Control.c
 * @author jinchl (jcl709905683@gmail.com)
 * @brief 
 * @version 0.1
 * @date 2026-06-22
 * 
 * @copyright Copyright (c) 2026
 * 
 */


#include "compliance_control.h"
#include "utils.h"

// https://zhuanlan.zhihu.com/p/1948488928983250459

/**
 * @brief 初始化柔顺控制器
 * 
 * @param pCtrl 控制器指针
 * @param dt 离散时间
 * @param pCfg 参数配置
 * @param pLimit 限幅配置
 */
void Compliance_Controller_Init(Compliance_Controller_t *pCtrl, float dt, Compliance_Config_t *pCfg, Compliance_Limit_t *pLimit)
{
    if (pCtrl == NULL || pCfg == NULL || pLimit == NULL)
    {
        return;
    }

    memset(pCtrl, 0, sizeof(Compliance_Controller_t));
    pCtrl->dt = dt;
    memcpy(&pCtrl->config, pCfg, sizeof(Compliance_Config_t));
    memcpy(&pCtrl->limit, pLimit, sizeof(Compliance_Limit_t));
}

/**
 * @brief 根据目标频率和虚拟质量整定完整导纳控制器参数
 *
 * 完整导纳控制器模型为：
 *
 *     Md * x_acc + Bd * x_vel + Kd * x_pos = F_err
 *
 * 其中：
 *
 *     Md：virtual_mass，虚拟质量，单位 kg
 *     Bd：virtual_damping，虚拟阻尼，单位 N*s/m
 *     Kd：virtual_stiffness，虚拟刚度，单位 N/m
 *
 * 该函数按照标准二阶系统形式整定参数：
 *
 *     x_acc + 2ζωn * x_vel + ωn² * x_pos = F_err / Md
 *
 * 因此有：
 *
 *     Bd = 2ζMdωn
 *     Kd = Mdωn²
 *
 * 其中：
 *
 *     Hz   ：导纳系统目标固有频率，单位 Hz
 *     Wn   ：导纳系统目标固有角频率，单位 rad/s
 *     zeta ：阻尼比
 *
 * 当 zeta = 1.0 时，系统接近临界阻尼，响应较稳，不容易产生振荡。
 *
 * @param pCtrl 控制器指针
 * @param Hz   目标导纳频率，单位 Hz
 *             值越大，导纳响应越快，但接触过程越容易变硬
 *             精密下压建议从 1~5 Hz 开始
 *
 * @param Md   虚拟质量，单位 kg
 *             值越大，加速度响应越慢，对力噪声越不敏感
 *             可先取接近实际动子质量
 */
void Compliance_Controller_Tune(Compliance_Controller_t *pCtrl, float Hz, float Md)
{
    /*
     * 空指针保护。
     */
    if (pCtrl == NULL)
    {
        return;
    }

    /*
     * 参数保护。
     *
     * Hz 必须大于 0，否则 Wn 无意义。
     * M 必须大于 0，否则虚拟质量无意义。
     */
    if ((Hz <= 0.0f) || (Md <= 0.0f))
    {
        return;
    }

    /*
     * 将目标频率 Hz 转换为角频率 rad/s。
     *
     * Wn = 2πf
     */
    float Wn = Hz * _2PI;

    /*
     * 阻尼比。
     *
     * zeta = 1.0 表示临界阻尼附近，适合精密接触场景。
     *
     * zeta < 1.0：
     *     响应更快，但可能有超调和振荡。
     *
     * zeta > 1.0：
     *     响应更慢，但更稳定。
     */
    float zeta = 1.0f;

    /*
     * 设置虚拟质量 Md。
     *
     * 单位：kg
     *
     * Md 越大：
     *     同样力误差下，加速度越小，响应越柔和。
     *
     * Md 越小：
     *     响应越快，但更容易受力传感器噪声影响。
     */
    pCtrl->config.virtual_mass = Md;

    /*
     * 设置虚拟阻尼 Bd。
     *
     * 公式：
     *
     *     Bd = 2ζMdωn
     *
     * 单位：
     *
     *     kg * rad/s = kg/s
     *
     * 而：
     *
     *     1 N = 1 kg*m/s²
     *
     * 所以：
     *
     *     N*s/m = kg/s
     *
     * 因此单位正确。
     *
     * Bd 越大：
     *     速度响应越慢，接触越柔和。
     *
     * Bd 越小：
     *     速度响应越快，但更容易振荡。
     */
    pCtrl->config.virtual_damping = 2.0f * zeta * pCtrl->config.virtual_mass * Wn;

    /*
     * 设置虚拟刚度 Kd。
     *
     * 公式：
     *
     *     Kd = Mdωn²
     *
     * 单位：
     *
     *     kg * 1/s² = kg/s² = N/m
     *
     * 因此单位正确。
     *
     * Kd 越大：
     *     导纳位移越小，系统表现越硬。
     *
     * Kd 越小：
     *     导纳位移越大，系统表现越软。
     *
     * 注意：
     *     对于精密下压力控，Kd 不宜过大。
     *     如果接触刚度为 Kc，建议：
     *
     *         Kd <= 0.05 * Kc
     *
     *     否则可能导致接触力稳态误差变大，或者接触过程变硬。
     */
    pCtrl->config.virtual_stiffness = pCtrl->config.virtual_mass * SQ(Wn);
}

/**
 * @brief 复位完整导纳控制器内部状态
 *
 * 使用场景：
 * 1. 从位置模式切换到力控模式时
 * 2. 接触状态丢失时
 * 3. 发生故障后重新开始时
 * 4. 目标物品更换时
 *
 * @param pCtrl 控制器指针
 */
void Compliance_Controller_Reset(Compliance_Controller_t *pCtrl)
{
    if (pCtrl == NULL)
    {
        return;
    }

    memset(&pCtrl->status, 0, sizeof(Compliance_Status_t));
}

/**
 * @brief 阻抗控制器周期更新
 *
 * 控制方程：
 *
 *     F_cmd = Md * (a_ref - a_fb)
 *           + Bd * (v_ref - v_fb)
 *           + Kd * (x_ref - x_fb)
 *           + F_ff
 *
 * 对直线电机：
 *
 *     Iq_cmd = F_cmd / Kt
 *
 * @param pCtrl 控制器指针
 * @param pos_err 位置误差
 * @param vel_err 速度误差
 * @param acc_err 加速度误差
 * @param force_ff 力前馈值
 * @param use_md 使能虚拟质量
 */
void Impedance_Update(Compliance_Controller_t *pCtrl, float pos_err, float vel_err, float acc_err, float force_ff, bool use_md)
{
    if (pCtrl == NULL)
    {
        return;
    }

    /******************** 1. 计算误差 ********************/

    pCtrl->status.pos_err = pos_err;

    pCtrl->status.vel_err = vel_err;

    pCtrl->status.acc_err = acc_err;

    pCtrl->status.force_ff = force_ff;

    /******************** 2. 误差限幅 ********************/

    /*
     * 限制位置误差，防止 Kd * pos_err 过大。
     */
    pCtrl->status.pos_err = CONSTRAIN(pCtrl->status.pos_err,
                                      -pCtrl->limit.max_pos_err,
                                      pCtrl->limit.max_pos_err);

    /*
     * 限制速度误差，防止 Bd * vel_err 过大。
     */
    pCtrl->status.vel_err = CONSTRAIN(pCtrl->status.vel_err,
                                      -pCtrl->limit.max_vel_err,
                                      pCtrl->limit.max_vel_err);

    if (use_md)
    {
        /*
        * 限制加速度误差。
        */
        pCtrl->status.acc_err = CONSTRAIN(pCtrl->status.acc_err,
                                          -pCtrl->limit.max_acc_err,
                                          pCtrl->limit.max_acc_err);
    }

    /******************** 3. 计算各项推力 ********************/

    if (use_md)
    {
        /*
        * 虚拟质量项。
        *
        * force_m = Md * acc_err
        *
        * 如果 acc_fb 噪声大，可以不使用该项：
        *
        *     pCtrl->force_m = 0.0f;
        */
        pCtrl->status.force_m = pCtrl->config.virtual_mass * pCtrl->status.acc_err;
    }

    /*
     * 虚拟阻尼项。
     *
     * force_d = Bd * vel_err
     *
     * 这是阻抗控制中抑制振荡和反弹的重要项。
     */
    pCtrl->status.force_d = pCtrl->config.virtual_damping * pCtrl->status.vel_err;

    /*
     * 虚拟刚度项。
     *
     * force_p = Kd * pos_err
     *
     * 该项决定系统的“硬度”。
     * 对精密下压，Kd 不要太大。
     */
    pCtrl->status.force_p = pCtrl->config.virtual_stiffness * pCtrl->status.pos_err;

    /******************** 4. 合成推力指令 ********************/

    /*
     * 最终推力指令。
     *
     * force_ff 可以用于：
     * 1. 重力补偿
     * 2. 摩擦补偿
     * 3. 期望恒定下压力
     */
    if (use_md)
    {
        pCtrl->status.force_cmd = pCtrl->status.force_m
                                  + pCtrl->status.force_d
                                  + pCtrl->status.force_p
                                  + pCtrl->status.force_ff;
    }
    else
    {
        pCtrl->status.force_cmd = pCtrl->status.force_d
                                  + pCtrl->status.force_p
                                  + pCtrl->status.force_ff;
    }

    /******************** 5. 推力限幅 ********************/

    pCtrl->status.force_cmd = CONSTRAIN(pCtrl->status.force_cmd,
                                        pCtrl->limit.min_force,
                                        pCtrl->limit.max_force);
}

/**
 * @brief 导纳控制器周期更新
 *
 * 控制方程：
 *
 *     Md * x_acc + Bd * x_vel + Kd * x_pos = F_ref - F_meas
 *
 * 整理为：
 *
 *     x_acc = (F_ref - F_meas - Bd * x_vel - Kd * x_pos) / Md
 *
 * 然后积分：
 *
 *     x_vel = x_vel + x_acc * dt
 *     x_pos = x_pos + x_vel * dt
 *
 * @param pCtrl 控制器指针
 * @param force_err 力误差值
 * @param use_kd 使能虚拟刚度
 */
void Admittance_Update(Compliance_Controller_t *pCtrl, float force_err, bool use_kd)
{
    if (pCtrl == NULL)
    {
        return;
    }

    /*
     * 防止虚拟质量过小导致除零或者加速度过大。
     */
    if (pCtrl->config.virtual_mass <= FLOAT_EPSILON)
    {
        pCtrl->config.virtual_mass = FLOAT_EPSILON;
    }

    /*
     * 1. 计算力误差。
     *
     * 向下为正时：
     * force_err > 0 表示实际力小于目标力，需要继续向下压。
     * force_err < 0 表示实际力大于目标力，需要向上释放。
     */
    pCtrl->status.force_err = force_err;

    /*
     * 2. 根据完整导纳方程计算虚拟加速度。
     *
     * Md * x_acc + Bd * x_vel + Kd * x_pos = force_err
     *
     * 因此：
     *
     * x_acc = (force_err - Bd * x_vel - Kd * x_pos) / Md
     *
     * 各项含义：
     *
     * force_err:
     *     外部驱动力，相当于推动虚拟质量运动的力。
     *
     * Bd * x_vel:
     *     虚拟阻尼力，速度越大，阻碍越大。
     *
     * Kd * x_pos:
     *     虚拟弹簧力，位移越大，回拉越大。
     */
    if (use_kd)
    {
        pCtrl->status.x_acc = (pCtrl->status.force_err
                               - pCtrl->config.virtual_damping * pCtrl->status.x_vel
                               - pCtrl->config.virtual_stiffness * pCtrl->status.x_pos)
                              / pCtrl->config.virtual_mass;
    }
    else
    {
        pCtrl->status.x_acc = (pCtrl->status.force_err
                               - pCtrl->config.virtual_damping * pCtrl->status.x_vel)
                              / pCtrl->config.virtual_mass;
    }

    /*
     * 3. 对虚拟加速度进行限幅。
     *
     * 这一步很重要。
     * 力反馈有尖峰时，如果不限制加速度，速度指令可能产生突变。
     */
    pCtrl->status.x_acc = CONSTRAIN(pCtrl->status.x_acc,
                                    pCtrl->limit.min_acceleration,
                                    pCtrl->limit.max_acceleration);

    /*
     * 4. 积分得到虚拟速度。
     */
    pCtrl->status.x_vel += pCtrl->status.x_acc * pCtrl->dt;

    /*
     * 5. 对虚拟速度进行限幅。
     *
     * 对精密下压，速度限制比导纳参数本身更关键。
     */
    pCtrl->status.x_vel = CONSTRAIN(pCtrl->status.x_vel,
                                    pCtrl->limit.min_velocity,
                                    pCtrl->limit.max_velocity);

    if (use_kd)
    {
        /*
        * 6. 积分得到虚拟位移。
        */
        pCtrl->status.x_pos += pCtrl->status.x_vel * pCtrl->dt;

        /*
        * 7. 对虚拟位移进行限幅。
        *
        * 防止导纳控制器因为传感器异常或者长期偏差而产生过大位移。
        */
        pCtrl->status.x_pos = CONSTRAIN(pCtrl->status.x_pos,
                                        pCtrl->limit.min_position,
                                        pCtrl->limit.max_position);
    }
}
