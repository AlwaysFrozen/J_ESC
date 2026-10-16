/**
 * @file filter.h
 * @author jinchl (jcl709905683@gmail.com)
 * @brief 
 * @version 0.2
 * @date 2026-09-18
 * 
 * @copyright Copyright (c) 2026
 * 
 */


#pragma once
#include "utils.h"

#define FIR_MAX_TAPS                    (64U)       // FIR最多支持64个抽头
#define FIR_MAX_ORDER                   (FIR_MAX_TAPS - 1U)
#define IIR_MAX_ORDER                   (4U)        // IIR最高支持4阶
#define IIR_MAX_SECTIONS                (2U)        // 最多串联两个二阶节
#define IIR_SECTION_COEF_COUNT          (3U)        // 每节b0~b2、a0~a2
#define FIR_BAND_TRANSITION_WIDTH_RATIO (0.25f)     // 带通/带阻过渡带宽占主体带宽的比例
#define FILTER_DEPTH_MIN_DB             (9.0f)      // Kaiser阶数估算公式要求阻带衰减大于8dB
#define FILTER_DEPTH_MAX_DB             (120.0f)    // 限制极端配置和I0计算范围
#define FILTER_FREQ_MIN                 (1.0e-6f)   // 归一化频率下限
#define FILTER_FREQ_MAX                 (0.499999f) // 归一化频率上限，0.5对应Nyquist频率
#define FILTER_EPS                      (1.0e-12f)  // 浮点零判断阈值


typedef enum
{
    Filter_Struct_None, // 不使用滤波器，初始化后直通
    Filter_Struct_IIR,  // 无限冲激响应滤波器
    Filter_Struct_FIR,  // 有限冲激响应滤波器
    Filter_Struct_MAX
} Filter_Struct_t;

typedef enum
{
    Filter_Type_None,     // 不指定类型，初始化后直通
    Filter_Type_Lowpass,  // 低通
    Filter_Type_Highpass, // 高通
    Filter_Type_Bandpass, // 带通
    Filter_Type_Bandstop, // 带阻
    Filter_Type_MAX
} Filter_Type_t;

typedef enum
{
    Filter_Status_OK = 0,            // 初始化成功或配置为直通
    Filter_Status_Invalid_Argument,  // 空指针或无效采样频率
    Filter_Status_Invalid_Struct,    // 滤波器结构枚举无效
    Filter_Status_Invalid_Type,      // 滤波器类型枚举无效
    Filter_Status_Invalid_Frequency, // 中心/截止/边界频率无效
    Filter_Status_Invalid_Width,     // 主体宽度或过渡带宽无效
    Filter_Status_Invalid_Depth,     // FIR目标阻带衰减超出9~120dB
    Filter_Status_Order_Unsupported, // IIR阶数/类型组合无效或FIR需要超过64抽头
} Filter_Status_t;

/**
 * @brief 滤波器配置。
 *
 * 参数生效规则：
 * - None：所有参数均忽略，输入直通。
 * - FIR低通/高通：filter_freq为截止频率Hz，filter_width为过渡带宽Hz，
 *   filter_depth为目标阻带衰减dB；filter_order忽略，由深度和过渡带宽自动计算。
 * - FIR带通/带阻：filter_freq为中心频率Hz，filter_width为通带/阻带主体宽度Hz，
 *   过渡带宽按filter_width的25%推导，filter_depth为目标阻带衰减dB；filter_order忽略。
 * - IIR低通/高通：filter_order支持1~4阶，filter_freq为截止频率Hz；
 *   filter_width和filter_depth忽略。
 * - IIR带通/带阻：filter_order支持2阶（一个biquad）或4阶（两个biquad串联）；
 *   filter_freq为中心频率Hz，filter_width为通带/阻带宽度Hz，filter_depth忽略。
 */
typedef struct
{
    Filter_Struct_t filter_struct;
    Filter_Type_t filter_type;
    int filter_order;
    float filter_freq;
    float filter_width;
    float filter_depth;
} Filter_Config_t;

typedef struct
{
    float b[FIR_MAX_TAPS]; // FIR系数
    float x[FIR_MAX_TAPS]; // FIR输入环形缓冲区
    uint16_t taps;         // 实际抽头数
    uint16_t head;         // 下一个输入写入位置
} FIR_Filter_t;

typedef struct
{
    float b[IIR_SECTION_COEF_COUNT]; // 分子系数b0~b2
    float a[IIR_SECTION_COEF_COUNT]; // 分母系数a0~a2，a0固定为1
    float z[2];                      // Direct Form II Transposed状态
    uint8_t order;                   // 当前节阶数，取值1或2
} IIR_Section_t;

typedef struct
{
    IIR_Section_t section[IIR_MAX_SECTIONS]; // 按数组顺序串联运行
    uint8_t section_count;                   // 实际节数，取值1~2
    uint8_t order;                           // 总阶数，支持1~4
} IIR_Filter_t;

typedef struct
{
    Filter_Struct_t filter_struct;
    Filter_Status_t status;
    union {
        FIR_Filter_t fir;
        IIR_Filter_t iir;
    } state;
} Filter_t;

/**
 * @brief 单步一阶低通滤波宏。
 *
 * @param sample 当前采样值，单位由调用方信号定义。
 * @param value 输入/输出变量，保存滤波后的值，单位与 sample 相同。
 * @param filter_constant 滤波系数，无量纲，取值范围建议 0.0~1.0。
 */
#define FirstOrder_LPF_Cal(sample, value, filter_constant) ((value) = (1.0f - filter_constant) * (value) + (filter_constant) * (sample))

/**
 * @brief 根据截止频率和采样周期计算一阶低通滤波系数。
 *
 * @param fc 截止频率，单位 Hz，取值范围建议 > 0。
 * @param dt 采样周期，单位 s，取值范围建议 > 0。
 * @return float 一阶低通滤波系数 alpha，范围 0.0~1.0。
 */
static inline float LPF_Alpha_Cal(float fc, float dt)
{
    float rc    = 1.0f / (_2PI * fc);
    float alpha = CONSTRAIN((dt / (dt + rc)), 0.0f, 1.0f);
    return alpha;
}

/**
 * @brief 对浮点数组执行一阶低通滤波。
 *
 * @param value 输入/输出数组，保存滤波后的值，单位与 sample 相同。
 * @param sample 当前采样数组，单位由调用方信号定义。
 * @param num 数组元素个数，单位 count，取值范围 0~UINT32_MAX。
 * @param fc 截止频率，单位 Hz，取值范围建议 > 0。
 * @param dt 采样周期，单位 s，取值范围建议 > 0。
 */
static inline void LPF_Mult_F32(float *value, float *sample, uint32_t num, float fc, float dt)
{
    float rc    = 1.0f / (_2PI * fc);
    float alpha = CONSTRAIN((dt / (dt + rc)), 0.0f, 1.0f);
    for (uint32_t i = 0; i < num; i++)
    {
        value[i] += (sample[i] - value[i]) * alpha;
    }
}

/**
 * @brief 对单个浮点值执行一阶低通滤波。
 *
 * @param value 输入/输出值指针，保存滤波后的值，单位与 sample 相同。
 * @param sample 当前采样值，单位由调用方信号定义。
 * @param fc 截止频率，单位 Hz，取值范围建议 > 0。
 * @param dt 采样周期，单位 s，取值范围建议 > 0。
 */
static inline void LPF_F32(float *value, float sample, float fc, float dt)
{
    float rc     = 1.0f / (_2PI * fc);
    float alpha  = CONSTRAIN((dt / (dt + rc)), 0.0f, 1.0f);
    *value      += (sample - *value) * alpha;
}

/**
 * @brief 对浮点数组执行一阶低通的逆滤波估计。
 *
 * @param value 输出数组，保存逆滤波后的估计值，单位与 sample 相同。
 * @param sample 当前采样数组，单位由调用方信号定义。
 * @param sample_last 上一次采样数组，单位与 sample 相同。
 * @param num 数组元素个数，单位 count，取值范围 0~UINT32_MAX。
 * @param fc 截止频率，单位 Hz；<=0 时直接输出 sample。
 * @param dt 采样周期，单位 s；<=0 时直接输出 sample。
 */
static inline void Inverse_LPF_Mult_F32(float *value, float *sample, float *sample_last, uint32_t num, float fc, float dt)
{
    if (value == NULL || sample == NULL || sample_last == NULL) return;

    if (fc <= 0.0f || dt <= 0.0f)
    {
        for (uint32_t i = 0; i < num; i++)
        {
            value[i] = sample[i];
        }
        return;
    }

    float rc = 1.0f / (_2PI * fc);
    for (uint32_t i = 0; i < num; i++)
    {
        value[i] = ((rc + dt) * sample[i] - rc * sample_last[i]) / dt;
    }
}

/**
 * @brief 对单个浮点值执行一阶低通的逆滤波估计。
 *
 * @param value 输出值指针，保存逆滤波后的估计值，单位与 sample 相同。
 * @param sample 当前采样值，单位由调用方信号定义。
 * @param sample_last 上一次采样值，单位与 sample 相同。
 * @param fc 截止频率，单位 Hz，取值范围建议 > 0；<=0 时直接输出 sample。
 * @param dt 采样周期，单位 s，取值范围建议 > 0；<=0 时直接输出 sample。
 */
static inline void Inverse_LPF_F32(float *value, float sample, float sample_last, float fc, float dt)
{
    if (value == NULL) return;

    if (fc <= 0.0f || dt <= 0.0f)
    {
        *value = sample;
        return;
    }

    float rc = 1.0f / (_2PI * fc);
    *value   = ((rc + dt) * sample - rc * sample_last) / dt;
}

/**
 * @brief 采用Kaiser窗设计FIR滤波器，阶数由过渡带宽和目标阻带衰减自动计算。
 *
 * @param f 滤波器对象指针。
 * @param type 滤波器类型。
 * @param f1 归一化截止频率；带通/带阻时为下边界。
 * @param f2 带通/带阻的归一化上边界，低通/高通忽略。
 * @param transition_width 归一化过渡带宽。
 * @param stopband_attenuation_db 目标阻带衰减，单位dB。
 * @return Filter_Status_t 初始化状态；无法在64抽头内满足指标时返回Order_Unsupported。
 */
Filter_Status_t Design_FIR(Filter_t *f, Filter_Type_t type, float f1, float f2, float transition_width, float stopband_attenuation_db);

/**
 * @brief 使用最多两个二阶节设计1~4阶IIR滤波器并清空历史状态。
 *
 * @param f 滤波器对象指针。
 * @param order 目标阶数，低通/高通支持1~4；带通/带阻支持2或4。
 * @param type 滤波器类型，支持低通、高通、带通、带阻。
 * @param f1 归一化频率；低通/高通为截止频率，带通/带阻为下边界。
 * @param f2 带通/带阻的归一化上边界，低通/高通忽略。
 * @return Filter_Status_t 初始化状态；参数无效时返回对应错误且输入直通。
 */
Filter_Status_t Design_IIR(Filter_t *f, int order, Filter_Type_t type, float f1, float f2);

/**
 * @brief 根据配置和采样频率初始化滤波器并校验所有生效参数。
 *
 * @param f 滤波器对象指针。
 * @param config 滤波器配置，字段生效规则见Filter_Config_t注释。
 * @param sample_freq 实际调用Process_Filter的采样频率，单位Hz。
 * @return Filter_Status_t 初始化状态；失败时滤波器保持输入直通。
 */
Filter_Status_t Init_Filter(Filter_t *f, const Filter_Config_t *config, float sample_freq);

/**
 * @brief 将滤波器状态预置为指定的稳态输入值。
 *
 * @param f 已初始化的滤波器对象指针。
 * @param value 预置输入值。
 */
void Prime_Filter(Filter_t *f, float value);

/**
 * @brief 输入一个采样点并计算滤波输出。
 *
 * @param f 已初始化的滤波器对象指针；为 NULL 时直接返回 input。
 * @param input 当前输入采样值，单位由调用方信号定义，取值范围由信号量程决定。
 * @return float 滤波后的输出值，单位与 input 相同。
 */
float Process_Filter(Filter_t *f, float input);
