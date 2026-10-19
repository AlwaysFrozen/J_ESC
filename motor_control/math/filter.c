/**
 * @file filter.c
 * @author jinchl (jcl709905683@gmail.com)
 * @brief FIR/IIR滤波器设计和运行实现。
 * @version 0.2
 * @date 2026-09-18
 *
 * @copyright Copyright (c) 2026
 *
 */

#include "filter.h"

static bool filter_type_is_valid(Filter_Type_t type)
{
    return type >= Filter_Type_Lowpass && type <= Filter_Type_Bandstop;
}

static float sinc(float x)
{
    if (fabsf(x) < FILTER_EPS) return 1.0f;
    return sinf(_PI * x) / (_PI * x);
}

static void set_passthrough(Filter_t *f, Filter_Status_t status)
{
    if (f == NULL) return;

    memset(f, 0, sizeof(*f));
    f->filter_struct = Filter_Struct_None;
    f->status        = status;
}

static float bessel_i0(float x)
{
    float y    = x * x * 0.25f;
    float sum  = 1.0f;
    float term = 1.0f;

    for (int k = 1; k <= 20; k++)
    {
        term *= y / ((float)k * (float)k);
        sum  += term;
        if (term < sum * 1.0e-7f) break;
    }

    return sum;
}

static float kaiser_beta(float attenuation_db)
{
    if (attenuation_db > 50.0f)
    {
        return 0.1102f * (attenuation_db - 8.7f);
    }
    if (attenuation_db >= 21.0f)
    {
        float value = attenuation_db - 21.0f;
        return 0.5842f * powf(value, 0.4f) + 0.07886f * value;
    }
    return 0.0f;
}

static Filter_Status_t calculate_fir_order(Filter_Type_t type, float transition_width, float attenuation_db, uint16_t *order)
{
    float delta_omega  = _2PI * transition_width;
    int required_order = (int)ceilf((attenuation_db - 8.0f) / (2.285f * delta_omega));

    if (required_order < 1) required_order = 1;

    // 对称高通和带阻需要奇数抽头（偶数阶），否则Nyquist频率处必然为零。
    if ((type == Filter_Type_Highpass || type == Filter_Type_Bandstop) && (required_order & 1))
    {
        required_order++;
    }

    if (required_order > (int)FIR_MAX_ORDER)
    {
        return Filter_Status_Order_Unsupported;
    }

    *order = (uint16_t)required_order;
    return Filter_Status_OK;
}

static bool normalize_fir(FIR_Filter_t *fir, Filter_Type_t type, float f1, float f2)
{
    float gain = 0.0f;

    if (type == Filter_Type_Highpass)
    {
        for (uint16_t i = 0; i < fir->taps; i++) gain += fir->b[i] * ((i & 1U) ? -1.0f : 1.0f);
    }
    else if (type == Filter_Type_Bandpass)
    {
        float center = 0.5f * (f1 + f2);
        float real   = 0.0f;
        float imag   = 0.0f;

        for (uint16_t i = 0; i < fir->taps; i++)
        {
            float phase  = _2PI * center * (float)i;
            real        += fir->b[i] * cosf(phase);
            imag        -= fir->b[i] * sinf(phase);
        }
        gain = sqrtf(real * real + imag * imag);
    }
    else
    {
        for (uint16_t i = 0; i < fir->taps; i++) gain += fir->b[i];
    }

    if (!isfinite(gain) || fabsf(gain) < FILTER_EPS) return false;

    for (uint16_t i = 0; i < fir->taps; i++) fir->b[i] /= gain;
    return true;
}

Filter_Status_t Design_FIR(Filter_t *f, Filter_Type_t type, float f1, float f2, float transition_width, float stopband_attenuation_db)
{
    if (f == NULL) return Filter_Status_Invalid_Argument;

    set_passthrough(f, Filter_Status_Invalid_Argument);

    if (!filter_type_is_valid(type))
    {
        f->status = Filter_Status_Invalid_Type;
        return f->status;
    }
    if (!isfinite(f1) || f1 <= FILTER_FREQ_MIN || f1 >= FILTER_FREQ_MAX)
    {
        f->status = Filter_Status_Invalid_Frequency;
        return f->status;
    }
    if (!isfinite(transition_width) || transition_width <= 0.0f)
    {
        f->status = Filter_Status_Invalid_Width;
        return f->status;
    }
    if (!isfinite(stopband_attenuation_db) || stopband_attenuation_db < FILTER_DEPTH_MIN_DB || stopband_attenuation_db > FILTER_DEPTH_MAX_DB)
    {
        f->status = Filter_Status_Invalid_Depth;
        return f->status;
    }

    float half_transition = transition_width * 0.5f;
    if (type == Filter_Type_Bandpass || type == Filter_Type_Bandstop)
    {
        if (!isfinite(f2) || f2 <= f1 || f2 >= FILTER_FREQ_MAX)
        {
            f->status = Filter_Status_Invalid_Frequency;
            return f->status;
        }
        if (f1 - half_transition <= 0.0f || f2 + half_transition >= 0.5f)
        {
            f->status = Filter_Status_Invalid_Width;
            return f->status;
        }
    }
    else if (f1 - half_transition <= 0.0f || f1 + half_transition >= 0.5f)
    {
        f->status = Filter_Status_Invalid_Width;
        return f->status;
    }

    uint16_t order         = 0;
    Filter_Status_t status = calculate_fir_order(type, transition_width, stopband_attenuation_db, &order);
    if (status != Filter_Status_OK)
    {
        f->status = status;
        return status;
    }

    memset(f, 0, sizeof(*f));
    f->filter_struct  = Filter_Struct_FIR;
    f->status         = Filter_Status_OK;
    FIR_Filter_t *fir = &f->state.fir;
    fir->taps         = order + 1U;
    float beta        = kaiser_beta(stopband_attenuation_db);
    float window_norm = bessel_i0(beta);
    float midpoint    = (float)order * 0.5f;

    for (uint16_t n = 0; n < fir->taps; n++)
    {
        float m = (float)n - midpoint;
        float h = 0.0f;

        switch (type)
        {
        case Filter_Type_Lowpass:
            h = 2.0f * f1 * sinc(2.0f * f1 * m);
            break;
        case Filter_Type_Highpass:
            h = sinc(m) - 2.0f * f1 * sinc(2.0f * f1 * m);
            break;
        case Filter_Type_Bandpass:
            h = 2.0f * f2 * sinc(2.0f * f2 * m) - 2.0f * f1 * sinc(2.0f * f1 * m);
            break;
        case Filter_Type_Bandstop:
            h = sinc(m) - (2.0f * f2 * sinc(2.0f * f2 * m) - 2.0f * f1 * sinc(2.0f * f1 * m));
            break;
        default:
            break;
        }

        float ratio  = order > 0U ? (2.0f * (float)n / (float)order - 1.0f) : 0.0f;
        float inside = MAX(0.0f, 1.0f - ratio * ratio);
        float window = bessel_i0(beta * sqrtf(inside)) / window_norm;
        fir->b[n]    = h * window;
    }

    if (!normalize_fir(fir, type, f1, f2))
    {
        set_passthrough(f, Filter_Status_Invalid_Argument);
    }
    return f->status;
}

static void clear_iir_section(IIR_Section_t *section)
{
    memset(section, 0, sizeof(*section));
}

static void set_iir_first_order(IIR_Section_t *section, Filter_Type_t type, float fc)
{
    float k    = tanf(_PI * fc);
    float norm = 1.0f / (1.0f + k);

    clear_iir_section(section);
    section->order = 1U;
    section->a[0]  = 1.0f;
    section->a[1]  = (k - 1.0f) * norm;

    if (type == Filter_Type_Highpass)
    {
        section->b[0] = norm;
        section->b[1] = -norm;
    }
    else
    {
        section->b[0] = k * norm;
        section->b[1] = section->b[0];
    }
}

static void set_iir_biquad(IIR_Section_t *section, Filter_Type_t type, float fc, float q)
{
    float w0    = _2PI * fc;
    float sn    = sinf(w0);
    float cs    = cosf(w0);
    float alpha = sn / (2.0f * q);
    float a0    = 1.0f + alpha;
    float b0    = 0.0f;
    float b1    = 0.0f;
    float b2    = 0.0f;
    float a1    = -2.0f * cs;
    float a2    = 1.0f - alpha;

    switch (type)
    {
    case Filter_Type_Highpass:
        b0 = (1.0f + cs) * 0.5f;
        b1 = -(1.0f + cs);
        b2 = b0;
        break;
    case Filter_Type_Bandpass:
        b0 = alpha;
        b2 = -alpha;
        break;
    case Filter_Type_Bandstop:
        b0 = 1.0f;
        b1 = -2.0f * cs;
        b2 = 1.0f;
        break;
    case Filter_Type_Lowpass:
    default:
        b0 = (1.0f - cs) * 0.5f;
        b1 = 1.0f - cs;
        b2 = b0;
        break;
    }

    clear_iir_section(section);
    section->order = 2U;
    section->a[0]  = 1.0f;
    section->a[1]  = a1 / a0;
    section->a[2]  = a2 / a0;
    section->b[0]  = b0 / a0;
    section->b[1]  = b1 / a0;
    section->b[2]  = b2 / a0;
}

Filter_Status_t Design_IIR(Filter_t *f, int order, Filter_Type_t type, float f1, float f2)
{
    if (f == NULL) return Filter_Status_Invalid_Argument;

    set_passthrough(f, Filter_Status_Invalid_Argument);

    if (!filter_type_is_valid(type))
    {
        f->status = Filter_Status_Invalid_Type;
        return f->status;
    }
    if (!isfinite(f1) || f1 <= FILTER_FREQ_MIN || f1 >= FILTER_FREQ_MAX)
    {
        f->status = Filter_Status_Invalid_Frequency;
        return f->status;
    }
    if (order < 1 || order > (int)IIR_MAX_ORDER)
    {
        f->status = Filter_Status_Order_Unsupported;
        return f->status;
    }
    if (type == Filter_Type_Bandpass || type == Filter_Type_Bandstop)
    {
        if (order != 2 && order != 4)
        {
            f->status = Filter_Status_Order_Unsupported;
            return f->status;
        }
        if (!isfinite(f2) || f2 <= f1 || f2 >= FILTER_FREQ_MAX)
        {
            f->status = Filter_Status_Invalid_Frequency;
            return f->status;
        }
    }

    float band_fc = 0.0f;
    float band_q  = 0.0f;
    if (type == Filter_Type_Bandpass || type == Filter_Type_Bandstop)
    {
        // f1/f2由“中心频率±半带宽”得到，算术平均值即配置的中心频率。
        band_fc = 0.5f * (f1 + f2);
        band_q  = band_fc / (f2 - f1);
        if (!isfinite(band_q) || band_q <= 0.0f)
        {
            f->status = Filter_Status_Invalid_Width;
            return f->status;
        }
        band_q = MAX(band_q, 0.1f);
    }

    memset(f, 0, sizeof(*f));
    f->filter_struct  = Filter_Struct_IIR;
    f->status         = Filter_Status_OK;
    IIR_Filter_t *iir = &f->state.iir;
    iir->order        = (uint8_t)order;

    if (type == Filter_Type_Bandpass || type == Filter_Type_Bandstop)
    {
        if (order == 2)
        {
            iir->section_count = 1U;
            set_iir_biquad(&iir->section[0], type, band_fc, band_q);
        }
        else
        {
            // 单节边界增益调到约-1.5dB，使两节级联后的目标边界接近-3dB。
            const float bandpass_q_scale = 0.643594253f;
            const float bandstop_q_scale = 1.553773974f;
            float q_scale                = type == Filter_Type_Bandpass ? bandpass_q_scale : bandstop_q_scale;
            float section_q              = MAX(band_q * q_scale, 0.1f);

            iir->section_count = 2U;
            set_iir_biquad(&iir->section[0], type, band_fc, section_q);
            set_iir_biquad(&iir->section[1], type, band_fc, section_q);
        }
    }
    else if (order == 1)
    {
        iir->section_count = 1U;
        set_iir_first_order(&iir->section[0], type, f1);
    }
    else if (order == 2)
    {
        iir->section_count = 1U;
        set_iir_biquad(&iir->section[0], type, f1, _1_SQRT2);
    }
    else if (order == 3)
    {
        iir->section_count = 2U;
        set_iir_first_order(&iir->section[0], type, f1);
        set_iir_biquad(&iir->section[1], type, f1, 1.0f);
    }
    else
    {
        // 四阶Butterworth由Q=0.5412和Q=1.3066的两个二阶节构成。
        iir->section_count = 2U;
        set_iir_biquad(&iir->section[0], type, f1, 0.541196100f);
        set_iir_biquad(&iir->section[1], type, f1, 1.306562965f);
    }

    return f->status;
}

Filter_Status_t Init_Filter(Filter_t *f, const Filter_Config_t *config, float sample_freq)
{
    if (f == NULL) return Filter_Status_Invalid_Argument;
    if (config == NULL || !isfinite(sample_freq) || sample_freq <= 0.0f)
    {
        set_passthrough(f, Filter_Status_Invalid_Argument);
        return f->status;
    }

    if (config->filter_struct < Filter_Struct_None || config->filter_struct >= Filter_Struct_MAX)
    {
        set_passthrough(f, Filter_Status_Invalid_Struct);
        return f->status;
    }
    if (config->filter_type < Filter_Type_None || config->filter_type >= Filter_Type_MAX)
    {
        set_passthrough(f, Filter_Status_Invalid_Type);
        return f->status;
    }
    if (config->filter_struct == Filter_Struct_None || config->filter_type == Filter_Type_None)
    {
        set_passthrough(f, Filter_Status_OK);
        return f->status;
    }
    if (!isfinite(config->filter_freq) || config->filter_freq <= 0.0f || config->filter_freq >= sample_freq * 0.5f)
    {
        set_passthrough(f, Filter_Status_Invalid_Frequency);
        return f->status;
    }

    float f1 = config->filter_freq / sample_freq;
    float f2 = 0.0f;

    if (config->filter_type == Filter_Type_Bandpass || config->filter_type == Filter_Type_Bandstop)
    {
        if (!isfinite(config->filter_width) || config->filter_width <= 0.0f)
        {
            set_passthrough(f, Filter_Status_Invalid_Width);
            return f->status;
        }
        f1 = (config->filter_freq - config->filter_width * 0.5f) / sample_freq;
        f2 = (config->filter_freq + config->filter_width * 0.5f) / sample_freq;
    }

    if (config->filter_struct == Filter_Struct_FIR)
    {
        float transition_width = config->filter_width / sample_freq;
        if (config->filter_type == Filter_Type_Bandpass || config->filter_type == Filter_Type_Bandstop)
        {
            transition_width *= FIR_BAND_TRANSITION_WIDTH_RATIO;
        }
        return Design_FIR(f, config->filter_type, f1, f2, transition_width, config->filter_depth);
    }

    return Design_IIR(f, config->filter_order, config->filter_type, f1, f2);
}

void Prime_Filter(Filter_t *f, float value)
{
    if (f == NULL || f->status != Filter_Status_OK || !isfinite(value)) return;

    if (f->filter_struct == Filter_Struct_FIR)
    {
        FIR_Filter_t *fir = &f->state.fir;
        if (fir->taps == 0U || fir->taps > FIR_MAX_TAPS) return;
        for (uint16_t i = 0; i < fir->taps; i++) fir->x[i] = value;
        fir->head = 0U;
        return;
    }

    if (f->filter_struct == Filter_Struct_IIR)
    {
        IIR_Filter_t *iir = &f->state.iir;
        if (iir->section_count == 0U || iir->section_count > IIR_MAX_SECTIONS) return;

        float section_input = value;
        for (uint8_t i = 0; i < iir->section_count; i++)
        {
            IIR_Section_t *section = &iir->section[i];
            if (section->order == 0U || section->order > 2U) return;
            float denominator = 1.0f + section->a[1] + section->a[2];
            if (fabsf(denominator) < FILTER_EPS) return;
            float section_output = (section->b[0] + section->b[1] + section->b[2]) * section_input / denominator;
            section->z[1]        = section->b[2] * section_input - section->a[2] * section_output;
            section->z[0]        = section_output - section->b[0] * section_input;
            section_input        = section_output;
        }
    }
}

static float process_fir(FIR_Filter_t *fir, float input)
{
    if (fir->taps == 0U || fir->taps > FIR_MAX_TAPS) return input;

    fir->x[fir->head] = input;
    uint16_t index    = fir->head;
    float output      = 0.0f;

    for (uint16_t i = 0; i < fir->taps; i++)
    {
        output += fir->b[i] * fir->x[index];
        index   = index == 0U ? fir->taps - 1U : index - 1U;
    }

    fir->head++;
    if (fir->head >= fir->taps) fir->head = 0U;
    return output;
}

static float process_iir_section(IIR_Section_t *section, float input)
{
    if (section->order == 0U || section->order > 2U) return input;

    // Transposed Direct Form II；一阶节的b2/a2为0，因此可复用同一路径。
    float output  = section->b[0] * input + section->z[0];
    section->z[0] = section->b[1] * input - section->a[1] * output + section->z[1];
    section->z[1] = section->b[2] * input - section->a[2] * output;

    return output;
}

static float process_iir(IIR_Filter_t *iir, float input)
{
    if (iir->order == 0U || iir->order > IIR_MAX_ORDER || iir->section_count == 0U || iir->section_count > IIR_MAX_SECTIONS)
    {
        return input;
    }

    float output = input;
    for (uint8_t i = 0; i < iir->section_count; i++)
    {
        output = process_iir_section(&iir->section[i], output);
    }
    return output;
}

float Process_Filter(Filter_t *f, float input)
{
    if (f == NULL || f->status != Filter_Status_OK) return input;

    if (f->filter_struct == Filter_Struct_FIR) return process_fir(&f->state.fir, input);
    if (f->filter_struct == Filter_Struct_IIR) return process_iir(&f->state.iir, input);
    return input;
}
