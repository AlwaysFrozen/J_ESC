/**
 * @file Sine_Generator.c
 * @author jinchl (jcl709905683@gmail.com)
 * @brief Sine wave generator with runtime adjustable parameters
 * @version 0.1
 * @date 2026-07-01
 *
 * @copyright Copyright (c) 2026
 *
 */


#include "sine_generator.h"

static float Sine_Generator_Normalize_Phase(float phase)
{
    phase = fmodf(phase, _2PI);
    if (phase < 0.0f)
    {
        phase += _2PI;
    }

    return phase;
}

void Sine_Generator_Init(Sine_Generator_t *pGenerator, float frequency, float amplitude, float offset, float dt)
{
    if (pGenerator == NULL)
    {
        return;
    }

    pGenerator->frequency = frequency;
    pGenerator->amplitude = amplitude;
    pGenerator->offset    = offset;
    pGenerator->phase     = 0.0f;
    pGenerator->dt        = dt;
    pGenerator->out       = offset;
}

void Sine_Generator_Set_Params(Sine_Generator_t *pGenerator, float frequency, float amplitude, float offset)
{
    if (pGenerator == NULL)
    {
        return;
    }

    pGenerator->frequency = frequency;
    pGenerator->amplitude = amplitude;
    pGenerator->offset    = offset;
}

void Sine_Generator_Set_Frequency(Sine_Generator_t *pGenerator, float frequency)
{
    if (pGenerator == NULL)
    {
        return;
    }

    pGenerator->frequency = frequency;
}

void Sine_Generator_Set_Amplitude(Sine_Generator_t *pGenerator, float amplitude)
{
    if (pGenerator == NULL)
    {
        return;
    }

    pGenerator->amplitude = amplitude;
}

void Sine_Generator_Set_Offset(Sine_Generator_t *pGenerator, float offset)
{
    if (pGenerator == NULL)
    {
        return;
    }

    pGenerator->offset = offset;
}

void Sine_Generator_Set_Dt(Sine_Generator_t *pGenerator, float dt)
{
    if (pGenerator == NULL)
    {
        return;
    }

    pGenerator->dt = dt;
}

void Sine_Generator_Set_Phase(Sine_Generator_t *pGenerator, float phase)
{
    if (pGenerator == NULL)
    {
        return;
    }

    pGenerator->phase = Sine_Generator_Normalize_Phase(phase);
}

float Sine_Generator_Update(Sine_Generator_t *pGenerator)
{
    if (pGenerator == NULL)
    {
        return 0.0f;
    }

    pGenerator->out = pGenerator->offset + pGenerator->amplitude * sinf(pGenerator->phase);

    if (pGenerator->dt > 0.0f)
    {
        pGenerator->phase += _2PI * pGenerator->frequency * pGenerator->dt;
        pGenerator->phase  = Sine_Generator_Normalize_Phase(pGenerator->phase);
    }

    return pGenerator->out;
}
