/**
 * @file sine_generator.h
 * @author jinchl (jcl709905683@gmail.com)
 * @brief Sine wave generator with runtime adjustable parameters
 * @version 0.1
 * @date 2026-07-01
 *
 * @copyright Copyright (c) 2026
 *
 */


#pragma once
#include "utils.h"

typedef struct
{
    float frequency; // Hz
    float amplitude;
    float offset;
    float phase; // rad, limited to 0~2pi
    float dt;    // s
    float out;
} Sine_Generator_t;

void Sine_Generator_Init(Sine_Generator_t *pGenerator, float frequency, float amplitude, float offset, float dt);
void Sine_Generator_Set_Params(Sine_Generator_t *pGenerator, float frequency, float amplitude, float offset);
void Sine_Generator_Set_Frequency(Sine_Generator_t *pGenerator, float frequency);
void Sine_Generator_Set_Amplitude(Sine_Generator_t *pGenerator, float amplitude);
void Sine_Generator_Set_Offset(Sine_Generator_t *pGenerator, float offset);
void Sine_Generator_Set_Dt(Sine_Generator_t *pGenerator, float dt);
void Sine_Generator_Set_Phase(Sine_Generator_t *pGenerator, float phase);
float Sine_Generator_Update(Sine_Generator_t *pGenerator);
