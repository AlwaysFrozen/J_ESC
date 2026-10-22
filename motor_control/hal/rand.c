/**
 * @file rand.c
 * @author jinchl (jinchl@jodell.com)
 * @brief 
 * @version 0.1
 * @date 2026-10-19
 * 
 * @copyright Copyright (c) 2026
 * 
 */

#include "rand.h"

#include "board.h"

#include "main.h"
#include "rng.h"

bool rand_generate(uint32_t *rand)
{
    return HAL_OK == HAL_RNG_GenerateRandomNumber(&hrng, rand);
}
