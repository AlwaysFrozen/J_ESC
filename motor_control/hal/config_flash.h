#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/* Two independent erase sectors; addresses/size are supplied by the linker. */
const uint8_t *config_flash_data(unsigned int slot);
size_t config_flash_capacity(void);
bool config_flash_begin(void);
bool config_flash_erase(unsigned int slot);
bool config_flash_program(unsigned int slot, size_t offset, uint32_t word);
bool config_flash_end(void);
bool config_flash_in_thread(void);
