#include "config_flash.h"
#include "stm32f4xx_hal.h"

extern uint8_t __config_slot_a;
extern uint8_t __config_slot_b;
extern uint8_t __config_slot_size;

const uint8_t *config_flash_data(unsigned int slot)
{
    return slot == 0U ? &__config_slot_a : slot == 1U ? &__config_slot_b : NULL;
}

size_t config_flash_capacity(void)
{
    return (size_t)(uintptr_t)&__config_slot_size;
}

bool config_flash_in_thread(void)
{
    /* HAL timeouts require the SysTick interrupt to remain enabled. */
    return __get_IPSR() == 0U && __get_PRIMASK() == 0U &&
           __get_BASEPRI() == 0U && __get_FAULTMASK() == 0U;
}

bool config_flash_begin(void)
{
    if (HAL_FLASH_Unlock() != HAL_OK)
        return false;

    __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR |
                          FLASH_FLAG_PGAERR | FLASH_FLAG_PGPERR | FLASH_FLAG_PGSERR);
    return true;
}

bool config_flash_erase(unsigned int slot)
{
    if (slot > 1U)
        return false;

    FLASH_EraseInitTypeDef erase = {0};
    uint32_t sector_error = 0xFFFFFFFFU;
    erase.TypeErase = FLASH_TYPEERASE_SECTORS;
    erase.Sector = slot == 0U ? FLASH_SECTOR_10 : FLASH_SECTOR_11;
    erase.NbSectors = 1U;
    /* Board MCU supply is 3.3 V; x32 programming requires 2.7--3.6 V. */
    erase.VoltageRange = FLASH_VOLTAGE_RANGE_3;
    return HAL_FLASHEx_Erase(&erase, &sector_error) == HAL_OK && sector_error == 0xFFFFFFFFU;
}

bool config_flash_program(unsigned int slot, size_t offset, uint32_t word)
{
    const uint8_t *base = config_flash_data(slot);
    size_t capacity = config_flash_capacity();
    if (base == NULL || (offset & 3U) != 0U || capacity < sizeof(word) || offset > capacity - sizeof(word))
        return false;

    if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, (uint32_t)(uintptr_t)(base + offset), word) != HAL_OK)
        return false;

    /* Programming may leave stale lines in the ART data cache. */
    FLASH_FlushCaches();
    return *(volatile const uint32_t *)(base + offset) == word;
}

bool config_flash_end(void)
{
    return HAL_FLASH_Lock() == HAL_OK;
}
