#include "config_storage.h"
#include "config_flash.h"
#include "global.h"
#include <string.h>

#define CHECK(condition) do { if (!(condition)) return __LINE__; } while (0)
#define CAPACITY 256U
#define HEADER_SIZE 24U

Global_Configs_t global_config;
volatile Global_Status_t global_status;
static uint8_t flash[2][CAPACITY];
static uint8_t backup[2][CAPACITY];
static uint8_t two_valid[2][CAPACITY];
static bool thread_context = true, unlocked, erase_fail, unlock_fail, lock_fail, corrupt_write;
static int words_left = -1;
static unsigned int erases, programs;

void *memcpy(void *dst, const void *src, size_t size)
{
    uint8_t *d = dst;
    const uint8_t *s = src;
    while (size--) *d++ = *s++;
    return dst;
}
void *memset(void *dst, int value, size_t size)
{
    uint8_t *d = dst;
    while (size--) *d++ = (uint8_t)value;
    return dst;
}
int memcmp(const void *a, const void *b, size_t size)
{
    const uint8_t *x = a, *y = b;
    while (size--) { if (*x != *y) return *x - *y; ++x; ++y; }
    return 0;
}

const uint8_t *config_flash_data(unsigned int slot) { return slot < 2U ? flash[slot] : NULL; }
size_t config_flash_capacity(void) { return CAPACITY; }
bool config_flash_in_thread(void) { return thread_context; }
bool config_flash_begin(void) { unlocked = !unlock_fail; return unlocked; }
bool config_flash_end(void) { unlocked = false; return !lock_fail; }
bool config_flash_erase(unsigned int slot)
{
    if (!unlocked || erase_fail || slot > 1U) return false;
    ++erases;
    memset(flash[slot], 0xFF, CAPACITY);
    return true;
}
bool config_flash_program(unsigned int slot, size_t offset, uint32_t word)
{
    if (!unlocked || slot > 1U || offset + 4U > CAPACITY || (offset & 3U) || words_left == 0)
        return false;
    if (words_left > 0) --words_left;
    ++programs;
    const uint8_t *bytes = (const uint8_t *)&word;
    for (unsigned int i = 0; i < 4U; ++i)
    {
        if ((flash[slot][offset + i] & bytes[i]) != bytes[i]) return false;
        flash[slot][offset + i] &= bytes[i];
    }
    if (corrupt_write && offset == HEADER_SIZE) flash[slot][offset] ^= 1U;
    return true;
}

static void boot_state(void)
{
    memset((void *)&global_status, 0, sizeof(global_status));
}
static void idle_state(void)
{
    for (unsigned int i = 0; i < BOARD_SUPPORTED_MOTOR_NUMBER; ++i)
        global_status.motor_status[i].motor_state = MS_IDLE;
}
static int load_value(uint8_t value)
{
    boot_state();
    memset(&global_config, 0xCC, sizeof(global_config));
    if (config_storage_load() != CONFIG_STORAGE_OK) return 1;
    for (size_t i = 0; i < sizeof(global_config); ++i)
        if (global_config.bytes[i] != value) return 1;
    return 0;
}

int main(void)
{
    memset(flash, 0xFF, sizeof(flash));
    memset(&global_config, 0x55, sizeof(global_config));
    CHECK(config_storage_load() == CONFIG_STORAGE_EMPTY);
    CHECK(global_config.bytes[0] == 0x55);
    CHECK(config_storage_last_status() == CONFIG_STORAGE_EMPTY);
    CHECK(config_storage_save() == CONFIG_STORAGE_BUSY); /* No save during initialization. */
    idle_state();
    thread_context = false;
    CHECK(config_storage_save() == CONFIG_STORAGE_BUSY);
    CHECK(config_storage_load() == CONFIG_STORAGE_BUSY);
    thread_context = true;
    global_status.motor_status[1].is_enabled = 1;
    CHECK(config_storage_save() == CONFIG_STORAGE_BUSY);
    global_status.motor_status[1].is_enabled = 0;
    global_status.motor_status[1].motor_state = MS_Calibrate;
    CHECK(config_storage_save() == CONFIG_STORAGE_BUSY);
    global_status.motor_status[1].motor_state = MS_Running;
    CHECK(config_storage_save() == CONFIG_STORAGE_BUSY);
    idle_state();
    CHECK(erases == 0 && programs == 0);
    CHECK(config_storage_save() == CONFIG_STORAGE_OK);
    CHECK(!unlocked && erases == 1U);
    unsigned int saved_programs = programs;
    CHECK(config_storage_save() == CONFIG_STORAGE_OK);
    CHECK(erases == 1U && programs == saved_programs);
    CHECK(config_storage_load() == CONFIG_STORAGE_BUSY); /* Loading is boot-only. */
    CHECK(load_value(0x55) == 0);
    memcpy(backup, flash, sizeof(flash));

    /* Fail at every word boundary, including the final commit word. */
    const int total_words = 5 + (sizeof(global_config) + 3U) / 4U + 1;
    for (int cut = 0; cut < total_words; ++cut)
    {
        memcpy(flash, backup, sizeof(flash));
        idle_state();
        memset(&global_config, 0xAA, sizeof(global_config));
        words_left = cut;
        CHECK(config_storage_save() == CONFIG_STORAGE_FLASH_ERROR);
        CHECK(!unlocked);
        words_left = -1;
        CHECK(load_value(0x55) == 0);
    }
    idle_state();
    memset(&global_config, 0xAA, sizeof(global_config));
    corrupt_write = true;
    CHECK(config_storage_save() == CONFIG_STORAGE_VERIFY_ERROR);
    corrupt_write = false;
    CHECK(load_value(0x55) == 0);
    idle_state();
    memset(&global_config, 0xAA, sizeof(global_config));
    erase_fail = true;
    CHECK(config_storage_save() == CONFIG_STORAGE_FLASH_ERROR);
    erase_fail = false;
    unlock_fail = true;
    CHECK(config_storage_save() == CONFIG_STORAGE_FLASH_ERROR);
    unlock_fail = false;
    CHECK(!unlocked);
    CHECK(config_storage_save() == CONFIG_STORAGE_OK);
    CHECK(load_value(0xAA) == 0);

    /* With both sectors valid, the next save must target A and preserve B. */
    memcpy(two_valid, flash, sizeof(flash));
    idle_state();
    memset(&global_config, 0x66, sizeof(global_config));
    words_left = 2;
    CHECK(config_storage_save() == CONFIG_STORAGE_FLASH_ERROR);
    words_left = -1;
    CHECK(load_value(0xAA) == 0);
    idle_state();
    memset(&global_config, 0x66, sizeof(global_config));
    CHECK(config_storage_save() == CONFIG_STORAGE_OK);
    CHECK(load_value(0x66) == 0);
    memcpy(flash, two_valid, sizeof(flash));

    /* New record corruption must fall back to the older sector. */
    flash[1][HEADER_SIZE] ^= 1U;
    CHECK(load_value(0x55) == 0);
    memcpy(flash, backup, sizeof(flash));
    flash[0][12] ^= 1U; /* Sequence is covered by CRC. */
    boot_state();
    memset(&global_config, 0x33, sizeof(global_config));
    CHECK(config_storage_load() == CONFIG_STORAGE_CORRUPT);
    CHECK(global_config.bytes[0] == 0x33);
    memcpy(flash, backup, sizeof(flash));
    flash[0][4] ^= 1U;
    CHECK(config_storage_load() == CONFIG_STORAGE_INCOMPATIBLE);
    CHECK(global_config.bytes[0] == 0x33);
    memcpy(flash, backup, sizeof(flash));
    flash[0][8] ^= 1U;
    CHECK(config_storage_load() == CONFIG_STORAGE_INCOMPATIBLE);

    /* Sequence wrap: make a valid older record with sequence UINT32_MAX. */
    memcpy(flash, backup, sizeof(flash));
    memset(flash[0] + 12U, 0xFF, 4U);
    uint32_t crc = UINT32_MAX;
    for (size_t i = 0; i < 16U + sizeof(global_config); ++i)
    {
        crc ^= i < 16U ? flash[0][i] : flash[0][HEADER_SIZE + i - 16U];
        for (int bit = 0; bit < 8; ++bit) crc = (crc >> 1) ^ ((crc & 1U) ? UINT32_C(0xEDB88320) : 0U);
    }
    crc = ~crc;
    memcpy(flash[0] + 16U, &crc, sizeof(crc));
    idle_state();
    memset(&global_config, 0x77, sizeof(global_config));
    lock_fail = true;
    CHECK(config_storage_save() == CONFIG_STORAGE_FLASH_ERROR);
    lock_fail = false;
    CHECK(!unlocked);
    CHECK(load_value(0x77) == 0);
    CHECK(flash[1][12] == 0U); /* Wrapped record wins over UINT32_MAX. */
    return 0;
}
