#include "config_storage.h"
#include "config_flash.h"
#include "global.h"

#include <stddef.h>
#include <string.h>

#define CONFIG_MAGIC UINT32_C(0x4A455343)
#define CONFIG_COMMITTED UINT32_C(0x43464731)

typedef struct
{
    uint32_t magic;
    uint32_t version;
    uint32_t size;
    uint32_t sequence;
    uint32_t crc;
    uint32_t committed; /* Written last, after header/payload read-back. */
} Config_Header_t;

_Static_assert(sizeof(Config_Header_t) == 24U, "configuration header layout changed");
_Static_assert(sizeof(Global_Configs_t) + sizeof(Config_Header_t) <= 128U * 1024U,
               "configuration exceeds one reserved sector");

static Config_Storage_Status_t last_status = CONFIG_STORAGE_EMPTY;
static bool busy;
/* Avoid placing a potentially large configuration snapshot on the ISR stack. */
static Global_Configs_t snapshot;

static uint32_t crc_update(uint32_t crc, const void *data, size_t size)
{
    const uint8_t *bytes = data;
    for (size_t i = 0; i < size; ++i)
    {
        crc ^= bytes[i];
        for (unsigned int bit = 0; bit < 8U; ++bit)
            crc = (crc >> 1) ^ ((crc & 1U) ? UINT32_C(0xEDB88320) : 0U);
    }
    return crc;
}

static uint32_t record_crc(const Config_Header_t *header, const void *payload)
{
    uint32_t crc = crc_update(UINT32_MAX, header, offsetof(Config_Header_t, crc));
    return ~crc_update(crc, payload, sizeof(Global_Configs_t));
}

static Config_Storage_Status_t inspect(unsigned int slot, Config_Header_t *header)
{
    const uint8_t *data = config_flash_data(slot);
    if (data == NULL || config_flash_capacity() < sizeof(*header) + sizeof(Global_Configs_t))
        return CONFIG_STORAGE_FLASH_ERROR;

    memcpy(header, data, sizeof(*header));
    Config_Header_t erased;
    memset(&erased, 0xFF, sizeof(erased));
    if (memcmp(header, &erased, sizeof(erased)) == 0)
        return CONFIG_STORAGE_EMPTY;
    if (header->magic != CONFIG_MAGIC || header->committed != CONFIG_COMMITTED)
        return CONFIG_STORAGE_CORRUPT;
    if (header->version != CONFIG_STORAGE_VERSION || header->size != sizeof(Global_Configs_t))
        return CONFIG_STORAGE_INCOMPATIBLE;
    return record_crc(header, data + sizeof(*header)) == header->crc ?
        CONFIG_STORAGE_OK : CONFIG_STORAGE_CORRUPT;
}

static int newest(Config_Header_t headers[2], Config_Storage_Status_t statuses[2])
{
    statuses[0] = inspect(0U, &headers[0]);
    statuses[1] = inspect(1U, &headers[1]);
    if (statuses[0] != CONFIG_STORAGE_OK)
        return statuses[1] == CONFIG_STORAGE_OK ? 1 : -1;
    if (statuses[1] != CONFIG_STORAGE_OK)
        return 0;

    /* Modular comparison also handles UINT32_MAX -> 0 sequence wrap. */
    uint32_t delta = headers[1].sequence - headers[0].sequence;
    return delta != 0U && delta < UINT32_C(0x80000000) ? 1 : 0;
}

static bool motors_stopped(bool loading)
{
    for (unsigned int i = 0; i < BOARD_SUPPORTED_MOTOR_NUMBER; ++i)
    {
        const volatile Motor_Status_t *motor = &global_status.motor_status[i];
        if (motor->is_enabled)
            return false;
        if (loading ? motor->motor_state != MS_Init :
            (motor->motor_state != MS_IDLE && motor->motor_state != MS_Err))
            return false;
    }
    return true;
}

Config_Storage_Status_t config_storage_load(void)
{
    if (!config_flash_in_thread() || busy || !motors_stopped(true))
        return last_status = CONFIG_STORAGE_BUSY;

    busy = true;
    Config_Header_t headers[2];
    Config_Storage_Status_t statuses[2];
    int active = newest(headers, statuses);
    if (active >= 0)
    {
        memcpy(&global_config, config_flash_data((unsigned int)active) + sizeof(Config_Header_t), sizeof(global_config));
        last_status = CONFIG_STORAGE_OK;
    }
    else if (statuses[0] == CONFIG_STORAGE_FLASH_ERROR || statuses[1] == CONFIG_STORAGE_FLASH_ERROR)
        last_status = CONFIG_STORAGE_FLASH_ERROR;
    else if (statuses[0] == CONFIG_STORAGE_INCOMPATIBLE || statuses[1] == CONFIG_STORAGE_INCOMPATIBLE)
        last_status = CONFIG_STORAGE_INCOMPATIBLE;
    else if (statuses[0] == CONFIG_STORAGE_EMPTY && statuses[1] == CONFIG_STORAGE_EMPTY)
        last_status = CONFIG_STORAGE_EMPTY;
    else
        last_status = CONFIG_STORAGE_CORRUPT;
    busy = false;
    return last_status;
}

static Config_Storage_Status_t write_record(unsigned int slot, const Config_Header_t *header)
{
    if (!config_flash_erase(slot))
        return CONFIG_STORAGE_FLASH_ERROR;

    for (size_t offset = 0; offset < offsetof(Config_Header_t, committed); offset += sizeof(uint32_t))
    {
        uint32_t word;
        memcpy(&word, (const uint8_t *)header + offset, sizeof(word));
        if (!config_flash_program(slot, offset, word))
            return CONFIG_STORAGE_FLASH_ERROR;
    }
    for (size_t offset = 0; offset < sizeof(snapshot); offset += sizeof(uint32_t))
    {
        uint32_t word = UINT32_MAX;
        size_t remaining = sizeof(snapshot) - offset;
        memcpy(&word, (const uint8_t *)&snapshot + offset, remaining < sizeof(word) ? remaining : sizeof(word));
        if (!config_flash_program(slot, sizeof(*header) + offset, word))
            return CONFIG_STORAGE_FLASH_ERROR;
    }

    Config_Header_t readback;
    const uint8_t *data = config_flash_data(slot);
    memcpy(&readback, data, sizeof(readback));
    if (memcmp(&readback, header, offsetof(Config_Header_t, committed)) != 0 ||
        memcmp(data + sizeof(*header), &snapshot, sizeof(snapshot)) != 0 ||
        record_crc(&readback, data + sizeof(*header)) != header->crc)
        return CONFIG_STORAGE_VERIFY_ERROR;

    if (!config_flash_program(slot, offsetof(Config_Header_t, committed), CONFIG_COMMITTED))
        return CONFIG_STORAGE_FLASH_ERROR;
    return inspect(slot, &readback) == CONFIG_STORAGE_OK ? CONFIG_STORAGE_OK : CONFIG_STORAGE_VERIFY_ERROR;
}

Config_Storage_Status_t config_storage_save(void)
{
    if (!config_flash_in_thread() || busy || !motors_stopped(false))
        return last_status = CONFIG_STORAGE_BUSY;

    busy = true;
    memcpy(&snapshot, &global_config, sizeof(snapshot));
    Config_Header_t headers[2];
    Config_Storage_Status_t statuses[2];
    int active = newest(headers, statuses);
    if (active >= 0 && memcmp(config_flash_data((unsigned int)active) + sizeof(Config_Header_t), &snapshot, sizeof(snapshot)) == 0)
        last_status = CONFIG_STORAGE_OK;
    else if (statuses[0] == CONFIG_STORAGE_FLASH_ERROR || statuses[1] == CONFIG_STORAGE_FLASH_ERROR)
        last_status = CONFIG_STORAGE_FLASH_ERROR;
    else if (!config_flash_begin())
    {
        (void)config_flash_end();
        last_status = CONFIG_STORAGE_FLASH_ERROR;
    }
    else
    {
        unsigned int target = active == 0 ? 1U : 0U;
        Config_Header_t header = {
            .magic = CONFIG_MAGIC,
            .version = CONFIG_STORAGE_VERSION,
            .size = sizeof(snapshot),
            .sequence = active < 0 ? 0U : headers[active].sequence + 1U,
            .committed = UINT32_MAX
        };
        header.crc = record_crc(&header, &snapshot);
        last_status = write_record(target, &header);
        if (!config_flash_end())
            last_status = CONFIG_STORAGE_FLASH_ERROR;
    }
    busy = false;
    return last_status;
}

Config_Storage_Status_t config_storage_last_status(void)
{
    return last_status;
}
