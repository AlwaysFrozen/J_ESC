#pragma once

#include <stdint.h>

/* Increment whenever Global_Configs_t or a nested type changes layout/meaning,
 * including enum values and build options affecting their layout. No migration
 * is performed: incompatible records are rejected. */
#define CONFIG_STORAGE_VERSION UINT32_C(1)

typedef enum
{
    CONFIG_STORAGE_OK = 0,
    CONFIG_STORAGE_EMPTY,
    CONFIG_STORAGE_INCOMPATIBLE,
    CONFIG_STORAGE_CORRUPT,
    CONFIG_STORAGE_BUSY,
    CONFIG_STORAGE_FLASH_ERROR,
    CONFIG_STORAGE_VERIFY_ERROR
} Config_Storage_Status_t;

/* Boot only, after global_init_default(), before motor_init()/encoder_init().
 * Copies the newest valid record to global_config. Failure changes no config.
 * Motor state must still be MS_Init, with all inverters disabled. */
Config_Storage_Status_t config_storage_load(void);

/* Persist a snapshot of global_config; identical data causes no erase/write.
 * Call synchronously from the foreground with interrupts enabled, all motors
 * disabled and in MS_IDLE/MS_Err. Never call from an ISR/control callback.
 * The caller must serialize configuration changes and motor enable commands
 * until this returns. Single-bank Flash operations stall execution/interrupts;
 * this API is unsuitable for periodic saving or real-time operation. */
Config_Storage_Status_t config_storage_save(void);

/* Result of the last load/save, including the automatic boot load. */
Config_Storage_Status_t config_storage_last_status(void);
