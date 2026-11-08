#include "od_commands.h"
#include "od_generated.h"
#include "global.h"
#include "motor_api.h"
#include "config_storage.h"

static const uint8_t control_max = 4, system_max = 5;
static uint32_t flash_status, config_pending;

void od_commands_config_changed(void) { config_pending = 1; }
bool od_commands_config_pending(void) {return config_pending != 0;}

static bool stopped(void)
{
    const volatile Motor_Status_t *motor = &global_status.motor_status[0];
    return !motor->is_enabled && (motor->motor_state == MS_IDLE || motor->motor_state == MS_Err);
}

static bool finite_config(void)
{
    for (size_t i = 0; i < od_dictionary.count; ++i) {
        const OD_Entry *entry = &od_dictionary.entries[i];
        if (entry->index < 0x2000 || entry->index > 0x2006 || entry->type != OD_REAL32) continue;
        uint8_t bytes[4];
        if (od_read(&od_dictionary, entry->index, entry->subindex, bytes, sizeof(bytes)) != OD_OK) return false;
        /* IEEE binary32 exponent all ones denotes either Inf or NaN. */
        if ((bytes[3] & 0x7FU) == 0x7FU && (bytes[2] & 0x80U) != 0) return false;
    }
    return true;
}

static OD_Result execute_motor(uint8_t subindex, uint32_t value)
{
    volatile Motor_Status_t *motor = &global_status.motor_status[0];
    if (subindex == 1 && value == 0) {
        if (motor->motor_state == MS_Init || motor->motor_state == MS_Calibrate) return OD_BUSY;
        (void)motor_disable(0);
        /* Disabling never clears a latched fault. */
        if (motor->is_error) motor->motor_state = MS_Err;
        return OD_OK;
    }
    if (subindex == 3 && value >= 5 && value <= 8) return OD_UNSUPPORTED;
    if ((subindex == 1 || subindex == 2) ? value != 1 : (value < 1 || value > 8)) return OD_VALUE_INVALID;
    if (config_pending || !finite_config()) return OD_CONFIG_INVALID;
    if (motor->is_error || motor->is_braked || !motor->is_calibrated) return OD_BUSY;
    if (global_config.motor_config[0].motor_type != Motor_Type_DC &&
        (global_config.motor_config[0].motor_type != Motor_Type_BLDC_3_Phase ||
         global_config.motor_controller_config[0].motor_controller_type != Motor_Controller_Type_FOC)) return OD_UNSUPPORTED;
    if (subindex == 1) {
        if (motor->is_enabled && motor->motor_state == MS_Running) return OD_OK;
        if (!stopped()) return OD_BUSY;
        motor_reset(0);
        if (motor->is_error) { motor->motor_state = MS_Err; return OD_CONFIG_INVALID; }
        return motor_enable(0) ? OD_OK : OD_BUSY;
    }
    if (!motor->is_enabled || motor->motor_state != MS_Running || motor->motor_ctrl_mode != MCM_None) return OD_BUSY;
    /* The active operation must be disabled before starting another operation. */
    if (subindex == 3 && global_config.motor_config[0].motor_type != Motor_Type_BLDC_3_Phase) return OD_UNSUPPORTED;
    bool accepted = false;
    if (subindex == 2) accepted = motor_set_align(0);
    else switch (value) {
    case 1: accepted = motor_set_rs_identify(0); break;
    case 2: accepted = motor_set_ls_identify(0); break;
    case 3: accepted = motor_set_pp_identify(0); break;
    case 4: accepted = motor_set_flux_identify(0); break;
    default: return OD_VALUE_INVALID;
    }
    return accepted ? OD_OK : OD_BUSY;
}

static OD_Result command_write(const OD_Entry *entry, const uint8_t *data, size_t length)
{
    (void)length; /* OD validates exact type, permissions and length first. */
    if (__get_IPSR() != 0 || __get_PRIMASK() != 0) return OD_BUSY;
    uint32_t value = (uint32_t)data[0] | ((uint32_t)data[1] << 8) |
                     ((uint32_t)data[2] << 16) | ((uint32_t)data[3] << 24);
    if (entry->index == 0x1000) {
        if (value != 1) return OD_VALUE_INVALID;
        if (!stopped()) return OD_BUSY;
        if (entry->subindex == 2) {
            /* Foreground, IRQs enabled: Flash must never run under the OD lock.
             * The single command/parameter consumer serializes the entire save. */
            if (config_pending || !finite_config()) return OD_CONFIG_INVALID;
            flash_status = (uint32_t)config_storage_save();
            if (flash_status == CONFIG_STORAGE_BUSY) return OD_BUSY;
            return flash_status == CONFIG_STORAGE_OK ? OD_OK : OD_OPERATION_FAILED;
        }
    }
    uint32_t state = __get_PRIMASK();
    __disable_irq();
    OD_Result result;
    if (entry->index == 0) result = execute_motor(entry->subindex, value);
    else if (!stopped()) result = OD_BUSY;
    else if (entry->subindex == 1) {
        if (!finite_config()) result = OD_CONFIG_INVALID;
        else {
            motor_reset_err(0);
            motor_config_check(0);
            result = global_status.motor_status[0].is_error ? OD_CONFIG_INVALID : OD_OK;
        }
    } else if (entry->subindex == 3) {
        /* Rebuild derived controller/encoder state only with the bridge off.
         * ISR exclusion also prevents calibration starting halfway through. */
        config_pending = 1;
        if (!finite_config()) result = OD_CONFIG_INVALID;
        else {
            motor_reset_err(0);
            motor_init();
            encoder_init();
            motor_reset(0);
            if (global_status.motor_status[0].is_error) {
                global_status.motor_status[0].motor_state = MS_Err;
                result = OD_CONFIG_INVALID;
            } else {
                global_status.motor_status[0].is_aligned = 0;
                config_pending = 0; result = OD_OK;
            }
        }
    } else result = OD_SUBINDEX_NOT_FOUND;
    __set_PRIMASK(state);
    return result;
}

OD_Result od_commands_bind(void)
{
    if (od_dictionary.frozen) return OD_FROZEN;
    flash_status = (uint32_t)config_storage_last_status();
    config_pending = global_status.motor_status[0].is_error ? 1U : 0U;
    OD_Result result;
#define BIND_RO(name, pointer) do { result = OD_BIND_RO(od_dictionary, name, pointer); if (result != OD_OK) return result; } while (0)
#define BIND_CMD(name) do { result = od_bind_command(&od_dictionary, OD_NAME_##name, command_write); if (result != OD_OK) return result; } while (0)
    BIND_RO(control0_max_subindex, &control_max);
    BIND_CMD(control0_enable);
    BIND_CMD(control0_align);
    BIND_CMD(control0_identify);
    BIND_RO(control0_identify_state, &global_status.foc_status[0].identify_state);
    BIND_RO(system_max_subindex, &system_max);
    BIND_CMD(system_fault_reset);
    BIND_CMD(system_save_flash);
    BIND_CMD(system_apply_config);
    BIND_RO(system_flash_status, &flash_status);
    BIND_RO(system_config_pending, &config_pending);
#undef BIND_RO
#undef BIND_CMD
    return OD_OK;
}
