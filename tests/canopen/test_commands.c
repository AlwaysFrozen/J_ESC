#include "od_commands.h"
#include "od_generated.h"
#include "global.h"
#include "motor_api.h"
#include "config_storage.h"

#define CHECK(test) do { if (!(test)) return __LINE__; } while (0)
volatile Global_Status_t global_status;
Global_Configs_t global_config;
uint32_t test_primask, test_ipsr;
static int enables, disables, resets, saves, inits;
static bool invalid_config;
static Config_Storage_Status_t save_result = CONFIG_STORAGE_OK;
static float config_value = 24.0f;
static uint32_t u32;
static int32_t i32;
static uint16_t u16;
static uint8_t u8;
static bool boolean;

#ifdef OD_TEST_NO_CRT
int _fltused;
void *memcpy(void *destination, const void *source, size_t size)
{
    unsigned char *to = destination;
    const unsigned char *from = source;
    for (size_t i = 0; i < size; ++i) to[i] = from[i];
    return destination;
}
#endif

void motor_config_check(int index) { (void)index; if (invalid_config) global_status.motor_status[0].is_error = 1; }
void motor_reset_err(int index) { (void)index; global_status.motor_status[0].is_error = 0; }
void motor_reset(int index)
{
    ++resets;
    motor_config_check(index);
    global_status.motor_status[0].motor_state = MS_IDLE;
    global_status.motor_status[0].motor_ctrl_mode = MCM_None;
}
void motor_init(void) { ++inits; motor_config_check(0); }
void encoder_init(void) { ++inits; }
bool motor_enable(int index)
{
    (void)index; ++enables;
    global_status.motor_status[0].is_enabled = 1;
    global_status.motor_status[0].motor_state = MS_Running;
    return true;
}
bool motor_disable(int index)
{
    (void)index; ++disables;
    global_status.motor_status[0].is_enabled = 0;
    global_status.motor_status[0].motor_state = MS_IDLE;
    global_status.motor_status[0].motor_ctrl_mode = MCM_None;
    return true;
}
static bool mode(Motor_Control_Mode_t value)
{
    global_status.motor_status[0].motor_ctrl_mode = value;
    return true;
}
bool motor_set_align(int index) { (void)index; global_status.motor_status[0].is_aligned = 0; return mode(MCM_Align); }
bool motor_set_rs_identify(int index) { (void)index; return mode(MCM_RS_IDENTIFY); }
bool motor_set_ls_identify(int index) { (void)index; return mode(MCM_LS_IDENTIFY); }
bool motor_set_pp_identify(int index) { (void)index; return mode(MCM_PP_IDENTIFY); }
bool motor_set_flux_identify(int index)
{
    (void)index;
    return global_status.motor_status[0].is_aligned && mode(MCM_FLUX_IDENTIFY);
}
Config_Storage_Status_t config_storage_last_status(void) { return CONFIG_STORAGE_EMPTY; }
Config_Storage_Status_t config_storage_save(void)
{
    ++saves;
    if (test_primask || test_ipsr || global_status.motor_status[0].is_enabled) return CONFIG_STORAGE_BUSY;
    return save_result;
}

static OD_Result write_value(uint16_t index, uint8_t subindex, uint32_t value)
{
    uint8_t bytes[] = {value, value >> 8, value >> 16, value >> 24};
    return od_write(&od_dictionary, index, subindex, bytes, sizeof(bytes));
}

int main(void)
{
    CHECK(od_commands_bind() == OD_OK);
    CHECK(od_commands_bind() == OD_ALREADY_BOUND);
    for (size_t i = 0; i < od_dictionary.count; ++i) {
        const OD_Entry *entry = &od_dictionary.entries[i];
        if (entry->index < 0x2000) continue;
        OD_Result result;
        switch (entry->type) {
        case OD_REAL32: result = od_bind_f32(&od_dictionary, i, &config_value); break;
        case OD_UNSIGNED32: result = od_bind_u32(&od_dictionary, i, &u32); break;
        case OD_INTEGER32: result = od_bind_i32(&od_dictionary, i, &i32); break;
        case OD_UNSIGNED16: result = od_bind_u16(&od_dictionary, i, &u16); break;
        case OD_UNSIGNED8: result = od_bind_u8(&od_dictionary, i, &u8); break;
        case OD_BOOLEAN: result = od_bind_boolean(&od_dictionary, i, &boolean); break;
        default: return __LINE__;
        }
        CHECK(result == OD_OK);
    }
    CHECK(od_freeze(&od_dictionary, NULL) == OD_OK);
    CHECK(od_commands_bind() == OD_FROZEN);
    uint8_t bytes[4] = {1};
    CHECK(od_read(&od_dictionary, 0, 1, bytes, 4) == OD_ACCESS_DENIED);
    CHECK(od_write(&od_dictionary, 0, 1, bytes, 1) == OD_LENGTH_MISMATCH);
    CHECK(write_value(0, 1, 256) == OD_VALUE_INVALID);
    CHECK(write_value(0, 2, 0) == OD_VALUE_INVALID);
    CHECK(write_value(0x1000, 2, 0) == OD_VALUE_INVALID);
    CHECK(enables == 0 && saves == 0);
    global_status.motor_status[0].motor_state = MS_Init;
    CHECK(write_value(0, 1, 0) == OD_BUSY);
    global_status.motor_status[0].motor_state = MS_IDLE;
    CHECK(write_value(0, 1, 1) == OD_BUSY);
    global_status.motor_status[0].is_calibrated = 1;
    global_config.motor_config[0].motor_type = Motor_Type_BLDC_3_Phase;
    global_config.motor_controller_config[0].motor_controller_type = Motor_Controller_Type_FOC;
    global_config.motor_controller_config[0].motor_controller_type = Motor_Controller_Type_DTC;
    CHECK(write_value(0, 1, 1) == OD_UNSUPPORTED && enables == 0);
    global_config.motor_controller_config[0].motor_controller_type = Motor_Controller_Type_FOC;
    CHECK(write_value(0, 1, 1) == OD_OK && enables == 1 && resets == 1);
    CHECK(write_value(0, 1, 1) == OD_OK && enables == 1 && resets == 1);
    CHECK(write_value(0x1000, 2, 1) == OD_BUSY && saves == 0);
    CHECK(write_value(0x1000, 1, 1) == OD_BUSY);
    CHECK(write_value(0x1000, 3, 1) == OD_BUSY);
    CHECK(write_value(0, 2, 1) == OD_OK);
    CHECK(write_value(0, 3, 1) == OD_BUSY); /* Cannot replace an active operation. */
    CHECK(write_value(0, 1, 0) == OD_OK);
    CHECK(write_value(0, 2, 1) == OD_BUSY); /* No implicit enable. */
    CHECK(write_value(0, 1, 1) == OD_OK);
    CHECK(write_value(0, 3, 4) == OD_BUSY); /* Flux requires alignment. */
    for (unsigned int value = 1; value <= 4; ++value) {
        CHECK(write_value(0, 1, 0) == OD_OK);
        CHECK(write_value(0, 1, 1) == OD_OK);
        global_status.motor_status[0].is_aligned = 1;
        CHECK(write_value(0, 3, value) == OD_OK);
    }
    for (unsigned int value = 5; value <= 8; ++value)
        CHECK(write_value(0, 3, value) == OD_UNSUPPORTED);
    CHECK(write_value(0, 3, 9) == OD_VALUE_INVALID);
    global_status.motor_status[0].is_error = 1;
    CHECK(write_value(0, 1, 0) == OD_OK);
    CHECK(global_status.motor_status[0].is_error && global_status.motor_status[0].motor_state == MS_Err);
    CHECK(write_value(0, 1, 1) == OD_BUSY);
    invalid_config = true;
    CHECK(write_value(0x1000, 1, 1) == OD_CONFIG_INVALID);
    invalid_config = false;
    CHECK(write_value(0x1000, 1, 1) == OD_OK);
    od_commands_config_changed();
    CHECK(write_value(0, 1, 1) == OD_CONFIG_INVALID);
    CHECK(write_value(0x1000, 2, 1) == OD_CONFIG_INVALID && saves == 0);
    invalid_config = true;
    CHECK(write_value(0x1000, 3, 1) == OD_CONFIG_INVALID);
    CHECK(global_status.motor_status[0].motor_state == MS_Err);
    invalid_config = false;
    const union { uint32_t bits; float value; } nan = {.bits = UINT32_C(0x7FC00000)};
    config_value = nan.value;
    CHECK(write_value(0x1000, 3, 1) == OD_CONFIG_INVALID);
    config_value = 24.0f;
    CHECK(write_value(0x1000, 3, 1) == OD_OK);
    CHECK(global_status.motor_status[0].is_aligned == 0 && inits == 4);
    CHECK(od_read(&od_dictionary, 0x1000, 5, bytes, 4) == OD_OK && bytes[0] == 0);
    for (int value = CONFIG_STORAGE_OK; value <= CONFIG_STORAGE_VERIFY_ERROR; ++value) {
        save_result = (Config_Storage_Status_t)value;
        OD_Result expected = value == CONFIG_STORAGE_OK ? OD_OK : value == CONFIG_STORAGE_BUSY ? OD_BUSY : OD_OPERATION_FAILED;
        CHECK(write_value(0x1000, 2, 1) == expected);
        CHECK(od_read(&od_dictionary, 0x1000, 4, bytes, 4) == OD_OK && bytes[0] == value);
    }
    int previous_saves = saves;
    test_ipsr = 1;
    CHECK(write_value(0x1000, 2, 1) == OD_BUSY);
    test_ipsr = 0; test_primask = 1;
    CHECK(write_value(0x1000, 2, 1) == OD_BUSY);
    CHECK(saves == previous_saves);
    test_primask = 0;
    CHECK(disables > 0);
    return 0;
}
