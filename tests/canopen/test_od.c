#include "od_generated.h"

#ifdef OD_TEST_NO_CRT
/* Minimal support for standalone Windows /nodefaultlib test executables. */
int _fltused = 0;
void *memcpy(void *destination, const void *source, size_t size)
{
    unsigned char *to = destination;
    const unsigned char *from = source;
    for (size_t i = 0; i < size; ++i) to[i] = from[i];
    return destination;
}
void *memset(void *destination, int value, size_t size)
{
    unsigned char *to = destination;
    for (size_t i = 0; i < size; ++i) to[i] = (unsigned char)value;
    return destination;
}
#endif

#define CHECK(test) do { if (!(test)) return __LINE__; } while (0)

static int test_generated(void)
{
    const OD_Entry *missing = NULL;
    volatile float voltage = 24.5f;
    float speed = 0.0f, limit = 10.0f;
    uint32_t wrong_type = 0;
    uint8_t buffer[4] = {0x55, 0x55, 0x55, 0x55};
    const uint8_t wire_speed[4] = {0, 0, 0x48, 0x41}; /* 12.5 */

    CHECK(od_freeze(&od_dictionary, &missing) == OD_NOT_BOUND);
    CHECK(missing == od_find(&od_dictionary, 0x2000, 0));
    CHECK(od_read(&od_dictionary, 0x2000, 0, buffer, 4) == OD_NOT_BOUND);
    CHECK(buffer[0] == 0x55);
    CHECK(OD_BIND(od_dictionary, bus_voltage, &wrong_type) == OD_TYPE_MISMATCH);
    CHECK(OD_BIND_RO(od_dictionary, bus_voltage, &voltage) == OD_OK);
    CHECK(OD_BIND_RO(od_dictionary, motor0_target_speed, &speed) == OD_ACCESS_DENIED);
    CHECK(OD_BIND(od_dictionary, motor0_target_speed, &speed) == OD_OK);
    CHECK(OD_BIND(od_dictionary, motor0_target_speed, &speed) == OD_ALREADY_BOUND);
    CHECK(od_freeze(&od_dictionary, &missing) == OD_NOT_BOUND);
    CHECK(missing == od_find(&od_dictionary, 0x2002, 0));
    CHECK(od_read(&od_dictionary, 0x2000, 0, buffer, 4) == OD_NOT_READY);
    CHECK(OD_BIND(od_dictionary, motor0_current_limit, &limit) == OD_OK);
    CHECK(od_freeze(&od_dictionary, &missing) == OD_OK && missing == NULL);
    CHECK(OD_BIND(od_dictionary, motor0_current_limit, &limit) == OD_FROZEN);
    CHECK(od_read(&od_dictionary, 0x2000, 0, buffer, 4) == OD_OK);
    CHECK(buffer[0] == 0 && buffer[1] == 0 && buffer[2] == 0xC4 && buffer[3] == 0x41);
    voltage = 25.0f;
    CHECK(od_read(&od_dictionary, 0x2000, 0, buffer, 4) == OD_OK && buffer[2] == 0xC8);
    CHECK(od_write(&od_dictionary, 0x2000, 0, wire_speed, 4) == OD_ACCESS_DENIED);
    CHECK(voltage == 25.0f);
    CHECK(od_write(&od_dictionary, 0x2001, 0, wire_speed, 3) == OD_LENGTH_MISMATCH);
    CHECK(speed == 0.0f);
    CHECK(od_write(&od_dictionary, 0x2001, 0, wire_speed, 4) == OD_OK && speed == 12.5f);
    CHECK(od_read(&od_dictionary, 0x1999, 0, buffer, 4) == OD_INDEX_NOT_FOUND);
    CHECK(od_read(&od_dictionary, 0x2000, 1, buffer, 4) == OD_SUBINDEX_NOT_FOUND);
    CHECK(od_read(&od_dictionary, 0x2002, 255, buffer, 4) == OD_SUBINDEX_NOT_FOUND);
    CHECK(od_read(&od_dictionary, 0xFFFF, 0, buffer, 4) == OD_INDEX_NOT_FOUND);
    CHECK(od_read(NULL, 0, 0, buffer, 4) == OD_BAD_ARGUMENT);
    CHECK(od_write(&od_dictionary, 0x2001, 0, NULL, 4) == OD_BAD_ARGUMENT);
    CHECK(od_freeze(NULL, NULL) == OD_BAD_ARGUMENT);
    return 0;
}

static int test_scalars(void)
{
    static const OD_Entry entries[] = {
        {"b", 0x2100, 0, OD_BOOLEAN, OD_RW, 1},
        {"i8", 0x2101, 0, OD_INTEGER8, OD_RW, 1},
        {"i16", 0x2102, 0, OD_INTEGER16, OD_RW, 2},
        {"i32", 0x2103, 0, OD_INTEGER32, OD_RW, 4},
        {"i64", 0x2104, 0, OD_INTEGER64, OD_RW, 8},
        {"u8", 0x2105, 0, OD_UNSIGNED8, OD_RW, 1},
        {"u16", 0x2106, 0, OD_UNSIGNED16, OD_RW, 2},
        {"u32", 0x2107, 0, OD_UNSIGNED32, OD_RW, 4},
        {"u64", 0x2108, 0, OD_UNSIGNED64, OD_RW, 8},
        {"f32", 0x2109, 0, OD_REAL32, OD_RW, 4},
        {"f64", 0x210A, 0, OD_REAL64, OD_RW, 8},
        {"constant", 0x2200, 0, OD_UNSIGNED32, OD_RO, 4},
        {"command", 0x2200, 2, OD_UNSIGNED8, OD_WO, 1},
    };
    OD_Binding bindings[13] = {0};
    OD_Dictionary d = {entries, bindings, 13, false};
    bool b = false;
    int8_t i8 = 0; int16_t i16 = 0; int32_t i32 = 0; int64_t i64 = 0;
    uint8_t u8 = 0, command = 0; uint16_t u16 = 0; uint32_t u32 = 0; uint64_t u64 = 0;
    float f32 = 0; double f64 = 0;
    const uint32_t constant = 0x12345678;
    const uint8_t values[11][8] = {
        {1}, {0xFE}, {0xFE, 0xFF}, {0xFE, 0xFF, 0xFF, 0xFF},
        {0xFE, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF},
        {0xA5}, {0x34, 0x12}, {0x78, 0x56, 0x34, 0x12},
        {8, 7, 6, 5, 4, 3, 2, 1}, {0, 0, 0x48, 0x41},
        {0, 0, 0, 0, 0, 0, 0x29, 0x40}
    };
    uint8_t output[8];
    CHECK(od_bind_boolean(&d, 0, &b) == OD_OK);
    CHECK(od_bind_i8(&d, 1, &i8) == OD_OK);
    CHECK(od_bind_i16(&d, 2, &i16) == OD_OK);
    CHECK(od_bind_i32(&d, 3, &i32) == OD_OK);
    CHECK(od_bind_i64(&d, 4, &i64) == OD_OK);
    CHECK(od_bind_u8(&d, 5, &u8) == OD_OK);
    CHECK(od_bind_u16(&d, 6, &u16) == OD_OK);
    CHECK(od_bind_u32(&d, 7, &u32) == OD_OK);
    CHECK(od_bind_u64(&d, 8, &u64) == OD_OK);
    CHECK(od_bind_f32(&d, 9, &f32) == OD_OK);
    CHECK(od_bind_f64(&d, 10, &f64) == OD_OK);
    CHECK(od_bind_ro_u32(&d, 11, &constant) == OD_OK);
    CHECK(od_bind_u8(&d, 12, &command) == OD_OK);
    CHECK(od_freeze(&d, NULL) == OD_OK);
    for (size_t i = 0; i < 11; ++i) {
        CHECK(od_write(&d, entries[i].index, 0, values[i], entries[i].size) == OD_OK);
        CHECK(od_read(&d, entries[i].index, 0, output, entries[i].size) == OD_OK);
        for (size_t j = 0; j < entries[i].size; ++j) CHECK(output[j] == values[i][j]);
    }
    CHECK(b && i8 == -2 && i16 == -2 && i32 == -2 && i64 == -2);
    CHECK(u8 == 0xA5 && u16 == 0x1234 && u32 == 0x12345678);
    CHECK(u64 == UINT64_C(0x0102030405060708) && f32 == 12.5f && f64 == 12.5);
    output[0] = 2;
    CHECK(od_write(&d, 0x2100, 0, output, 1) == OD_VALUE_INVALID && b);
    output[0] = 0;
    CHECK(od_write(&d, 0x2100, 0, output, 1) == OD_OK && !b);
    CHECK(od_read(&d, 0x2200, 0, output, 4) == OD_OK && output[0] == 0x78);
    CHECK(od_write(&d, 0x2200, 2, values[5], 1) == OD_OK && command == 0xA5);
    CHECK(od_read(&d, 0x2200, 2, output, 1) == OD_ACCESS_DENIED);
    CHECK(od_read(&d, 0x2200, 1, output, 1) == OD_SUBINDEX_NOT_FOUND);
    return 0;
}

int test_protocol(void);
int test_cdc(void);

static int test_native_storage(void)
{
    enum { OD_NAME_native_signed, OD_NAME_native_unsigned, OD_NAME_wide_enum };
    const OD_Entry entries[] = {
        {"native_signed", 0x2300, 1, OD_INTEGER32, OD_RW, 4},
        {"native_unsigned", 0x2300, 2, OD_UNSIGNED32, OD_RW, 4},
        {"wide_enum", 0x2300, 3, OD_UNSIGNED32, OD_RW, 4},
    };
    OD_Binding bindings[3] = {0};
    OD_Dictionary d = {entries, bindings, 3, false};
    int signed_value = -7;
    unsigned int unsigned_value = 0x12345678U;
    struct { uint8_t value; uint8_t guard[3]; } short_enum = {1, {0xAA, 0xBB, 0xCC}};
    CHECK(OD_BIND(d, native_signed, &signed_value) == OD_OK);
    CHECK(OD_BIND(d, native_unsigned, &unsigned_value) == OD_OK);
    CHECK(od_bind_u8_as_u32(&d, OD_NAME_wide_enum, &short_enum.value) == OD_OK);
    CHECK(od_freeze(&d, NULL) == OD_OK);
    uint8_t output[4];
    CHECK(od_read(&d, 0x2300, 1, output, 4) == OD_OK);
    CHECK(output[0] == 0xF9 && output[1] == 0xFF && output[3] == 0xFF);
    CHECK(od_read(&d, 0x2300, 2, output, 4) == OD_OK);
    CHECK(output[0] == 0x78 && output[3] == 0x12);
    const uint8_t negative[] = {0xFE, 0xFF, 0xFF, 0xFF};
    CHECK(od_write(&d, 0x2300, 1, negative, 4) == OD_OK && signed_value == -2);
    CHECK(od_write(&d, 0x2300, 2, negative, 4) == OD_OK && unsigned_value == 0xFFFFFFFEU);
    CHECK(od_read(&d, 0x2300, 3, output, 4) == OD_OK);
    CHECK(output[0] == 1 && output[1] == 0 && output[2] == 0 && output[3] == 0);
    const uint8_t valid[] = {2, 0, 0, 0}, invalid[] = {0, 1, 0, 0};
    CHECK(od_write(&d, 0x2300, 3, valid, 4) == OD_OK && short_enum.value == 2);
    CHECK(short_enum.guard[0] == 0xAA && short_enum.guard[1] == 0xBB && short_enum.guard[2] == 0xCC);
    CHECK(od_write(&d, 0x2300, 3, invalid, 4) == OD_VALUE_INVALID && short_enum.value == 2);
    return 0;
}

static int test_readonly_runtime(void)
{
    static const OD_Entry entries[] = {
        {"short_state", 0x2400, 1, OD_UNSIGNED32, OD_RO, 4},
        {"wide_state", 0x2400, 2, OD_UNSIGNED32, OD_RO, 4},
        {"feedback", 0x2400, 3, OD_REAL32, OD_RO, 4},
        {"flag", 0x2400, 4, OD_BOOLEAN, OD_RO, 1},
        {"writable", 0x2400, 5, OD_UNSIGNED32, OD_RW, 4},
    };
    OD_Binding bindings[5] = {0};
    OD_Dictionary d = {entries, bindings, 5, false};
    volatile struct { uint8_t value; uint8_t guard[3]; } short_state = {3, {0xAA, 0xBB, 0xCC}};
    const volatile unsigned int wide_state = 0x21000001U;
    volatile float feedback = 12.5f;
    volatile bool flag = true;
    uint32_t writable = 0;
    CHECK(od_bind_ro_u8_as_u32(&d, 4, &short_state.value) == OD_ACCESS_DENIED);
    CHECK(od_bind_ro_u8_as_u32(&d, 2, &short_state.value) == OD_TYPE_MISMATCH);
    CHECK(od_bind_ro_u8_as_u32(&d, 0, &short_state.value) == OD_OK);
    CHECK(!bindings[0].writable && bindings[0].widen_u8);
    CHECK(od_bind_ro_native_u32(&d, 1, &wide_state) == OD_OK);
    CHECK(od_bind_ro_f32(&d, 2, &feedback) == OD_OK);
    CHECK(od_bind_ro_boolean(&d, 3, &flag) == OD_OK);
    CHECK(od_bind_u32(&d, 4, &writable) == OD_OK);
    CHECK(od_freeze(&d, NULL) == OD_OK);
    uint8_t output[4];
    CHECK(od_read(&d, 0x2400, 1, output, 4) == OD_OK);
    CHECK(output[0] == 3 && output[1] == 0 && output[2] == 0 && output[3] == 0);
    short_state.value = 7;
    CHECK(od_read(&d, 0x2400, 1, output, 4) == OD_OK && output[0] == 7);
    CHECK(od_read(&d, 0x2400, 2, output, 4) == OD_OK);
    CHECK(output[0] == 1 && output[1] == 0 && output[2] == 0 && output[3] == 0x21);
    CHECK(od_read(&d, 0x2400, 3, output, 4) == OD_OK && output[2] == 0x48 && output[3] == 0x41);
    CHECK(od_read(&d, 0x2400, 4, output, 1) == OD_OK && output[0] == 1);
    const uint8_t zero[4] = {0};
    for (uint8_t subindex = 1; subindex <= 4; ++subindex)
        CHECK(od_write(&d, 0x2400, subindex, zero, subindex == 4 ? 1 : 4) == OD_ACCESS_DENIED);
    CHECK(short_state.value == 7 && short_state.guard[0] == 0xAA && short_state.guard[2] == 0xCC);
    CHECK(wide_state == 0x21000001U && feedback == 12.5f && flag);
    CHECK(od_bind_ro_u8_as_u32(&d, 0, &short_state.value) == OD_FROZEN);
    return 0;
}

int main(void)
{
    int result = test_generated();
    if (result != 0) return result;
    result = test_native_storage();
    if (result != 0) return result;
    result = test_scalars();
    if (result != 0) return result;
    result = test_readonly_runtime();
    if (result != 0) return result;
    result = test_protocol();
    return result != 0 ? result : test_cdc();
}
