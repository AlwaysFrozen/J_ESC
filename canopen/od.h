#ifndef CANOPEN_OD_H
#define CANOPEN_OD_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/* CANopen basic data type identifiers. */
typedef enum {
    OD_BOOLEAN = 0x0001, OD_INTEGER8 = 0x0002, OD_INTEGER16 = 0x0003,
    OD_INTEGER32 = 0x0004, OD_UNSIGNED8 = 0x0005, OD_UNSIGNED16 = 0x0006,
    OD_UNSIGNED32 = 0x0007, OD_REAL32 = 0x0008, OD_REAL64 = 0x0011,
    OD_INTEGER64 = 0x0015, OD_UNSIGNED64 = 0x001B
} OD_Type;

typedef enum { OD_RO = 1, OD_WO = 2, OD_RW = 3 } OD_Access;
typedef enum {
    OD_OK = 0, OD_BAD_ARGUMENT, OD_INDEX_NOT_FOUND, OD_SUBINDEX_NOT_FOUND,
    OD_ACCESS_DENIED, OD_LENGTH_MISMATCH, OD_TYPE_MISMATCH, OD_NOT_BOUND,
    OD_ALREADY_BOUND, OD_FROZEN, OD_NOT_READY, OD_VALUE_INVALID,
    OD_BUSY, OD_UNSUPPORTED, OD_OPERATION_FAILED, OD_CONFIG_INVALID
} OD_Result;

typedef struct {
    const char *name;
    uint16_t index;
    uint8_t subindex;
    OD_Type type;
    OD_Access access;
    uint8_t size;
} OD_Entry;

/* WO commands execute synchronously in the caller's foreground context.
 * The handler owns synchronization; a successful write is never queued/replayed. */
typedef OD_Result (*OD_Write_Handler)(const OD_Entry *, const uint8_t *, size_t);

typedef struct {
    /* The cast in od_write is permitted only for a writable typed binding. */
    const volatile void *data;
    bool writable;
    bool native_int; /* Access as int/unsigned int, which may differ from int32_t. */
    bool widen_u8; /* Byte-sized enum storage exposed as UNSIGNED32 on the wire. */
    OD_Write_Handler write_handler;
} OD_Binding;

typedef struct {
    const OD_Entry *entries; /* Sorted by (index, subindex). */
    OD_Binding *bindings;
    size_t count;
    bool frozen;
} OD_Dictionary;

OD_Result od_bind_command(OD_Dictionary *, size_t, OD_Write_Handler);
bool od_is_command(const OD_Dictionary *, uint16_t index, uint8_t subindex);

/* All bindings are required. Failure leaves the dictionary unfrozen.
 * first_missing may be NULL. Bind/freeze before enabling communication. */
OD_Result od_freeze(OD_Dictionary *dictionary, const OD_Entry **first_missing);
const OD_Entry *od_find(const OD_Dictionary *dictionary, uint16_t index, uint8_t subindex);

/* Exact-length, little-endian wire values; no heap allocation.
 * Caller must serialize accesses and provide any ISR/task synchronization.
 * The output buffer is untouched on read failure. Writes validate before storing. */
OD_Result od_read(const OD_Dictionary *dictionary, uint16_t index, uint8_t subindex,
                  uint8_t *buffer, size_t length);
OD_Result od_write(OD_Dictionary *dictionary, uint16_t index, uint8_t subindex,
                   const uint8_t *buffer, size_t length);

/* A read plan is prepared once in foreground on an immutable frozen dictionary.
 * Keep the dictionary alive. Prepared reads preserve typed volatile loads, but
 * the caller still owns synchronization and 64-bit atomicity. */
typedef struct { const OD_Entry *entry; const OD_Binding *binding; } OD_ReadPlan;
OD_Result od_prepare_read(const OD_Dictionary *, uint16_t index, uint8_t subindex, OD_ReadPlan *);
OD_Result od_read_prepared(const OD_ReadPlan *, uint8_t *buffer, size_t length);

/* Typed entry points preserve const/volatile. Prefer the C11 generic macros below. */
#define OD_DECLARE_BIND(suffix, c_type) \
    OD_Result od_bind_##suffix(OD_Dictionary *, size_t, volatile c_type *); \
    OD_Result od_bind_ro_##suffix(OD_Dictionary *, size_t, const volatile c_type *)
OD_DECLARE_BIND(boolean, bool);
OD_DECLARE_BIND(i8, int8_t);
OD_DECLARE_BIND(i16, int16_t);
OD_DECLARE_BIND(i32, int32_t);
OD_DECLARE_BIND(i64, int64_t);
OD_DECLARE_BIND(u8, uint8_t);
OD_DECLARE_BIND(u16, uint16_t);
OD_DECLARE_BIND(u32, uint32_t);
OD_DECLARE_BIND(u64, uint64_t);
OD_DECLARE_BIND(f32, float);
OD_DECLARE_BIND(f64, double);
OD_DECLARE_BIND(native_i32, int);
OD_DECLARE_BIND(native_u32, unsigned int);
#undef OD_DECLARE_BIND

/* Selected only for unsupported types: its strict prototype rejects the
 * three-argument binding call at compile time. No implementation is needed. */
void od_bind_unsupported_pointer(void);

/* Explicit width conversion, preserving the application's short-enum ABI. */
OD_Result od_bind_u8_as_u32(OD_Dictionary *, size_t, volatile uint8_t *);
OD_Result od_bind_ro_u8_as_u32(OD_Dictionary *, size_t, const volatile uint8_t *);

#define OD_BIND_CASE(suffix, c_type) \
    c_type *: od_bind_##suffix, volatile c_type *: od_bind_##suffix
#define OD_BIND_RO_CASE(suffix, c_type) \
    c_type *: od_bind_ro_##suffix, volatile c_type *: od_bind_ro_##suffix, \
    const c_type *: od_bind_ro_##suffix, const volatile c_type *: od_bind_ro_##suffix

/* name is the CSV name token, not a string. Pointer expressions evaluate once.
 * Unsupported pointer types (including void*) are a compile-time error. */
#define OD_FIXED_BIND_FUNCTION(pointer) \
    _Generic((pointer), \
        OD_BIND_CASE(boolean, bool), OD_BIND_CASE(i8, int8_t), \
        OD_BIND_CASE(i16, int16_t), OD_BIND_CASE(i32, int32_t), \
        OD_BIND_CASE(i64, int64_t), OD_BIND_CASE(u8, uint8_t), \
        OD_BIND_CASE(u16, uint16_t), OD_BIND_CASE(u32, uint32_t), \
        OD_BIND_CASE(u64, uint64_t), OD_BIND_CASE(f32, float), \
        OD_BIND_CASE(f64, double), default: od_bind_unsupported_pointer)

#define OD_FIXED_BIND_RO_FUNCTION(pointer) \
    _Generic((pointer), \
        OD_BIND_RO_CASE(boolean, bool), OD_BIND_RO_CASE(i8, int8_t), \
        OD_BIND_RO_CASE(i16, int16_t), OD_BIND_RO_CASE(i32, int32_t), \
        OD_BIND_RO_CASE(i64, int64_t), OD_BIND_RO_CASE(u8, uint8_t), \
        OD_BIND_RO_CASE(u16, uint16_t), OD_BIND_RO_CASE(u32, uint32_t), \
        OD_BIND_RO_CASE(u64, uint64_t), OD_BIND_RO_CASE(f32, float), \
        OD_BIND_RO_CASE(f64, double), default: od_bind_unsupported_pointer)

/* Nesting avoids duplicate _Generic associations when int32_t aliases int.
 * GNU ARM may instead typedef int32_t as long: retain the actual access type. */
#define OD_BIND(dictionary, name, pointer) \
    _Generic((pointer), OD_BIND_CASE(native_i32, int), OD_BIND_CASE(native_u32, unsigned int), \
        default: OD_FIXED_BIND_FUNCTION(pointer))(&(dictionary), OD_NAME_##name, (pointer))

#define OD_BIND_RO(dictionary, name, pointer) \
    _Generic((pointer), OD_BIND_RO_CASE(native_i32, int), OD_BIND_RO_CASE(native_u32, unsigned int), \
        default: OD_FIXED_BIND_RO_FUNCTION(pointer))(&(dictionary), OD_NAME_##name, (pointer))

#endif
