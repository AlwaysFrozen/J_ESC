#include "od.h"
#include <float.h>
#include <limits.h>

_Static_assert(CHAR_BIT == 8, "OD requires 8-bit bytes");
_Static_assert(sizeof(int) == 4, "native integer bindings require 32-bit int");
_Static_assert(sizeof(float) == 4 && FLT_RADIX == 2 && FLT_MANT_DIG == 24 &&
               FLT_MAX_EXP == 128, "REAL32 requires IEEE binary32");
_Static_assert(sizeof(double) == 8 && DBL_MANT_DIG == 53 && DBL_MAX_EXP == 1024,
               "REAL64 requires IEEE binary64");
_Static_assert(INT32_MIN == (-INT32_MAX - 1), "OD requires two's complement integers");

static bool valid_dictionary(const OD_Dictionary *d)
{
    return d != NULL && d->entries != NULL && d->bindings != NULL && d->count != 0;
}

static OD_Result bind_pointer(OD_Dictionary *d, size_t slot, const volatile void *data,
                              OD_Type type, bool writable, bool native_int)
{
    if (!valid_dictionary(d) || slot >= d->count || data == NULL) return OD_BAD_ARGUMENT;
    if (d->frozen) return OD_FROZEN;
    if (d->entries[slot].type != type) return OD_TYPE_MISMATCH;
    if (!writable && d->entries[slot].access != OD_RO) return OD_ACCESS_DENIED;
    if (d->bindings[slot].data != NULL || d->bindings[slot].write_handler != NULL) return OD_ALREADY_BOUND;
    d->bindings[slot].data = data;
    d->bindings[slot].writable = writable;
    d->bindings[slot].native_int = native_int;
    d->bindings[slot].widen_u8 = false;
    return OD_OK;
}

OD_Result od_bind_command(OD_Dictionary *d, size_t slot, OD_Write_Handler handler)
{
    if (!valid_dictionary(d) || slot >= d->count || handler == NULL) return OD_BAD_ARGUMENT;
    if (d->frozen) return OD_FROZEN;
    if (d->entries[slot].access != OD_WO) return OD_ACCESS_DENIED;
    if (d->entries[slot].type != OD_UNSIGNED32 || d->entries[slot].size != 4U) return OD_TYPE_MISMATCH;
    if (d->bindings[slot].data != NULL || d->bindings[slot].write_handler != NULL) return OD_ALREADY_BOUND;
    d->bindings[slot].write_handler = handler;
    d->bindings[slot].writable = true;
    return OD_OK;
}

bool od_is_command(const OD_Dictionary *d, uint16_t index, uint8_t subindex)
{
    const OD_Entry *entry = od_find(d, index, subindex);
    return entry != NULL && d->bindings[entry - d->entries].write_handler != NULL;
}

#define OD_DEFINE_BIND(suffix, c_type, od_type) \
    OD_Result od_bind_##suffix(OD_Dictionary *d, size_t slot, volatile c_type *p) \
    { return bind_pointer(d, slot, p, od_type, true, false); } \
    OD_Result od_bind_ro_##suffix(OD_Dictionary *d, size_t slot, const volatile c_type *p) \
    { return bind_pointer(d, slot, p, od_type, false, false); }
OD_DEFINE_BIND(boolean, bool, OD_BOOLEAN)
OD_DEFINE_BIND(i8, int8_t, OD_INTEGER8)
OD_DEFINE_BIND(i16, int16_t, OD_INTEGER16)
OD_DEFINE_BIND(i32, int32_t, OD_INTEGER32)
OD_DEFINE_BIND(i64, int64_t, OD_INTEGER64)
OD_DEFINE_BIND(u8, uint8_t, OD_UNSIGNED8)
OD_DEFINE_BIND(u16, uint16_t, OD_UNSIGNED16)
OD_DEFINE_BIND(u32, uint32_t, OD_UNSIGNED32)
OD_DEFINE_BIND(u64, uint64_t, OD_UNSIGNED64)
OD_DEFINE_BIND(f32, float, OD_REAL32)
OD_DEFINE_BIND(f64, double, OD_REAL64)
#undef OD_DEFINE_BIND

#define OD_DEFINE_NATIVE_BIND(suffix, c_type, od_type) \
    OD_Result od_bind_##suffix(OD_Dictionary *d, size_t slot, volatile c_type *p) \
    { return bind_pointer(d, slot, p, od_type, true, true); } \
    OD_Result od_bind_ro_##suffix(OD_Dictionary *d, size_t slot, const volatile c_type *p) \
    { return bind_pointer(d, slot, p, od_type, false, true); }
OD_DEFINE_NATIVE_BIND(native_i32, int, OD_INTEGER32)
OD_DEFINE_NATIVE_BIND(native_u32, unsigned int, OD_UNSIGNED32)
#undef OD_DEFINE_NATIVE_BIND

OD_Result od_bind_u8_as_u32(OD_Dictionary *d, size_t slot, volatile uint8_t *p)
{
    OD_Result result = bind_pointer(d, slot, p, OD_UNSIGNED32, true, false);
    if (result == OD_OK) d->bindings[slot].widen_u8 = true;
    return result;
}

OD_Result od_bind_ro_u8_as_u32(OD_Dictionary *d, size_t slot, const volatile uint8_t *p)
{
    OD_Result result = bind_pointer(d, slot, p, OD_UNSIGNED32, false, false);
    if (result == OD_OK) d->bindings[slot].widen_u8 = true;
    return result;
}

OD_Result od_freeze(OD_Dictionary *d, const OD_Entry **first_missing)
{
    if (first_missing != NULL) *first_missing = NULL;
    if (!valid_dictionary(d)) return OD_BAD_ARGUMENT;
    for (size_t i = 0; i < d->count; ++i) {
        if (d->bindings[i].data == NULL && d->bindings[i].write_handler == NULL) {
            if (first_missing != NULL) *first_missing = &d->entries[i];
            return OD_NOT_BOUND;
        }
    }
    d->frozen = true;
    return OD_OK;
}

static OD_Result locate(const OD_Dictionary *d, uint16_t index, uint8_t subindex, size_t *slot)
{
    if (!valid_dictionary(d)) return OD_BAD_ARGUMENT;
    size_t lo = 0, hi = d->count;
    while (lo < hi) {
        size_t mid = lo + (hi - lo) / 2;
        const OD_Entry *e = &d->entries[mid];
        if (e->index < index || (e->index == index && e->subindex < subindex)) lo = mid + 1;
        else hi = mid;
    }
    if (lo < d->count && d->entries[lo].index == index && d->entries[lo].subindex == subindex) {
        *slot = lo;
        return OD_OK;
    }
    if ((lo < d->count && d->entries[lo].index == index) ||
        (lo > 0 && d->entries[lo - 1].index == index)) return OD_SUBINDEX_NOT_FOUND;
    return OD_INDEX_NOT_FOUND;
}

const OD_Entry *od_find(const OD_Dictionary *d, uint16_t index, uint8_t subindex)
{
    size_t slot;
    return locate(d, index, subindex, &slot) == OD_OK ? &d->entries[slot] : NULL;
}

/* Access the object representation only after a single typed volatile load,
 * and perform a single typed volatile store after decoding. No bytewise I/O
 * on the shared application variable. A critical section may still be needed. */
static bool little_endian(void)
{
    const uint16_t one = 1;
    return *(const unsigned char *)&one == 1;
}

static void encode(uint8_t *wire, const unsigned char *native, size_t size)
{
    bool little = little_endian();
    for (size_t i = 0; i < size; ++i) wire[i] = native[little ? i : size - 1 - i];
}

static void decode(unsigned char *native, const uint8_t *wire, size_t size)
{
    bool little = little_endian();
    for (size_t i = 0; i < size; ++i) native[little ? i : size - 1 - i] = wire[i];
}

static OD_Result check_access(const OD_Dictionary *d, uint16_t index, uint8_t subindex,
                               size_t length, bool write, size_t *slot)
{
    OD_Result result = locate(d, index, subindex, slot);
    if (result != OD_OK) return result;
    const OD_Entry *e = &d->entries[*slot];
    if ((e->access & (write ? OD_WO : OD_RO)) == 0) return OD_ACCESS_DENIED;
    if (length != e->size) return OD_LENGTH_MISMATCH;
    if (d->bindings[*slot].data == NULL && d->bindings[*slot].write_handler == NULL) return OD_NOT_BOUND;
    if (!d->frozen) return OD_NOT_READY;
    if (write && !d->bindings[*slot].writable) return OD_ACCESS_DENIED;
    return OD_OK;
}

#define OD_SCALAR_TYPES(X) \
    X(OD_INTEGER8, int8_t) X(OD_INTEGER16, int16_t) X(OD_INTEGER32, int32_t) \
    X(OD_INTEGER64, int64_t) X(OD_UNSIGNED8, uint8_t) X(OD_UNSIGNED16, uint16_t) \
    X(OD_UNSIGNED32, uint32_t) X(OD_UNSIGNED64, uint64_t) X(OD_REAL32, float) X(OD_REAL64, double)

OD_Result od_read(const OD_Dictionary *d, uint16_t index, uint8_t subindex,
                  uint8_t *buffer, size_t length)
{
    if (buffer == NULL) return OD_BAD_ARGUMENT;
    size_t slot;
    OD_Result result = check_access(d, index, subindex, length, false, &slot);
    if (result != OD_OK) return result;
    OD_ReadPlan plan = {&d->entries[slot], &d->bindings[slot]};
    return od_read_prepared(&plan, buffer, length);
}

OD_Result od_prepare_read(const OD_Dictionary *d, uint16_t index, uint8_t subindex, OD_ReadPlan *plan)
{
    if (plan == NULL) return OD_BAD_ARGUMENT;
    const OD_Entry *entry = od_find(d, index, subindex);
    size_t slot;
    OD_Result result = check_access(d, index, subindex, entry == NULL ? 0 : entry->size, false, &slot);
    if (result != OD_OK) return result;
    *plan = (OD_ReadPlan){&d->entries[slot], &d->bindings[slot]};
    return OD_OK;
}

OD_Result od_read_prepared(const OD_ReadPlan *plan, uint8_t *buffer, size_t length)
{
    if (plan == NULL || plan->entry == NULL || plan->binding == NULL || buffer == NULL)
        return OD_BAD_ARGUMENT;
    if (length != plan->entry->size) return OD_LENGTH_MISMATCH;
    if ((plan->entry->access & OD_RO) == 0) return OD_ACCESS_DENIED;
    if (plan->binding->data == NULL) return OD_NOT_BOUND;
    const volatile void *data = plan->binding->data;
    if (plan->binding->widen_u8) {
        uint32_t value = *(const volatile uint8_t *)data;
        encode(buffer, (const unsigned char *)&value, sizeof(value));
        return OD_OK;
    }
    if (plan->binding->native_int) {
        if (plan->entry->type == OD_INTEGER32) {
            int value = *(const volatile int *)data;
            encode(buffer, (const unsigned char *)&value, sizeof(value));
        } else {
            unsigned int value = *(const volatile unsigned int *)data;
            encode(buffer, (const unsigned char *)&value, sizeof(value));
        }
        return OD_OK;
    }
    switch (plan->entry->type) {
    case OD_BOOLEAN: buffer[0] = *(const volatile bool *)data ? 1U : 0U; return OD_OK;
#define OD_READ_CASE(od_type, c_type) \
    case od_type: { \
        c_type value = *(const volatile c_type *)data; \
        encode(buffer, (const unsigned char *)&value, sizeof(value)); return OD_OK; }
    OD_SCALAR_TYPES(OD_READ_CASE)
#undef OD_READ_CASE
    default: return OD_TYPE_MISMATCH;
    }
}

OD_Result od_write(OD_Dictionary *d, uint16_t index, uint8_t subindex,
                   const uint8_t *buffer, size_t length)
{
    if (buffer == NULL) return OD_BAD_ARGUMENT;
    size_t slot;
    OD_Result result = check_access(d, index, subindex, length, true, &slot);
    if (result != OD_OK) return result;
    if (d->bindings[slot].write_handler != NULL)
        return d->bindings[slot].write_handler(&d->entries[slot], buffer, length);
    /* Only writable typed binders can authorize this removal of const. */
    volatile void *data = (volatile void *)d->bindings[slot].data;
    if (d->bindings[slot].widen_u8) {
        uint32_t value;
        decode((unsigned char *)&value, buffer, sizeof(value));
        if (value > UINT8_MAX) return OD_VALUE_INVALID;
        *(volatile uint8_t *)data = (uint8_t)value;
        return OD_OK;
    }
    if (d->bindings[slot].native_int) {
        if (d->entries[slot].type == OD_INTEGER32) {
            int value;
            decode((unsigned char *)&value, buffer, sizeof(value));
            *(volatile int *)data = value;
        } else {
            unsigned int value;
            decode((unsigned char *)&value, buffer, sizeof(value));
            *(volatile unsigned int *)data = value;
        }
        return OD_OK;
    }
    switch (d->entries[slot].type) {
    case OD_BOOLEAN:
        if (buffer[0] > 1U) return OD_VALUE_INVALID;
        *(volatile bool *)data = buffer[0] != 0;
        return OD_OK;
#define OD_WRITE_CASE(od_type, c_type) \
    case od_type: { \
        c_type value; decode((unsigned char *)&value, buffer, sizeof(value)); \
        *(volatile c_type *)data = value; return OD_OK; }
    OD_SCALAR_TYPES(OD_WRITE_CASE)
#undef OD_WRITE_CASE
    default: return OD_TYPE_MISMATCH;
    }
}
