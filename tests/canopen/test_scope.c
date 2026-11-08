#include "od_scope.h"
#include "od_wire.h"
#define CHECK(test) do { if (!(test)) return __LINE__; } while (0)

int test_scope(void)
{
    static OD_Scope s;
    const OD_Entry entries[] = {
        {"value", 0x2100, 1, OD_UNSIGNED32, OD_RO, 4},
        {"enum", 0x2100, 2, OD_UNSIGNED32, OD_RO, 4},
        {"wide", 0x2100, 3, OD_UNSIGNED64, OD_RO, 8},
        {"flag", 0x2100, 4, OD_BOOLEAN, OD_RO, 1},
        {"bad", 0x2100, 5, OD_UNSIGNED32, OD_RW, 4}
    };
    OD_Binding bindings[5] = {0};
    OD_Dictionary d = {entries, bindings, 5, false};
    uint32_t value = 0, writable = 0;
    uint8_t narrow = 7;
    uint64_t wide = 1;
    bool flag = true;
    CHECK(od_bind_ro_u32(&d, 0, &value) == OD_OK);
    CHECK(od_bind_ro_u8_as_u32(&d, 1, &narrow) == OD_OK);
    CHECK(od_bind_ro_u64(&d, 2, &wide) == OD_OK);
    CHECK(od_bind_ro_boolean(&d, 3, &flag) == OD_OK);
    CHECK(od_bind_u32(&d, 4, &writable) == OD_OK);
    CHECK(od_freeze(&d, NULL) == OD_OK);
    od_scope_init(&s);
    CHECK(od_scope_start(&s) == OD_NOT_READY);
    OD_ScopeChannel channels[2] = {{0x2100, 1, OD_UNSIGNED32}, {0x2100, 2, OD_UNSIGNED32}};
    CHECK(od_scope_configure(&s, &d, channels, 2, 2, OD_SCOPE_BURST) == OD_OK);
    CHECK(s.stride == 8 && s.samples_per_block == 128);
    CHECK(od_scope_start(&s) == OD_OK);
    CHECK(od_scope_start(&s) == OD_BUSY);
    const unsigned samples = OD_SCOPE_BLOCK_COUNT * 128;
    for (unsigned i = 0; i < samples * 2; ++i) {
        value = i;
        od_scope_tick(&s);
        if (i < samples * 2 - 2) CHECK(od_scope_peek(&s) == NULL);
    }
    CHECK(atomic_load(&s.state) == OD_SCOPE_DRAINING && s.captured == samples);
    CHECK(od_scope_start(&s) == OD_BUSY);
    for (unsigned i = 0; i < OD_SCOPE_BLOCK_COUNT; ++i) {
        const OD_ScopeBlock *b = od_scope_peek(&s);
        CHECK(b != NULL && b->count == 128 && b->first_tick == i * 256);
        for (unsigned j = 0; j < b->count; ++j) {
            CHECK(od_wire_u32(b->data + j * 8) == i * 256 + j * 2);
            CHECK(od_wire_u32(b->data + j * 8 + 4) == 7);
        }
        od_scope_release(&s);
    }
    CHECK(od_scope_empty(&s) && od_scope_peek(&s) == NULL);
    CHECK(od_scope_start(&s) == OD_OK && s.segment == 2);
    od_scope_tick(&s); od_scope_stop(&s);
    CHECK(od_scope_peek(&s)->count == 1 && od_scope_peek(&s)->first_tick == samples * 2);
    od_scope_release(&s);
    CHECK(od_scope_configure(&s, &d, channels, 1, 1, OD_SCOPE_CONTINUOUS) == OD_OK);
    CHECK(od_scope_start(&s) == OD_OK);
    for (unsigned i = 0; i < OD_SCOPE_BLOCK_COUNT * 256 + 1; ++i) od_scope_tick(&s);
    CHECK(atomic_load(&s.state) == OD_SCOPE_OVERFLOW && s.overflows == 1);
    CHECK(s.captured == OD_SCOPE_BLOCK_COUNT * 256);
    while (od_scope_peek(&s) != NULL) od_scope_release(&s);
    CHECK(od_scope_start(&s) == OD_OK);
    /* Many ring wraps with a consumer: published data is never overwritten. */
    for (unsigned block = 0; block < 100; ++block) {
        for (unsigned i = 0; i < 256; ++i) { value = block * 256 + i; od_scope_tick(&s); }
        CHECK(od_scope_peek(&s) != NULL);
        CHECK(od_wire_u32(od_scope_peek(&s)->data) == block * 256);
        od_scope_release(&s);
    }
    od_scope_stop(&s);
    channels[0].subindex = 3; channels[0].type = OD_UNSIGNED64;
    CHECK(od_scope_configure(&s, &d, channels, 1, 1, OD_SCOPE_BURST) == OD_UNSUPPORTED);
    channels[0].subindex = 5; channels[0].type = OD_UNSIGNED32;
    CHECK(od_scope_configure(&s, &d, channels, 1, 1, OD_SCOPE_BURST) == OD_ACCESS_DENIED);
    channels[0].subindex = 1; channels[0].type = OD_REAL32;
    CHECK(od_scope_configure(&s, &d, channels, 1, 1, OD_SCOPE_BURST) == OD_TYPE_MISMATCH);
    channels[0].type = OD_UNSIGNED32;
    CHECK(od_scope_configure(&s, &d, channels, 1, 0, OD_SCOPE_BURST) == OD_BAD_ARGUMENT);
    channels[1] = channels[0];
    CHECK(od_scope_configure(&s, &d, channels, 2, 1, OD_SCOPE_BURST) == OD_BAD_ARGUMENT);
    channels[0].index = 0x2000;
    CHECK(od_scope_configure(&s, &d, channels, 1, 1, OD_SCOPE_BURST) == OD_ACCESS_DENIED);
    return 0;
}
