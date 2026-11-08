#include "od_wire.h"
#include "od.h"
#define CHECK(test) do { if (!(test)) return __LINE__; } while (0)

int test_wire(void)
{
    CHECK(od_wire_crc32((const uint8_t *)"123456789", 9) == 0xCBF43926U);
    static OD_WireFrame in, out;
    static OD_WireParser parser;
    uint8_t bytes[OD_WIRE_ENCODED_SIZE];
    od_wire_parser_init(&parser);
    in.type = OD_MSG_SAMPLES; in.session = 0xAB000001; in.sequence = UINT32_MAX;
    for (unsigned size = 0; size <= OD_WIRE_MAX_PAYLOAD; ++size) {
        in.length = (uint16_t)size;
        for (unsigned i = 0; i < size; ++i) in.payload[i] = (uint8_t)i;
        size_t count = od_wire_encode(&in, bytes, sizeof(bytes));
        CHECK(count > 0 && bytes[count - 1] == 0);
        for (size_t i = 0; i < count - 1; ++i) CHECK(!od_wire_feed(&parser, bytes[i], &out));
        CHECK(od_wire_feed(&parser, 0, &out));
        CHECK(out.type == in.type && out.session == in.session && out.sequence == in.sequence && out.length == size);
        for (unsigned i = 0; i < size; ++i) CHECK(out.payload[i] == (uint8_t)i);
    }
    CHECK(parser.errors == 0);
    size_t count = od_wire_encode(&in, bytes, sizeof(bytes));
    bytes[5] ^= 0x80;
    for (size_t i = 0; i < count; ++i) CHECK(!od_wire_feed(&parser, bytes[i], &out));
    CHECK(parser.errors == 1);
    for (unsigned i = 0; i < OD_WIRE_ENCODED_SIZE + 10; ++i) CHECK(!od_wire_feed(&parser, 1, &out));
    CHECK(!od_wire_feed(&parser, 0, &out) && parser.errors == 2);
    count = od_wire_encode(&in, bytes, sizeof(bytes));
    for (size_t i = 0; i < count; ++i) {
        bool complete = od_wire_feed(&parser, bytes[i], &out);
        CHECK(complete == (i == count - 1));
    }
    CHECK(od_wire_encode(&in, bytes, 1) == 0);
    in.length = OD_WIRE_MAX_PAYLOAD + 1;
    CHECK(od_wire_encode(&in, bytes, sizeof(bytes)) == 0);

    const OD_Entry entries[] = {{"value", 0x2100, 1, OD_UNSIGNED32, OD_RO, 4}};
    OD_Binding bindings[1] = {0};
    OD_Dictionary d = {entries, bindings, 1, false};
    volatile uint32_t value = 0x12345678;
    OD_ReadPlan plan = {0};
    CHECK(od_prepare_read(&d, 0x2100, 1, &plan) == OD_NOT_BOUND);
    CHECK(od_bind_ro_u32(&d, 0, &value) == OD_OK);
    CHECK(od_prepare_read(&d, 0x2100, 1, &plan) == OD_NOT_READY);
    CHECK(od_freeze(&d, NULL) == OD_OK);
    CHECK(od_prepare_read(&d, 0x2100, 1, &plan) == OD_OK);
    CHECK(od_read_prepared(&plan, bytes, 4) == OD_OK && od_wire_u32(bytes) == value);
    value = 42;
    CHECK(od_read_prepared(&plan, bytes, 4) == OD_OK && od_wire_u32(bytes) == 42);
    CHECK(od_read_prepared(&plan, bytes, 3) == OD_LENGTH_MISMATCH);
    CHECK(od_prepare_read(NULL, 0, 0, &plan) == OD_BAD_ARGUMENT);
    CHECK(od_prepare_read(&d, 0x2100, 2, &plan) == OD_SUBINDEX_NOT_FOUND);
    return 0;
}
