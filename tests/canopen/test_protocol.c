#include "od_protocol.h"

#define CHECK(test) do { if (!(test)) return __LINE__; } while (0)

typedef struct { OD_Dictionary dictionary; unsigned int writes; bool busy; } Context;

static uint8_t access_value(void *context, bool write, uint16_t index, uint8_t subindex,
                            OD_Type type, uint8_t *data, size_t length)
{
    Context *c = context;
    const OD_Entry *entry = od_find(&c->dictionary, index, subindex);
    if (entry != NULL && entry->type != type) return OD_TYPE_MISMATCH;
    if (write && c->busy) return OD_PROTOCOL_BUSY;
    OD_Result result = write ? od_write(&c->dictionary, index, subindex, data, length) :
                               od_read(&c->dictionary, index, subindex, data, length);
    if (write && result == OD_OK) ++c->writes;
    return (uint8_t)result;
}

static void feed(OD_Protocol *p, const char *text)
{
    while (*text != '\0') od_protocol_feed(p, (uint8_t)*text++);
}

static bool equals(const OD_Protocol *p, const char *text)
{
    size_t i = 0;
    while (text[i] != '\0') {
        if (i >= p->output_length || p->output[i] != (uint8_t)text[i]) return false;
        ++i;
    }
    return i == p->output_length;
}

int test_protocol(void)
{
    const OD_Entry entries[] = {
        {"maximum", 0x2000, 0, OD_UNSIGNED8, OD_RO, 1},
        {"voltage", 0x2000, 3, OD_REAL32, OD_RW, 4},
    };
    OD_Binding bindings[2] = {0};
    Context c = {{entries, bindings, 2, false}, 0, false};
    const uint8_t maximum = 3;
    float voltage = 12.5f;
    CHECK(od_bind_ro_u8(&c.dictionary, 0, &maximum) == OD_OK);
    CHECK(od_bind_f32(&c.dictionary, 1, &voltage) == OD_OK);
    CHECK(od_freeze(&c.dictionary, NULL) == OD_OK);
    OD_Protocol p;
    od_protocol_init(&p, access_value, &c);
    feed(&p, "R 00000001 2000 ");
    CHECK(p.output_length == 0);
    feed(&p, "00 0005 01\r\n");
    CHECK(equals(&p, "O 00000001 00 03\n"));
    /* Output backpressure: input must wait until transport drains the reply. */
    feed(&p, "ignored");
    CHECK(equals(&p, "O 00000001 00 03\n"));
    p.output_length = 0;
    feed(&p, "R 00000002 2000 03 0008 04\n");
    CHECK(equals(&p, "O 00000002 00 00004841\n"));
    p.output_length = 0;
    feed(&p, "W 00000003 2000 03 0008 04 0000C841");
    CHECK(c.writes == 0 && voltage == 12.5f);
    feed(&p, "\n");
    CHECK(equals(&p, "O 00000003 00 -\n") && voltage == 25.0f && c.writes == 1);
    p.output_length = 0;
    feed(&p, "W 00000004 2000 00 0005 01 00\n");
    CHECK(equals(&p, "O 00000004 04 -\n") && maximum == 3 && c.writes == 1);
    p.output_length = 0;
    feed(&p, "R 00000005 2000 03 0007 04\n");
    CHECK(equals(&p, "O 00000005 06 -\n"));
    p.output_length = 0;
    feed(&p, "R 00000006 2000 03 0008 01\n");
    CHECK(equals(&p, "O 00000006 05 -\n"));
    p.output_length = 0;
    feed(&p, "R 00000007 2001 03 0008 04\n");
    CHECK(equals(&p, "O 00000007 02 -\n"));
    p.output_length = 0;
    feed(&p, "R 00000008 2000 01 0008 04\n");
    CHECK(equals(&p, "O 00000008 03 -\n"));
    p.output_length = 0;
    feed(&p, "W 00000009 2000 03 0008 04 BAD_HEXX\n");
    CHECK(equals(&p, "O 00000009 80 -\n") && c.writes == 1);
    p.output_length = 0;
    feed(&p, "W 0000000A 2000 03 0008 04 00000000!\n");
    CHECK(equals(&p, "O 0000000A 80 -\n") && voltage == 25.0f);
    p.output_length = 0;
    for (unsigned int i = 0; i < OD_PROTOCOL_LINE_SIZE + 20U; ++i) od_protocol_feed(&p, 'x');
    feed(&p, "\n");
    CHECK(equals(&p, "O 00000000 80 -\n"));
    p.output_length = 0;
    c.busy = true;
    feed(&p, "W 0000000B 2000 03 0008 04 00000000\n");
    CHECK(equals(&p, "O 0000000B 81 -\n") && voltage == 25.0f && c.writes == 1);
    od_protocol_init(&p, NULL, NULL);
    feed(&p, "R 0000000C 2000 03 0008 04\n");
    CHECK(equals(&p, "O 0000000C 0A -\n"));
    return 0;
}
