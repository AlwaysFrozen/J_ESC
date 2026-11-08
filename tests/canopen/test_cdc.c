#include "od_cdc.h"
#include "od_user.h"
#include "global.h"
#include "usbd_cdc_if.h"
#include "od_wire.h"
#include "od_scope.h"
#include "od_motion.h"
OD_Result od_motion_submit(const uint8_t *p,size_t n){(void)p;(void)n;return OD_UNSUPPORTED;}
OD_Result od_motion_heartbeat(void){return OD_UNSUPPORTED;}
void od_motion_cancel(void){}
void od_motion_status(uint8_t p[16]){for(unsigned i=0;i<16;++i)p[i]=0;}

#define CHECK(test) do { if (!(test)) return __LINE__; } while (0)

volatile Global_Status_t global_status;
uint32_t test_primask, test_ipsr;
static int config_changes;
void od_commands_config_changed(void) { ++config_changes; }
static float voltage = 24.5f, speed, limit;
static const uint8_t *transmitted;
static uint16_t transmitted_length;
static bool transport_busy;
static int command_count;
static OD_Result command_result;
static OD_Result execute_command(const OD_Entry *entry, const uint8_t *data, size_t length)
{
    (void)entry; (void)data; (void)length;
    if (test_primask != 0) return OD_BUSY;
    ++command_count;
    return command_result;
}

/* Test fixture initializer only; product mappings are compiled separately. */
OD_Result od_user_init(const OD_Entry **missing)
{
    od_dictionary.frozen = false;
    for (size_t i = 0; i < od_dictionary.count; ++i) od_dictionary.bindings[i].data = NULL;
    OD_Result result = OD_BIND_RO(od_dictionary, bus_voltage, &voltage);
    if (result != OD_OK) return result;
    result = OD_BIND(od_dictionary, motor0_target_speed, &speed);
    if (result != OD_OK) return result;
    result = OD_BIND(od_dictionary, motor0_current_limit, &limit);
    if (result != OD_OK) return result;
    return od_freeze(&od_dictionary, missing);
}

uint8_t CDC_Transmit_FS(uint8_t *buffer, uint16_t length)
{
    if (transport_busy) return USBD_BUSY;
    transmitted = buffer; /* Deliberately retain the pointer, as USB does. */
    transmitted_length = length;
    return USBD_OK;
}

static bool equals(const char *text)
{
    size_t i = 0;
    while (text[i] != '\0') {
        if (i >= transmitted_length || transmitted[i] != (uint8_t)text[i]) return false;
        ++i;
    }
    return i == transmitted_length;
}

static void receive(const char *text)
{
    size_t length = 0;
    while (text[length] != '\0') ++length;
    od_cdc_receive((const uint8_t *)text, length);
}

static void complete(void)
{
    od_cdc_transmit_complete();
    transmitted = NULL;
    transmitted_length = 0;
}

static OD_WireFrame binary_reply;
static OD_WireParser binary_parser;

static void binary_request(uint8_t type, uint32_t session, uint32_t sequence, const uint8_t *data, uint16_t size)
{
    static OD_WireFrame frame;
    static uint8_t packet[OD_WIRE_ENCODED_SIZE];
    frame.type = type; frame.session = session; frame.sequence = sequence; frame.length = size;
    for (unsigned i = 0; i < size; ++i) frame.payload[i] = data[i];
    size_t length = od_wire_encode(&frame, packet, sizeof(packet));
    od_cdc_receive(packet, length);
    for (unsigned i = 0; i < 32 && transmitted == NULL; ++i) od_cdc_poll();
}

static bool decode_transmitted(void)
{
    if (transmitted == NULL) return false;
    od_wire_parser_init(&binary_parser);
    bool ready = false;
    for (unsigned i = 0; i < transmitted_length; ++i)
        if (od_wire_feed(&binary_parser, transmitted[i], &binary_reply)) ready = true;
    return ready;
}

/* Exported only by the host-test DLL link: a serial-like bridge used by Python
 * to exercise the real C parser, CDC scheduler and sampler together. */
int test_bridge_open(void)
{
    static OD_Dictionary original;
    static bool saved;
    if (!saved) { original = od_dictionary; saved = true; }
    od_dictionary = original;
    for (size_t i = 0; i < od_dictionary.count; ++i) od_dictionary.bindings[i] = (OD_Binding){0};
    transmitted = NULL; transmitted_length = 0; transport_busy = false;
    command_count = 0; command_result = OD_OK;
    od_cdc_usb_close();
    OD_Result result = od_cdc_init();
    if (result != OD_OK) return result;
    static const OD_Entry runtime[] = {
        {"enable", 0, 1, OD_UNSIGNED32, OD_WO, 4},
        {"save", 0x1000, 2, OD_UNSIGNED32, OD_WO, 4},
        {"runtime", 0x2100, 1, OD_REAL32, OD_RO, 4},
        {"short_enum", 0x2100, 2, OD_UNSIGNED32, OD_RO, 4}
    };
    static OD_Binding bindings[4];
    static uint8_t narrow;
    for (unsigned i = 0; i < 4; ++i) bindings[i] = (OD_Binding){0};
    voltage = 12.5f; narrow = 7;
    od_dictionary = (OD_Dictionary){runtime, bindings, 4, false};
    result = od_bind_command(&od_dictionary, 0, execute_command);
    if (result != OD_OK) return result;
    result = od_bind_command(&od_dictionary, 1, execute_command);
    if (result != OD_OK) return result;
    result = od_bind_ro_f32(&od_dictionary, 2, &voltage);
    if (result != OD_OK) return result;
    result = od_bind_ro_u8_as_u32(&od_dictionary, 3, &narrow);
    if (result != OD_OK) return result;
    result = od_freeze(&od_dictionary, NULL);
    od_cdc_usb_open(); od_cdc_poll();
    return result;
}

void test_bridge_write(const uint8_t *data, uint32_t size) { od_cdc_receive(data, size); }
void test_bridge_ticks(uint32_t count)
{
    for (uint32_t i = 0; i < count; ++i) { voltage += 0.25f; od_cdc_sample_tick(); }
}
uint32_t test_bridge_read(uint8_t *out, uint32_t capacity)
{
    for (unsigned i = 0; i < 16; ++i) od_cdc_poll();
    if (transmitted == NULL) return 0;
    uint32_t count = transmitted_length < capacity ? transmitted_length : capacity;
    for (uint32_t i = 0; i < count; ++i) out[i] = transmitted[i];
    transmitted += count; transmitted_length = (uint16_t)(transmitted_length - count);
    if (transmitted_length == 0) complete();
    return count;
}
int test_bridge_commands(void) { return command_count; }

static int test_binary_cdc(void)
{
    receive("B 00000010\n");
    od_cdc_poll();
    CHECK(equals("O 00000010 00 02\n"));
    /* Negotiation ACK remains intact until completion. */
    od_cdc_poll(); CHECK(equals("O 00000010 00 02\n"));
    complete(); od_cdc_poll();
    binary_request(OD_MSG_CAPABILITIES, 0, 1, NULL, 0);
    CHECK(decode_transmitted() && binary_reply.type == (OD_MSG_CAPABILITIES | OD_MSG_REPLY));
    CHECK(binary_reply.length == 19 && binary_reply.payload[0] == OD_OK);
    CHECK(od_wire_u32(binary_reply.payload + 1) == BOARD_HRT_FREQ);
    uint32_t session = binary_reply.session;
    CHECK(session != 0); complete();
    uint8_t write[] = {0, 0, 1, OD_UNSIGNED32, 0, 4, 0, 0, 0, 0};
    int previous = command_count;
    command_result = OD_OK;
    transport_busy = true;
    binary_request(OD_MSG_WRITE, session, 2, write, sizeof(write));
    CHECK(transmitted == NULL && command_count == previous + 1);
    transport_busy = false; od_cdc_poll();
    CHECK(decode_transmitted() && binary_reply.payload[0] == OD_OK);
    complete();
    binary_request(OD_MSG_WRITE, session, 2, write, sizeof(write));
    CHECK(transmitted == NULL && command_count == previous + 1);
    binary_request(OD_MSG_WRITE, session + 1, 3, write, sizeof(write));
    CHECK(transmitted == NULL && command_count == previous + 1);
    uint8_t benchmark[] = {2, 0};
    binary_request(OD_MSG_BENCHMARK, session, 3, benchmark, sizeof(benchmark));
    CHECK(decode_transmitted() && binary_reply.payload[0] == OD_OK); complete();
    for (unsigned block = 0; block < 2; ++block) {
        od_cdc_poll(); od_cdc_poll();
        CHECK(decode_transmitted() && binary_reply.type == OD_MSG_BENCHMARK_DATA);
        CHECK(binary_reply.sequence == block && binary_reply.length == 1024);
        for (unsigned i = 0; i < 1024; ++i) CHECK(binary_reply.payload[i] == (uint8_t)i);
        complete();
    }

    /* Temporarily expose a real runtime variable for end-to-end sample transport. */
    OD_Dictionary command_dictionary = od_dictionary;
    const OD_Entry runtime[] = {{"runtime", 0x2100, 1, OD_REAL32, OD_RO, 4}};
    OD_Binding binding[1] = {0};
    od_dictionary = (OD_Dictionary){runtime, binding, 1, false};
    CHECK(od_bind_ro_f32(&od_dictionary, 0, &voltage) == OD_OK);
    CHECK(od_freeze(&od_dictionary, NULL) == OD_OK);
    uint8_t config[] = {OD_SCOPE_BURST, 1, 1, 0, 0, 0, 0, 0x21, 1, OD_REAL32, 0};
    binary_request(OD_MSG_SCOPE_CONFIG, session, 4, config, sizeof(config));
    CHECK(decode_transmitted() && binary_reply.payload[0] == OD_OK); complete();
    binary_request(OD_MSG_SCOPE_START, session, 5, NULL, 0);
    CHECK(decode_transmitted() && binary_reply.payload[0] == OD_OK); complete();
    for (unsigned i = 0; i < OD_SCOPE_BLOCK_COUNT * 256; ++i) od_cdc_sample_tick();
    /* Control response wins over ready sample blocks. */
    binary_request(OD_MSG_SCOPE_STATUS, session, 6, NULL, 0);
    CHECK(decode_transmitted() && binary_reply.type == (OD_MSG_SCOPE_STATUS | OD_MSG_REPLY));
    CHECK(binary_reply.payload[1] == OD_SCOPE_DRAINING); complete();
    for (unsigned block = 0; block < OD_SCOPE_BLOCK_COUNT; ++block) {
        od_cdc_poll(); od_cdc_poll();
        CHECK(decode_transmitted() && binary_reply.type == OD_MSG_SAMPLES);
        CHECK(binary_reply.length == 1056 && od_wire_u32(binary_reply.payload + 8) == block);
        CHECK(od_wire_u16(binary_reply.payload + 28) == 256);
        CHECK(od_wire_u16(binary_reply.payload + 30) == 4);
        CHECK(binary_reply.payload[34] == 0xC4 && binary_reply.payload[35] == 0x41);
        /* USB retains stable bytes even if ticks and poll continue. */
        od_cdc_sample_tick(); od_cdc_poll(); CHECK(decode_transmitted());
        complete();
    }
    binary_request(OD_MSG_LEAVE, session, 7, NULL, 0);
    CHECK(decode_transmitted() && binary_reply.payload[0] == OD_OK); complete(); od_cdc_poll();
    receive("B 00000011\n"); od_cdc_poll(); CHECK(equals("O 00000011 00 02\n")); complete();
    binary_request(OD_MSG_CAPABILITIES, 0, 1, NULL, 0);
    CHECK(decode_transmitted() && binary_reply.session != session);
    uint16_t retained_length = transmitted_length;
    uint8_t retained[64];
    CHECK(retained_length <= sizeof(retained));
    for (unsigned i = 0; i < retained_length; ++i) retained[i] = transmitted[i];
    od_cdc_session_reset(); od_cdc_poll();
    CHECK(transmitted_length == retained_length);
    for (unsigned i = 0; i < retained_length; ++i) CHECK(transmitted[i] == retained[i]);
    complete(); od_cdc_poll();
    od_dictionary = command_dictionary;
    /* DTR/USB session reset recovers v1, including while a stream was configured. */
    receive("W 00000012 0000 01 0007 04 00000000\n"); od_cdc_poll();
    CHECK(equals("O 00000012 00 -\n")); complete();
    return 0;
}

int test_cdc(void)
{
    CHECK(od_cdc_init() == OD_OK);
    od_cdc_usb_open();
    global_status.motor_status[0].motor_state = MS_IDLE;
    receive("R 00000001 2000 ");
    od_cdc_poll();
    CHECK(transmitted == NULL);
    receive("00 0008 04\nR 00000002 2000 00 0008 04\n");
    transport_busy = true;
    od_cdc_poll();
    CHECK(transmitted == NULL);
    transport_busy = false;
    od_cdc_poll();
    CHECK(equals("O 00000001 00 0000C441\n"));
    /* Second queued command cannot overwrite a buffer still owned by USB. */
    od_cdc_poll();
    CHECK(equals("O 00000001 00 0000C441\n"));
    complete();
    od_cdc_poll();
    CHECK(equals("O 00000002 00 0000C441\n"));
    complete();
    global_status.motor_status[0].motor_state = MS_Running;
    receive("W 00000003 2001 00 0008 04 00004841\n");
    od_cdc_poll();
    CHECK(equals("O 00000003 81 -\n") && speed == 0);
    complete();
    /* Runtime monitoring remains available while the motor is running. */
    receive("R 00000030 2000 00 0008 04\n");
    od_cdc_poll();
    CHECK(equals("O 00000030 00 0000C441\n"));
    complete();
    receive("W 00000031 2000 00 0008 04 00000000\n");
    od_cdc_poll();
    CHECK(equals("O 00000031 04 -\n") && voltage == 24.5f);
    complete();
    global_status.motor_status[0].motor_state = MS_IDLE;
    global_status.motor_status[0].is_enabled = 1;
    receive("W 00000004 2001 00 0008 04 00004841\n");
    od_cdc_poll();
    CHECK(equals("O 00000004 81 -\n") && speed == 0);
    complete();
    global_status.motor_status[0].is_enabled = 0;
    global_status.motor_status[0].motor_state = MS_Err;
    receive("W 00000005 2001 00 0008 04 00004841\n");
    od_cdc_poll();
    CHECK(equals("O 00000005 00 -\n") && speed == 12.5f);
    CHECK(config_changes == 1 && test_primask == 0);
    /* USB reset aborts the pending session, even with a reply in flight. */
    od_cdc_usb_close();
    od_cdc_poll();
    transmitted = NULL;
    od_cdc_usb_open();
    receive("R 00000006 2000 00 0008 04\n");
    od_cdc_poll();
    CHECK(equals("O 00000006 00 0000C441\n"));
    complete();
    static uint8_t overflow[2049];
    for (size_t i = 0; i < sizeof(overflow); ++i) overflow[i] = 'x';
    od_cdc_receive(overflow, sizeof(overflow));
    od_cdc_poll();
    receive("!\nR 00000007 2000 00 0008 04\n");
    od_cdc_poll();
    CHECK(equals("O 00000000 80 -\n"));
    complete();
    od_cdc_poll();
    CHECK(equals("O 00000007 00 0000C441\n"));
    complete();
    /* Real WO callback path bypasses the config-only state lock and executes
     * once even if USB is busy or loses the response. */
    OD_Dictionary saved = od_dictionary;
    static const OD_Entry commands[] = {
        {"enable", 0, 1, OD_UNSIGNED32, OD_WO, 4},
        {"save", 0x1000, 2, OD_UNSIGNED32, OD_WO, 4}
    };
    OD_Binding bindings[2] = {0};
    od_dictionary = (OD_Dictionary){commands, bindings, 2, false};
    CHECK(od_bind_command(&od_dictionary, 0, execute_command) == OD_OK);
    CHECK(od_bind_command(&od_dictionary, 1, execute_command) == OD_OK);
    CHECK(od_freeze(&od_dictionary, NULL) == OD_OK);
    global_status.motor_status[0].is_enabled = 1;
    global_status.motor_status[0].motor_state = MS_Running;
    receive("W 00000008 0000 01 0007 04 00000000\n");
    transport_busy = true;
    od_cdc_poll();
    CHECK(command_count == 1 && transmitted == NULL);
    od_cdc_poll();
    CHECK(command_count == 1);
    transport_busy = false;
    od_cdc_poll();
    CHECK(equals("O 00000008 00 -\n") && command_count == 1);
    complete();
    command_result = OD_OPERATION_FAILED;
    receive("W 00000009 1000 02 0007 04 01000000\n");
    od_cdc_poll();
    CHECK(equals("O 00000009 0E -\n") && command_count == 2);
    complete();
    receive("W 0000000A 0000 01 0008 04 00000000\n");
    od_cdc_poll();
    CHECK(equals("O 0000000A 06 -\n") && command_count == 2);
    complete();
    int binary_result = test_binary_cdc();
    if (binary_result != 0) return binary_result;
    od_dictionary = saved;
    return 0;
}
