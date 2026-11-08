#include "od_cdc.h"
#include "od_protocol.h"
#include "od_user.h"
#include "od_commands.h"
#include "global.h"
#include "usbd_cdc_if.h"
#include "od_wire.h"
#include "od_scope.h"
#include "od_motion.h"

#define RX_SIZE 2048U
static uint8_t rx[RX_SIZE];
static volatile uint16_t rx_head, rx_tail;
static volatile bool rx_overflow, usb_active, tx_inflight;
static volatile uint32_t usb_epoch;
static volatile uint32_t usb_tx_epoch;
static uint32_t seen_epoch;
static uint32_t seen_tx_epoch;
static bool initialized;
static OD_Protocol protocol;
static OD_WireParser wire_parser;
static OD_WireFrame wire_input, wire_output;
static uint8_t wire_tx[OD_WIRE_ENCODED_SIZE];
static size_t wire_tx_length;
static bool binary_mode, leave_binary;
static uint32_t last_sequence, session_id;
static uint16_t benchmark_blocks, benchmark_sequence;
static OD_Scope scope;

void od_cdc_sample_tick(void) { od_scope_tick(&scope); }

static uint32_t lock(void)
{
    uint32_t state = __get_PRIMASK();
    __disable_irq();
    return state;
}

static void unlock(uint32_t state) { __set_PRIMASK(state); }

static uint8_t access_value(void *context, bool write, uint16_t index,
                            uint8_t subindex, OD_Type type, uint8_t *data, size_t length)
{
    OD_Dictionary *dictionary = context;
    const OD_Entry *entry = od_find(dictionary, index, subindex);
    if (entry != NULL && entry->type != type) return OD_TYPE_MISMATCH;
    if (write && index == 0x1000 && (subindex == 2 || subindex == 3) &&
        (atomic_load_explicit(&scope.state, memory_order_acquire) == OD_SCOPE_RUNNING ||
         !od_scope_empty(&scope) || benchmark_blocks != 0)) return OD_BUSY;
    if (write && od_is_command(dictionary, index, subindex))
        return (uint8_t)od_write(dictionary, index, subindex, data, length);
    /* The check and scalar access share one short critical section, so a
     * control ISR cannot enable the motor between the state check and store. */
    uint32_t state = lock();
    uint8_t result;
    if (write && entry != NULL && (entry->access & OD_WO) != 0) {
        for (unsigned int i = 0; i < BOARD_SUPPORTED_MOTOR_NUMBER; ++i) {
            const volatile Motor_Status_t *motor = &global_status.motor_status[i];
            if (motor->is_enabled || (motor->motor_state != MS_IDLE && motor->motor_state != MS_Err)) {
                unlock(state);
                return OD_PROTOCOL_BUSY;
            }
        }
    }
    result = (uint8_t)(write ? od_write(dictionary, index, subindex, data, length) :
                              od_read(dictionary, index, subindex, data, length));
    if (write && result == OD_OK && index >= 0x2000 && index <= 0x2006)
        od_commands_config_changed();
    unlock(state);
    return result;
}

OD_Result od_cdc_init(void)
{
    OD_Result result = od_user_init(NULL);
    if (result != OD_OK) return result;
    uint32_t state = lock();
    od_protocol_init(&protocol, access_value, &od_dictionary);
    od_wire_parser_init(&wire_parser);
    od_scope_init(&scope);
    binary_mode = false; leave_binary = false; wire_tx_length = 0; last_sequence = 0;
    benchmark_blocks = 0;
    seen_epoch = usb_epoch;
    seen_tx_epoch = usb_tx_epoch;
    initialized = true;
    unlock(state);
    return OD_OK;
}

void od_cdc_usb_open(void)
{
    atomic_store_explicit(&scope.state, OD_SCOPE_IDLE, memory_order_release);
    rx_head = rx_tail = 0;
    rx_overflow = false;
    usb_active = true;
    ++usb_epoch;
    ++usb_tx_epoch;
}

void od_cdc_usb_close(void)
{
    od_motion_cancel();
    atomic_store_explicit(&scope.state, OD_SCOPE_IDLE, memory_order_release);
    usb_active = false;
    rx_head = rx_tail = 0;
    rx_overflow = false;
    ++usb_epoch;
    ++usb_tx_epoch;
}

void od_cdc_session_reset(void)
{
    od_motion_cancel();
    atomic_store_explicit(&scope.state, OD_SCOPE_IDLE, memory_order_release);
    rx_head = rx_tail = 0;
    rx_overflow = false;
    ++usb_epoch;
}

void od_cdc_receive(const uint8_t *data, size_t length)
{
    if (!usb_active || rx_overflow) return;
    uint16_t head = rx_head;
    size_t available = (rx_tail + RX_SIZE - head - 1U) % RX_SIZE;
    if (length > available) { rx_overflow = true; return; }
    for (size_t i = 0; i < length; ++i) {
        rx[head] = data[i];
        head = (uint16_t)((head + 1U) % RX_SIZE);
    }
    __DMB();
    rx_head = head;
}

void od_cdc_transmit_complete(void) { tx_inflight = false; }

static void scope_status(uint8_t *out)
{
    uint32_t state = lock();
    out[0] = (uint8_t)atomic_load_explicit(&scope.state, memory_order_acquire);
    out[1] = scope.mode; out[2] = scope.channels; out[3] = 0;
    od_wire_put32(out + 4, scope.config_id);
    od_wire_put32(out + 8, scope.segment);
    od_wire_put32(out + 12, scope.captured);
    od_wire_put32(out + 16, scope.overflows);
    od_wire_put32(out + 20, scope.divisor);
    od_wire_put16(out + 24, scope.stride);
    od_wire_put16(out + 26, scope.samples_per_block);
    unlock(state);
}

static void process_binary(void)
{
    const OD_WireFrame *in = &wire_input;
    /* Only a capability query may bootstrap with session zero. Requests are
     * strictly increasing in a session: duplicates never execute a command twice.
     * Reconnect before sequence wrap; no automatic non-idempotent retry. */
    if ((in->session != session_id && !(in->type == OD_MSG_CAPABILITIES && in->session == 0)) ||
        in->sequence == 0 || in->sequence <= last_sequence || (in->type & OD_MSG_REPLY) != 0) return;
    last_sequence = in->sequence;
    wire_output.type = in->type | OD_MSG_REPLY;
    wire_output.session = session_id; wire_output.sequence = in->sequence;
    wire_output.length = 1;
    uint8_t *out = wire_output.payload;
    const uint8_t *data = in->payload;
    out[0] = OD_LENGTH_MISMATCH;
    switch (in->type) {
    case OD_MSG_MOTION_SUBMIT:
        if(benchmark_blocks){out[0]=OD_BUSY;break;}
        out[0]=(uint8_t)od_motion_submit(data,in->length);
        if(out[0]==OD_OK){od_motion_status(out+1);wire_output.length=17;}break;
    case OD_MSG_MOTION_STATUS:
        if(in->length)break;
        out[0]=OD_OK;od_motion_status(out+1);wire_output.length=17;break;
    case OD_MSG_MOTION_HEARTBEAT:
        if(in->length)break;
        out[0]=(uint8_t)od_motion_heartbeat();break;
    case OD_MSG_MOTION_STOP:
        if(in->length)break;
        od_motion_cancel();out[0]=OD_OK;break;
    case OD_MSG_BENCHMARK:
        if (in->length != 2) break;
        if (atomic_load_explicit(&scope.state, memory_order_acquire) == OD_SCOPE_RUNNING ||
            !od_scope_empty(&scope) || benchmark_blocks != 0) { out[0] = OD_BUSY; break; }
        benchmark_blocks = od_wire_u16(data);
        if (benchmark_blocks == 0 || benchmark_blocks > 512) {
            benchmark_blocks = 0; out[0] = OD_VALUE_INVALID; break;
        }
        benchmark_sequence = 0; out[0] = OD_OK;
        break;
    case OD_MSG_CAPABILITIES:
        if (in->length != 0) break;
        out[0] = OD_OK;
        od_wire_put32(out + 1, BOARD_HRT_FREQ);
        od_wire_put32(out + 5, BOARD_RT_FREQ);
        od_wire_put32(out + 9, OD_SCOPE_BLOCK_COUNT * OD_SCOPE_BLOCK_BYTES);
        od_wire_put16(out + 13, OD_SCOPE_BLOCK_BYTES);
        out[15] = OD_SCOPE_MAX_CHANNELS;
        /* Bits: continuous, burst, <=32-bit runtime values, benchmark, motion.
         * Trigger and automatic planning are not device capabilities here. */
        out[16] = 31;
        od_wire_put16(out + 17, OD_WIRE_MAX_PAYLOAD);
        wire_output.length = 19;
        break;
    case OD_MSG_LEAVE:
        if (in->length != 0) break;
        { uint32_t state = lock(); od_scope_stop(&scope); unlock(state); }
        benchmark_blocks = 0;
        od_motion_cancel();
        leave_binary = true; out[0] = OD_OK;
        break;
    case OD_MSG_READ:
    case OD_MSG_WRITE:
        if (in->length < 6) break;
        {
            bool write = in->type == OD_MSG_WRITE;
            uint8_t size = data[5];
            if (size == 0 || size > 8 || in->length != (write ? 6U + size : 6U)) break;
            uint8_t value[8] = {0};
            if (write) for (unsigned i = 0; i < size; ++i) value[i] = data[6 + i];
            out[0] = access_value(&od_dictionary, write, od_wire_u16(data), data[2],
                                  (OD_Type)od_wire_u16(data + 3), value, size);
            if (!write && out[0] == OD_OK) {
                for (unsigned i = 0; i < size; ++i) out[1 + i] = value[i];
                wire_output.length = (uint16_t)(1U + size);
            }
        }
        break;
    case OD_MSG_SCOPE_CONFIG:
        if (benchmark_blocks != 0) { out[0] = OD_BUSY; break; }
        if (in->length < 6) break;
        {
            uint8_t count = data[1];
            if (count == 0 || count > OD_SCOPE_MAX_CHANNELS || in->length != 6U + count * 5U) break;
            uint32_t divisor = od_wire_u32(data + 2);
            /* First milestone only accepts >=10kHz exact integer divisors. */
            if (divisor == 0 || divisor > BOARD_HRT_FREQ / 10000U) { out[0] = OD_VALUE_INVALID; break; }
            OD_ScopeChannel channels[OD_SCOPE_MAX_CHANNELS];
            for (unsigned i = 0; i < count; ++i) {
                const uint8_t *c = data + 6 + i * 5;
                channels[i] = (OD_ScopeChannel){od_wire_u16(c), c[2], (OD_Type)od_wire_u16(c + 3)};
            }
            /* Configure only when stopped and empty. Validation/lookup is outside
             * the IRQ lock; there is no concurrent foreground starter. */
            out[0] = (uint8_t)od_scope_configure(&scope, &od_dictionary, channels, count, divisor, data[0]);
            if (out[0] == OD_OK) { scope_status(out + 1); wire_output.length = 29; }
        }
        break;
    case OD_MSG_SCOPE_START:
        if (benchmark_blocks != 0) { out[0] = OD_BUSY; break; }
        if (in->length != 0) break;
        {
            uint32_t state = lock();
            out[0] = (uint8_t)od_scope_start(&scope);
            unlock(state);
            if (out[0] == OD_OK) { scope_status(out + 1); wire_output.length = 29; }
        }
        break;
    case OD_MSG_SCOPE_STOP:
        if (in->length != 0) break;
        { uint32_t state = lock(); od_scope_stop(&scope); unlock(state); }
        benchmark_blocks = 0;
        out[0] = OD_OK;
        break;
    case OD_MSG_SCOPE_STATUS:
        if (in->length != 0) break;
        out[0] = OD_OK; scope_status(out + 1); wire_output.length = 29;
        break;
    default: out[0] = OD_UNSUPPORTED; break;
    }
    wire_tx_length = od_wire_encode(&wire_output, wire_tx, sizeof(wire_tx));
}

static void prepare_samples(void)
{
    if (benchmark_blocks != 0) {
        wire_output.type = OD_MSG_BENCHMARK_DATA; wire_output.session = session_id;
        wire_output.sequence = benchmark_sequence++;
        wire_output.length = OD_SCOPE_BLOCK_BYTES;
        for (unsigned i = 0; i < OD_SCOPE_BLOCK_BYTES; ++i) wire_output.payload[i] = (uint8_t)i;
        --benchmark_blocks;
        wire_tx_length = od_wire_encode(&wire_output, wire_tx, sizeof(wire_tx));
        return;
    }
    const OD_ScopeBlock *block = od_scope_peek(&scope);
    if (block == NULL) return;
    wire_output.type = OD_MSG_SAMPLES; wire_output.session = session_id;
    wire_output.sequence = scope.block_sequence;
    uint8_t *out = wire_output.payload;
    od_wire_put32(out, scope.config_id); od_wire_put32(out + 4, scope.segment);
    od_wire_put32(out + 8, scope.block_sequence); od_wire_put32(out + 12, BOARD_HRT_FREQ);
    od_wire_put32(out + 16, (uint32_t)block->first_tick);
    od_wire_put32(out + 20, (uint32_t)(block->first_tick >> 32));
    od_wire_put32(out + 24, scope.divisor);
    od_wire_put16(out + 28, block->count); od_wire_put16(out + 30, scope.stride);
    size_t size = (size_t)block->count * scope.stride;
    for (size_t i = 0; i < size; ++i) out[32 + i] = block->data[i];
    wire_output.length = (uint16_t)(32U + size);
    wire_tx_length = od_wire_encode(&wire_output, wire_tx, sizeof(wire_tx));
    /* wire_tx now owns a copy until USB completion, including USBD_BUSY retries. */
    od_scope_release(&scope);
}

void od_cdc_poll(void)
{
    if (!initialized) return;
    /* Bound work per main-loop iteration; no waits inside an IRQ or foreground. */
    for (unsigned int budget = 0; budget < 128U; ++budget) {
        uint32_t state = lock();
        if (seen_epoch != usb_epoch) {
            /* DTR close is not an endpoint reset. The old TX storage remains
             * owned by USB until completion; physical reset cancels ownership. */
            if (tx_inflight && seen_tx_epoch == usb_tx_epoch) { unlock(state); return; }
            seen_epoch = usb_epoch;
            seen_tx_epoch = usb_tx_epoch;
            tx_inflight = false;
            od_protocol_init(&protocol, access_value, &od_dictionary);
            od_wire_parser_init(&wire_parser);
            od_scope_init(&scope);
            binary_mode = false; leave_binary = false; wire_tx_length = 0; last_sequence = 0;
            benchmark_blocks = 0;
        }
        if (!usb_active || tx_inflight) { unlock(state); return; }
        if (!binary_mode && protocol.enter_binary && protocol.output_length == 0) {
            binary_mode = true;
            if (++session_id == 0) ++session_id;
            last_sequence = 0;
            od_wire_parser_init(&wire_parser);
        }
        if (binary_mode && leave_binary && wire_tx_length == 0) {
            binary_mode = false; leave_binary = false;
            od_protocol_init(&protocol, access_value, &od_dictionary);
        }
        if (binary_mode && wire_tx_length != 0) {
            tx_inflight = true;
            uint8_t status = CDC_Transmit_FS(wire_tx, (uint16_t)wire_tx_length);
            if (status == USBD_OK) wire_tx_length = 0;
            else tx_inflight = false;
            unlock(state);
            return;
        }
        if (protocol.output_length != 0) {
            /* Keep protocol.output intact until the completion callback.
             * Prevent disconnect/completion races during USB submission. */
            tx_inflight = true;
            uint8_t status = CDC_Transmit_FS(protocol.output, (uint16_t)protocol.output_length);
            if (status == USBD_OK) protocol.output_length = 0;
            else tx_inflight = false; /* Preserve pending reply and retry later. */
            unlock(state);
            return;
        }
        if (rx_overflow) {
            rx_tail = rx_head;
            rx_overflow = false;
            protocol.input_length = 0;
            protocol.discard = true; /* Resynchronize at a subsequent newline. */
            wire_parser.used = 0; wire_parser.discard = true;
            /* Fail the capture visibly on input overflow; never execute a suffix. */
            od_scope_stop(&scope);
        }
        if (rx_tail == rx_head) {
            unlock(state);
            if (binary_mode) prepare_samples();
            /* Keep packet preparation outside the IRQ lock. Submit on next poll. */
            return;
        }
        uint8_t byte = rx[rx_tail];
        rx_tail = (uint16_t)((rx_tail + 1U) % RX_SIZE);
        unlock(state);
        if (binary_mode) {
            if (od_wire_feed(&wire_parser, byte, &wire_input)) process_binary();
        } else od_protocol_feed(&protocol, byte);
    }
}
