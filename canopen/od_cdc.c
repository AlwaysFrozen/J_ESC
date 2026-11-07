#include "od_cdc.h"
#include "od_protocol.h"
#include "od_user.h"
#include "global.h"
#include "usbd_cdc_if.h"

#define RX_SIZE 512U
static uint8_t rx[RX_SIZE];
static volatile uint16_t rx_head, rx_tail;
static volatile bool rx_overflow, usb_active, tx_inflight;
static volatile uint32_t usb_epoch;
static uint32_t seen_epoch;
static bool initialized;
static OD_Protocol protocol;

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
    unlock(state);
    return result;
}

OD_Result od_cdc_init(void)
{
    OD_Result result = od_user_init(NULL);
    if (result != OD_OK) return result;
    uint32_t state = lock();
    od_protocol_init(&protocol, access_value, &od_dictionary);
    seen_epoch = usb_epoch;
    initialized = true;
    unlock(state);
    return OD_OK;
}

void od_cdc_usb_open(void)
{
    rx_head = rx_tail = 0;
    rx_overflow = false;
    usb_active = true;
    ++usb_epoch;
}

void od_cdc_usb_close(void)
{
    usb_active = false;
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

void od_cdc_poll(void)
{
    if (!initialized) return;
    /* Bound work per main-loop iteration; no waits inside an IRQ or foreground. */
    for (unsigned int budget = 0; budget < 128U; ++budget) {
        uint32_t state = lock();
        if (seen_epoch != usb_epoch) {
            seen_epoch = usb_epoch;
            tx_inflight = false;
            od_protocol_init(&protocol, access_value, &od_dictionary);
        }
        if (!usb_active || tx_inflight) { unlock(state); return; }
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
        }
        if (rx_tail == rx_head) { unlock(state); return; }
        uint8_t byte = rx[rx_tail];
        rx_tail = (uint16_t)((rx_tail + 1U) % RX_SIZE);
        unlock(state);
        od_protocol_feed(&protocol, byte);
    }
}
