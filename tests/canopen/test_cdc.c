#include "od_cdc.h"
#include "od_user.h"
#include "global.h"
#include "usbd_cdc_if.h"

#define CHECK(test) do { if (!(test)) return __LINE__; } while (0)

volatile Global_Status_t global_status;
static float voltage = 24.5f, speed, limit;
static const uint8_t *transmitted;
static uint16_t transmitted_length;
static bool transport_busy;

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
    /* USB reset aborts the pending session, even with a reply in flight. */
    od_cdc_usb_close();
    od_cdc_poll();
    transmitted = NULL;
    od_cdc_usb_open();
    receive("R 00000006 2000 00 0008 04\n");
    od_cdc_poll();
    CHECK(equals("O 00000006 00 0000C441\n"));
    complete();
    uint8_t overflow[513];
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
    return 0;
}
