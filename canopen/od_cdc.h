#ifndef CANOPEN_OD_CDC_H
#define CANOPEN_OD_CDC_H

#include "od.h"

/* Foreground initialization/polling. RX callbacks only queue bytes. */
OD_Result od_cdc_init(void);
void od_cdc_poll(void);
/* Called from USB CDC callbacks only. */
void od_cdc_usb_open(void);
void od_cdc_usb_close(void);
/* DTR drop resets parsing only after any hardware-owned TX buffer is drained. */
void od_cdc_session_reset(void);
void od_cdc_receive(const uint8_t *data, size_t length);
void od_cdc_transmit_complete(void);
/* Once per high-rate control cycle, after motor_update_frt(). */
void od_cdc_sample_tick(void);

#endif
