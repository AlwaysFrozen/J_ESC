#ifndef CANOPEN_OD_CDC_H
#define CANOPEN_OD_CDC_H

#include "od.h"

/* Foreground initialization/polling. RX callbacks only queue bytes. */
OD_Result od_cdc_init(void);
void od_cdc_poll(void);
/* Called from USB CDC callbacks only. */
void od_cdc_usb_open(void);
void od_cdc_usb_close(void);
void od_cdc_receive(const uint8_t *data, size_t length);
void od_cdc_transmit_complete(void);

#endif
