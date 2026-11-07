#ifndef TEST_OD_CDC_IF_H
#define TEST_OD_CDC_IF_H
#include <stdint.h>
#define USBD_OK 0U
#define USBD_BUSY 1U
uint8_t CDC_Transmit_FS(uint8_t *buffer, uint16_t length);
#endif
