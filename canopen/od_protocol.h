#ifndef CANOPEN_OD_PROTOCOL_H
#define CANOPEN_OD_PROTOCOL_H

#include "od.h"

#define OD_PROTOCOL_LINE_SIZE 96U
#define OD_PROTOCOL_REPLY_SIZE 40U
#define OD_PROTOCOL_MALFORMED 0x80U
#define OD_PROTOCOL_BUSY 0x81U

/* data is a wire-format value. For reads the callback fills it on success. */
typedef uint8_t (*OD_ProtocolAccess)(void *context, bool write, uint16_t index,
                                    uint8_t subindex, OD_Type type, uint8_t *data, size_t length);
typedef struct {
    char input[OD_PROTOCOL_LINE_SIZE];
    uint8_t output[OD_PROTOCOL_REPLY_SIZE];
    size_t input_length;
    size_t output_length;
    bool discard;
    OD_ProtocolAccess access;
    void *context;
} OD_Protocol;

void od_protocol_init(OD_Protocol *protocol, OD_ProtocolAccess access, void *context);
/* Feed only while output_length == 0. Retain output until transport completes. */
void od_protocol_feed(OD_Protocol *protocol, uint8_t byte);

#endif
