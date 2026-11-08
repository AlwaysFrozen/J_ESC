#ifndef OD_WIRE_H
#define OD_WIRE_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* v2: COBS(raw header + payload + CRC32), followed by one zero byte.
 * No packed C structs are sent. All multibyte values are little endian. */
#define OD_WIRE_VERSION 2U
#define OD_WIRE_HEADER_SIZE 16U
#define OD_WIRE_MAX_PAYLOAD 1088U
#define OD_WIRE_RAW_SIZE (OD_WIRE_HEADER_SIZE + OD_WIRE_MAX_PAYLOAD + 4U)
#define OD_WIRE_ENCODED_SIZE (OD_WIRE_RAW_SIZE + OD_WIRE_RAW_SIZE / 254U + 2U)

enum {
    OD_MSG_CAPABILITIES = 1, OD_MSG_LEAVE = 2,
    OD_MSG_READ = 3, OD_MSG_WRITE = 4,
    OD_MSG_SCOPE_CONFIG = 0x10, OD_MSG_SCOPE_START = 0x11,
    OD_MSG_SCOPE_STOP = 0x12, OD_MSG_SCOPE_STATUS = 0x13,
    OD_MSG_BENCHMARK = 0x20,
    OD_MSG_MOTION_SUBMIT=0x30, OD_MSG_MOTION_STATUS=0x31,
    OD_MSG_MOTION_HEARTBEAT=0x32, OD_MSG_MOTION_STOP=0x33,
    OD_MSG_SAMPLES = 0x40, OD_MSG_BENCHMARK_DATA = 0x41, OD_MSG_REPLY = 0x80
};

typedef struct {
    uint8_t type;
    uint32_t session, sequence;
    uint16_t length;
    uint8_t payload[OD_WIRE_MAX_PAYLOAD];
} OD_WireFrame;

typedef struct {
    uint8_t encoded[OD_WIRE_ENCODED_SIZE];
    size_t used;
    bool discard;
    uint32_t errors;
} OD_WireParser;

uint16_t od_wire_u16(const uint8_t *p);
uint32_t od_wire_u32(const uint8_t *p);
void od_wire_put16(uint8_t *p, uint16_t v);
void od_wire_put32(uint8_t *p, uint32_t v);
uint32_t od_wire_crc32(const uint8_t *p, size_t length);
void od_wire_parser_init(OD_WireParser *p);
/* Returns true only for a complete validated frame. A bad packet is discarded
 * through its delimiter; the next packet is independently decodable. */
bool od_wire_feed(OD_WireParser *p, uint8_t byte, OD_WireFrame *frame);
size_t od_wire_encode(const OD_WireFrame *frame, uint8_t *out, size_t capacity);

#ifdef __cplusplus
}
#endif

#endif
