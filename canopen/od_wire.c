#include "od_wire.h"

uint16_t od_wire_u16(const uint8_t *p) { return (uint16_t)(p[0] | (uint16_t)p[1] << 8); }
uint32_t od_wire_u32(const uint8_t *p)
{
    return (uint32_t)p[0] | (uint32_t)p[1] << 8 | (uint32_t)p[2] << 16 | (uint32_t)p[3] << 24;
}
void od_wire_put16(uint8_t *p, uint16_t v) { p[0] = (uint8_t)v; p[1] = (uint8_t)(v >> 8); }
void od_wire_put32(uint8_t *p, uint32_t v)
{
    for (unsigned i = 0; i < 4; ++i) p[i] = (uint8_t)(v >> (8U * i));
}
static uint32_t crc_update(uint32_t crc, const uint8_t *p, size_t length)
{
    for (size_t i = 0; i < length; ++i) {
        crc ^= p[i];
        for (unsigned bit = 0; bit < 8; ++bit)
            crc = (crc >> 1) ^ ((0U - (crc & 1U)) & 0xEDB88320U);
    }
    return crc;
}
uint32_t od_wire_crc32(const uint8_t *p, size_t length)
{
    return ~crc_update(UINT32_MAX, p, length);
}

void od_wire_parser_init(OD_WireParser *p)
{
    p->used = 0; p->discard = false; p->errors = 0;
}

bool od_wire_feed(OD_WireParser *p, uint8_t byte, OD_WireFrame *f)
{
    if (byte != 0) {
        if (!p->discard) {
            if (p->used == sizeof(p->encoded)) { p->discard = true; p->used = 0; }
            else p->encoded[p->used++] = byte;
        }
        return false;
    }
    if (p->discard) { p->discard = false; ++p->errors; return false; }
    if (p->used == 0) return false;
    /* Decode in place: the output never overtakes the input. */
    size_t input = 0, output = 0, length = p->used;
    p->used = 0;
    while (input < length) {
        uint8_t code = p->encoded[input++];
        if (code == 0 || (size_t)(code - 1U) > length - input) goto invalid;
        for (unsigned i = 1; i < code; ++i) p->encoded[output++] = p->encoded[input++];
        if (code != 255 && input < length) p->encoded[output++] = 0;
    }
    if (output < OD_WIRE_HEADER_SIZE + 4U || output > OD_WIRE_RAW_SIZE) goto invalid;
    const uint8_t *raw = p->encoded;
    uint16_t size = od_wire_u16(raw + 4);
    if (raw[0] != 'J' || raw[1] != 'E' || raw[2] != OD_WIRE_VERSION ||
        raw[6] != 0 || raw[7] != 0 || size > OD_WIRE_MAX_PAYLOAD ||
        output != OD_WIRE_HEADER_SIZE + size + 4U ||
        od_wire_crc32(raw, output - 4U) != od_wire_u32(raw + output - 4U)) goto invalid;
    f->type = raw[3]; f->length = size;
    f->session = od_wire_u32(raw + 8); f->sequence = od_wire_u32(raw + 12);
    for (size_t i = 0; i < size; ++i) f->payload[i] = raw[OD_WIRE_HEADER_SIZE + i];
    return true;
invalid:
    ++p->errors;
    return false;
}

size_t od_wire_encode(const OD_WireFrame *f, uint8_t *out, size_t capacity)
{
    /* Require worst-case capacity, so no partial frame can be submitted. */
    if (f == NULL || out == NULL || f->length > OD_WIRE_MAX_PAYLOAD) return 0;
    size_t size = OD_WIRE_HEADER_SIZE + f->length + 4U;
    if (capacity < size + size / 254U + 2U) return 0;
    /* Stream three pieces into COBS instead of putting a 1 KiB raw copy on the
     * foreground stack (which also has to accommodate preempting control ISRs). */
    uint8_t header[OD_WIRE_HEADER_SIZE], checksum[4];
    header[0] = 'J'; header[1] = 'E'; header[2] = OD_WIRE_VERSION; header[3] = f->type;
    od_wire_put16(header + 4, f->length); header[6] = 0; header[7] = 0;
    od_wire_put32(header + 8, f->session); od_wire_put32(header + 12, f->sequence);
    uint32_t crc = crc_update(UINT32_MAX, header, sizeof(header));
    od_wire_put32(checksum, ~crc_update(crc, f->payload, f->length));
    size_t code_at = 0, used = 1;
    uint8_t code = 1;
    for (size_t i = 0; i < size; ++i) {
        uint8_t byte = i < sizeof(header) ? header[i] :
            (i < size - 4U ? f->payload[i - sizeof(header)] : checksum[i - (size - 4U)]);
        if (byte == 0) { out[code_at] = code; code_at = used++; code = 1; }
        else {
            out[used++] = byte;
            if (++code == 255) { out[code_at] = code; code_at = used++; code = 1; }
        }
    }
    out[code_at] = code; out[used++] = 0;
    return used;
}
