#include "od_protocol.h"

static bool parse_hex(const char *text, size_t length, uint32_t *value)
{
    *value = 0;
    for (size_t i = 0; i < length; ++i) {
        unsigned int digit;
        char c = text[i];
        if (c >= '0' && c <= '9') digit = (unsigned int)(c - '0');
        else if (c >= 'A' && c <= 'F') digit = (unsigned int)(c - 'A') + 10U;
        else if (c >= 'a' && c <= 'f') digit = (unsigned int)(c - 'a') + 10U;
        else return false;
        *value = (*value << 4) | digit;
    }
    return true;
}

static void print_hex(uint8_t *text, uint32_t value, size_t length)
{
    static const char digits[] = "0123456789ABCDEF";
    while (length != 0) {
        text[--length] = (uint8_t)digits[value & 15U];
        value >>= 4;
    }
}

static void reply(OD_Protocol *p, uint32_t sequence, uint8_t status,
                  const uint8_t *data, size_t length)
{
    p->output[0] = 'O'; p->output[1] = ' ';
    print_hex(&p->output[2], sequence, 8);
    p->output[10] = ' ';
    print_hex(&p->output[11], status, 2);
    p->output[13] = ' ';
    size_t size = 14;
    if (length == 0) p->output[size++] = '-';
    for (size_t i = 0; i < length; ++i) {
        print_hex(&p->output[size], data[i], 2);
        size += 2;
    }
    p->output[size++] = '\n';
    p->output_length = size;
}

static void process_line(OD_Protocol *p)
{
    size_t size = p->input_length;
    const char *s = p->input;
    uint32_t sequence = 0, index, subindex, type, length, value;
    uint8_t data[8];
    bool write = s[0] == 'W';
    /* Even malformed commands echo their sequence if it can be recovered. */
    if (size >= 10 && !parse_hex(&s[2], 8, &sequence)) sequence = 0;
    if (size < 26 || (s[0] != 'R' && !write) ||
        s[1] != ' ' || s[10] != ' ' || s[15] != ' ' || s[18] != ' ' || s[23] != ' ' ||
        !parse_hex(&s[2], 8, &sequence) || !parse_hex(&s[11], 4, &index) ||
        !parse_hex(&s[16], 2, &subindex) || !parse_hex(&s[19], 4, &type) ||
        !parse_hex(&s[24], 2, &length) || length == 0 || length > sizeof(data)) {
        reply(p, sequence, OD_PROTOCOL_MALFORMED, NULL, 0);
        return;
    }
    if ((!write && size != 26) || (write && (size != 27 + length * 2 || s[26] != ' '))) {
        reply(p, sequence, OD_PROTOCOL_MALFORMED, NULL, 0);
        return;
    }
    if (write) {
        for (size_t i = 0; i < length; ++i) {
            if (!parse_hex(&s[27 + i * 2], 2, &value)) {
                reply(p, sequence, OD_PROTOCOL_MALFORMED, NULL, 0);
                return;
            }
            data[i] = (uint8_t)value;
        }
    }
    uint8_t status = p->access == NULL ? OD_NOT_READY :
        p->access(p->context, write, (uint16_t)index, (uint8_t)subindex, (OD_Type)type, data, length);
    reply(p, sequence, status, data, status == OD_OK && !write ? length : 0);
}

void od_protocol_init(OD_Protocol *p, OD_ProtocolAccess access, void *context)
{
    p->input_length = 0;
    p->output_length = 0;
    p->discard = false;
    p->access = access;
    p->context = context;
}

void od_protocol_feed(OD_Protocol *p, uint8_t byte)
{
    if (p->output_length != 0) return;
    if (byte == '\n') {
        if (p->discard) reply(p, 0, OD_PROTOCOL_MALFORMED, NULL, 0);
        else if (p->input_length != 0) {
            if (p->input[p->input_length - 1] == '\r') --p->input_length;
            if (p->input_length != 0) process_line(p);
        }
        p->input_length = 0;
        p->discard = false;
    } else if (!p->discard) {
        if (p->input_length == sizeof(p->input)) {
            p->input_length = 0;
            p->discard = true;
        } else p->input[p->input_length++] = (char)byte;
    }
}
