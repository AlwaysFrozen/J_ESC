"""Bounded v2 COBS/CRC framing, compatible with canopen/od_wire.c."""
from dataclasses import dataclass
import struct
import zlib

VERSION = 2
MAX_PAYLOAD = 1088
RAW_SIZE = 16 + MAX_PAYLOAD + 4
ENCODED_SIZE = RAW_SIZE + RAW_SIZE // 254 + 2
HEADER = struct.Struct("<2sBBHHII")
CAPABILITIES, LEAVE, READ, WRITE = 1, 2, 3, 4
CONFIG, START, STOP, STATUS = 0x10, 0x11, 0x12, 0x13
BENCHMARK, SAMPLES, BENCHMARK_DATA, REPLY = 0x20, 0x40, 0x41, 0x80


def cobs_encode(raw):
    out = bytearray(b"\0")
    code_at, code = 0, 1
    for value in raw:
        if value == 0:
            out[code_at] = code
            code_at, code = len(out), 1
            out.append(0)
        else:
            out.append(value)
            code += 1
            if code == 255:
                out[code_at] = code
                code_at, code = len(out), 1
                out.append(0)
    out[code_at] = code
    return bytes(out)


def cobs_decode(packet):
    out, offset = bytearray(), 0
    while offset < len(packet):
        code = packet[offset]
        offset += 1
        if code == 0 or offset + code - 1 > len(packet):
            raise ValueError("invalid COBS packet")
        out.extend(packet[offset:offset + code - 1])
        offset += code - 1
        if code != 255 and offset < len(packet):
            out.append(0)
    return bytes(out)


@dataclass(frozen=True)
class Frame:
    kind: int
    session: int
    sequence: int
    payload: bytes = b""

    def encode(self):
        if len(self.payload) > MAX_PAYLOAD:
            raise ValueError("payload too large")
        raw = HEADER.pack(b"JE", VERSION, self.kind, len(self.payload), 0, self.session, self.sequence) + self.payload
        raw += struct.pack("<I", zlib.crc32(raw))
        return cobs_encode(raw) + b"\0"

    @classmethod
    def decode(cls, packet):
        raw = cobs_decode(packet)
        if len(raw) < 20 or len(raw) > RAW_SIZE:
            raise ValueError("invalid frame length")
        magic, version, kind, size, flags, session, sequence = HEADER.unpack_from(raw)
        if magic != b"JE" or version != VERSION or flags != 0 or len(raw) != size + 20:
            raise ValueError("invalid frame header")
        if zlib.crc32(raw[:-4]) != struct.unpack_from("<I", raw, len(raw) - 4)[0]:
            raise ValueError("CRC mismatch")
        return cls(kind, session, sequence, raw[16:-4])


class Parser:
    def __init__(self):
        self.packet, self.discard, self.errors = bytearray(), False, 0

    def feed(self, data):
        frames = []
        for value in data:
            if value:
                if not self.discard:
                    if len(self.packet) == ENCODED_SIZE:
                        self.packet.clear()
                        self.discard = True
                    else:
                        self.packet.append(value)
            else:
                if self.discard:
                    self.errors += 1
                elif self.packet:
                    try:
                        frames.append(Frame.decode(self.packet))
                    except ValueError:
                        self.errors += 1
                self.packet.clear()
                self.discard = False
        return frames
