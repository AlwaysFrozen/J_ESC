"""Foreground reference client. No motion command is issued by acquisition.

The production Qt client should use asynchronous I/O and bounded worker queues.
This standard-library client exists to measure and validate the firmware first.
"""
from collections import deque
from dataclasses import dataclass
import json
import math
import struct
import time

from . import wire

CONTINUOUS, BURST = 1, 2
RUNNING, DRAINING, OVERFLOW = 1, 2, 3
STATUS_LAYOUT = struct.Struct("<BBBBIIIIIHH")
SAMPLE_LAYOUT = struct.Struct("<IIIIQIHH")
RECORD_MAGIC = b"JESCOPE2\n"


@dataclass(frozen=True)
class Capabilities:
    tick_hz: int
    slow_hz: int
    buffer_bytes: int
    block_bytes: int
    max_channels: int
    features: int
    max_payload: int

    @classmethod
    def decode(cls, payload):
        return cls(*struct.unpack("<IIIHBBH", payload))


@dataclass(frozen=True)
class ScopeStatus:
    state: int
    mode: int
    channels: int
    reserved: int
    config_id: int
    segment: int
    captured: int
    overflows: int
    divisor: int
    stride: int
    samples_per_block: int

    @classmethod
    def decode(cls, payload):
        result = cls(*STATUS_LAYOUT.unpack(payload))
        if result.reserved != 0:
            raise ValueError("unsupported status flags")
        return result


@dataclass(frozen=True)
class SampleBlock:
    config_id: int
    segment: int
    sequence: int
    tick_hz: int
    first_tick: int
    divisor: int
    count: int
    stride: int
    data: bytes

    @classmethod
    def decode(cls, frame):
        if frame.kind != wire.SAMPLES or len(frame.payload) < SAMPLE_LAYOUT.size:
            raise ValueError("invalid sample frame")
        fields = SAMPLE_LAYOUT.unpack_from(frame.payload)
        result = cls(*fields, frame.payload[SAMPLE_LAYOUT.size:])
        if (result.count == 0 or result.stride == 0 or result.divisor == 0 or result.tick_hz == 0
                or len(result.data) != result.count * result.stride or result.sequence != frame.sequence):
            raise ValueError("inconsistent sample block")
        return result


class ModePlanner:
    """Conservative initial bandwidth decision; an overflow latches burst mode.

    Missing measurements always choose burst. No automatic return to continuous
    after an overflow in this milestone, so an unstable link cannot mode-flap.
    """
    def __init__(self, payload_bps=None, margin=0.70):
        if not 0 < margin < 1:
            raise ValueError("margin must be between zero and one")
        if payload_bps is not None and (not math.isfinite(payload_bps) or payload_bps <= 0):
            raise ValueError("throughput must be finite and positive")
        self.payload_bps, self.margin, self.overflowed = payload_bps, margin, False

    def choose(self, sample_hz, stride, block_bytes=1024):
        if sample_hz <= 0 or stride <= 0 or stride > block_bytes:
            raise ValueError("invalid sample budget")
        useful = block_bytes // stride * stride
        required = sample_hz * stride * (1 + 64 / useful)
        if self.overflowed or self.payload_bps is None or required > self.payload_bps * self.margin:
            return BURST
        return CONTINUOUS


class Client:
    def __init__(self, port, timeout=1.5, stream=None):
        self.port, self.timeout, self.stream = port, timeout, stream
        self.parser, self.pending = wire.Parser(), deque()
        self.session, self.sequence, self.failed = 0, 0, False

    def _write(self, data):
        try:
            self.port.write(data)
        except OSError:
            self.failed = True
            raise

    def _read(self, size):
        try:
            return self.port.read(size)
        except OSError:
            self.failed = True
            raise

    def negotiate(self):
        self._write(b"B 00000001\n")
        deadline, line = time.monotonic() + self.timeout, bytearray()
        # Read exactly through the text ACK; never discard following binary bytes.
        while time.monotonic() < deadline:
            byte = self._read(1)
            if not byte:
                continue
            line.extend(byte)
            if len(line) > 96:
                self.failed = True
                raise ValueError("oversized negotiation response")
            if byte == b"\n":
                response = bytes(line).strip()
                line.clear()
                if response == b"O 00000001 00 02":
                    return Capabilities.decode(self.request(wire.CAPABILITIES))
                # SerialPort opens with !\n to terminate a stale text command.
                if response == b"O 00000000 80 -":
                    continue
                self.failed = True
                raise ValueError(f"v2 negotiation rejected: {response!r}")
        self.failed = True
        raise TimeoutError("v2 negotiation timed out; reconnect before retrying")

    def next_frame(self, timeout=None):
        deadline = time.monotonic() + (self.timeout if timeout is None else timeout)
        while True:
            if self.pending:
                frame = self.pending.popleft()
                if self.session and frame.session != self.session:
                    continue
                return frame
            if time.monotonic() >= deadline:
                return None
            frames = self.parser.feed(self._read(4096))
            if len(frames) > 64:
                self.failed = True
                raise ValueError("receive queue limit exceeded")
            self.pending.extend(frames)

    def request(self, kind, payload=b""):
        if self.failed:
            raise RuntimeError("session failed; reconnect before issuing commands")
        if self.sequence == 0xFFFFFFFF:
            raise RuntimeError("request sequence exhausted; reconnect")
        self.sequence += 1
        self._write(wire.Frame(kind, self.session, self.sequence, payload).encode())
        deadline = time.monotonic() + self.timeout
        while time.monotonic() < deadline:
            frame = self.next_frame(max(0, deadline - time.monotonic()))
            if frame is None:
                break
            if frame.kind == (kind | wire.REPLY) and frame.sequence == self.sequence:
                if not frame.payload:
                    raise ValueError("empty command response")
                if self.session == 0:
                    if frame.session == 0:
                        raise ValueError("device supplied a zero session")
                    self.session = frame.session
                if frame.payload[0]:
                    raise RuntimeError(f"device returned status 0x{frame.payload[0]:02X}")
                return frame.payload[1:]
            if self.stream:
                self.stream(frame)
        self.failed = True
        raise TimeoutError("request timed out; result may be unknown; reconnect without retry")

    def configure(self, channels, divisor, mode):
        payload = struct.pack("<BBI", mode, len(channels), divisor)
        for index, subindex, type_id in channels:
            payload += struct.pack("<HBH", index, subindex, type_id)
        return ScopeStatus.decode(self.request(wire.CONFIG, payload))

    def start(self):
        return ScopeStatus.decode(self.request(wire.START))

    def status(self):
        return ScopeStatus.decode(self.request(wire.STATUS))

    def measure_throughput(self, blocks=256, repeats=3):
        if not 1 <= blocks <= 512 or repeats < 1:
            raise ValueError("invalid benchmark size")
        rates = []
        for _ in range(repeats):
            self.request(wire.BENCHMARK, struct.pack("<H", blocks))
            start, total, sequence = time.monotonic(), 0, 0
            while sequence < blocks:
                frame = self.next_frame()
                if frame is None:
                    self.failed = True
                    raise TimeoutError("throughput benchmark stalled")
                if frame.kind != wire.BENCHMARK_DATA or frame.sequence != sequence or len(frame.payload) != 1024:
                    raise ValueError("incomplete throughput benchmark")
                total += len(frame.payload)
                sequence += 1
            elapsed = time.monotonic() - start
            if elapsed < 0.02:
                raise ValueError("benchmark too short to measure reliably")
            rates.append(total / elapsed)
        return min(rates)

    def close(self):
        try:
            if self.session and not self.failed:
                self.request(wire.STOP)
                self.request(wire.LEAVE)
        finally:
            self.port.close()


class Recorder:
    """Append-only bounded records. Each frame retains its own CRC and device time."""
    def __init__(self, file, metadata):
        self.file = file
        header = json.dumps(metadata, ensure_ascii=False, allow_nan=False).encode("utf-8")
        if len(header) > 1024 * 1024:
            raise ValueError("metadata too large")
        file.write(RECORD_MAGIC + struct.pack("<I", len(header)) + header)

    def write(self, frame):
        packet = frame.encode()
        self.file.write(struct.pack("<I", len(packet)) + packet)


def read_recording(file):
    if file.read(len(RECORD_MAGIC)) != RECORD_MAGIC:
        raise ValueError("invalid recording magic")
    def exact(size):
        data = file.read(size)
        if len(data) != size:
            raise ValueError("truncated recording")
        return data
    size = struct.unpack("<I", exact(4))[0]
    if size > 1024 * 1024:
        raise ValueError("metadata too large")
    metadata = json.loads(exact(size))
    def frames():
        while True:
            prefix = file.read(4)
            if not prefix:
                return
            if len(prefix) != 4:
                raise ValueError("truncated record length")
            length = struct.unpack("<I", prefix)[0]
            if not 1 <= length <= wire.ENCODED_SIZE:
                raise ValueError("invalid record length")
            packet = exact(length)
            if packet[-1] != 0:
                raise ValueError("missing frame delimiter")
            yield wire.Frame.decode(packet[:-1])
    return metadata, frames()
