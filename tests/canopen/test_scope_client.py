"""Wire/recording tests plus Python -> real C CDC/sampler integration when built."""
import ctypes
import io
import os
from pathlib import Path
import struct
import sys
import tempfile
import time
import unittest
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
from tools.scope_client import wire
from tools.scope_client.client import (BURST, CONTINUOUS, Client, ModePlanner, Recorder,
                                       SampleBlock, read_recording)
from tools.scope_client.__main__ import Collector, acquire, export_recording


def sample_frame(sequence=0, tick=100, segment=1):
    header = struct.pack("<IIIIQIHH", 1, segment, sequence, 20000, tick, 1, 2, 4)
    return wire.Frame(wire.SAMPLES, 9, sequence, header + struct.pack("<ff", 1.5, -2))


class WireTests(unittest.TestCase):
    def test_all_payload_lengths_and_fragmentation(self):
        parser = wire.Parser()
        for size in range(wire.MAX_PAYLOAD + 1):
            frame = wire.Frame(wire.SAMPLES, 0xAB000001, 0xFFFFFFFF,
                               bytes(i % 256 for i in range(size)))
            packet = frame.encode()
            self.assertLessEqual(len(packet), wire.ENCODED_SIZE)
            result = []
            for offset in range(0, len(packet), 7):
                result.extend(parser.feed(packet[offset:offset + 7]))
            self.assertEqual(result, [frame])
        self.assertEqual(parser.errors, 0)

    def test_corruption_oversize_and_recovery(self):
        parser = wire.Parser()
        frame = sample_frame()
        broken = bytearray(frame.encode())
        broken[8] ^= 64
        self.assertEqual(parser.feed(broken), [])
        self.assertEqual(parser.feed(b"x" * (wire.ENCODED_SIZE + 1) + b"\0"), [])
        self.assertEqual(parser.feed(frame.encode() * 2), [frame, frame])
        self.assertEqual(parser.errors, 2)
        with self.assertRaises(ValueError):
            wire.Frame(1, 1, 1, b"x" * (wire.MAX_PAYLOAD + 1)).encode()

    def test_rejects_future_version_even_with_valid_crc(self):
        import zlib
        raw = wire.HEADER.pack(b"JE", 3, 1, 0, 0, 1, 1)
        packet = wire.cobs_encode(raw + struct.pack("<I", zlib.crc32(raw)))
        with self.assertRaises(ValueError):
            wire.Frame.decode(packet)

    def test_mode_budget_and_overflow_latch(self):
        self.assertEqual(ModePlanner().choose(20000, 16), BURST)
        planner = ModePlanner(700000)
        self.assertEqual(planner.choose(20000, 16), CONTINUOUS)
        self.assertEqual(planner.choose(20000, 32), BURST)
        planner.overflowed = True
        self.assertEqual(planner.choose(10000, 4), BURST)
        for invalid in [float("nan"), float("inf"), 0, -1]:
            with self.assertRaises(ValueError):
                ModePlanner(invalid)

    def test_record_roundtrip_truncation_and_crc(self):
        file = io.BytesIO()
        recorder = Recorder(file, {"sample_hz": 20000})
        recorder.write(sample_frame())
        metadata, frames = read_recording(io.BytesIO(file.getvalue()))
        self.assertEqual(metadata["sample_hz"], 20000)
        self.assertEqual(list(frames), [sample_frame()])
        for cut in [1, 5]:
            _, frames = read_recording(io.BytesIO(file.getvalue()[:-cut]))
            with self.assertRaises(ValueError):
                list(frames)
        broken = bytearray(file.getvalue())
        broken[-5] ^= 32
        _, frames = read_recording(io.BytesIO(broken))
        with self.assertRaises(ValueError):
            list(frames)

    def test_collector_detects_time_and_sequence_gaps(self):
        collector = Collector(Recorder(io.BytesIO(), {}), 4, 20000, 1)
        collector.expect(1, 1)
        collector(sample_frame())
        with self.assertRaises(ValueError):
            collector(sample_frame(sequence=2, tick=102))
        with self.assertRaises(ValueError):
            collector(sample_frame(sequence=1, tick=103))
        collector(sample_frame(sequence=1, tick=102))
        self.assertEqual(collector.total_samples, 4)
        bad = sample_frame().payload[:-1]
        with self.assertRaises(ValueError):
            SampleBlock.decode(wire.Frame(wire.SAMPLES, 9, 0, bad))

    def test_timeout_does_not_retry(self):
        class SilentPort:
            def __init__(self): self.writes = []
            def write(self, data): self.writes.append(data)
            def read(self, size): return b""
            def close(self): pass
        port = SilentPort()
        client = Client(port, timeout=0.01)
        client.session = 1
        with self.assertRaises(TimeoutError):
            client.request(wire.WRITE, b"example")
        client.close()
        self.assertEqual(len(port.writes), 1)
        with self.assertRaises(RuntimeError):
            client.request(wire.STOP)

    def test_write_failure_does_not_issue_cleanup_commands(self):
        class FailingPort:
            def __init__(self): self.writes = 0; self.closed = False
            def write(self, data):
                self.writes += 1
                raise OSError("partial transport write")
            def close(self): self.closed = True
        port = FailingPort()
        client = Client(port)
        client.session = 1
        with self.assertRaises(OSError):
            client.request(wire.WRITE, b"example")
        client.close()
        self.assertEqual(port.writes, 1)
        self.assertTrue(port.closed)

    def test_export_refuses_a_recording_without_final_status(self):
        # Stay within the workspace; this test does not require system temp access.
        temporary_root = ROOT / "build/od-tests"
        temporary_root.mkdir(parents=True, exist_ok=True)
        with tempfile.TemporaryDirectory(dir=temporary_root) as temporary:
            source, target = Path(temporary) / "record", Path(temporary) / "export.csv"
            with source.open("wb") as file:
                Recorder(file, {"session": 9, "channels": [{"name": "test", "format": "<f"}]}).write(sample_frame())
            with self.assertRaisesRegex(ValueError, "final capture status"):
                export_recording(source, target)

    def test_export_checks_device_final_sample_count(self):
        from tools.scope_client.client import STATUS_LAYOUT
        temporary_root = ROOT / "build/od-tests"
        temporary_root.mkdir(parents=True, exist_ok=True)
        with tempfile.TemporaryDirectory(dir=temporary_root) as temporary:
            source, target = Path(temporary) / "record", Path(temporary) / "export.csv"
            with source.open("wb") as file:
                recorder = Recorder(file, {"session": 9, "channels": [{"name": "test", "format": "<f"}]})
                recorder.write(sample_frame())
                status = STATUS_LAYOUT.pack(0, BURST, 1, 0, 1, 1, 3, 0, 1, 4, 256)
                recorder.write(wire.Frame(wire.STATUS | wire.REPLY, 9, 1, b"\0" + status))
            with self.assertRaisesRegex(ValueError, "sample count"):
                export_recording(source, target)


class BridgePort:
    def __init__(self, fragment=97, ticks=64, delay=0):
        self.api = ctypes.CDLL(os.environ["J_ESC_TEST_BRIDGE"])
        self.api.test_bridge_open.restype = ctypes.c_int
        self.api.test_bridge_write.argtypes = [ctypes.c_void_p, ctypes.c_uint32]
        self.api.test_bridge_ticks.argtypes = [ctypes.c_uint32]
        self.api.test_bridge_read.argtypes = [ctypes.c_void_p, ctypes.c_uint32]
        self.api.test_bridge_read.restype = ctypes.c_uint32
        self.api.test_bridge_commands.restype = ctypes.c_int
        if self.api.test_bridge_open():
            raise RuntimeError("C bridge init failed")
        self.fragment, self.ticks, self.delay, self.closed = fragment, ticks, delay, False
        self.write(b"!\n")

    def write(self, data):
        self.api.test_bridge_write(ctypes.c_char_p(bytes(data)), len(data))

    def read(self, size):
        if self.delay:
            time.sleep(self.delay)
        self.api.test_bridge_ticks(self.ticks)
        buffer = ctypes.create_string_buffer(min(size, self.fragment))
        count = self.api.test_bridge_read(buffer, len(buffer))
        return buffer.raw[:count]

    def close(self):
        self.closed = True


@unittest.skipUnless(os.environ.get("J_ESC_TEST_BRIDGE"), "build bridge with tests/canopen/run_tests.py")
class FirmwareIntegrationTests(unittest.TestCase):
    def test_negotiation_scalar_short_enum_and_burst(self):
        port = BridgePort(fragment=13)
        client = Client(port)
        caps = client.negotiate()
        self.assertEqual((caps.tick_hz, caps.buffer_bytes, caps.max_channels), (20000, 16384, 16))
        value = client.request(wire.READ, struct.pack("<HBHB", 0x2100, 2, 7, 4))
        self.assertEqual(value, struct.pack("<I", 7))
        status = client.configure([(0x2100, 1, 8), (0x2100, 2, 7)], 2, BURST)
        started = client.start()
        blocks = []
        while len(blocks) < 16:
            frame = client.next_frame()
            self.assertIsNotNone(frame)
            blocks.append(SampleBlock.decode(frame))
        for i, block in enumerate(blocks):
            self.assertEqual((block.config_id, block.segment, block.sequence, block.stride),
                             (status.config_id, started.segment, i, 8))
            self.assertEqual(block.first_tick, blocks[0].first_tick + i * 128 * 2)
            for _, narrow in struct.iter_unpack("<fI", block.data):
                self.assertEqual(narrow, 7)
        client.close()
        self.assertEqual(port.api.test_bridge_commands(), 0)
        self.assertTrue(port.closed)

    def test_native_crc_failure_recovers_without_executing(self):
        port = BridgePort()
        client = Client(port)
        client.negotiate()
        request = wire.Frame(wire.WRITE, client.session, client.sequence + 1,
                             struct.pack("<HBHBI", 0, 1, 7, 4, 0)).encode()
        broken = bytearray(request)
        broken[-3] ^= 64
        port.write(broken)
        port.api.test_bridge_ticks(10)
        client.status()
        self.assertEqual(port.api.test_bridge_commands(), 0)
        # A valid command submitted once increments the real callback exactly once.
        client.request(wire.WRITE, struct.pack("<HBHBI", 0, 1, 7, 4, 0))
        self.assertEqual(port.api.test_bridge_commands(), 1)
        client.close()

    def test_benchmark_and_planner(self):
        port = BridgePort(fragment=4096, delay=0.001)
        client = Client(port)
        client.negotiate()
        throughput = client.measure_throughput(blocks=32, repeats=2)
        self.assertGreater(throughput, 0)
        self.assertEqual(port.api.test_bridge_commands(), 0)
        client.close()

    def test_flash_command_is_rejected_during_capture(self):
        port = BridgePort()
        client = Client(port)
        client.negotiate()
        client.configure([(0x2100, 1, 8)], 1, BURST)
        client.start()
        with self.assertRaisesRegex(RuntimeError, "0x0C"):
            client.request(wire.WRITE, struct.pack("<HBHBI", 0x1000, 2, 7, 4, 1))
        self.assertEqual(port.api.test_bridge_commands(), 0)
        client.close()

    def test_capture_record_export_with_real_firmware(self):
        from argparse import Namespace
        temporary_root = ROOT / "build/od-tests"
        temporary_root.mkdir(parents=True, exist_ok=True)
        with tempfile.TemporaryDirectory(dir=temporary_root) as temporary:
            directory = Path(temporary)
            dictionary = directory / "od.csv"
            dictionary.write_text("name,index,subindex,data_type,access\nvalue,0x2100,0x01,REAL32,ro\n", encoding="utf-8")
            recording, exported = directory / "capture.jescscope", directory / "capture.csv"
            port = BridgePort()
            args = Namespace(csv=str(dictionary), channels="value", seconds=0.04, output=str(recording),
                             port="COM_TEST", rate=10000, mode="burst")
            with patch("tools.scope_client.__main__.SerialPort", return_value=port):
                acquire(args)
            self.assertTrue(port.closed)
            export_recording(recording, exported)
            self.assertIn("device_tick", exported.read_text(encoding="utf-8-sig"))
            with recording.open("rb") as file:
                metadata, frames = read_recording(file)
                samples = [SampleBlock.decode(frame) for frame in frames if frame.kind == wire.SAMPLES]
            self.assertEqual(metadata["sample_hz"], 10000)
            self.assertGreater(sum(block.count for block in samples), 0)
            self.assertEqual(port.api.test_bridge_commands(), 0)


if __name__ == "__main__":
    unittest.main()
