"""Run with python -m tools.scope_client from the repository root."""
import argparse
import csv
from dataclasses import asdict
import hashlib
import math
from pathlib import Path
import struct
import sys
import time

from tools.od_monitor.model import load_dictionary
from tools.od_monitor.serial_win32 import SerialPort, list_ports
from . import wire
from .client import (BURST, CONTINUOUS, OVERFLOW, Capabilities, Client, ModePlanner,
                     Recorder, SampleBlock, ScopeStatus, read_recording)

ROOT = Path(__file__).resolve().parents[2]


class Collector:
    def __init__(self, recorder, stride, tick_hz, divisor):
        self.recorder, self.stride, self.tick_hz, self.divisor = recorder, stride, tick_hz, divisor
        self.config_id, self.segment = None, None
        self.expected_sequence, self.next_tick = 0, None
        self.segment_samples, self.total_samples, self.blocks = 0, 0, 0

    def expect(self, config_id, segment):
        self.config_id, self.segment = config_id, segment
        self.expected_sequence, self.next_tick, self.segment_samples = 0, None, 0

    def __call__(self, frame):
        if frame.kind != wire.SAMPLES:
            return
        block = SampleBlock.decode(frame)
        if (block.config_id != self.config_id or block.segment != self.segment
                or block.sequence != self.expected_sequence or block.stride != self.stride
                or block.tick_hz != self.tick_hz or block.divisor != self.divisor):
            raise ValueError("sample configuration/sequence mismatch; recording is incomplete")
        if self.next_tick is not None and block.first_tick != self.next_tick:
            raise ValueError("sample time gap inside a capture segment")
        self.next_tick = block.first_tick + block.count * block.divisor
        self.expected_sequence += 1
        self.segment_samples += block.count
        self.total_samples += block.count
        self.blocks += 1
        self.recorder.write(frame)


def export_recording(source, destination):
    with open(source, "rb") as file, open(destination, "x", newline="", encoding="utf-8-sig") as output:
        metadata, frames = read_recording(file)
        channels = metadata["channels"]
        row_format = struct.Struct("<" + "".join(item["format"].lstrip("<") for item in channels))
        writer = csv.writer(output)
        writer.writerow(["segment", "device_tick", "device_time_s", *[item["name"] for item in channels]])
        count = 0
        expected, next_tick, previous_segment = 0, None, None
        totals, endings = {}, {}
        for frame in frames:
            if frame.session != metadata["session"]:
                raise ValueError("recording session mismatch")
            if frame.kind == (wire.STATUS | wire.REPLY):
                if not frame.payload or frame.payload[0] != 0:
                    raise ValueError("invalid final capture status")
                status = ScopeStatus.decode(frame.payload[1:])
                if status.state != 0:
                    raise ValueError("capture was not stopped cleanly")
                endings[status.segment] = status
            if frame.kind != wire.SAMPLES:
                continue
            block = SampleBlock.decode(frame)
            if block.stride != row_format.size:
                raise ValueError("recording channel layout mismatch")
            if block.segment != previous_segment:
                expected, next_tick = 0, None
                # Blank row preserves discontinuous capture segments in CSV.
                if previous_segment is not None:
                    writer.writerow([])
                previous_segment = block.segment
            if block.sequence != expected or (next_tick is not None and next_tick != block.first_tick):
                raise ValueError("recording has a missing or reordered block")
            expected += 1
            totals[block.segment] = totals.get(block.segment, 0) + block.count
            next_tick = block.first_tick + block.count * block.divisor
            for i, values in enumerate(row_format.iter_unpack(block.data)):
                tick = block.first_tick + i * block.divisor
                writer.writerow([block.segment, tick, f"{tick / block.tick_hz:.9f}", *values])
                count += 1
        if not endings:
            raise ValueError("recording has no final capture status; it may be interrupted")
        for segment, total in totals.items():
            if segment not in endings or endings[segment].captured != total:
                raise ValueError("recording sample count does not match final device status")
        for segment, status in endings.items():
            if totals.get(segment, 0) != status.captured:
                raise ValueError("recording is missing captured samples")
        if any(status.overflows for status in endings.values()):
            print("记录中包含采集溢出事件；段之间的空隙已保留。", file=sys.stderr)
    print(f"导出 {count} 个采样点到 {destination}")


def acquire(args):
    dictionary = Path(args.csv)
    entries = {entry.name: entry for entry in load_dictionary(dictionary)}
    names = args.channels.split(",")
    try:
        channels = [entries[name.strip()] for name in names]
    except KeyError as error:
        raise ValueError(f"对象字典中没有变量：{error.args[0]}") from error
    if len({entry.name for entry in channels}) != len(channels):
        raise ValueError("通道不能重复")
    for entry in channels:
        if entry.access != "ro" or not 0x2100 <= entry.index <= 0x2109 or entry.subindex == 0 or entry.size > 4:
            raise ValueError(f"不支持采样通道：{entry.name}")
    if not math.isfinite(args.seconds) or args.seconds <= 0:
        raise ValueError("采集时长必须是有限正数")
    with open(args.output, "xb") as output:
        client = Client(SerialPort(args.port))
        try:
            caps: Capabilities = client.negotiate()
            if not 1 <= len(channels) <= caps.max_channels:
                raise ValueError("通道数超出设备能力")
            if args.rate < 10000 or caps.tick_hz % args.rate:
                raise ValueError(f"采样率须至少 10 kHz 且整除设备频率 {caps.tick_hz}")
            divisor = caps.tick_hz // args.rate
            stride = sum(entry.size for entry in channels)
            throughput = None
            if args.mode == "auto":
                print("正在测量当前 CDC 链路吞吐（不会启动电机）…", flush=True)
                throughput = client.measure_throughput()
                print(f"保守实测样本负载吞吐：{throughput / 1000:.1f} kB/s", flush=True)
            planner = ModePlanner(throughput)
            mode = {"continuous": CONTINUOUS, "burst": BURST}.get(args.mode)
            if mode is None:
                mode = planner.choose(args.rate, stride, caps.block_bytes)
            metadata = {
                "format_version": 1, "protocol_version": 2, "session": client.session,
                "capabilities": asdict(caps), "sample_hz": args.rate, "divisor": divisor,
                "dictionary_sha256": hashlib.sha256(dictionary.read_bytes()).hexdigest(),
                "dictionary_verified_by_device": False, "measured_payload_bps": throughput,
                "channels": [dict(name=e.name, index=e.index, subindex=e.subindex,
                                  type=e.type_code, format=e.format, size=e.size) for e in channels],
                "requested_mode": args.mode, "initial_mode": mode,
            }
            recorder = Recorder(output, metadata)
            collector = Collector(recorder, stride, caps.tick_hz, divisor)
            client.stream = collector
            channel_spec = [(entry.index, entry.subindex, entry.type_code) for entry in channels]
            configured = client.configure(channel_spec, divisor, mode)
            deadline = time.monotonic() + args.seconds
            capture_active = False
            while time.monotonic() < deadline:
                started = client.start()
                capture_active = True
                collector.expect(configured.config_id, started.segment)
                print(f"段 {started.segment}：{'连续' if mode == CONTINUOUS else '间断'}，{args.rate} Hz，{len(channels)} 通道", flush=True)
                expected = (caps.buffer_bytes // caps.block_bytes) * configured.samples_per_block
                overflow = False
                last_check = time.monotonic()
                while time.monotonic() < deadline:
                    frame = client.next_frame(0.10)
                    if frame is not None:
                        collector(frame)
                    if mode == BURST and collector.segment_samples == expected:
                        break
                    if mode == CONTINUOUS and time.monotonic() - last_check >= 0.25:
                        status = client.status()
                        last_check = time.monotonic()
                        if status.state == OVERFLOW:
                            overflow = True
                            break
                # Stop publishes any partial block; query final count and drain all
                # records before starting another segment or returning to v1.
                client.request(wire.STOP)
                final = client.status()
                recorder.write(wire.Frame(wire.STATUS | wire.REPLY, client.session,
                                          client.sequence, b"\0" + client_status_bytes(final)))
                drain_deadline = time.monotonic() + 5
                while collector.segment_samples < final.captured and time.monotonic() < drain_deadline:
                    frame = client.next_frame(0.10)
                    if frame is not None:
                        collector(frame)
                if collector.segment_samples != final.captured:
                    raise ValueError("采样段未完整上传；保留文件供检查")
                overflow |= final.overflows > started.overflows
                if overflow:
                    print("连续采样缓存溢出，已停止当前段；设备时间保留了后续采样空隙。", flush=True)
                    if args.mode != "auto":
                        raise RuntimeError("强制连续模式无法持续，请减少通道或使用间断模式")
                    planner.overflowed = True
                    mode = planner.choose(args.rate, stride, caps.block_bytes)
                    configured = client.configure(channel_spec, divisor, mode)
                if mode == CONTINUOUS:
                    break
            if not capture_active:
                raise ValueError("采集时长过短，未启动采样")
            output.flush()
            print(f"已记录 {collector.total_samples} 点、{collector.blocks} 块；帧校验错误 {client.parser.errors}；文件 {args.output}")
        finally:
            client.close()


def client_status_bytes(status):
    from .client import STATUS_LAYOUT
    return STATUS_LAYOUT.pack(*asdict(status).values())


def main():
    parser = argparse.ArgumentParser(description="J_ESC v2 高速采集参考客户端；不发送运动命令")
    commands = parser.add_subparsers(dest="command", required=True)
    commands.add_parser("ports", help="列出 COM 端口")
    capture = commands.add_parser("capture", help="测量吞吐并采集原始数据")
    capture.add_argument("--port", required=True)
    capture.add_argument("--csv", default=str(ROOT / "canopen/object_dictionary.csv"))
    capture.add_argument("--channels", required=True, help="CSV 中变量名称，以逗号分隔")
    capture.add_argument("--rate", type=int, default=20000)
    capture.add_argument("--mode", choices=["auto", "continuous", "burst"], default="auto")
    capture.add_argument("--seconds", type=float, default=10)
    capture.add_argument("--output", required=True, help="新的 .jescscope 文件，不覆盖已有文件")
    export = commands.add_parser("export", help="将原始记录转换为 CSV，间断段之间留空行")
    export.add_argument("source")
    export.add_argument("destination")
    args = parser.parse_args()
    try:
        if args.command == "ports":
            print("\n".join(list_ports()) or "未发现 COM 端口")
        elif args.command == "export":
            export_recording(args.source, args.destination)
        else:
            acquire(args)
    except (OSError, ValueError, RuntimeError, KeyError, struct.error) as error:
        print(f"错误：{error}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
