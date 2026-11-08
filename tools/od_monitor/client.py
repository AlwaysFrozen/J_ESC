"""Single-request USB CDC protocol client; never retries writes automatically."""
import re
import secrets
import time

STATUS = {
    1: "参数错误", 2: "主索引不存在", 3: "子索引不存在", 4: "权限拒绝",
    5: "长度不匹配", 6: "数据类型不匹配，请检查 CSV", 7: "变量未绑定",
    8: "重复绑定", 9: "映射已冻结", 10: "字典未就绪", 11: "数据值无效",
    12: "控制状态不允许：请检查失能/使能、校准、寻相和当前模式",
    13: "当前电机类型或辨识算法尚不支持此操作",
    14: "操作失败：请读取 Flash 状态及电机故障码",
    15: "参数无效或尚未应用：请失能后应用参数并检查故障码",
    0x80: "协议格式错误", 0x81: "设备忙：电机须失能且处于 IDLE 或 Err",
}


class DeviceError(Exception):
    def __init__(self, status):
        self.status = status
        super().__init__(STATUS.get(status, f"设备错误 0x{status:02X}"))


class ProtocolError(OSError):
    pass


class Client:
    def __init__(self, port, timeout=1.5):
        self.port = port
        self.timeout = timeout
        self.sequence = secrets.randbits(32)
        self.pending = bytearray()

    def exchange(self, entry, data=None, *, timeout=None):
        write = data is not None
        if write and entry.access == "ro":
            raise ValueError("只读条目不能写入")
        if not write and entry.access == "wo":
            raise ValueError("只写条目不能读取")
        if write and len(data) != entry.size:
            raise ValueError("写入长度不匹配")
        self.sequence = (self.sequence + 1) & 0xFFFFFFFF
        sequence = self.sequence
        request = (f"{'W' if write else 'R'} {sequence:08X} {entry.index:04X} "
                   f"{entry.subindex:02X} {entry.type_code:04X} {entry.size:02X}")
        if write:
            request += " " + data.hex().upper()
        self.port.write((request + "\n").encode("ascii"))
        deadline = time.monotonic() + (self.timeout if timeout is None else timeout)
        while time.monotonic() < deadline:
            while b"\n" in self.pending:
                raw, _, rest = self.pending.partition(b"\n")
                self.pending = bytearray(rest)
                match = re.fullmatch(rb"O ([0-9A-Fa-f]{8}) ([0-9A-Fa-f]{2}) ([0-9A-Fa-f]+|-)", raw.rstrip(b"\r"))
                if match is None:
                    continue  # Ignore boot logs or an interrupted session's partial line.
                if int(match[1], 16) != sequence:
                    continue
                status = int(match[2], 16)
                if status:
                    raise DeviceError(status)
                try:
                    payload = b"" if match[3] == b"-" else bytes.fromhex(match[3].decode("ascii"))
                except ValueError as error:
                    raise ProtocolError("响应的十六进制数据无效") from error
                if len(payload) != (0 if write else entry.size):
                    raise ProtocolError("响应长度与操作不匹配")
                return payload
            block = self.port.read(64)
            self.pending.extend(block)
            if len(self.pending) > 4096:
                raise ProtocolError("响应过长或缺少换行符")
        operation = "写入结果未知，请重连并读取确认，勿直接重试" if write else "读取超时，请重新连接"
        raise TimeoutError(operation)

    def read(self, entry):
        return entry.decode(self.exchange(entry))

    def write(self, entry, text, *, timeout=None):
        data = entry.encode(text)
        self.exchange(entry, data, timeout=timeout)
        return entry.decode(data)

    def close(self):
        self.port.close()
