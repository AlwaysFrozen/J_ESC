"""CSV validation shared with the firmware generator, plus scalar conversion."""
from dataclasses import dataclass
import math
from pathlib import Path
import struct
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from generate_od import load_entries  # noqa: E402

# CANopen identifiers and standard little-endian wire representations.
FORMATS = {
    "BOOLEAN": (0x0001, "B"), "INTEGER8": (0x0002, "b"),
    "INTEGER16": (0x0003, "h"), "INTEGER32": (0x0004, "i"),
    "UNSIGNED8": (0x0005, "B"), "UNSIGNED16": (0x0006, "H"),
    "UNSIGNED32": (0x0007, "I"), "REAL32": (0x0008, "f"),
    "REAL64": (0x0011, "d"), "INTEGER64": (0x0015, "q"),
    "UNSIGNED64": (0x001B, "Q"),
}


@dataclass(frozen=True)
class Entry:
    name: str
    index: int
    subindex: int
    data_type: str
    access: str
    description: str = ""

    @property
    def type_code(self):
        return FORMATS[self.data_type][0]

    @property
    def format(self):
        return "<" + FORMATS[self.data_type][1]

    @property
    def size(self):
        return struct.calcsize(self.format)

    def encode(self, text):
        try:
            if self.data_type.startswith("REAL"):
                value = float(text)
                if not math.isfinite(value):
                    raise ValueError("不接受 NaN 或 Inf")
            else:
                stripped = text.strip()
                value = int(stripped, 16 if stripped.lstrip("+-").lower().startswith("0x") else 10)
                if self.data_type == "BOOLEAN" and value not in (0, 1):
                    raise ValueError("BOOLEAN 只能为 0 或 1")
            return struct.pack(self.format, value)
        except (ValueError, OverflowError, struct.error) as error:
            raise ValueError(f"{self.data_type} 数值无效：{text}（{error}）") from error

    def decode(self, data):
        if len(data) != self.size:
            raise ValueError(f"返回长度错误：需要 {self.size} 字节，实际 {len(data)}")
        value = struct.unpack(self.format, data)[0]
        if self.data_type == "BOOLEAN" and value not in (0, 1):
            raise ValueError("设备返回了无效 BOOLEAN")
        return str(value)


def load_dictionary(path):
    return [Entry(e.name, e.index, e.subindex, e.data_type, e.access, e.description)
            for e in load_entries(path)]
