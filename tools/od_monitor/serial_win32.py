"""Blocking Windows COM port using only the Python standard library.

All I/O belongs to the GUI's background worker, never to the Tk thread.
"""
import ctypes
from ctypes import wintypes as w
import os
import re


class DCB(ctypes.Structure):
    _fields_ = [("length", w.DWORD), ("baud", w.DWORD), ("flags", w.DWORD),
                ("reserved", w.WORD), ("xon_limit", w.WORD), ("xoff_limit", w.WORD),
                ("byte_size", w.BYTE), ("parity", w.BYTE), ("stop_bits", w.BYTE),
                ("xon", ctypes.c_char), ("xoff", ctypes.c_char), ("error", ctypes.c_char),
                ("eof", ctypes.c_char), ("event", ctypes.c_char), ("reserved1", w.WORD)]


class Timeouts(ctypes.Structure):
    _fields_ = [(name, w.DWORD) for name in ("interval", "multiplier", "constant", "write_multiplier", "write_constant")]


def list_ports():
    if os.name != "nt":
        return []
    import winreg
    ports = set()
    try:
        with winreg.OpenKey(winreg.HKEY_LOCAL_MACHINE, r"HARDWARE\DEVICEMAP\SERIALCOMM") as key:
            for number in range(winreg.QueryInfoKey(key)[1]):
                port = winreg.EnumValue(key, number)[1]
                if re.fullmatch(r"COM\d+", port, re.IGNORECASE):
                    ports.add(port.upper())
    except FileNotFoundError:
        pass
    return sorted(ports, key=lambda port: int(port[3:]))


class SerialPort:
    def __init__(self, port):
        if os.name != "nt":
            raise OSError("当前上位机串口后端支持 Windows")
        if not re.fullmatch(r"COM[1-9]\d*", port, re.IGNORECASE):
            raise ValueError("请输入串口名称，例如 COM3")
        self.api = ctypes.WinDLL("kernel32", use_last_error=True)
        signatures = {
            "CreateFileW": ([w.LPCWSTR, w.DWORD, w.DWORD, ctypes.c_void_p, w.DWORD, w.DWORD, w.HANDLE], w.HANDLE),
            "CloseHandle": ([w.HANDLE], w.BOOL),
            "GetCommState": ([w.HANDLE, ctypes.POINTER(DCB)], w.BOOL),
            "SetCommState": ([w.HANDLE, ctypes.POINTER(DCB)], w.BOOL),
            "SetCommTimeouts": ([w.HANDLE, ctypes.POINTER(Timeouts)], w.BOOL),
            "PurgeComm": ([w.HANDLE, w.DWORD], w.BOOL),
            "ReadFile": ([w.HANDLE, ctypes.c_void_p, w.DWORD, ctypes.POINTER(w.DWORD), ctypes.c_void_p], w.BOOL),
            "WriteFile": ([w.HANDLE, ctypes.c_void_p, w.DWORD, ctypes.POINTER(w.DWORD), ctypes.c_void_p], w.BOOL),
        }
        for name, (arguments, result) in signatures.items():
            function = getattr(self.api, name)
            function.argtypes = arguments
            function.restype = result
        self.handle = self.api.CreateFileW("\\\\.\\" + port.upper(), 0xC0000000, 0, None, 3, 0, None)
        if self.handle == ctypes.c_void_p(-1).value:
            self.handle = None
            raise ctypes.WinError(ctypes.get_last_error())
        try:
            dcb = DCB()
            dcb.length = ctypes.sizeof(DCB)
            self._check(self.api.GetCommState(self.handle, ctypes.byref(dcb)))
            dcb.baud, dcb.byte_size, dcb.parity, dcb.stop_bits = 115200, 8, 0, 0
            dcb.flags = 1 | (1 << 4) | (1 << 12)  # binary; DTR/RTS enabled, no flow control
            self._check(self.api.SetCommState(self.handle, ctypes.byref(dcb)))
            self._check(self.api.SetCommTimeouts(self.handle, ctypes.byref(
                Timeouts(0xFFFFFFFF, 0xFFFFFFFF, 50, 0, 1000))))
            self._check(self.api.PurgeComm(self.handle, 0x0F))
            # An invalid suffix aborts even a previously complete-but-unterminated
            # write command; a bare newline could accidentally execute it.
            self.write(b"!\n")
        except Exception:
            self.close()
            raise

    @staticmethod
    def _check(result):
        if not result:
            raise ctypes.WinError(ctypes.get_last_error())

    def read(self, size):
        buffer = ctypes.create_string_buffer(size)
        transferred = w.DWORD()
        self._check(self.api.ReadFile(self.handle, buffer, size, ctypes.byref(transferred), None))
        return buffer.raw[:transferred.value]

    def write(self, data):
        buffer = ctypes.create_string_buffer(data)
        transferred = w.DWORD()
        self._check(self.api.WriteFile(self.handle, buffer, len(data), ctypes.byref(transferred), None))
        if transferred.value != len(data):
            raise OSError("串口写入不完整；若为参数写入，请重新连接后读取确认")

    def close(self):
        if self.handle is not None:
            self.api.CloseHandle(self.handle)
            self.handle = None
