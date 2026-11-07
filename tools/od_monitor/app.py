"""Simple CSV-driven object dictionary monitor for Windows USB CDC."""
import argparse
from pathlib import Path
import queue
import threading
import tkinter as tk
from tkinter import filedialog, messagebox, ttk

from client import Client, DeviceError
from model import load_dictionary
from serial_win32 import SerialPort, list_ports

DEFAULT_CSV = Path(__file__).resolve().parents[2] / "canopen/object_dictionary.csv"


class Monitor:
    def __init__(self, root, csv_path):
        self.root = root
        self.entries = []
        self.connected = False
        self.busy = False
        self.jobs, self.events = queue.Queue(), queue.Queue()
        self.stop = threading.Event()
        self.path = tk.StringVar(value=str(csv_path))
        self.port = tk.StringVar()
        self.value = tk.StringVar()
        self.state = tk.StringVar(value="未连接")
        self.detail = tk.StringVar(value="选择参数，读取后可编辑写入值。")
        root.title("J_ESC 参数工具 · USB CDC")
        root.geometry("1180x680")
        root.minsize(850, 460)
        self._build()
        self.refresh_ports()
        self.load_csv()
        threading.Thread(target=self._worker, daemon=True).start()
        root.after(50, self._drain)
        root.protocol("WM_DELETE_WINDOW", self.close)

    def _build(self):
        frame = ttk.Frame(self.root, padding=12)
        frame.pack(fill="both", expand=True)
        csv_bar = ttk.Frame(frame)
        csv_bar.pack(fill="x", pady=(0, 8))
        ttk.Label(csv_bar, text="CSV 文件").pack(side="left")
        self.path_entry = ttk.Entry(csv_bar, textvariable=self.path)
        self.path_entry.pack(side="left", fill="x", expand=True, padx=8)
        self.browse_button = ttk.Button(csv_bar, text="浏览导入…", command=self.browse)
        self.browse_button.pack(side="left")
        self.load_button = ttk.Button(csv_bar, text="加载 / 重新加载", command=self.load_csv)
        self.load_button.pack(side="left", padx=(6, 0))
        bar = ttk.Frame(frame)
        bar.pack(fill="x", pady=(0, 10))
        ttk.Label(bar, text="串口").pack(side="left")
        self.ports = ttk.Combobox(bar, textvariable=self.port, width=15)
        self.ports.pack(side="left", padx=8)
        self.refresh_button = ttk.Button(bar, text="刷新串口", command=self.refresh_ports)
        self.refresh_button.pack(side="left")
        self.connect_button = ttk.Button(bar, text="连接", command=self.toggle_connection)
        self.connect_button.pack(side="left", padx=8)
        ttk.Label(bar, textvariable=self.state).pack(side="left", padx=8)

        table_frame = ttk.Frame(frame)
        table_frame.pack(fill="both", expand=True)
        columns = ("name", "index", "sub", "type", "access", "value", "status")
        self.table = ttk.Treeview(table_frame, columns=columns, show="headings", selectmode="extended")
        for key, title, width in zip(columns,
                ("名称", "主索引", "子索引", "类型", "权限", "当前值 / 已确认写入值", "结果"),
                (260, 80, 70, 100, 55, 180, 190)):
            self.table.heading(key, text=title)
            self.table.column(key, width=width, minwidth=45)
        scroll = ttk.Scrollbar(table_frame, orient="vertical", command=self.table.yview)
        horizontal = ttk.Scrollbar(table_frame, orient="horizontal", command=self.table.xview)
        self.table.configure(yscrollcommand=scroll.set, xscrollcommand=horizontal.set)
        self.table.grid(row=0, column=0, sticky="nsew")
        scroll.grid(row=0, column=1, sticky="ns")
        horizontal.grid(row=1, column=0, sticky="ew")
        table_frame.rowconfigure(0, weight=1)
        table_frame.columnconfigure(0, weight=1)
        self.table.bind("<<TreeviewSelect>>", self._selection_changed)
        actions = ttk.Frame(frame)
        actions.pack(fill="x", pady=10)
        self.read_button = ttk.Button(actions, text="读取选中", command=self.read_selected)
        self.read_button.pack(side="left")
        self.read_all_button = ttk.Button(actions, text="读取全部", command=self.read_all)
        self.read_all_button.pack(side="left", padx=6)
        ttk.Label(actions, text="写入值").pack(side="left", padx=(16, 6))
        self.value_entry = ttk.Entry(actions, textvariable=self.value, width=24)
        self.value_entry.pack(side="left")
        self.write_button = ttk.Button(actions, text="写入选中参数", command=self.write_selected)
        self.write_button.pack(side="left", padx=6)
        ttk.Label(frame, textvariable=self.detail, wraplength=1100).pack(fill="x", anchor="w")
        ttk.Label(frame, text="写入仅修改 RAM；电机须失能并处于 IDLE / Err。枚举值使用数值输入。",
                  foreground="#666666").pack(fill="x", pady=(6, 0))
        self._controls()

    def _controls(self):
        selected = self.table.selection()
        idle = not self.busy
        for widget in (self.path_entry, self.browse_button, self.load_button, self.refresh_button):
            widget.configure(state="normal" if idle else "disabled")
        self.ports.configure(state="normal" if idle and not self.connected else "disabled")
        self.connect_button.configure(text="断开" if self.connected else "连接", state="normal" if idle else "disabled")
        readable = any(self.entries[int(item)].access != "wo" for item in selected)
        self.read_button.configure(state="normal" if idle and self.connected and readable else "disabled")
        self.read_all_button.configure(state="normal" if idle and self.connected and
                                       any(entry.access != "wo" for entry in self.entries) else "disabled")
        writable = len(selected) == 1 and self.entries[int(selected[0])].access != "ro"
        self.write_button.configure(state="normal" if idle and self.connected and writable else "disabled")
        self.value_entry.configure(state="normal" if idle and writable else "disabled")

    def refresh_ports(self):
        try:
            ports = list_ports()
            self.ports["values"] = ports
            if not self.port.get() and ports:
                self.port.set(ports[0])
        except OSError as error:
            self.detail.set(f"无法列出串口：{error}；可手动输入 COM 名称。")

    def browse(self):
        path = filedialog.askopenfilename(title="选择对象字典 CSV", filetypes=[("CSV 文件", "*.csv"), ("所有文件", "*.*")])
        if path:
            self.path.set(path)
            self.load_csv()

    def load_csv(self):
        try:
            entries = load_dictionary(Path(self.path.get()).expanduser())
        except (OSError, ValueError) as error:
            messagebox.showerror("CSV 加载失败", str(error))
            return
        self.entries = entries
        self.table.delete(*self.table.get_children())
        self.value.set("")
        for number, entry in enumerate(entries):
            self.table.insert("", "end", iid=str(number), values=(entry.name,
                f"0x{entry.index:04X}", f"0x{entry.subindex:02X}", entry.data_type, entry.access, "", "未读取"))
        self.detail.set(f"已加载 {len(entries)} 个条目：{self.path.get()}")
        self._controls()

    def _selection_changed(self, _event=None):
        selection = self.table.selection()
        if len(selection) == 1:
            number = int(selection[0])
            self.value.set(self.table.set(selection[0], "value"))
            self.detail.set(self.entries[number].description or self.entries[number].name)
        self._controls()

    def _start(self, operation, payload=None):
        if self.busy:
            return
        self.busy = True
        self.state.set("处理中…")
        self.jobs.put((operation, payload))
        self._controls()

    def toggle_connection(self):
        self._start("disconnect" if self.connected else "connect", self.port.get().strip())

    def read_selected(self):
        items = [(int(i), self.entries[int(i)]) for i in self.table.selection()
                 if self.entries[int(i)].access != "wo"]
        if items:
            self._start("read", items)

    def read_all(self):
        self._start("read", [(i, entry) for i, entry in enumerate(self.entries) if entry.access != "wo"])

    def write_selected(self):
        selection = self.table.selection()
        if len(selection) != 1:
            return
        number = int(selection[0])
        entry = self.entries[number]
        try:
            entry.encode(self.value.get())
        except ValueError as error:
            messagebox.showerror("数值无效", str(error))
            return
        self._start("write", (number, entry, self.value.get()))

    def _worker(self):
        client = None
        try:
            while not self.stop.is_set():
                try:
                    operation, payload = self.jobs.get(timeout=0.1)
                except queue.Empty:
                    continue
                try:
                    if operation == "connect":
                        client = Client(SerialPort(payload))
                        self.events.put(("message", "串口已连接，可以读取参数。"))
                    elif operation == "disconnect":
                        if client:
                            client.close()
                        client = None
                    elif operation == "read":
                        if client is None:
                            raise OSError("尚未连接串口")
                        for number, entry in payload:
                            if self.stop.is_set():
                                break
                            try:
                                self.events.put(("row", number, client.read(entry), "读取成功"))
                            except DeviceError as error:
                                self.events.put(("row", number, None, str(error)))
                    elif operation == "write":
                        if client is None:
                            raise OSError("尚未连接串口")
                        number, entry, text = payload
                        value = client.write(entry, text)
                        self.events.put(("row", number, value, "写入已确认，未回读"))
                except DeviceError as error:
                    if operation == "write":
                        self.events.put(("row", payload[0], None, str(error)))
                    self.events.put(("message", str(error)))
                except (OSError, ValueError) as error:
                    if operation == "write":
                        self.events.put(("row", payload[0], None, "写入结果未知，请重连读取确认"))
                    elif operation == "read" and payload:
                        self.events.put(("row", number, None, str(error)))
                    if client:
                        client.close()
                    client = None
                    hint = "写入结果未知，请重连读取确认。" if operation == "write" else ""
                    self.events.put(("message", f"{hint}操作失败，连接已关闭：{error}"))
                finally:
                    self.events.put(("done", client is not None))
        finally:
            if client:
                client.close()

    def _drain(self):
        while True:
            try:
                event = self.events.get_nowait()
            except queue.Empty:
                break
            if event[0] == "row":
                _, number, value, status = event
                if value is not None:
                    self.table.set(str(number), "value", value)
                self.table.set(str(number), "status", status)
            elif event[0] == "message":
                self.detail.set(event[1])
            elif event[0] == "done":
                if event[1] and not self.connected:
                    for item in self.table.get_children():
                        self.table.set(item, "value", "")
                        self.table.set(item, "status", "未读取（新连接）")
                self.connected = event[1]
                self.busy = False
                self.state.set(f"已连接 {self.port.get()}" if self.connected else "未连接")
                self._controls()
        self.root.after(50, self._drain)

    def close(self):
        self.stop.set()
        self.root.destroy()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--csv", type=Path, default=DEFAULT_CSV, help="默认读取的对象字典 CSV 路径")
    args = parser.parse_args()
    root = tk.Tk()
    Monitor(root, args.csv)
    root.mainloop()


if __name__ == "__main__":
    main()
