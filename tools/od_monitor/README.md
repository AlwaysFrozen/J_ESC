# USB CDC 参数上位机

Windows + Python 3.10 或更新版本，使用 Tkinter 和 Win32 COM 接口，仅依赖 Python 标准库。常规 Windows Python 安装需包含 Tcl/Tk；不需要安装 pyserial。

## 启动

在项目根目录执行：

```text
python tools/od_monitor/app.py
```

也可以双击本目录下的 `start.bat`。默认加载项目的 `canopen/object_dictionary.csv`，不依赖当前工作目录。

指定其他 CSV：

```text
python tools/od_monitor/app.py --csv "D:\parameters\object_dictionary.csv"
```

界面中可用“浏览导入”选择文件，也可直接编辑路径再点击“加载 / 重新加载”。导入的是对象定义，不会自动向设备写入参数。CSV 与代码生成器共用校验逻辑，格式见 `canopen/README.md`。请加载与设备固件对应的 CSV；设备会检查地址、类型和长度，但不会检查名称或完整字典版本。

## 使用

1. 编译并烧录更新后的固件，使用 USB 数据线连接板卡。
2. 在串口列表选择设备的 COM 端口；插拔设备后可以刷新，也可手动输入 COM 名称。
3. 点击“连接”，选择条目后“读取选中”，或“读取全部”。支持多选读取，只写条目会跳过。
4. 选择单个可写条目，在“写入值”中输入数值，然后点击“写入选中参数”。

整数支持十进制、带 `0x` 前缀的十六进制和负数，浮点数支持小数及科学计数法。枚举使用对应头文件中的数值；`have_brake` 是 INTEGER32。BOOLEAN 只接受 0/1，超范围整数、NaN 和 Inf 会在发送前拒绝。REAL32 会按单精度转换，界面显示实际发送后归一化的值。

子索引 0 的最大子索引条目是只读的，界面禁用写入。正常写入须在电机失能且状态为 `MS_IDLE` 或 `MS_Err` 时执行，否则设备返回“设备忙”。配置值的业务范围、枚举合法值和重新初始化要求仍以应用代码为准；不能把“写入已确认”理解为参数组合已通过电机配置检查。

产品 CSV 的 `0x2000`–`0x2006` 为配置参数，`0x2100`–`0x2109` 为内核、电源、采样、电机、编码器、观测器、控制环、FOC 和 HFI 的只读运行变量。运行中可以读取状态；连续读取多个条目不是同一控制周期的快照。更新固件后点击“加载 / 重新加载”同步最新映射。

“写入已确认，未回读”表示设备已接受写入，可再次读取确认。写入仅修改 RAM，本界面没有保存 Flash 或使能电机的命令，断电后按原配置加载逻辑恢复。

串口读写在后台线程串行执行，处理中禁用连接/CSV修改按钮。每次只发送一个请求，匹配请求编号的响应后才执行下一个。超时或断线会关闭连接；写入不会自动重试，因为设备可能已经执行写入但响应丢失。出现“写入结果未知”后应重新连接并读取确认。

## 固件连接位置

- `main.c`：业务初始化后调用 `od_cdc_init()`，主循环中调用 `od_cdc_poll()`。
- `usbd_cdc_if.c`：接收回调缓存字节；发送完成回调释放发送缓冲区；初始化/反初始化回调切换会话。
- `canopen/od_cdc.c`：512 字节接收队列、断线处理、发送忙重试、状态检查与临界区访问。
- `canopen/od_protocol.c`：独立的文本协议解析器，不依赖 USB HAL。

不要在这个 CDC 端口混入其他命令解析器；不完整或过长请求会被丢弃或返回格式错误。打开串口时发送 `!` 加换行以终止旧会话残留，避免把旧的未结束写命令补全执行。

## 协议 v1

协议为本项目的 USB 调试协议，直接访问 CANopen 对象字典，不是 CAN 总线上的 SDO 报文。所有字段均为定宽十六进制 ASCII，以一个空格分隔，以 LF 结束，设备也接受 CRLF。

```text
读取：R SSSSSSSS IIII SS TTTT LL\n
写入：W SSSSSSSS IIII SS TTTT LL DATA\n
响应：O SSSSSSSS EE DATA\n
```

| 字段 | 含义 |
| --- | --- |
| SSSSSSSS | 32 位请求编号，响应原样返回 |
| IIII / SS | 主索引 / 子索引 |
| TTTT | CANopen 基本数据类型编号，例如 REAL32 为 0008 |
| LL | 数据字节数，支持 01–08，必须与条目类型匹配 |
| DATA | 线缆字节逐字节十六进制编码，按小端序；无数据用 `-` |
| EE | 状态码：00 成功，01–0B 对应 OD_Result，80 格式错误，81 电机状态不允许写入 |

例如读取额定电压（`0x2000:03`），设备值为 12.5：

```text
R 00000001 2000 03 0008 04
O 00000001 00 00004841
```

将同一参数设置为 12.5：

```text
W 00000002 2000 03 0008 04 00004841
O 00000002 00 -
```

请求格式错误且无法恢复编号、接收溢出或超长行重同步时，响应编号可能为 00000000。正常客户端会忽略不匹配的编号，超时后重连。

Win32 读取使用短超时，避免阻塞界面；相关系统接口语义见 [Microsoft COMMTIMEOUTS 文档](https://learn.microsoft.com/en-us/windows/win32/api/winbase/ns-winbase-commtimeouts)。

## 测试

```text
python tests/canopen/test_monitor.py
python tests/canopen/run_tests.py --cc clang
```

第一条无需串口设备，验证 CSV/类型转换及分片响应、状态码、超时行为。第二条还运行生成器测试，并编译执行真实 C 字典、协议解析器及带 USB 桩的传输测试。只有 ARM Compiler 时可用 `--cc <armclang.exe路径> --arm --compile-only`，该模式只做 C 编译检查，不执行 C 断言。

当前环境已通过14项 Python 测试、Tk 隐藏窗口控件检查及 GNU ARM 编译检查。Debug 固件已经使用 ARM GNU Toolchain 15.3.Rel2、CMake 和 Ninja 完成全量编译和链接，生成 `build/Debug/J_ESC.elf`，并已导出 HEX/BIN。现有电机控制代码仍有丢弃 volatile 限定及未使用变量等编译警告。C 运行测试、真实 USB 枚举/读写、断线重连和电机状态限制仍需验证。
