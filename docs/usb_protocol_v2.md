# J_ESC USB CDC 协议 v2（阶段一实现）

日期：2026-11-08。实现见 `canopen/od_wire.*`、`od_scope.*`、`od_cdc.c`；参考客户端见 `tools/scope_client/`。设计和实施进度见 [host_tool_design.md](host_tool_design.md)。

## 会话与兼容

1. 连接时以 `!\n` 终止残留文本请求；忽略序号为零的格式错误响应。
2. 发送 `B SSSSSSSS\n`，设备返回 `O SSSSSSSS 00 02\n`。
3. 设备等待这条文本 ACK 的 USB 发送完成，再进入二进制模式。
4. 先发送会话号 0 的能力查询，使用返回的非零会话号进行后续请求。
5. 二进制请求序号从 1 开始严格递增；重复、过期、错误会话或序号零的请求被忽略，不执行，也不补发旧响应。序号耗尽前重新协商；超时或短写后关闭连接并核对状态，不盲目重试。
6. `LEAVE` 在其响应发送完成后返回文本模式。下一次协商产生新的会话号。
7. DTR 下降或物理 USB 会话重置停止采样并清除会话。DTR 下降会等待在途 USB TX 完成后复用缓冲；物理端点复位才直接取消旧 TX 所有权。

这些命令是项目自定义 USB 调试协议，不是 CANopen SDO / CiA 402 报文。

## 帧格式

所有多字节字段小端，禁止直接传输 C 结构体。

```text
COBS(16 字节头 + payload + CRC32) + 0x00
```

| 原始偏移 | 字段 | 长度 |
| --- | --- | --- |
| 0 | ASCII `JE` | 2 |
| 2 | 版本，固定 2 | 1 |
| 3 | 消息类型 | 1 |
| 4 | payload 长度，0–1088 | 2 |
| 6 | 保留标志，固定 0 | 2 |
| 8 | 会话号 | 4 |
| 12 | 请求序号或数据块序号 | 4 |
| 16 | payload | 可变 |
| 16 + payload 长度 | CRC32 | 4 |

CRC32/IEEE：反射多项式 `0xEDB88320`、初值 `0xFFFFFFFF`、最终异或 `0xFFFFFFFF`，覆盖原始头和 payload。标准向量 `123456789` → `0xCBF43926`。CRC 本身也编码在 COBS 内。

解析器支持任意拆包、粘包；校验、长度或版本错误只丢弃当前包；超长包丢弃至下一个零分隔。空分隔可忽略。编码缓冲上限 1,114 字节（含末尾零的容量预算）。

## 请求和响应

普通响应类型为 `请求类型 | 0x80`，回显请求序号；payload 第一个字节为状态码，余下为成功结果。状态码沿用 `OD_Result`，普通配置状态拒绝仍可能返回 `0x81`。错误响应不带成功结果。

| 类型 | 名称 | 请求 payload | 成功结果（不含状态字节） |
| --- | --- | --- | --- |
| `0x01` | CAPABILITIES | 空 | 能力描述，18 B |
| `0x02` | LEAVE | 空 | 空 |
| `0x03` | READ | `index:u16, sub:u8, type:u16, size:u8` | 标量小端值 |
| `0x04` | WRITE | READ 的字段 + 标量值 | 空 |
| `0x10` | SCOPE_CONFIG | 见下文 | 采集状态，28 B |
| `0x11` | SCOPE_START | 空 | 采集状态，28 B |
| `0x12` | SCOPE_STOP | 空 | 空 |
| `0x13` | SCOPE_STATUS | 空 | 采集状态，28 B |
| `0x20` | BENCHMARK | `blocks:u16`，1–512 | 空，随后推送测试数据块 |
| `0x30` | MOTION_SUBMIT | `mode:u8, reserved:3 B=0, target:f32, speed:f32, acceleration:f32, jerk:f32` | 运动状态 16 B |
| `0x31` | MOTION_STATUS | 空 | 运动状态 16 B |
| `0x32` | MOTION_HEARTBEAT | 空 | 空 |
| `0x33` | MOTION_STOP | 空 | 空；请求在实时入口取消在线运动 |

READ / WRITE 的标量大小为 1–8 B，类型必须与 OD 一致。复用原有配置权限、使能状态、应用/保存条件和命令回调，不能直接写运行变量。采样进行中、已采数据未排空或吞吐测试未结束时，拒绝 Flash 保存和参数应用，避免打断采样与通信。

能力描述：

```text
tick_hz:u32, slow_hz:u32, buffer_bytes:u32,
block_bytes:u16, max_channels:u8, features:u8, max_payload:u16
```

`features` 位 0=连续采样、位 1=间断采样、位 2=32 位及以下运行变量采样、位 3=吞吐测试、位 4=基础电流/速度/位置完整目标接口。当前值 31，不声明触发、字典哈希或周期同步轨迹序列。

## 基础运动接口

mode 1=电流、2=速度、3=位置。电流以 A 为单位，其他目标使用轴单位（旋转 deg，直线 mm）；速度、加速度、jerk 为相应时间导数。电流模式的其余三个浮点值必须为 0；速度模式 speed 必须为 0；位置模式 speed > 0。速度/位置的 acceleration、jerk 必须为正且有限。位置是绝对目标，不在此接口新增软件行程限制。

提交只允许已使能、校准/寻相完成、无故障/抱闸、配置已应用的 DC 或三相 BLDC FOC，且当前模式为 None/Current/Speed/Pos；不自动使能。超出实时电流/速度限制会拒绝，不静默截断。前台准备整条轨迹，拒绝非有限系数或超过 1 小时的规划；实时入口在没有其他被挂起中断的边界原子发布，并复核门控、限幅、10 ms 快照时效和取消代号。

运动状态布局：`id:u32, phase:u8, result:u8, requested_mode:u8, reserved:u8=0, actual_motor_mode:u32, lease_remaining_ticks:u32`。phase 0=已停止、1=已接收、2=已生效/保持、3=位置到达、4=执行失败、5=租约超时或状态变化后失能。`actual_motor_mode` 使用固件 `Motor_Control_Mode_t`，不是 1/2/3 的请求模式编号。请求 ACK 不代表执行或位置到达。

主机应每约 100 ms 发心跳，租约为半秒高频回调计数。STOP、LEAVE、DTR/USB 断线取消待执行命令；仅当该接口拥有活动运动时才失能，不改变原手动命令的断线语义。位置到达仍须续约。此机制依赖控制 ISR 正常运行，未声称硬件急停能力。重复请求仍按会话序号规则拒绝执行，不自动重试。

## 采集配置和状态

配置 payload：

```text
mode:u8, channel_count:u8, divisor:u32,
重复 channel_count 次 { index:u16, subindex:u8, type:u16 }
```

- mode：1 连续、2 间断。
- 采样率 = `tick_hz / divisor`。当前只接受不低于 10 kHz 的整数分频；当前板级 20 kHz 对应分频 1 或 2。
- 最多 16 个通道，仅接受 `0x2100`–`0x2109` 下子索引非零、1/2/4 B 的 RO 标量；拒绝重复通道与 64 位值。
- 不进行采样、缓存已排空时才允许重配或启动。停止会发布最后的非满块；主机应收完最后状态中的 `captured` 点后再启动新段。
- 高速采样点位于 `motor_update_frt(0)` 后。慢环字段可能重复，跨 ISR 的变量不是统一原子快照。

状态结果：

```text
state:u8, mode:u8, channels:u8, reserved:u8,
config_id:u32, segment:u32, captured:u32, overflows:u32, divisor:u32,
stride:u16, samples_per_block:u16
```

- state：0 停止/空闲、1 采样、2 间断已冻结待排空、3 连续缓存溢出。
- `captured` 为当前段已采样点数；不是已上传点数。
- `overflows` 为会话内连续缓存溢出累计次数；显式 STOP 后 state 为 0，但累计计数保留。
- `stride` 为一个同步采样点中所有通道字节数之和。
- `samples_per_block = floor(1024 / stride)`；间断段完整容量为此值 × 16。
- 连续模式满缓存时停止，不覆盖旧数据。间断模式采满 16 块后冻结，冻结前不发送样本。

## 推送数据

### `0x40` SAMPLES

头中的 sequence 等于 payload 的 block_sequence，无普通响应状态字节。

```text
config_id:u32, segment:u32, block_sequence:u32, tick_hz:u32,
first_tick:u64, divisor:u32, count:u16, stride:u16,
sample_data:count*stride 字节
```

一个样本按配置顺序紧凑排列，无对齐填充。短枚举按 OD 类型展开，如 UNSIGNED32 仍占 4 B。块序号每段从 0 开始。设备 tick 在同一会话内跨段递增；设备/会话重置后重新计数。tick 表示高频控制回调计数，并非 USB 接收时间，也不是独立硬件绝对时钟。

第 i 个样本的时间轴为 `(first_tick + i * divisor) / tick_hz`。主机须检查段内序号与 tick 连续，跨段保留空隙。真实周期抖动、截止时间与 USB 吞吐仍须上板验证。

### `0x41` BENCHMARK_DATA

payload 固定 1,024 B，内容为重复的 `0..255`，头序号为 `0..blocks-1`。不会调用电机控制命令。

只允许采样停止且无待上传样本时启动，测量期间拒绝配置/启动采样。STOP / LEAVE 可取消未发送测试块；已提交 USB 的块仍需完成。参考客户端取三轮测量最小值作为保守负载吞吐，另留 30% 余量；此值受当时电脑、USB、固件和控制负载影响。

## 原始记录格式

参考客户端的 `.jescscope`：

```text
ASCII "JESCOPE2\n"
metadata_length:u32
metadata:UTF-8 JSON
重复 { encoded_frame_length:u32, encoded_frame:含 COBS 及末尾零 }
```

JSON 最多 1 MiB，单帧记录最多 1,114 B。元数据保存通道类型、本地字典 SHA256、采样频率、能力和会话；`dictionary_verified_by_device=false` 明确表示尚未由固件核对字典版本。

记录样本帧及每段停止后的最终状态响应。最终状态可能在剩余样本前写入；读取结束时核对每段样本总数与最终 `captured`。缺少最终状态、损坏、截断或数量不匹配均不能当作完整记录。CSV 导出保留设备 tick、段号和跨段空行。

## 阶段一边界

已实现 [C++/Qt 参数方案、基础运动目标接口、GPU 示波器、FFT/统计、多坐标轴与录制/回放/CSV 导出](../tools/esc_studio/README.md)。Qt 录制逐段停止/核对最终状态/排空后重布防，最终状态记录帧序号可为 0（仅用于文件保存，不作为设备请求重放）；样本保留原块序号。录制在 `.partial` 写入，完整结束才生成正式文件。预/后触发、固件字典哈希、原子配置事务、周期同步目标序列及自动从间断恢复连续尚未实现。

参考 CLI 已具备启动前自动选择模式及溢出后转间断。无电机真实 USB 与 ISR 初测见 [硬件验证](hardware_validation.md)；不代表真实电机负载或全场景实时性验收。
