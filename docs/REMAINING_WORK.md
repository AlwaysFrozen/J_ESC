# J_ESC 未完成工作检查清单

检查日期：2026-11-08（Asia/Shanghai）。检查基线：`d0ff895`（增加上位机）。

## 结论与检查边界

项目已具备对象字典、USB v2、配置持久化、Qt 参数管理、基础运动目标接口、示波采集/录制/回放和 FFT 等基础功能，但还没有完成实际电机运动及带载验收。**当前首要阻塞是 PWM 工作时的 AS5048A 反馈异常，最新现场记录停在编码器接线整改，不能将项目标记为完成。**

本次检查了项目源码、现有设计/交接/调试文档、构建配置和测试，并重新运行本地回归。未连接串口或 J-Link，未烧录、写设备参数、保存 Flash 或启动电机；下面的硬件状态来自已有记录，不是本次实时读回。

状态分为“未实现”“部分实现”“已实现待验收”“待排查”。P0 为当前阻塞，P1 为基础功能验收前应完成，P2 为后续完善，P3 为按需求决定的扩展。优先级为本次建议；扩展功能不自动成为当前版本的交付要求。

## 1. P0：当前现场阻塞

| 编号 | 工作与状态 | 现有证据 | 完成条件 |
| --- | --- | --- | --- |
| HW-01 | AS5048A 在 PWM 下异常：待排查 | 两次限压寻相及零目标 PWM 均触发故障 51；缓存错误寄存器为 2。编码器 5 V 供电、约 20 cm SPI 线与三相线并行且无屏蔽。见 [实际电机调试记录](MOTOR_COMMISSIONING.md) | 先确认断电整改走线已完成；复核失能读取、零目标 PWM、限时低电流寻相。记录错误状态、有效角度、速度及电流，寻相完成且反馈稳定；必要时用仪器检查供电和通信波形。当前走线只是强疑点，不能直接认定根因 |
| HW-02 | 相电流 ADC 噪声：待排查 | 失能时三相电流标准差约 0.49–0.75 A，暂停 SPI 后仍有相近 ADC 噪声，见上述记录 | 检查采样链路、偏置校准和 PWM 同步采样，区分原始 ADC 噪声与换算结果；确定适合当前低电流测试的测量能力及保护阈值。不能仅靠放宽保护阈值宣布解决 |

前置状态：测试方案只写入 RAM，未保存配置 Flash；断电后需在失能状态重新载入 `build/hardware-validation/phase6-optimized-profile.jescconfig`、逐项回读、应用并等待校准。`phase6-confirmed-draft.jescconfig` 是未完成草稿，不能直接用于运行。方案和日志位于被 Git 忽略的 `build/` 中。

## 2. P1：基础控制和硬件验收

| 编号 | 工作与状态 | 依据/入口 | 完成条件 |
| --- | --- | --- | --- |
| CTRL-01 | 电流、速度、位置真实执行：已实现待验收 | [od_motion.c](../canopen/od_motion.c)、[MotorPage.qml](../tools/esc_studio/qml/MotorPage.qml)、[实际电机调试记录](MOTOR_COMMISSIONING.md) | HW-01 通过后，逐项验证小电流、低速、短距离目标；核对相序、编码器方向、单位、限幅、跟踪误差、到达与保持，每项结束确认失能。目前尚无成功的完整寻相及三种目标验收 |
| CTRL-02 | 停止、心跳超时、断线取消：已实现待验收 | 新接口已有约 100 ms 心跳、500 ms 租约及取消代号；C 测试使用电机/规划器桩 | 在真实运动中验证显式停止、停发心跳、USB 断开、主机退出及准备目标期间取消，记录实际停机时序和最终状态。原手动使能/寻相/辨识没有同样的租约，不能混用结论 |
| CTRL-03 | 轨迹质量和目标生效延迟：已实现待验收 | [od_motion.c](../canopen/od_motion.c)、[motion_planner.c](../motor_control/math/motion_planner.c) | 测量提交、接收、生效及到达的时序，验证加速度/jerk、连续换目标、模式切换和最坏发布延迟；补足真实规划器参与的集成验证 |
| RT-01 | 活动 FOC 与全负载实时预算：已实现待验收 | [CMakeLists.txt](../CMakeLists.txt) 对控制热路径启用 `-O2`；最后调试观察到高频累计耗时约 36.55 μs | 在真实运动及最大目标通道数采样、USB、低频控制中断并行条件下测量最坏耗时/抖动、栈与内存余量、溢出和故障路径。现有 20 kHz 记录及短暂寻相统计不能证明完整 50 μs 周期预算 |
| TEMP-01 | 温度采集与保护闭环：部分实现 | [realtime.c](../motor_control/hal/realtime.c) 中 `core_temp`、`board_temp` 固定写 0；已有原始 ADC 通道和温度保护相关逻辑 | 实现所需传感器换算、校准和有效性判断，核对电机温度来源，验证断线/异常值和过温保护。字典可读或有阈值不等于温度测量已生效 |
| PROT-01 | 超速、过流、热模型等保护：已实现待验收 | [motor.c](../motor_control/motor.c)、`tests/canopen/test_motor_policy.c`；已有速度越限与无效热参数回归 | 核对真实参数、热累积量及故障锁存/显式复位；在受控条件下补硬件验收。转矩常数口径尚未确认，当前使用磁链换算，不应宣称转矩精度 |
| CFG-01 | 真实配置持久化与恢复：已实现待验收 | [配置存储说明](../motor_control/config/README.md)、[config_storage.c](../motor_control/config/config_storage.c) | 在有效方案稳定后安排保存、读回、断电重启恢复及受控掉电恢复验证，确认失败可回退旧记录。模拟 Flash 测试已经通过，现有硬件记录仅证明配置扇区未被固件烧录改变 |
| HW-03 | J-Link NRST 问题根因：待排查 | [硬件验证记录](hardware_validation.md)：断开 NRST 后退出调试器可独立运行 | 检查探针/转接板引脚、MCU NRST 实际电平与复位电路，恢复正确连接后复测接入、退出和独立启动。当前断开 NRST 是隔离措施，不能标记为硬件修复 |

## 3. P2：协议、上位机和可靠性完善

| 编号 | 工作与状态 | 当前边界与建议验收 |
| --- | --- | --- |
| PROTO-01 | 固件字典哈希及固件版本查询：未实现 | 当前 `.jescconfig` / `.jescscope` 保存本地字典 SHA256，设备未核对，录制标记 `dictionary_verified_by_device=false`。增加设备版本和字典标识，验证错配拒绝、旧固件兼容和文件追溯。见 [USB v2 协议](usb_protocol_v2.md) |
| CFG-02 | 原子配置事务：未实现 | 方案导入预览已有完整本地校验，但写设备仍逐项执行；中途失败可能只更新部分 RAM。需要暂存、整体校验、提交/取消及断线处理，证明失败不会部分生效；不应把本地方案管理描述成设备事务 |
| UI-01 | 参数树与依赖联动：部分实现 | 七页、分组筛选、已知枚举和单位已实现；完整折叠树、全部字段依赖/业务范围联动未完成。按电机/编码器/控制器类型展示有效字段，并保留固件最终校验 |
| SCOPE-01 | 下位机预/后触发：未实现 | 增加触发源、边沿/阈值、预/后触发缓存和超时状态，验证触发点定位、满缓存、重布防及录制/回放一致性 |
| SCOPE-02 | 自动从间断恢复连续：未实现 | 当前自动模式可因溢出从连续降为间断，尚无恢复策略；如需要，增加重新测预算、余量与滞回，验证切换不丢失段/时间信息 |
| FILE-01 | 长文件索引与操作取消：待完善 | 当前回放定位通过后台顺序扫描，长文件耗时随长度增长，尚无磁盘索引。完善定位、取消和关闭时响应，保留原始文件完整性校验及有界显示内存 |
| REL-01 | 长期录制和异常恢复：已实现基础处理，待压力验收 | 已有 8 MiB 写盘队列、`.partial`、CRC 和最终点数检查，也已有约 65 秒连续记录；仍需慢盘、磁盘满、长时间录制、USB 拔插、主机异常退出及重连测试，核对明确报错、停采、文件状态和数据完整性 |
| PERF-01 | 完整应用性能：待验收 | 独立 RHI 八通道测试已通过；需要测完整 UI 在 USB/磁盘/FFT 并发、16 通道、长历史、缩放和分通道坐标下的端到端帧时与响应。不能用独立合成 FPS 或“准备 ms”替代 |
| TEST-01 | 完整运动协议集成覆盖：待补充 | 运动邮箱已有测试，但依赖电机/规划器桩。补实际 CDC 运动帧分发、主机请求、状态/心跳/停止的集成验证，覆盖畸形帧、取消竞争与错误返回；已有采样 C 桥互通不等于运动端到端覆盖 |

上位机现状和功能边界详见 [J_ESC Studio README](../tools/esc_studio/README.md)。

## 4. P3：按产品范围决定的扩展与适配

这些是源码或设计中可见的缺口，是否纳入下一版本需按实际硬件和功能需求决定。

| 编号 | 工作与状态 | 可核对的缺口 |
| --- | --- | --- |
| ENC-01 | ABZ 编码器：未完成板级适配 | [encoder.c](../motor_control/encoder/encoder.c) 的 ABZ 初始化仍有 `todo`，高频路径把 `encoder_delta_cnt` 写成 0。需接入定时器计数、方向/回绕、Z 相及位置有效性验证；不能因枚举存在就认为支持 |
| ENC-02 | 禾川/多摩川 RS485 编码器：未完成板级适配 | [encoder_rs485.c](../motor_control/hal/encoder_rs485.c) 的初始化和传输均返回 `false`，abort 留空；协议解析代码已有。需补 UART/DMA、DE、超时、收发完成及错误恢复并实测 |
| MOTOR-01 | DTC、DC 辨识及扩展辨识：未实现/未接入 | [motor.c](../motor_control/motor.c) 明确标注 DTC 未实现；[od_commands.c](../canopen/od_commands.c) 拒绝辨识编号 5–8（齿槽、惯量、摩擦、重力），DC 辨识未接入。现有电阻/电感/极对数/磁链入口也仍需真实算法验收 |
| MOTION-01 | 统一软件行程边界、自动回零、周期同步目标序列及更多模式：未实现 | 当前基础接口提供电流/速度/位置单目标及软件租约，无 CST/CSV/CSP 序列接口。需要按机械结构补状态机、边界与测试，不能直接套用为标准驱动器能力 |
| CAN-01 | 完整 CANopen 节点：未实现 | [canopen/README.md](../canopen/README.md) 明确只有对象字典基础，无 SDO/PDO/NMT/Heartbeat 和自动 CAN 通信；字符串、DOMAIN、EDS 导出也不包含。USB 运动心跳与 CANopen Heartbeat 是不同机制 |
| FFT-01 | 实时 FFT、PSD、对数轴、每通道手动 Y 和单位换算：未实现 | 现有 FFT 是按钮触发的后台分析，分通道 Y 仅独立自动范围；按需要扩展并验证数值准确性与显示性能 |
| CFG-03 | 跨固件配置格式迁移：未实现 | 当前 Flash 保存 ABI 结构体快照，格式不兼容时拒绝加载，不自动迁移。若有升级保参需求，需设计版本迁移，并遵守结构/枚举变化时递增存储版本的要求 |

## 5. 已完成，不应重复列为待办

- 参数方案：七组 166 项读取、导出、差异预览、草稿载入、字典/类型/数值校验已实现。
- 基础电流/速度/位置命令与状态、心跳、停止已实现；缺口是真实运动及完整集成验收。
- GPU 波形、连续/间断采集、原始录制、回放、CSV、FFT/统计及分通道自动坐标已实现。
- 步骤 22 的构建/部署 EXE 同步和固件 8 B 差异同步已完成；旧文档中的“未编译/未同步”描述不是当前待办。
- 步骤 23 已修复 AS5048A 主动读取、SPI 启用、流水线/校验及显式恢复路径，增加实测速度越限和无效热参数保护；不能因修复已合入便将 PWM 下故障 51 视为解决。

本次本地固件大小为 **213,528 B**，SHA256 为 `53bedd5d094dcaed815de979d86d644a16887114d77a5ebd717b8f076ff85c4c`，与 `build/hardware-validation/phase6-final-integrity.json` 一致；历史记录报告应用读回一致、配置扇区未变。本次未重新读取板上固件。

本次构建版和部署版 `esc_studio.exe` SHA256 均为 `ae2d82a5a3885c646b7652bb68c24d8ea19ce375f3600cc5f80cbd126846ca9b`。

## 6. 本次实际执行的验证

| 检查 | 本次结果 | 证据 |
| --- | --- | --- |
| Python 与 C 回归 | Python 8 + 10 + 15 = 33 项通过；C 字典/协议/CDC/采样、命令、运动邮箱、速度保护及 AS5048A 测试通过 | `build/work-audit/canopen.log` |
| 配置存储回归 | 通过；包括状态门控、相同内容不擦写、模拟写入中断、错误及回退等 | `tests/config_storage/run_tests.py` 本次执行退出码 0 |
| Qt 构建与 CTest | 4/4 组通过；核心 9、采集 9、RHI 3、UI 11，均无失败或跳过 | `build/work-audit/qt.log`、`build/esc-studio/{qt,scope,render,ui}-tests.txt` |
| ARM Debug 构建 | 成功，Ninja 报告无需重建 | `cmake --build --preset Debug` 退出码 0 |
| 工作区 | 检查开始时 `git status --short` 为空；本次只新增本清单，测试产物留在忽略的 `build/` | Git 状态 |

复现命令（仓库根目录 PowerShell，本机已有工具链）：

```powershell
python tests/canopen/run_tests.py --optimize-scope --cc E:/MATLAB/R2023b/toolbox/sldrt/clang/win64/clang.exe --lld E:/MATLAB/R2023b/toolbox/sldrt/clang/win64/lld.exe
python tests/config_storage/run_tests.py --cc E:/MATLAB/R2023b/toolbox/sldrt/clang/win64/clang.exe --lld E:/MATLAB/R2023b/toolbox/sldrt/clang/win64/lld.exe
powershell -NoProfile -ExecutionPolicy Bypass -File tools/esc_studio/build.ps1 -Test
cmake --build --preset Debug
```

这些通过结果证明当前本地构建和已有回归未发现失败，不替代真实电机、Flash 掉电、长期压力或完整性能验收。本次未新增测试用例，也未重新部署。

## 7. 推荐后续顺序

1. 先关闭 HW-01 / HW-02：确认编码器接线整改，核查反馈和电流测量，复测零目标 PWM，最后完成限时低电流寻相。
2. 再做 CTRL-01 / CTRL-02 / CTRL-03：小目标逐项验收，并记录停止、心跳及断线结果。
3. 完成 RT-01、TEMP-01、PROT-01：真实控制实时预算、有效温度反馈和保护验证。
4. 安排 CFG-01、HW-03、REL-01、PERF-01：持久化、复位电路、长期可靠性与完整应用性能。
5. 补协议版本/字典核对、运动集成覆盖，再按需要实现配置事务、触发和其他 P2/P3 项。

记录维护：以 [MOTOR_COMMISSIONING.md](MOTOR_COMMISSIONING.md) 的最新现场为准；[HANDOFF_NEXT.md](HANDOFF_NEXT.md)、[hardware_validation.md](hardware_validation.md) 与 [host_tool_design.md](host_tool_design.md) 含大量历史阶段描述。后续更新应注明替代关系，避免再次把已完成的同步、FFT 或基础运动接口列为未实现。关键方案与证据应另做持久归档，因为 `build/` 不受 Git 跟踪。
