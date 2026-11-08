# 无电机 USB / J-Link 验证记录

日期：2026-11-08（Asia/Shanghai）。用户已授权必要的自主调试；板卡使用 24 V 独立供电，USB 和 J-Link SWD 已接，nRESET 已接板卡复位脚，未连接电机。

**最新：已开始实际电机的步骤 23，限压寻相及零目标 PWM 出现编码器故障 51，当前失能，等待接线整改，见 [MOTOR_COMMISSIONING.md](MOTOR_COMMISSIONING.md)。本文前面无电机测试均为历史阶段；NRST 后来已断开，目前仍保持断开。**

## 固件与备份

- J-Link PLUS，序列号 601012352；本机软件 V7.96t，VTref 约 3.12–3.14 V。识别为 STM32F405 Cortex-M4。
- 初始板上程序与当前工程不同。先在复位暂停条件下备份完整 1 MiB Flash：`build/hardware-validation/flash-before-debug.bin`，SHA256 `cd13e67f273701ed69ae8403799a3d43973d837e34f42433ef94cbbfb89afebb`；向量 `SP=0x200175F0`、`Reset=0x08000345`。
- `cmake --build --preset Debug` 通过。通过 J-Link `loadbin` 写入 `build/Debug/J_ESC.bin`（215,036 B），`verifybin` 成功。只涉及应用区域对应的前 256 KiB 擦写范围，无整片擦除。
- 烧录后读回 `0x080C0000` 起 256 KiB（配置扇区 10/11），与完整备份对应部分逐字节一致。证据：`config-after-load.bin`、`load-current.log`。
- 未发送使能、寻相、辨识、故障复位、应用参数或保存配置 Flash 命令。这里的“未保存 Flash”指业务配置命令，**不代表没有烧录应用固件**。

## 真实 USB 结果

协议 v2 在 COM3 协商成功。三轮吞吐测量取最低值，当前约 **211 kB/s** 样本负载；非 USB 理论带宽，不代表其他电脑/运行负载。

| 验证 | 实际结果 | 产物（均在 `build/hardware-validation/`） |
| --- | --- | --- |
| Python，3 通道、20 kHz、自动 | 选择间断，27 段、36,720 点、432 块，CRC 错误 0 | `usb-20k-3ch-live.jescscope` |
| Qt 生产采集核心，3 通道、20 kHz、自动 | 9 段、12,240 点，离线完整校验通过 | `qt-usb-20k-3ch-v2.jescscope` |
| Qt，1 通道、10 kHz、自动 | 选择连续，22,707 点，溢出 0，正常完成录制 | `qt-usb-10k-1ch.jescscope` / `.json` |
| Qt，ISR 统计 3 通道、20 kHz、自动 | 选择间断，16 段、20,801 点，溢出 0，正常完成录制 | `qt-usb-20k-isr.jescscope` / `.json` |
| Python 文件 → Qt 解析与 CSV | 36,720 点，CRC/设备时间/最终点数通过 | `python-to-qt.csv` |
| Qt 文件 → Python 导出 | 成功；早期短记录为 2,720 点 | `qt-to-python.csv` |

Qt 探针的总计时包含三轮启动前吞吐测量，实际采样时长短于探针运行总时长；不能用总时长倒推采样率。段内 tick / divisor 与 10/20 kHz 校验通过。

ISR 统计记录中的高频间隔均值约 49.9996 μs，最小约 34.86 μs、最大约 65.14 μs；当前已观察到 ISR 到达间隔抖动。`hrt_consume_us` 采样值最大约 21.16 μs，固件报告的累计最大值约 **21.21 μs**。这些是当前 Debug、失能、配置错误状态下的统计，不是电机运行、最坏嵌套中断或所有 16 通道的实时预算证明，也不是独立仪器测量。

最终只读状态见 `final-device-state.json`：母线约 23.84 V，`is_enabled=0`，`motor_state=4`，主故障 255、扩展故障 3。按当前 `motor.h` 枚举，扩展故障 3 为电机额定电压配置错误。未填写虚构电机参数来绕过配置检查；当前电流记录不能作为电机电流反馈有效性证明。

## J-Link 退出后复位异常（步骤 16 时的历史记录）

反复观察到：连接目标并复位/运行后 COM3 可用；退出 Commander 后 COM3 消失；再次接入 SWD 时内核无法暂停、Flash/外设读取为零，正常复位会报告 `S_RESET_ST never gets cleared`，随后硬件复位可恢复。

直接查询探针：目标连接外 `ShowHWStatus` 报告 `TRES=0 TRST=0`；目标连接并运行后报告 `TRES=1 TRST=1`。此证据指向复位线/探针退出状态，需要进一步检查 NRST 电平、接线和探针行为；目前不能据此确认具体损坏或根因。

尝试显式 `SetRESET` / `SetTRST`、`SetSkipDebugDeInit = 1`、`SetRestartOnClose = 0` 后仍会复现。未升级探针固件、修改 MCU 选项字节或读取保护。命令依据：[SEGGER Commander](https://kb.segger.com/J-Link_Commander)、[SEGGER 命令字符串](https://kb.segger.com/J-Link_Command_Strings)。

本轮收尾暂时保留 J-Link 会话并让 MCU 运行，USB 串口已关闭释放。接下来应通过实际测量或临时断开 nRESET 做对照，确认离开调试器后独立运行的问题。保留会话是临时措施，不能标记为已修复。

## 步骤 17：NRST 对照与独立计时修复（2026-11-08）

用户按要求断电后仅断开 J-Link 到板卡的 NRST 线，随后恢复供电。退出原先保留的 Commander 后，COM3 和设备读回持续正常；不再需要保留调试器进程。说明异常与这条复位连接直接相关。**这是隔离后的恢复，不是已查清并修复硬件根因**；还需确认探针/转接板引脚、NRST 电平及板上复位电路。尤其在 NRST 已断开的条件下，探针 `TRES=0` 不能代表 MCU 引脚电平，也不能单凭探针状态确定输出在主动拉低。

同时发现独立运行的计时缺陷：`DWT_Init()` 原先只定义、未调用；默认 J-Link 退出还会清除跟踪使能，导致 `core_hrt_interval_us` / `consume_us` 变为 0。修复如下：

- `Core/Src/main.c` 在启动控制中断前初始化 DWT，并在前台检查跟踪/周期计数使能，缺失时恢复；恢复过程保持 CYCCNT，不无条件清零计数。
- 恢复时保存/恢复 PRIMASK，短暂排除中断并重新建立计时间隔基线。ADC/TIM2 ISR 遇到无效计数器时不更新最大耗时；恢复后的第一个间隔以 0 表示未知，避免把测量间隙作为控制周期。
- 用户再次断电、重新上电，未启动 J-Link 即成功读回约 50 μs 高频间隔、正常耗时及递增控制计数；证据 `cold-boot-no-debug-state.json`。
- 连续三轮 SWD 接入/退出后 COM3 可用、控制计数递增、计时非零、使能保持 0。证据 `attach-detach-{1,2,3}.log` 与 `after-detach-{1,2,3}-state.json`。这三轮在后续采样性能优化前执行。
- 在没有 J-Link 进程时，10 kHz 单通道强制连续运行约 65 秒，记录 **649,656 点**，溢出 0，CRC/时间/最终点数校验与全部 CSV 导出通过。证据 `standalone-65s-10k.jescscope` / `.json` / `.csv` / `-validation.json`。
- `scope_probe` 的固定 20 秒总超时改为“请求时长 + 20 秒”，时长校验范围 1–3,600,000 ms；错误参数在打开串口前拒绝。65 秒用例实际覆盖了原超时缺陷。

所有文件在 `build/hardware-validation/`。65 秒测试是分钟级连续记录初测，不是数小时/慢盘/断线压力验收。

## 步骤 18：16 通道实时预算问题与修复（2026-11-08）

未优化的 Debug 固件在 8 通道、20 kHz 自动间断下收到 19,456 点 / 38 段，固件报告的累计高频最大耗时约 34.33 μs；16 通道收到 9,728 点 / 38 段，但最大耗时达到 **54.42 μs**，采样阶段高频中断间隔均值约 **99.81 μs**。因此 `cold-boot-20k-16ch.jescscope` 虽然格式完整，**不代表实际 20 kHz 通过**。tick 是回调计数，不能单凭 tick 校验掩盖实际控制周期丢失。

定位为 `-O0` 下 `od_scope_tick → od_read_prepared` 的逐通道访问开销。固件 CMake 仅对 `canopen/od.c` 与 `od_scope.c` 追加 `-O2`，其余 Debug 代码保持 `-O0 -g3`；没有通过修改采样率、放宽校验或改用未检查的变量指针来回避问题。代价是这两个文件的调试局部变量可能被优化。

优化后重新烧录并校验，16 通道、20 kHz 自动间断：

| 指标 | 结果 |
| --- | --- |
| 原始记录 | `optimized-20k-16ch.jescscope`，11,008 点 / 43 段，溢出 0 |
| 段内间隔统计 | 均值约 49.9990 μs，观察范围约 34.18–65.82 μs |
| 高频耗时 | 采样值最大约 21.74 μs，固件累计最大约 **21.77 μs** |
| 保守链路吞吐 | 约 204.8 kB/s；自动选择间断，未尝试将 16 通道原始数据强制持续上传 |
| 文件完整性 | Qt 离线 CRC、序号、tick、最终点数校验通过 |

统计详见 `standalone-timing-summary.json`；仍有中断到达抖动，这些是无电机、失能/配置故障条件下的固件统计，不是带载最坏实时性或独立仪器测量。

回归同时覆盖默认优化级别和 `--optimize-scope` 的 Python/C 测试（33 项 Python 与 C 标量/字典/CDC/协议/采样/命令回归）；Qt 原核心、采集核心通过优化后的真实 C 桥，无跳过。编译命令已核对两个热路径文件最后使用 `-O2`。

最终 ARM Debug 镜像 **212,572 B**，RAM 42,656 B、CCM 6,940 B；SHA256 `13418510206a3fbecf1bb7b701db05cc287cf39e94f6da3e88303fdcf13a6653`。烧录 `verifybin` 成功，配置扇区 10/11 与最初备份仍逐字节一致。`scope-optimized-load.log`、`config-after-timing-fix.bin` 为证据。

最终状态见 `step18-final-device-state.json`：失能、故障 255 / 扩展 3，原额定电压配置错误保持；没有发送使能、寻相、辨识、故障复位、应用参数或配置保存命令。没有 J-Link 或采集进程保留，串口已释放。NRST 目前保持断开，恢复这条线前仍需检查复位链路硬件原因。

## 下一步硬件验收

确认 NRST/探针/转接板电路原因；继续数小时记录、USB 运行中拔插及重连、慢盘/满盘、完整 GUI/USB/磁盘并行帧时。8/16 通道更多类型组合与实际电机负载下的 ISR 预算仍需验证。连接电机后的寻相、辨识、使能、带载控制和配置 Flash 保存另行验收。

## 步骤 19–21 本轮新增证据与交接（2026-11-08）

- 新运动接口与 Qt 方案/FFT/分通道显示代码已接入；真实板卡仅查能力、运动闲置状态、166 项配置和采样，没有提交真实运动目标或使能。
- `phase5-readonly-results.txt`：features=31、运动状态闲置；实际 166 项方案为 `phase5-device.jescconfig`，Qt 离线导入/导出为 `phase5-device-roundtrip.jescconfig`。没有把导入草稿应用到设备。
- `phase5-final-16ch.jescscope` / `.json`：20 kHz 自动间断，6,656 点 / 26 段、无溢出、完整校验通过；`phase5-timing-summary.json` 记录间隔均值约 49.9979 μs、范围约 34.27–65.74 μs、累计最大耗时约 23.10 μs。无运动/配置故障状态，不是带载或目标生效周期验收。
- `phase5-before-update.bin` 与 `phase5-final-before.bin` 保存烧录前完整 1 MiB；`phase5-final-config.bin` 与备份的配置扇区一致。保留备份，后续使用新的文件名。
- 板上最后 verify 的镜像为 216,484 B，本地 BIN 为 216,492 B（切换电流模式清除旧规划标志的 8 B 差异）；最后的桌面源码也比构建/部署产物新。当前尚未做最终同步，哈希和下一步命令详见 `HANDOFF_NEXT.md`。
- 交接时没有保留调试器、采集或测试进程，NRST 仍断开，串口释放。原失能/额定电压配置故障保持。

## 步骤 22：接手后的最终回归及同步（2026-11-08）

- 已重新运行交接要求的 Python/C 优化回归、`build.ps1 -Test -Deploy`、ARM Debug 构建，全部通过。Qt 核心 9、采集核心 9、RHI 3、UI 11 项无失败和跳过。
- 最新桌面构建与部署 EXE SHA256 一致：`ae2d82a5a3885c646b7652bb68c24d8ea19ce375f3600cc5f80cbd126846ca9b`。仅系统 PATH 的部署版演示采样 smoke 与真实记录回放 smoke 退出 0；演示采样使用 Direct3D 11 / RHI、显示 5,508 点。隐藏窗口首次未渲染导致退出 4，未算通过，相关日志保留。
- J-Link 更新最后的电流模式规划标志修正，BIN 216,492 B，SHA256 `a9f1e3729bb4bbe92daa6a16d1e12485d75f78dbbb39442949d3f90a8596bbc0`。`verifybin` 成功，读回应用字节一致；新完整备份 `phase5-sync-before.bin`，独立配置读回 `phase5-sync-config.bin` 与烧录前及初始备份一致。证据 `phase5-sync-install.*`、`phase5-sync-integrity.json`。
- Commander 将无前缀长度 `216492` 按十六进制解释，应用读回发生越过 Flash 末尾的部分读取提示，实际文件为完整 1 MiB；校验脚本确认应用前 216,492 B 一致。后续长度须用显式 `0x`，不依据该提示声称越界读取成功。
- 新固件查询 features=31、运动状态闲置；166 项配置只读导出 `phase5-sync-device.jescconfig`。16 通道 20 kHz 自动间断：`phase5-sync-final-16ch.jescscope`，7,936 点 / 31 段、零溢出，Qt 离线完整性校验通过。预检占用首次 3 秒探针，未生成有效记录；7 秒重跑成功，均保留日志。
- `phase5-sync-timing-summary.json`：段内高频间隔均值约 49.9966 μs，范围 34.25–65.82 μs，累计最大高频耗时约 23.05 μs。仍为无电机、失能/配置故障下的统计，真实运动、带载最坏实时性未验收。
- 最终 `phase5-sync-final-state.json`：使能 0，主故障 255、扩展 3。未发送真实运动目标、使能、寻相、辨识、故障复位、应用参数或配置保存。NRST 仍断开；无调试/采集/测试进程保留，串口释放。没有创建 Git 提交。

## 步骤 23：实际电机准备、限压寻相与编码器故障（2026-11-08）

用户已连接并固定空载电机，补齐 24 V、1 A 额定、3 A 相电流峰值、4000 rpm 额定、24 g·cm² 惯量、4 极对、Rs=0.534 Ω、Ld=Lq=247 μH、磁链 0.004498 Wb。转矩常数 0.039 N·m/A 口径未注明，测试沿用磁链换算而不混用。

修复 AS5048A 高频读取、SPI 启用、有界等待、无效帧与显式故障恢复；增加实际转速越限故障 157，修复无效参数导致热模型 NaN。最终保留原正惯量校验，控制热路径 `-O2`，SPI1 2.625 MHz。Python 33 项及 C 原回归、新驱动/保护测试和 Qt 四组回归全部通过。

最终固件 213,528 B，SHA256 `53bedd5d094dcaed815de979d86d644a16887114d77a5ebd717b8f076ff85c4c`，应用完整读回一致；配置扇区与初始备份一致。见 `phase6-final-integrity.json`、`phase6-optimized-install.*`。方案只写 RAM、未保存 Flash，断电后会丢失。

实际 ADC 失能电流噪声标准差约 0.49–0.75 A，暂停 SPI 后仍有相近 ADC 噪声；用户确认此板噪声较大。限压方案使用约 0.416 V 电压矢量上限、50 Hz 电流环、0.3 A 寻相目标和 60 rpm 测试速度边界。配置校准通过，但两次限压寻相分别在阶段 1/2 触发编码器故障 51。显式故障复位可恢复角度，寄存器缓存错误位为 2（无效命令）。零目标、模式 None 的短时 PWM 对照也产生角度/速度反馈异常并失能。没有继续发送电流/速度/位置目标，不能报告运动验收通过。

用户确认编码器 5 V 供电、SPI 线约 20 cm、与三相线并行且无屏蔽；已要求先断电整改走线，再复查失能读取、零目标 PWM 和寻相。尚无独立仪器证明电气根因。最后确认失能、故障 51、未寻相，NRST 仍断开，无调试/采集进程保留。详情及所有日志入口见 [实际电机调试记录](MOTOR_COMMISSIONING.md)。
