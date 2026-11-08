# 下一位接手者先读这里

交接日期：2026-11-08（Asia/Shanghai）。**接手后的构建、回归、桌面部署与板上固件同步已完成，最新结果见下节。** 三个方向的基础功能已实现；真实运动、带载实时性和长期压力验收仍未完成。

**最新步骤 23：已连接固定空载电机和 AS5048A；用户补齐额定 4000 rpm、转子惯量 24 g·cm²，其余为 24 V、额定 1 A、相电流峰值 3 A、4 极对及已给 Rs/Ld/Lq/磁链。参数已写 RAM 并应用，低压寻相两次及零目标 PWM 对照均出现编码器故障 51，当前已失能，停止重试，等待用户断电整改接线。编码器 5 V、20 cm SPI 线与三相线并行且无屏蔽，是强疑点但非已证明根因。先读 [MOTOR_COMMISSIONING.md](MOTOR_COMMISSIONING.md)，旧“未使能/未接电机”仅描述此前测试。**

步骤 23 最新固件为 **213,528 B**，SHA256 `53bedd5d094dcaed815de979d86d644a16887114d77a5ebd717b8f076ff85c4c`，已烧录并完整读回一致。配置 Flash 与初始备份未变，电机参数只在 RAM，断电后须重新载入 `phase6-optimized-profile.jescconfig`。最新 Python/C（含驱动及保护测试）与 Qt 四组回归通过，ARM 构建通过。修复了 AS5048A 未主动读取/未启用 SPI、流水线及错误恢复，增加真实转速越限故障 157 和热模型无效参数保护；真实惯量已提供，最终保持原正惯量校验。控制热路径另启用 `-O2`，SPI 降至 2.625 MHz。无调试或采集进程残留，NRST 仍断开。步骤 23 的镜像和现场优先于下文步骤 22 的历史状态。

## 接手收尾结果（2026-11-08，优先于下文历史状态）

- 已重新执行交接指定的三条命令：Python 33 项与 C 回归通过；Qt 四组 CTest 全通过（核心 9、采集核心 9、RHI 3、UI 11，无跳过）；ARM Debug 构建成功，Ninja 确认无需重建。桌面最新 `Backend.cpp` 和 QML 用例均已编译、测试、部署。
- 构建与部署 EXE SHA256 均为 `ae2d82a5a3885c646b7652bb68c24d8ea19ce375f3600cc5f80cbd126846ca9b`。部署目录：`build/esc-studio/package/`。仅系统 PATH 下 demo/scope smoke 退出 0，Direct3D 11 / RHI，显示点 5,508；真实记录回放 smoke 退出 0。证据 `package/sync-visible-demo-result.txt`、`sync-replay-result.txt`。最初隐藏窗口 demo 退出 4（窗口未渲染、显示点 0），不计作通过。
- 已复看四张 `phase5-*.png`。这些仍为软件回退 UI 测试截图；独立 RHI 测试和部署 smoke 覆盖 GPU。分通道区域需滚动查看其余通道，未据此声明所有通道完整应用帧时验收。
- 板上已同步 **216,492 B** BIN，SHA256 `a9f1e3729bb4bbe92daa6a16d1e12485d75f78dbbb39442949d3f90a8596bbc0`，`verifybin` 成功，读回应用区与 BIN 逐字节一致。新完整备份 `phase5-sync-before.bin`；配置扇区 `phase5-sync-config.bin` 与烧录前及最初 `flash-before-debug.bin` 均一致。证据 `phase5-sync-install.jlink` / `.log`、`phase5-sync-integrity.json`。
- 读回脚本中长度 `216492` 被 Commander 按十六进制解析，日志包含越过 Flash 末尾的部分读取提示；实际取得完整 1 MiB `phase5-sync-readback.bin`。已核对长度，严格比较前 216,492 B 应用及独立读取的配置扇区，不将越界读取视为成功证据。后续长度必须显式带 `0x`。
- 更新后 features=31、运动状态全部为 0（闲置），只读 166 项配置保存到 `phase5-sync-device.jescconfig`。没有提交运动目标、使能、寻相、辨识、故障复位、应用参数或保存配置 Flash。
- 最新 16 通道 20 kHz 自动间断记录 `phase5-sync-final-16ch.jescscope`：**7,936 点 / 31 段，溢出 0、完整校验通过**。段内高频间隔均值约 49.9966 μs，观察范围 34.25–65.82 μs，固件累计高频最大约 23.05 μs。见 `phase5-sync-final-validation.json`、`phase5-sync-timing-summary.json`。首次 3 秒探针在吞吐预检结束前到期，未生成有效采样，保留失败日志 `phase5-sync-16ch.json`，随后以 7 秒重跑成功。
- 最终 `phase5-sync-final-state.json` 确认失能、故障 255 / 扩展 3，NRST 接线未改变。无 J-Link、采集、桌面或 UI 测试进程保留，串口已释放。未创建 Git 提交，也未清理用户工作区。

本轮同步已收尾，无需再次执行下面历史待办。下一步按用户后续指示选择：真实电机/轨迹和目标生效耗时验收、NRST 电路原因检查、长期录制/慢盘/断线恢复、触发或配置事务等。保持本文件列出的功能边界，不能把无电机失能测试当作整个项目验收。

## 用户目标与现场

- 按顺序继续：① 参数方案管理与界面增强 → ② 基础电流/速度/位置控制 → ③ FFT 与多坐标轴。不要重新询问是否继续这些已授权工作。
- 用户已授权必要的 SWD/USB 调试及固件烧录；真实板卡使用 24 V 独立供电，USB COM3，J-Link SWD 已接。用户已物理断开 NRST，电机没有连接。保持这个接线状态，不需要保留 J-Link 进程。
- 本轮真实板卡只查询能力、运动状态、166 项配置和采样；未提交运动目标、未使能/寻相/辨识/故障复位/应用参数/保存配置 Flash。演示界面的这些动作只发生在本地模拟设备。
- 板卡当前仍失能，主故障 255 / 扩展 3，含义为额定电压配置错误。不要填虚构参数绕过，也不要将无电机结果当作带载验收。
- Git 中有大量之前就存在的修改与未跟踪代码。没有创建提交；不要 reset/clean 或覆盖原工作区。

## 已写入的代码

### 1. 参数方案与界面

- `tools/esc_studio/src/ParameterScheme.*`：完整七组 166 项 `.jescconfig` JSON；本地字典 SHA256、地址/类型/条目集合/重复项/数值校验；先差异预览、后载入草稿；预览后数据或字典改变则拒绝旧预览。导入不自动写设备，写入/应用/Flash 仍显式执行。导出要求每项已读取或有草稿，不补零，拒绝已有目标文件。
- `Dictionary.*` 增加单位、分组、枚举选项角色和字典哈希。枚举按源码核对，电机 `motor0_type` 为 **0=BLDC 三相，1=DC**。不要误用旧测试桩枚举的顺序。
- `ParameterFilter` 增加分组筛选，QML 参数页增加完整读取、方案保存/预览/载入、写全部修改、已知枚举下拉和单位显示。仍不是完整折叠树，也未覆盖所有参数依赖/业务范围联动。
- `Backend` 接入方案对象及全组读取/写修改。真实板卡读取的方案在 `build/hardware-validation/phase5-device.jescconfig`，Qt 离线导入/再导出成功，输出 `phase5-device-roundtrip.jescconfig`，166 项草稿。

### 2. 基础运动目标接口

- `canopen/od_motion.*`、`od_wire.h`、`od_cdc.c`、`od_commands.*`、`motor_control/hal/realtime.c`：新增二进制 `0x30` 提交、`0x31` 状态、`0x32` 心跳、`0x33` 停止；能力 features 当前 31，位 4 表示新接口。没有扩充 OD CSV，因此字典仍为 626 项。
- 完整 20 B 目标 payload，电流 A；旋转轴 deg / deg/s，直线轴 mm / mm/s，附相应加速度和 jerk。速度/位置调用现有规划器在前台准备，不在 ISR 内计算轨迹。
- 校验状态、配置、限幅、非有限值、轨迹系数/时长；发布前后核对 10 ms 快照时效。取消代号防止轨迹准备中断线、随后又发布旧命令。
- 高频 ADC 回调在 `RETTOBASE` 表示没有其他被挂起 ISR 的边界发布，避免替换被 TIM2 更新到一半的规划器。实时入口只复制准备好的结果并更新有限数量字段。
- 新接口拥有的活动运动须约 100 ms 心跳；500 ms 无心跳、断线或停止会失能。原手动使能/寻相/辨识的断线语义不因此改变。位置到达仍需续约。该机制依赖控制 ISR，不能称为硬件急停。
- Qt `Backend.cpp` 工作线程与 `MotorPage.qml` 接入能力门控、完整目标、状态与心跳、停止。最新一次修改修正了演示设备的实际模式/停止状态显示，**这次 Backend.cpp 改动尚未重新编译**。
- 仍未实现统一软件行程边界、自动回零、周期同步目标序列或扭矩等其他模式；真实电机执行、轨迹质量、目标生效周期最坏耗时未验证。

### 3. FFT / 统计与分通道坐标

- `ScopeAnalysis.*`：后台分析当前窗口内最后一段连续、有限的原始数据，不跨段/丢块/tick 缺口/NaN；32–65,536 点，取 2 的幂；均值、RMS（含 DC）、最小/最大、峰峰值。
- FFT 去均值、Hann 窗、窗增益归一化的单边幅值；主峰/分辨率按全部频点计算，显示取各桶最大值并保留 Nyquist 端点。已验证 625 Hz、幅值 2、DC=3 的原始信号及窗口外段不影响结果。
- `ScopeViewModel` 使用现有文件工作线程执行一次分析；`WaveformItem` 增加单通道选择、独立自动 Y、频谱点输入，继续使用场景图。QML 增加 FFT 对话框、分通道坐标/单位、共享时间轴与滚轮/游标；分通道区域宽度、颜色及隐藏的共用 Y 标签已修正。
- 当前是按按钮分析，不是自动实时 FFT；没有 PSD、对数轴、每通道手动 Y、任意单位换算，也未验证全部通道和长历史下的完整应用帧时。

## 验证与产物的准确状态

### 已通过

- Python 33 项与 C 字典/协议/CDC/采样/命令回归，通过 `--optimize-scope`；新增 `test_motion.c` 测真实命令邮箱模块，使用规划器/电机测试桩，覆盖门控、延迟到安全边界生效、取消、心跳续约/超时、位置到达、NaN、准备中取消。**不是实际电机或完整运动协议 C 桥端到端验收。**
- Qt 核心 9、采集核心 9、RHI 3 项通过；新增方案原子预览、哈希错误、过期预览及 FFT 数值测试。
- 最新单独运行的 UI 测试 11 项全通过，含实际鼠标启动录制、方案读取/载入、演示运动提交/停止、FFT 与分通道坐标。`build/esc-studio/ui-tests.txt` 为最新结果。
- 旧焦点用例曾间歇失败；本轮查到页面切换后旧编辑委托引用会失效。用例现在先等待页面完成 polish，再重新查找编辑框、确认窗口激活并建立焦点，仍跨 1.2 秒后台轮询检查，不削弱断言。最后这次 QML 测试修正尚未再执行一次完整 `build.ps1 -Test -Deploy`。
- `phase5-config.png` / `phase5-motor.png` / `phase5-fft.png` / `phase5-axes.png` 在 `build/esc-studio/`；已查看。测试程序补了与主程序一致的 Windows 字体载入，早先方框字体截图已被覆盖。这些截图走软件回退，GPU 由独立 RHI 测试证明。
- 真实板卡 features=31，`MOTION_STATUS` 为闲置状态；只读 166 项配置成功。十六通道 20 kHz 自动间断记录 6,656 点 / 26 段、溢出 0，完整校验通过：`phase5-final-16ch.jescscope` / `.json`。
- 上述记录高频间隔均值约 49.9979 μs，观察范围约 34.27–65.74 μs，固件累计最大耗时约 23.10 μs。仍是失能、配置故障下的统计，不能作电机带载实时性承诺。分析在 `phase5-timing-summary.json`。
- 烧录前完整备份和配置扇区读回在 `build/hardware-validation/phase5-before-update.bin`、`phase5-final-before.bin`、`phase5-final-config.bin`；配置扇区与备份一致。不要覆盖已有备份。

### 尚未同步的最后几处修改（历史，已由上述接手收尾解决）

- **桌面源码比 EXE 新**：`Backend.cpp` 最新演示模式显示修正尚未编译。最新 QML 焦点用例修正仅通过单独 UI 测试。
- **构建版与部署版 EXE 当前不同**：构建版 SHA256 `8444ce16fed183038b98e16c422b35f4b416732ada64c8202e28fa4930a66454`；部署版 `001712c53ac2e9b30e85730acb799394286229117266ac8f4d01a5425eedaba4`。后者仍是较早一次成功部署，不能当作最新源码产物。
- **ARM 本地 BIN 比板上新 8 B**：本地 `build/Debug/J_ESC.bin` 为 **216,492 B**，SHA256 `a9f1e3729bb4bbe92daa6a16d1e12485d75f78dbbb39442949d3f90a8596bbc0`，RAM 42,808 B、CCM 6,940 B。板上最后校验镜像为 216,484 B。差异为切入电流模式时清除旧 `is_planning` 标志；最后一次无运动目标采样结果对应板上前一版。
- 未创建提交。交接时没有 J-Link、采集或 UI 测试进程保留，串口已释放。

## 原接手执行步骤（历史，已执行完毕）

1. 读本文件及 `docs/usb_protocol_v2.md` 的新运动协议，快速审阅 `od_motion.c` 的并发、取消与门控。不重搭项目、不重装工具链。
2. 在仓库根目录运行以下命令，检查实际结果，失败则修复后再部署；不要根据旧日志宣布全通过：

```powershell
python tests/canopen/run_tests.py --optimize-scope --cc E:/MATLAB/R2023b/toolbox/sldrt/clang/win64/clang.exe --lld E:/MATLAB/R2023b/toolbox/sldrt/clang/win64/lld.exe
powershell -NoProfile -ExecutionPolicy Bypass -File tools/esc_studio/build.ps1 -Test -Deploy
cmake --build --preset Debug
```

3. 核对部署/构建 EXE 哈希一致、重看四张截图，必要时更新布局。可在只有系统 PATH 的环境运行部署版 `--demo --scope-demo --smoke-test` 或 `--replay 绝对文件路径 --smoke-test`，证明没有 SDK PATH 依赖。
4. 若继续硬件验证，可使用已授权的 J-Link 更新最后 8 B 差异并 verify。写新的备份和日志文件名，不重复覆盖 `phase5-final-install.jlink` 里已有的备份路径；确认配置扇区未改变。只读查能力/运动状态、采样并确认仍失能，不发送真实运动目标。
5. 完成同步后更新交接主文档，明确完成范围与仍未验证项。之后再考虑触发、长文件索引/取消、配置事务/固件字典哈希、额外运动模式、长期录制和全负载验证；不要把这轮三个基础功能当作整个项目已完成。

本轮完整说明已补入 `tools/esc_studio/README.md`、`docs/usb_protocol_v2.md`。旧主文档步骤 18 的“尚无基础运动/FFT”等状态已过时，以本文件与后续步骤 19–21 为准。
