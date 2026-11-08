# 高速采集参考客户端

这是第一阶段的标准库验证工具，配合新增固件 v2 使用。[C++/Qt 图形上位机](../esc_studio/README.md)已提供配置、电机控制和 GPU 示波器；本工具继续承担独立采集验证与文件记录/导出。设计、已完成步骤及接手入口见 [交接文档](../../docs/host_tool_design.md)，字节格式见 [协议 v2](../../docs/usb_protocol_v2.md)。

在仓库根目录运行，Python 3.10+；真实 COM 后端当前支持 Windows，无第三方 Python 依赖。新固件需要先由用户烧录；工具不自动烧录、不发送电机运动命令。

```powershell
python -m tools.scope_client ports
python -m tools.scope_client capture --port COM3 --channels motor_basic0_phase_current_U,motor_basic0_phase_current_V,motor_basic0_phase_current_W --rate 20000 --mode auto --seconds 10 --output build/current.jescscope
python -m tools.scope_client export build/current.jescscope build/current.csv
```

- 默认使用仓库 `canopen/object_dictionary.csv`，可用 `--csv` 指定与固件对应的字典。目前设备不核对完整字典哈希，文件内明确记录该限制。
- 当前板级设置支持 `--rate 10000` 或 `20000`。仅支持最多 16 个、32 位及以下的只读运行变量，不支持配置项和 64 位通道。
- `--mode auto`：先测三轮 CDC 吞吐，取最小值，保留 30% 余量并计入分块开销，选择连续或间断。连续溢出后排空当前段、转间断，本次采集不自动返回连续。
- `--mode continuous`：强制连续；缓存溢出时报错，保留已记录文件。`--mode burst`：反复以目标频率采集一段，再上传，段间存在空隙。
- `--seconds` 是采集阶段的主机时间预算，不含开始前的吞吐测量、末段排空或 CSV 导出；采集停止后仍需等待剩余数据上传。
- 固件缓存为 16 × 1 KiB，段长依通道步长变化。GPU 绘图不在本工具范围内。
- 文件以独占创建方式写入，不覆盖已有路径。原始记录与 CSV 导出均流式处理。
- 原始文件保留设备 tick、配置/段/块号与最终采集状态；CSV 保留段号、设备 tick、设备时间，段间插入空行。
- 通讯超时或短写后停止本次会话，不自动重复命令。CRC 错误后能恢复帧边界，但缺块会使采样完整性检查失败。
- DTR 下降重置协议会话；关闭工具停止的是采集，不自动改变电机运动状态。

## 本机验证

```powershell
python tests/canopen/run_tests.py --cc E:/MATLAB/R2023b/toolbox/sldrt/clang/win64/clang.exe --lld E:/MATLAB/R2023b/toolbox/sldrt/clang/win64/lld.exe
cmake --build --preset Debug
```

示例 Clang/LLD 路径来自本机已安装工具，其他环境可指定自己的编译器。完整运行器会构建 C 测试桥，由 Python 通过 ctypes 驱动真实 C 协议、CDC 调度和采样器；不访问真实 USB 或电机。

只运行 `python tests/canopen/test_scope_client.py` 时，不带 C 桥路径会跳过硬件无关的 C/Python 集成部分；完整验证应运行上面的运行器。软件通过不代表已验证真实 USB 吞吐、控制中断预算或 GPU 性能。
