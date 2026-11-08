# CANopen 对象字典基础层

CSV 是对象定义的唯一来源，只有 `name`，没有独立的 `id`。每行描述一个数据项；生成器创建只读属性表和初始为空的变量指针表，不创建业务变量。用户在 `od_user.c` 中完成实际映射。

本模块提供对象字典基础能力，未实现 SDO、PDO、NMT、Heartbeat 或完整 CANopen 节点，也不会自动启动 CAN 通信。当前支持固定长度标量，暂不包含字符串、DOMAIN 和 EDS 导出；支持只写命令回调。

已通过 USB CDC 接入读写，配套 Windows 上位机见 [USB CDC 参数工具](../tools/od_monitor/README.md)。这是对象字典的 USB 调试接口，不代表已实现 CANopen 总线协议。

新增高速采集保留文本协议兼容，通过显式协商进入二进制 v2；参见 [v2 协议](../docs/usb_protocol_v2.md)、[参考采集工具](../tools/scope_client/README.md)及[实施交接记录](../docs/host_tool_design.md#13-实施记录)。[C++/Qt 上位机](../tools/esc_studio/README.md)已接入七个配置页、电机控制与 GPU 示波器；真实 USB 吞吐和控制中断预算尚需上板验证。

## 文件

| 文件 | 职责 |
| --- | --- |
| `object_dictionary.csv` | 轴0的166个配置参数、432个运行变量和17个只读最大子索引，另有11个控制/系统条目，共626项 |
| `../tools/generate_od.py` | Python 3.10+ 标准库生成器，无第三方依赖 |
| `generated/od_generated.h/.c` | 提交到仓库供查看和手动构建的生成结果，禁止手改 |
| `od.h/.c` | 类型绑定、查找、冻结、权限检查和字节序处理 |
| `od_user.h/.c` | 用户维护的真实变量映射，生成器不会覆盖 |
| `od_protocol.h/.c` | USB 文本请求及响应编解码 |
| `od_cdc.h/.c` | USB 缓存、主循环处理、状态保护及断线处理 |
| `od_commands.h/.c` | 只写命令回调、电机操作、参数应用及前台 Flash 保存 |

属性表为 `static const`，由链接器放入只读区；RAM 中保存各项的数据指针、可写标志、命令回调指针及字典状态。没有堆分配和第二份业务数据。`OD_NAME_<name>` 是从 name 派生的 C 枚举符号，其数值随排序变化，不应保存到 Flash 或作为通信地址。

## CSV 格式

必需列：`name,index,subindex,data_type,access`；可选列：`description`。

- UTF-8 编码，兼容 Excel 导出的 UTF-8 BOM。需要逗号或换行的说明应按标准 CSV 加引号。
- `name` 全局唯一，以英文字母开头，仅含字母、数字和下划线，建议小写加下划线；大小写敏感。
- `index` 范围 `0x0000`–`0xFFFF`，`subindex` 范围 `0x00`–`0xFF`，必须带十六进制前缀；二者组合唯一。
- `access` 严格使用 `ro`、`wo`、`rw`，均指通信侧权限，不限制业务程序本身。
- 不填写 C 变量路径、地址、初始值或单独的存储长度。标量长度由类型决定。

| data_type | C 类型 | 通信字节数 |
| --- | --- | --- |
| BOOLEAN | bool | 1（只允许 0 或 1） |
| INTEGER8 / UNSIGNED8 | int8_t / uint8_t | 1 |
| INTEGER16 / UNSIGNED16 | int16_t / uint16_t | 2 |
| INTEGER32 / UNSIGNED32 | int32_t / uint32_t | 4 |
| INTEGER64 / UNSIGNED64 | int64_t / uint64_t | 8 |
| REAL32 / REAL64 | float / double | 4 / 8 |

本层将索引和子索引视为扁平数据地址，不推断 VAR、ARRAY、RECORD，也不自动插入子索引 0。普通 VAR 使用子索引 0；定义数组或记录时，应由用户按相应 CANopen 对象语义显式列出必要的子索引 0 和其他条目。能够通过生成器校验不代表已满足 CiA 301/402 的完整对象要求。

## 生成和构建

在项目根目录执行：

```text
python tools/generate_od.py
python tools/generate_od.py --check
```

第一条更新仓库内的生成文件；第二条仅检查是否与 CSV 一致，过期时返回非零。还可指定 `--csv 路径 --out-dir 目录`。

生成器先校验全部输入，错误会标出数据行和名称，校验失败不修改现有输出。内容相同的文件不重复写入。

CMake 已接入自动生成，要求构建主机安装 Python 3.10+。构建使用构建目录内的 `canopen_generated` 文件，不修改源码目录；CSV 或生成脚本变化后自动更新。修改 CSV 后仍应手动执行一次生成命令，将仓库中的生成结果一起提交。正常固件构建还需要项目原有的 ARM GNU 工具链、CMake 和 Ninja。

## 用户映射

`od_user.c` 已将当前单轴板卡的 `Global_Configs_t` 全部七组配置绑定到实际 `global_config` 成员：

| 主索引 | 参数子索引 | 配置来源 | 参数数量 | 参数权限 |
| --- | --- | --- | --- | --- |
| `0x2000` | `0x01`–`0x1A` | `global_config.motor_config[0]` | 26 | rw |
| `0x2001` | `0x01`–`0x0B` | `global_config.encoder_config[0]` | 11 | rw |
| `0x2002` | `0x01`–`0x11` | `global_config.motor_controller_config[0]` | 17 | rw |
| `0x2003` | `0x01`–`0x08` | `global_config.motor_observer_config[0]` | 8 | rw |
| `0x2004` | `0x01`–`0x3A` | `global_config.loop_controller_config[0]` | 58 | rw |
| `0x2005` | `0x01`–`0x24` | `global_config.loop_filter_config[0]` | 36 | rw |
| `0x2006` | `0x01`–`0x0A` | `global_config.hfi_config[0]` | 10 | rw |

每组按 RECORD 方式组织，参数从子索引 `0x01` 起按结构体声明顺序排列，嵌套结构体按声明顺序递归展开为标量。子索引 `0x00` 为只读 `UNSIGNED8` 最大子索引，数值为上表各组最后的子索引，绑定到 `od_user.c` 中的静态常量。常量值由生成表中首尾条目的序号差计算，要求参数子索引连续。现有参数地址应保持稳定，新增参数应追加新的子索引。原有 `0x2000`、`0x2001` 地址保持不变。

新增条目名称使用组名前缀加字段路径，嵌套层级用下划线连接，例如 `loop_controller0_id_pid_config_kp` 对应 `global_config.loop_controller_config[0].id_pid_config.kp`。观测器的 `observser_config` 沿用现有结构体字段拼写。上位机重新加载 CSV 后即可访问新增条目。

枚举和 `uint32_t` 在线缆格式中使用 `UNSIGNED32`，普通 `int` 使用 `INTEGER32`，`float` 使用 `REAL32`。使能开关和 `auto_tune` 的实际类型为 `int`，所以不是 BOOLEAN。所有绑定引用原始变量；访问层保留实际 C 类型，不将 `int *` 强转为 `int32_t *`（GNU ARM 的 `int32_t` 可以是 `long`）。

当前 GNU ARM 默认将这些配置枚举保存为1字节，`od_user.c` 通过 `_Generic` 静态断言检查其与 `uint8_t` 兼容，使用显式 `od_bind_u8_as_u32()` 在读写时转换为4字节线缆格式；超出1字节存储范围的写入会被拒绝。业务结构体及原有配置持久化布局保持不变，不通过强制32位枚举编译选项修改 ABI。扩展多轴或改变枚举 ABI 时需要同步修改映射，当前有单轴及枚举类型静态检查。

### 运行变量

`Global_Status_t` 的运行变量从 `0x2100` 起映射，全部为 `ro`，嵌套结构和三相/αβ/dq 向量按声明顺序展开为标量。每组 `0x00` 仍为只读最大子索引。

| 主索引 | 数据子索引 | `global_status` 成员 | 变量数量 |
| --- | --- | --- | --- |
| `0x2100` | `0x01`–`0x0A` | `core_status` | 10 |
| `0x2101` | `0x01`–`0x08` | `power_status` | 8 |
| `0x2102` | `0x01`–`0x0A` | `peripheral_raw_status` | 10 |
| `0x2103` | `0x01`–`0x20` | `motor_basic_status[0]` | 32 |
| `0x2104` | `0x01`–`0x1B` | `encoder_status[0]` | 27 |
| `0x2105` | `0x01`–`0x58` | `motor_status[0]` | 88 |
| `0x2106` | `0x01`–`0x15` | `motor_observer[0]` | 21 |
| `0x2107` | `0x01`–`0x80` | `loop_controller_status[0]` | 128 |
| `0x2108` | `0x01`–`0x4E` | `foc_status[0]` | 78 |
| `0x2109` | `0x01`–`0x1E` | `hfi_status[0]` | 30 |

映射边界遵循头文件中 `not in od` 和“以下部分暂时不介入对象字典”注释，注释之后的内部字段不暴露；独立于 `Global_Status_t` 的滤波器历史缓冲区、轨迹规划器、正弦发生器及 PLL 等全局对象未纳入这组映射。原始采样数组和 PWM 时间数组映射当前单轴的 `[0]` 元素，名称以 `_0` 结尾。

运行变量绑定直接保留 `const volatile` 语义。1字节状态枚举经只读接口 `od_bind_ro_u8_as_u32()` 转换成 `UNSIGNED32`；`FOC_Run_State_t` 的数值包含 `0x21000001`，保持原生32位枚举访问，并有独立 ABI 静态检查，避免截断。`err_code` 使用 `UNSIGNED16`，`bool` 使用 `BOOLEAN`，以 `int` 保存的状态标志仍使用 `INTEGER32`。

USB CDC 允许电机运行时读取这些变量；写入返回 `OD_ACCESS_DENIED`，也不能通过字典覆盖控制状态、给定值或 PID 积分状态。每次读取一个标量时使用短临界区；连续读取多个参数并非同一控制周期的快照。配置项仍只允许停机失能时写入，状态变量不写入配置 Flash。

上位机重新加载 CSV 后即可查看新增变量。字典只反映当前固件变量值，不会补齐尚未实现的温度换算或其他采集逻辑。

`od_user_init()` 执行绑定并冻结字典；应在单线程初始化阶段调用一次，重复调用返回 `OD_FROZEN`。`main()` 已在业务初始化和配置加载之后通过 `od_cdc_init()` 调用它；主循环调用 `od_cdc_poll()` 处理 USB 请求。不要再单独重复调用 `od_user_init()`。

后续增加条目时，在 CSV 中声明属性，再在本文件中绑定，例如已有额定电压条目的等价写法为：

```c
OD_Result result = OD_BIND(od_dictionary, motor0_rated_voltage,
                          &global_config.motor_config[0].rated_voltage);
if (result != OD_OK) return result;
```

该条目已在当前映射函数中绑定，不应再次添加这段代码。

- `OD_BIND` 接受可写指针，保留 `volatile`；CSV 为 `ro` 时仍禁止通信写入。
- `OD_BIND_RO` 接受普通、`const`、`volatile` 或 `const volatile` 指针，只允许用于 CSV 中的 `ro` 条目。
- `od_bind_ro_u8_as_u32()` 用于只读的1字节枚举，在通信侧提供4字节 `UNSIGNED32` 值，不授予写入权限。
- C11 `_Generic` 按指针类型选择绑定函数；声明为 REAL32 的条目绑定到 `uint32_t` 时返回 `OD_TYPE_MISMATCH`。`void *`、结构体指针和使用 `OD_BIND` 传入的常量指针会编译失败。不要通过强制类型转换绕过类型检查。
- 每次绑定都要检查返回值。重复绑定返回 `OD_ALREADY_BOUND`，不会替换已有映射。
- 变量必须具有足够长的生命周期，通常为全局或静态变量；不能使用临时局部变量、位域或未对齐的 packed 成员。

函数末尾保留：

```c
return od_freeze(&od_dictionary, first_missing);
```

所有 CSV 条目都必须绑定。冻结失败时，`first_missing` 可获得首个未绑定条目的描述；用户可读取其 `name/index/subindex` 定位遗漏。全部绑定成功才进入冻结状态，之后禁止再绑定，也没有运行时重置接口。

启动时，在业务变量初始化、配置加载完成之后、启动通信之前调用 `od_user_init()` 并检查返回值。失败时保持通信服务未启动。当前代码不替用户猜测哪些控制变量可以直接写入。

## 读写

```c
uint8_t buffer[4];
OD_Result result = od_read(&od_dictionary, 0x2000, 0x01,
                           buffer, sizeof(buffer)); /* motor0_type: UNSIGNED32 */
```

`od_write()` 接收相同形式的地址和输入缓冲区。所有值使用小端线缆格式，例如 REAL32 的 12.5 为 `00 00 48 41`；不会直接发送 C 结构体。输入输出长度必须与类型完全匹配。读取失败不修改输出缓冲区，写入失败不修改目标变量。

`od_find()` 可在冻结前查询元数据，找不到返回空指针。查找按索引、子索引二分进行。读写会区分索引不存在、子索引不存在、权限错误、长度错误、未绑定以及尚未冻结；返回值是本模块状态，不是 SDO abort code。未来由协议层转换为对应的 SDO 响应。8 字节类型也不意味着可以直接使用 expedited SDO，应由协议层选择合适的传输方式。

## 并发、写入约束与持久化

绑定和冻结必须在单线程初始化阶段完成。运行期间调用方负责串行访问，以及与中断、控制任务之间的同步。实现通过类型化访问保留 `volatile` 语义，避免逐字节读取共享变量；这不保证 64 位访问或多字段快照的原子性。

当前配置条目都是直接访问 RAM 的 rw 映射。核心 `od_write()` 只校验属性、长度及 BOOLEAN 值，不校验电机状态、枚举取值、单位、参数范围或浮点数的 NaN/Inf，也不执行写后重计算。USB 适配层额外核对类型，并在短临界区内检查电机失能及 `MS_IDLE`/`MS_Err` 状态后执行写入；读操作也在短临界区内完成。上位机会拒绝 NaN/Inf 和类型数值溢出，业务范围检查复用电机、编码器及控制器初始化逻辑；通信写参后标记为待应用，须执行下述应用命令，成功后才能使能或保存。应用前检查配置 REAL32 的 NaN/Inf。

通信写入只改变 RAM。保存参数可写入 `0x1000:02`，命令层在前台、中断开启时显式调用 `config_storage_save()`；须失能且参数已应用。不能在每次字典写入时自动擦写 Flash。

## 验证

仅运行 Python 校验：

```text
python tests/canopen/test_generator.py
```

使用本机 C11 编译器编译并执行完整测试：

```text
python tests/canopen/run_tests.py --cc clang
```

Windows 缺少 CRT 开发环境时可增加 `--lld <lld.exe路径>`。只有 ARM Compiler for Embedded 时可运行编译验证：

```text
python tests/canopen/run_tests.py --cc <armclang.exe路径> --arm --compile-only
```

使用项目的 GNU ARM 工具链进行同类编译检查：

```text
python tests/canopen/run_tests.py --cc arm-none-eabi-gcc --arm-gcc --compile-only
```

编译验证不会运行 C 断言。C 测试使用固定测试 CSV，修改产品 CSV 不必同步修改测试夹具。测试覆盖漏绑定、冻结、重复绑定、错误类型、只读/只写权限、地址缺失、非法指针的编译拒绝、所有支持标量的线缆格式，以及协议分片、畸形命令、USB 发送忙、缓冲区生命周期、接收溢出、状态限制和会话重置。Python 测试还覆盖上位机编解码和超时不重试。

## 电机控制与系统命令

以下地址为本项目自定义调试接口，不属于标准 CiA 301/402 对象分配。既有 0x2000/0x2100 地址保持不变。单轴编号为0，未增加 MCU 重启或恢复出厂设置功能；“复位”指电机故障复位。

| 地址 | 名称 | 类型/权限 | 数值与语义 |
| --- | --- | --- | --- |
| 0x0000:00 | control0_max_subindex | UNSIGNED8 ro | 4 |
| 0x0000:01 | control0_enable | UNSIGNED32 wo | 1使能、0失能；重复使能不重置正在执行的操作 |
| 0x0000:02 | control0_align | UNSIGNED32 wo | 写1启动寻相 |
| 0x0000:03 | control0_identify | UNSIGNED32 wo | 1电阻、2电感、3极对数、4磁链；5齿槽、6惯量、7摩擦、8重力返回不支持 |
| 0x0000:04 | control0_identify_state | INTEGER32 ro | FOC辨识内部阶段 |
| 0x1000:00 | system_max_subindex | UNSIGNED8 ro | 5 |
| 0x1000:01 | system_fault_reset | UNSIGNED32 wo | 写1复位电机故障；须失能；重新检查电机配置 |
| 0x1000:02 | system_save_flash | UNSIGNED32 wo | 写1同步保存；成功响应表示底层保存成功（相同内容不擦写） |
| 0x1000:03 | system_apply_config | UNSIGNED32 wo | 写1应用RAM参数并重新初始化电机、编码器及控制器 |
| 0x1000:04 | system_flash_status | UNSIGNED32 ro | 最近加载/保存状态；使用 Config_Storage_Status_t 数值 |
| 0x1000:05 | system_config_pending | UNSIGNED32 ro | 1表示通信配置已修改或应用失败，禁止使能/保存；0表示无待应用参数 |

命令使用 `od_bind_command()` 绑定，不创建待执行的RAM命令字。`od_write()` 在校验地址、权限、长度及冻结状态后同步调用回调；读WO字段被拒绝。核心绑定仅允许 UNSIGNED32 WO。调用者必须在前台、中断开启时串行调用命令，不能给整个 `od_write()` 包裹关中断临界区；命令回调自身负责电机状态同步。USB层自动区分命令与普通变量访问。配置写入与命令由同一个前台消费者处理，Flash保存期间不会并发处理使能或写参。

使用流程：停机失能 → 写入参数 → 应用参数 → 等待采样校准结束（如需要）→ 使能 → 寻相或辨识。应用会清除已寻相标志；使能仅进入无运动模式。每次切换寻相/辨识前先失能再使能，以清理前一操作状态；磁链辨识需要已有有效寻相结果。DC辨识和DTC控制未接入。已有故障时失能保持故障锁存；初始化/采样校准期间失能命令返回忙，避免破坏校准状态机。应用和保存要求失能且为 IDLE/Err。

寻相/辨识成功响应仅表示已启动，算法仍由实时循环执行。通过“读取控制状态”检查故障、寻相标志、当前模式及辨识阶段，并读取相应配置确认结果。电阻通常终止于阶段5、电感阶段4、极对数阶段8、磁链阶段5；必须同时确认无故障，不能单凭阶段号判断成功。辨识结果由原有算法写回RAM参数，需要失能后显式保存才能持久化。

新增返回码：0C=状态忙，0D=当前类型/算法不支持，0E=操作失败（保存失败时读取 0x1000:04），0F=配置无效或待应用。上位机Flash保存超时为10秒，其他请求默认1.5秒；超时不重试，重连后先读取状态及参数。USB忙重发的是响应缓冲区，不重新执行命令。

命令回归测试使用产品CSV和真实OD/命令实现，电机及Flash接口通过桩模拟，覆盖状态门控、重复使能、活动操作互斥、未应用/非法配置、未实现辨识拒绝、前台限制以及Flash错误传递。CDC测试还覆盖运行时命令、USB忙时不重复执行及错误类型拒绝。当前已通过18项Python测试、本机C运行测试、配置存储C回归测试、隐藏Tk窗口控件检查与GNU ARM编译链接；真实USB及电机/Flash动作仍需上板验证。
