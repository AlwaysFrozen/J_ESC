# 配置持久化

`config_storage_save()` 将当前 `global_config` 的全部 `Global_Configs_t` 数据保存到片内 Flash，断电后可恢复。运行反馈 `global_status` 不保存。

## 调用

```c
#include "config_storage.h"

/* 在主循环或命令处理任务中执行。先停止、失能电机，等待进入 MS_IDLE/MS_Err。 */
Config_Storage_Status_t result = config_storage_save();
if (result != CONFIG_STORAGE_OK)
{
    /* 向调用者报告失败；不能将失败当成已保存。 */
}
```

必须在前台线程、中断开启时调用。所有电机必须失能且处于 `MS_IDLE` 或 `MS_Err`；初始化、校准、运行及中断上下文均返回 `CONFIG_STORAGE_BUSY`。从接口进入到返回期间，调用方必须串行处理配置修改及电机使能命令。配置有变化时才写入，相同内容不会擦写。

启动流程已接入：`global_init_default()` → `config_storage_load()` → `motor_init()` → `encoder_init()`。加载只允许在初始 `MS_Init` 状态、控制中断和逆变器尚未启动时调用。失败不会修改 `global_config`；通过 `config_storage_last_status()` 可读取自动加载的结果。现有默认初始化仍只清零，本模块不猜测具体电机参数；首次使用需要先配置有效参数再保存。

| 返回值 | 含义 |
| --- | --- |
| `CONFIG_STORAGE_OK` | 保存/加载成功，或当前配置与已保存内容相同 |
| `CONFIG_STORAGE_EMPTY` | 两个记录头都为空白，尚无已提交配置 |
| `CONFIG_STORAGE_INCOMPATIBLE` | 已提交记录的版本或配置长度不匹配 |
| `CONFIG_STORAGE_CORRUPT` | 记录不完整、标识错误或 CRC 错误 |
| `CONFIG_STORAGE_BUSY` | 电机状态、调用上下文不允许，或已有操作进行中 |
| `CONFIG_STORAGE_FLASH_ERROR` | Flash 解锁、擦除、编程、加锁失败或容量不足 |
| `CONFIG_STORAGE_VERIFY_ERROR` | 写后读回校验失败 |

## 存储布局与断电恢复

按 STM32F405 1 MB Flash 布局，保留两个独立的 128 KB 扇区：

| 区域 | 地址 | 用途 |
| --- | --- | --- |
| 扇区 0–9 | `0x08000000`–`0x080BFFFF` | 固件，链接容量调整为 768 KB |
| 扇区 10 | `0x080C0000`–`0x080DFFFF` | 配置 A |
| 扇区 11 | `0x080E0000`–`0x080FFFFF` | 配置 B |

地址和容量由链接脚本符号提供；Flash 后端擦除扇区 10/11。更换芯片或调整分区时必须同时检查链接脚本与后端扇区选择。布局依据：[ST RM0090](https://www.st.com/resource/en/reference_manual/rm0090-stm32f4xx-reference-manual-stmicroelectronics.pdf)。

记录头包含 magic、版本、长度、递增序号、CRC32 和提交标记。CRC 覆盖头部元数据和完整配置。先擦除非活动扇区、写入头部和配置、读回校验，最后写提交标记；旧的有效扇区不变。启动时选取序号最新且校验通过的记录，新记录未提交或损坏时回退旧记录。首次保存中断时尚无旧记录，启动会继续使用默认初始化结果。

每次有变化的保存会擦除一个扇区，不适合高频或周期性自动保存。STM32F405 是单 Bank Flash，擦写会阻塞取指和中断响应；必须在电机停机后执行。后端使用 32 位编程，要求 MCU 电源在 2.7–3.6 V 范围内（本板为 3.3 V）。

配置为当前 ABI 下的结构体快照。修改 `Global_Configs_t`、任意嵌套结构、枚举含义或影响布局的编译选项时，必须增加 `CONFIG_STORAGE_VERSION`，即使结构体总大小没变。本版本拒绝旧格式，不自动迁移参数。

烧录固件时应保留配置扇区；整片擦除、读保护级别变更导致的擦除，或其他工具对这两个扇区的写入都会清除配置。加锁失败时接口会报告错误，此时配置也可能已成功提交，应读取/重启确认。

## 验证

`tests/config_storage/run_tests.py` 使用真实存储逻辑与可注入失败的 Flash 模拟后端，覆盖空白加载、状态限制、去重、逐字写入中断、擦写错误、写后校验、损坏回退、版本/长度拒绝及序号回绕。测试用两轴和非 4 字节对齐长度的模拟配置检查边界行为。

```text
python tests/config_storage/run_tests.py --cc clang
```

Windows 无系统 C 运行库时可增加 `--lld <lld.exe路径>`。真实硬件仍需验证保存后断电重启、保存中掉电恢复，以及烧录时保留配置扇区。

## 通过USB上位机保存

参数工具已接入 `0x1000:02` 只写命令，写 UNSIGNED32 的1同步调用 `config_storage_save()`。界面点击“保存Flash”即可，固件要求停机失能且为IDLE/Err，并要求通信写入的参数先通过“应用参数”（`0x1000:03`）。返回成功后才显示保存成功；底层错误通过命令返回码和 `0x1000:04` 状态字段报告。断线/超时不自动重试，也不会因逐项写参自动保存。
