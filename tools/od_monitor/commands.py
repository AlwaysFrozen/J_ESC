"""Product command ABI. Require matching CSV metadata before enabling a button."""
from dataclasses import dataclass


@dataclass(frozen=True)
class Command:
    label: str
    name: str
    index: int
    subindex: int
    value: int
    result: str

    def matches(self, entry):
        return (entry.name, entry.index, entry.subindex, entry.data_type, entry.access) == (
            self.name, self.index, self.subindex, "UNSIGNED32", "wo")


COMMANDS = {
    "enable": Command("使能", "control0_enable", 0, 1, 1, "已使能，尚未启动运动"),
    "disable": Command("失能 / 停止", "control0_enable", 0, 1, 0, "已失能"),
    "align": Command("启动寻相", "control0_align", 0, 2, 1, "寻相已启动；请读取控制状态确认完成"),
    "fault_reset": Command("故障复位", "system_fault_reset", 0x1000, 1, 1, "故障复位已执行；请读取状态确认"),
    "save": Command("保存 Flash", "system_save_flash", 0x1000, 2, 1, "Flash 保存成功"),
    "apply": Command("应用参数", "system_apply_config", 0x1000, 3, 1, "参数已应用；未校准时将开始采样校准"),
}
IDENTIFICATIONS = {"电阻 Rs": 1, "电感 Ld/Lq": 2, "极对数": 3, "磁链": 4}
IDENTIFY = Command("启动辨识", "control0_identify", 0, 3, 1, "辨识已启动；请读取控制状态确认完成")
STATUS_NAMES = (
    "motor_status0_motor_state", "motor_status0_is_enabled", "motor_status0_is_calibrated",
    "motor_status0_is_aligned", "motor_status0_is_error", "motor_status0_motor_ctrl_mode",
    "motor_status0_err_code", "motor_status0_ext_err_code",
    "foc_status0_align_state", "control0_identify_state",
    "system_flash_status", "system_config_pending",
)
STATUS_LABELS = dict(zip(STATUS_NAMES, (
    "电机状态", "使能", "采样已校准", "已寻相", "故障", "控制模式", "故障码", "扩展故障码",
    "寻相阶段", "辨识阶段", "Flash结果", "参数待应用",
)))


def format_status(entry, value):
    if entry.name == "motor_status0_motor_state":
        value = {"0": "初始化", "1": "采样校准", "2": "空闲", "3": "运行", "4": "故障"}.get(value, value)
    elif entry.name == "system_flash_status":
        value = {"0": "成功", "1": "空白", "2": "不兼容", "3": "损坏", "4": "忙",
                 "5": "擦写失败", "6": "校验失败"}.get(value, value)
    return f"{STATUS_LABELS.get(entry.name, entry.name)}={value}"


def find_command(entries, command):
    return next(((i, entry) for i, entry in enumerate(entries) if command.matches(entry)), None)


def is_command(entry):
    return any(command.matches(entry) for command in (*COMMANDS.values(), IDENTIFY))
