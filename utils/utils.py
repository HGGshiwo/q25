import struct
from enum import IntEnum, unique
from typing import Optional, Union, List, Dict, Any
from dataclasses import dataclass
from typing import Union
import ctypes


def to_uint32(data):
    data = int(data)
    return ctypes.c_uint32(data).value


def to_int32(data):
    data = int(data)
    return ctypes.c_int32(data).value


# -------------------------- 核心枚举（控制指令+状态上报指令） --------------------------
@unique
class CommandType(IntEnum):
    """天狼Q25机器人指令枚举（控制指令+状态上报指令）"""

    # 控制指令（保留原有）
    MANUAL_HEARTBEAT = 0x21040001
    ENTER_L_MODE = 0x21010220
    EXIT_L_MODE = 0x21010221
    TOGGLE_STAND_DOWN = 0x21010202
    SET_AIM_POSE = 0x010B06  # 扩展指令（FireAim专属）
    RESET_AIM_POSE = 59
    MOTION_MODE_MANUAL = 0x21010C02
    MOTION_MODE_NAVIGATION = 0x21010C04
    MOVE_X_AXIS = 0x21010130
    MOVE_Y_AXIS = 0x21010131
    MOVE_YAW_AXIS = 0x21010135
    SET_PLATFORM_HEIGHT = 0x21010406
    GAIT_WALK = 0x21010420
    GAIT_RUN = 0x21010423
    SWITCH_SPEED_GEAR = 0x31010F11
    EMERGENCY_STOP = 0x21010C0E
    CAMERA_CONTROL = 0x21010F12
    CHARGE_REQUEST = 0x91910250  # response也是这个
    CHARGE_QUERY_STATUS = 0x91910253  # response也是这个

    # 状态上报指令（保留原有）
    RUN_STATUS_REPORT = 0x1008
    MOTION_STATE_REPORT = 0x1009
    SENSOR_DATA_REPORT = 0x100A
    CONTROLLER_SAFE_DATA_REPORT = 0x100B
    BATTERY_LEVEL_REPORT = 0x11050F01
    BATTERY_CHARGE_STATE_REPORT = 0x11050F21
    ERROR_CODE_REPORT = 0x12050F01


# -------------------------- 指令属性映射表（保留原有） --------------------------
command_info_map: Dict[CommandType, tuple[str, int, str]] = {
    # 控制指令
    CommandType.MANUAL_HEARTBEAT: ("手动模式下心跳指令", 0, "/heartbeat/manual"),
    CommandType.ENTER_L_MODE: ("进入L模式指令", 0, "/l-mode/enter"),
    CommandType.EXIT_L_MODE: ("退出L模式指令", 0, "/l-mode/exit"),
    CommandType.TOGGLE_STAND_DOWN: ("站立/趴下切换指令", 0, "/pose/toggle-stand-down"),
    CommandType.SET_AIM_POSE: ("对准姿态指令", 1, "/pose/set-aim"),
    CommandType.RESET_AIM_POSE: ("重置对准指令", 0, "/pose/reset-aim"),
    CommandType.MOTION_MODE_MANUAL: ("手动模式指令", 0, "/motion-mode/manual"),
    CommandType.MOTION_MODE_NAVIGATION: ("导航模式指令", 0, "/motion-mode/navigation"),
    CommandType.MOVE_X_AXIS: ("左摇杆Y轴指令（X轴速度）", 0, "/move/x-axis"),
    CommandType.MOVE_Y_AXIS: ("左摇杆X轴指令（Y轴速度）", 0, "/move/y-axis"),
    CommandType.MOVE_YAW_AXIS: ("右摇杆X轴指令（Yaw角速度）", 0, "/move/yaw-axis"),
    CommandType.SET_PLATFORM_HEIGHT: ("平台高度切换指令", 0, "/platform-height/set"),
    CommandType.GAIT_WALK: ("行走步态指令", 0, "/gait/set/walk"),
    CommandType.GAIT_RUN: ("跑步步态指令", 0, "/gait/set/run"),
    CommandType.SWITCH_SPEED_GEAR: ("速度切换指令", 0, "/speed-gear/switch"),
    CommandType.EMERGENCY_STOP: ("软急停指令", 0, "/emergency_stop"),
    CommandType.CAMERA_CONTROL: ("相机指令", 0, "/camera/control"),
    CommandType.CHARGE_REQUEST: (
        "自主充电请求指令（开始/结束/重置）",
        0,
        "/charge/request",
    ),
    CommandType.CHARGE_QUERY_STATUS: (
        "自主充电查询状态指令",
        0,
        "/charge/query-status",
    ),
    # 状态上报指令
    CommandType.RUN_STATUS_REPORT: ("运行状态数据上报", 1, "/report/run-status"),
    CommandType.MOTION_STATE_REPORT: ("运动状态数据上报", 1, "/report/motion-state"),
    CommandType.SENSOR_DATA_REPORT: (
        "运动控制传感器数据上报",
        1,
        "/report/sensor-data",
    ),
    CommandType.CONTROLLER_SAFE_DATA_REPORT: (
        "运动控制系统数据上报",
        1,
        "/report/controller-safe-data",
    ),
    CommandType.BATTERY_LEVEL_REPORT: ("电池电量状态上报", 0, "/report/battery/level"),
    CommandType.BATTERY_CHARGE_STATE_REPORT: (
        "电池充电状态上报",
        0,
        "/report/battery/charge-state",
    ),
    CommandType.ERROR_CODE_REPORT: ("错误码上报", 0, "/report/error-code"),
    # CommandType.CHARGE_RESPONSE: ("自主充电响应", 0, "/charge/response"),
    # CommandType.CHARGE_QUERY_RESPONSE: (
    #     "自主充电查询响应",
    #     0,
    #     "/charge/query-response",
    # ),
}


# -------------------------- 数据结构类（保留原有） --------------------------
@dataclass
class CommandHead:
    """指令头部（文档2.1.2/2.2.2）"""

    command_id: int  # uint32_t，指令码
    parameter_size: int  # uint32_t，有效数据长度（基本指令=指令值）
    command_type: int  # uint32_t，0=基本指令，1=扩展指令

    def to_bytes(self) -> bytes:
        """编码为小端字节流（12字节）"""
        return struct.pack(
            "<III", self.command_id, self.parameter_size, self.command_type
        )

    @classmethod
    def from_bytes(cls, data: bytes) -> "CommandHead":
        """从字节流解码（12字节）"""
        if len(data) < 12:
            raise ValueError("指令头部至少需要12字节")
        cmd_id, param_size, cmd_type = struct.unpack("<III", data[:12])
        return cls(command_id=cmd_id, parameter_size=param_size, command_type=cmd_type)


@dataclass
class FireAim:
    """对准指令数据（文档3.1.11）"""

    roll: int  # uint32_t，翻滚角（[-10°,10°]对应[1000,1000]）
    pitch: int  # uint32_t，俯仰角（[-22°,22°]对应[1000,1000]）
    yaw: int  # uint32_t，偏航角（[-22°,22°]对应[1000,1000]）

    def to_bytes(self) -> bytes:
        """编码为小端字节流（12字节）"""
        return struct.pack("<III", self.roll, self.pitch, self.yaw)

    @classmethod
    def from_bytes(cls, data: bytes) -> "FireAim":
        """从字节流解码（12字节）"""
        if len(data) < 12:
            raise ValueError("FireAim数据至少需要12字节")
        roll, pitch, yaw = struct.unpack("<III", data[:12])
        return cls(roll=roll, pitch=pitch, yaw=yaw)


@dataclass
class ImuSensorData:
    """IMU传感器数据（文档3.2.3）"""

    timestamp: int  # int32_t，时间戳
    roll: float  # 翻滚角（°）
    pitch: float  # 俯仰角（°）
    yaw: float  # 偏航角（°）
    omega_x: float  # X轴角速度（rad/s）
    omega_y: float  # Y轴角速度（rad/s）
    omega_z: float  # Z轴角速度（rad/s）
    acc_x: float  # X轴加速度（m/s²）
    acc_y: float  # Y轴加速度（m/s²）
    acc_z: float  # Z轴加速度（m/s²）

    def to_bytes(self) -> bytes:
        """编码为小端字节流（4+36=40字节）"""
        buffer_float = [
            self.roll,
            self.pitch,
            self.yaw,
            self.omega_x,
            self.omega_y,
            self.omega_z,
            self.acc_x,
            self.acc_y,
            self.acc_z,
        ]
        return struct.pack("<i9f", to_int32(self.timestamp), *buffer_float)

    @classmethod
    def from_bytes(cls, data: bytes) -> "ImuSensorData":
        """从字节流解码（40字节）"""
        if len(data) < 40:
            raise ValueError("ImuSensorData数据至少需要40字节")
        unpacked = struct.unpack("<i9f", data[:40])
        return cls(
            timestamp=unpacked[0],
            roll=unpacked[1],
            pitch=unpacked[2],
            yaw=unpacked[3],
            omega_x=unpacked[4],
            omega_y=unpacked[5],
            omega_z=unpacked[6],
            acc_x=unpacked[7],
            acc_y=unpacked[8],
            acc_z=unpacked[9],
        )


@dataclass
class LegJointData:
    """关节数据（位置/速度/力矩，文档3.2.3）"""

    # 左前腿
    fl_hipx: float  # 侧摆关节
    fl_hipy: float  # 髋关节
    fl_knee: float  # 膝关节
    # 右前腿
    fr_hipx: float  # 侧摆关节
    fr_hipy: float  # 髋关节
    fr_knee: float  # 膝关节
    # 左后腿
    hl_hipx: float  # 侧摆关节
    hl_hipy: float  # 髋关节
    hl_knee: float  # 膝关节
    # 右后腿
    hr_hipx: float  # 侧摆关节
    hr_hipy: float  # 髋关节
    hr_knee: float  # 膝关节

    def to_bytes(self) -> bytes:
        """编码为小端字节流（12×4=48字节）"""
        data_list = [
            self.fl_hipx,
            self.fl_hipy,
            self.fl_knee,
            self.fr_hipx,
            self.fr_hipy,
            self.fr_knee,
            self.hl_hipx,
            self.hl_hipy,
            self.hl_knee,
            self.hr_hipx,
            self.hr_hipy,
            self.hr_knee,
        ]
        return struct.pack("<12f", *data_list)

    @classmethod
    def from_bytes(cls, data: bytes) -> "LegJointData":
        """从字节流解码（48字节）"""
        if len(data) < 48:
            raise ValueError("LegJointData数据至少需要48字节")
        unpacked = struct.unpack("<12f", data[:48])
        return cls(
            fl_hipx=unpacked[0],
            fl_hipy=unpacked[1],
            fl_knee=unpacked[2],
            fr_hipx=unpacked[3],
            fr_hipy=unpacked[4],
            fr_knee=unpacked[5],
            hl_hipx=unpacked[6],
            hl_hipy=unpacked[7],
            hl_knee=unpacked[8],
            hr_hipx=unpacked[9],
            hr_hipy=unpacked[10],
            hr_knee=unpacked[11],
        )


@dataclass
class CpuInfo:
    """CPU状态数据（文档3.2.4）"""

    temperature: float  # 温度（℃）
    frequency: float  # 主频（MHz）

    def to_bytes(self) -> bytes:
        """编码为小端字节流（8字节）"""
        return struct.pack("<ff", self.temperature, self.frequency)

    @classmethod
    def from_bytes(cls, data: bytes) -> "CpuInfo":
        """从字节流解码（8字节）"""
        if len(data) < 8:
            raise ValueError("CpuInfo数据至少需要8字节")
        temp, freq = struct.unpack("<ff", data[:8])
        return cls(temperature=temp, frequency=freq)


# -------------------------- 状态上报核心数据类 --------------------------
@dataclass
class RcsData:
    """运行状态数据（文档3.2.1）"""

    robot_name: str  # 机器人名称（char[15]）
    current_milege: int  # 本次运行里程（cm，int32_t）
    total_milege: int  # 累计运行里程（cm，int32_t）
    current_run_time: int  # 本次运行时间（s，long）
    total_run_time: int  # 累计运行时间（s，long）
    current_motion_time: int  # 本次运动时间（s，long）
    total_motion_time: int  # 累计运动时间（s，long）
    joystick_lx: float  # 左摇杆X轴（[-1.0,1.0]）
    joystick_ly: float  # 左摇杆Y轴（[-1.0,1.0]）
    joystick_rx: float  # 右摇杆X轴（[-1.0,1.0]）
    joystick_ry: float  # 右摇杆Y轴（[-1.0,1.0]）
    is_nav_mode: int  # 0=手动模式，1=导航模式（uint8_t）
    # 错误状态位（文档3.2.1 error_state_bit）
    imu_error: bool
    wifi_error: bool
    driver_heat_warn: bool
    driver_error: bool
    motor_heat_warn: bool
    battery_low_warn: bool

    def to_bytes(self) -> bytes:
        """编码为小端字节流（15+4×2+8×4+4×4+1+4= 15+8+32+16+1+4=76字节）"""
        # 处理robot_name（补零到15字节）
        robot_name_bytes = self.robot_name.encode("utf-8")[:15].ljust(15, b"\x00")
        # 错误状态位打包（uint32_t）
        error_bits = 0
        error_bits |= (
            (1 << 31) if self.imu_error else 0
        )  # imu_error占最高位（文档位域定义）
        error_bits |= (1 << 0) if self.wifi_error else 0
        error_bits |= (1 << 1) if self.driver_heat_warn else 0
        error_bits |= (1 << 2) if self.driver_error else 0
        error_bits |= (1 << 3) if self.motor_heat_warn else 0
        error_bits |= (1 << 4) if self.battery_low_warn else 0

        return struct.pack(
            "<15siiqqqqffff10B I",  # 10B包含is_nav_mode和9个预留位
            robot_name_bytes,
            self.current_milege,
            self.total_milege,
            self.current_run_time,
            self.total_run_time,
            self.current_motion_time,
            self.total_motion_time,
            self.joystick_lx,
            self.joystick_ly,
            self.joystick_rx,
            self.joystick_ry,
            self.is_nav_mode,
            *[0] * 9,  # mode_reserved[9]
            error_bits,
        )

    @classmethod
    def from_bytes(cls, data: bytes) -> "RcsData":
        """从字节流解码（76字节）"""
        unpacked = struct.unpack("<15siiqqqqffff10B I", data)
        return cls(
            robot_name=unpacked[0].decode("utf-8").rstrip("\x00"),
            current_milege=unpacked[1],
            total_milege=unpacked[2],
            current_run_time=unpacked[3],
            total_run_time=unpacked[4],
            current_motion_time=unpacked[5],
            total_motion_time=unpacked[6],
            joystick_lx=unpacked[7],
            joystick_ly=unpacked[8],
            joystick_rx=unpacked[9],
            joystick_ry=unpacked[10],
            is_nav_mode=unpacked[11],
            imu_error=(unpacked[21] >> 31) & 1,
            wifi_error=(unpacked[21] >> 0) & 1,
            driver_heat_warn=(unpacked[21] >> 1) & 1,
            driver_error=(unpacked[21] >> 2) & 1,
            motor_heat_warn=(unpacked[21] >> 3) & 1,
            battery_low_warn=(unpacked[21] >> 4) & 1,
        )


@dataclass
class MotionStateData:
    """运动状态数据（文档3.2.2）"""

    # 0=趴下,1=起立中,2=初始站立,3=力控站立,4=踏步,5=趴下中,6=急停/摔倒
    basic_state: int
    gait_state: int  # 0=行走,1=跑步
    max_forward_vel: float  # 最大前进速度（m/s）
    max_backward_vel: float  # 最大后退速度（m/s）
    pos_x: float  # 世界坐标系X坐标（m）
    pos_y: float  # 世界坐标系Y坐标（m）
    pos_yaw: float  # 世界坐标系Yaw角（rad）
    vel_x: float  # 机身X轴速度（m/s）
    vel_y: float  # 机身Y轴速度（m/s）
    vel_yaw: float  # 机身Yaw角速度（rad/s）
    robot_distance: float  # 本次运行里程（cm）
    auto_charge_state: int  # 充电状态（uint8_t）
    pos_ctrl_state: int  # 位置控制状态（uint8_t）

    def to_bytes(self) -> bytes:
        """编码为小端字节流（2+4×8+4+10B+4= 2+32+4+10+4=52字节）"""
        control_state = 0  # 预留字段
        return struct.pack(
            "<2f6ffffI10B",
            self.basic_state,
            self.gait_state,
            self.max_forward_vel,
            self.max_backward_vel,
            self.pos_x,
            self.pos_y,
            self.pos_yaw,
            self.vel_x,
            self.vel_y,
            self.vel_yaw,
            self.robot_distance,
            control_state,
            self.auto_charge_state,
            self.pos_ctrl_state,
            *[0] * 8,  # task_reserved[8]
        )

    @classmethod
    def from_bytes(cls, data: bytes) -> "MotionStateData":
        """从字节流解码（52字节）"""
        unpacked = struct.unpack("<2f6ffffI10B", data)
        return cls(
            basic_state=unpacked[0],
            gait_state=unpacked[1],
            max_forward_vel=unpacked[2],
            max_backward_vel=unpacked[3],
            pos_x=unpacked[4],
            pos_y=unpacked[5],
            pos_yaw=unpacked[6],
            vel_x=unpacked[7],
            vel_y=unpacked[8],
            vel_yaw=unpacked[9],
            robot_distance=unpacked[10],
            auto_charge_state=unpacked[12],
            pos_ctrl_state=unpacked[13],
        )


@dataclass
class ControllerSensorData:
    """运动控制传感器数据（文档3.2.3）"""

    imu_data: ImuSensorData  # IMU数据
    joint_pos: LegJointData  # 关节位置（rad）
    joint_vel: LegJointData  # 关节速度（rad/s）
    joint_tau: LegJointData  # 关节力矩（N·m）

    def to_bytes(self) -> bytes:
        """编码为小端字节流（40+48×3= 40+144=184字节）"""
        return (
            self.imu_data.to_bytes()
            + self.joint_pos.to_bytes()
            + self.joint_vel.to_bytes()
            + self.joint_tau.to_bytes()
        )

    @classmethod
    def from_bytes(cls, data: bytes) -> "ControllerSensorData":
        """从字节流解码（184字节）"""
        if len(data) < 184:
            raise ValueError("ControllerSensorData数据至少需要184字节")
        imu = ImuSensorData.from_bytes(data[:40])
        joint_pos = LegJointData.from_bytes(data[40:88])
        joint_vel = LegJointData.from_bytes(data[88:136])
        joint_tau = LegJointData.from_bytes(data[136:184])
        return cls(
            imu_data=imu, joint_pos=joint_pos, joint_vel=joint_vel, joint_tau=joint_tau
        )


@dataclass
class Q20ControllerSafeData:
    """运动控制系统数据（文档3.2.4）"""

    motor_temperatures: List[float]  # 12个关节电机温度（℃）
    driver_temperatures: List[int]  # 12个关节驱动器温度（℃，uint8_t）
    cpu_info: CpuInfo  # CPU状态

    def to_bytes(self) -> "bytes":
        """编码为小端字节流（12×4 + 12×1 + 8= 48+12+8=68字节）"""
        if len(self.motor_temperatures) != 12 or len(self.driver_temperatures) != 12:
            raise ValueError("电机/驱动器温度列表必须包含12个元素")
        return struct.pack(
            "<12f12Bff",
            *self.motor_temperatures,
            *self.driver_temperatures,
            self.cpu_info.temperature,
            self.cpu_info.frequency,
        )

    @classmethod
    def from_bytes(cls, data: bytes) -> "Q20ControllerSafeData":
        """从字节流解码（68字节）"""
        if len(data) < 68:
            raise ValueError("Q20ControllerSafeData数据至少需要68字节")
        unpacked = struct.unpack("<12f12Bff", data[:68])
        return cls(
            motor_temperatures=list(unpacked[:12]),
            driver_temperatures=list(unpacked[12:24]),
            cpu_info=CpuInfo(temperature=unpacked[24], frequency=unpacked[25]),
        )


@dataclass
class BatteryLevel:
    """电池电量状态（文档3.2.5.1）"""

    level: int  # 电量（%，指令值）

    def to_bytes(self) -> bytes:
        """编码为指令头部（基本指令，12字节）"""
        head = CommandHead(
            command_id=CommandType.BATTERY_LEVEL_REPORT,
            parameter_size=self.level,
            command_type=0,
        )
        return head.to_bytes()

    @classmethod
    def from_bytes(cls, head: CommandHead) -> "BatteryLevel":
        """从指令头部解码"""
        return cls(level=head.parameter_size)


@dataclass
class BatteryChargeState:
    """电池充电状态（文档3.2.5.2）"""

    level: int  # 电量（%，高24位）
    is_charging: bool  # 是否充电（低8位：0=未充电，1=充电中）

    def to_bytes(self) -> bytes:
        """编码为指令头部（基本指令，12字节）"""
        param = (self.level << 8) | (1 if self.is_charging else 0)
        head = CommandHead(
            command_id=CommandType.BATTERY_CHARGE_STATE_REPORT,
            parameter_size=param,
            command_type=0,
        )
        return head.to_bytes()

    @classmethod
    def from_bytes(cls, head: CommandHead) -> "BatteryChargeState":
        """从指令头部解码"""
        level = (head.parameter_size >> 8) & 0xFFFFFF
        is_charging = (head.parameter_size & 0xFF) == 1
        return cls(level=level, is_charging=is_charging)


@dataclass
class ErrorCode:
    """错误码（文档3.2.6）"""

    error_level: int  # 0=通知，1=警告，2=错误（低8位）
    error_code: int  # 具体错误码（高24位）
    error_msg: str  # 错误描述（根据文档映射）

    # 错误码映射表（文档3.2.6）
    _ERROR_MAP = {
        0x00D00101: "IMU故障",
        0x00D00201: "地形图形传输超时",
        0x00D00502: "驱动器运行异常",
        0x00D00601: "机器人触发低电量保护",
        0x00D00701: "机器人触发关节电机过温保护",
    }

    def to_bytes(self) -> bytes:
        """编码为指令头部（基本指令，12字节）"""
        param = (self.error_code << 8) | self.error_level
        head = CommandHead(
            command_id=CommandType.ERROR_CODE_REPORT,
            parameter_size=param,
            command_type=0,
        )
        return head.to_bytes()

    @classmethod
    def from_bytes(cls, head: CommandHead) -> "ErrorCode":
        """从指令头部解码"""
        error_level = head.parameter_size & 0xFF
        error_code = (head.parameter_size >> 8) & 0xFFFFFF
        error_msg = cls._ERROR_MAP.get(error_code, f"未知错误（0x{error_code:06X}）")
        return cls(error_level=error_level, error_code=error_code, error_msg=error_msg)


@dataclass
class ChargeResponse:
    """自主充电响应（文档3.3.2）"""

    state: int  # charge_manager_state
    state_msg: str  # 状态描述

    # 充电状态映射表（文档3.3.2）
    _STATE_MAP = {
        0x0000: "空闲状态",
        0x0001: "充电任务状态",
        0x0002: "正在充电状态",
        0x0003: "结束充电状态",
        0x0004: "充电桩断电状态",
        0x0005: "安全告警",
        0x1002: "获取定位数据超时",
        0x1003: "开启定位算法失败",
        0x1004: "到达充电桩超时",
        0x1005: "获取定位数据失败",
        0x1006: "定位数据跳点",
        0x1007: "充电失败（趴在充电桩上）",
        0x100A: "充电失败（退出充电桩）",
    }

    def to_bytes(self) -> bytes:
        """编码为指令头部（基本指令，12字节）"""
        head = CommandHead(
            command_id=CommandType.CHARGE_RESPONSE,
            parameter_size=self.state,
            command_type=0,
        )
        return head.to_bytes()

    @classmethod
    def from_bytes(cls, head: CommandHead) -> "ChargeResponse":
        """从指令头部解码"""
        state = head.parameter_size
        state_msg = cls._STATE_MAP.get(state, f"未知状态（0x{state:04X}）")
        return cls(state=state, state_msg=state_msg)


# -------------------------- 核心映射：data类型→command_type（新增） --------------------------
_DATA_TYPE_TO_COMMAND = {
    FireAim: CommandType.SET_AIM_POSE  # 唯一支持自动匹配的场景：FireAim→对准指令
}


# -------------------------- 优化后的打包函数 --------------------------
def pack_q25_udp_cmd(
    command_type: Optional[CommandType] = None,
    parameter_size: int = 0,
    data: Optional[Union[List[int], bytes, FireAim]] = None,
) -> bytes:
    """
    打包天狼Q25机器人的UDP控制指令（支持通过data类型自动推导command_type）

    优化点：
    1. 当data是FireAim类型时，可无需指定command_type，自动匹配CommandType.SET_AIM_POSE
    2. 基本指令（无data）和其他扩展指令仍需指定command_type（避免歧义）
    3. 增强校验：确保cmd_type与data存在性一致（基本指令无data，扩展指令有data）

    :param command_type: 指令码（可选，FireAim类型data可自动推导）
    :param parameter_size: 有效数据长度（基本指令：指令值；扩展指令：数据字节数）
    :param data: 扩展指令数据（支持FireAim/list/bytes，基本指令需传None）
    :return: 可直接发送的UDP二进制数据包
    :raises ValueError: 指令无效、参数不匹配、无法自动推导command_type
    """
    # 第一步：自动推导command_type（仅支持FireAim类型）
    if command_type is None:
        if data is None:
            raise ValueError("基本指令必须指定command_type（data为None无法区分指令）")
        data_type = type(data)
        if data_type not in _DATA_TYPE_TO_COMMAND:
            raise ValueError(
                f"无法自动推导command_type：仅支持FireAim类型data自动匹配，当前data类型为{data_type}"
            )
        command_type = _DATA_TYPE_TO_COMMAND[data_type]
        print(f"自动匹配command_type：{command_type.name}（0x{command_type:08X}）")

    # 第二步：校验command_type有效性
    if command_type not in command_info_map:
        raise ValueError(f"未定义的指令类型：{command_type}")
    cmd_name, cmd_type, url = command_info_map[command_type]
    command_id = int(command_type)

    # 第三步：校验cmd_type与data的一致性（增强容错）
    if cmd_type == 0:  # 基本指令：必须无data
        if data is not None:
            raise ValueError(f"基本指令[{cmd_name}]不支持携带data，请勿传入data参数")
    else:  # 扩展指令：必须有data
        if data is None:
            raise ValueError(f"扩展指令[{cmd_name}]必须传入data参数")

    # 第四步：处理data编码（与原有逻辑一致）
    if cmd_type == 0:
        # 基本指令：无data，直接打包头部
        return struct.pack(
            "<III",
            to_uint32(command_id),
            to_uint32(parameter_size),
            to_uint32(cmd_type),
        )
    else:
        # 扩展指令：编码data
        if isinstance(data, FireAim):
            data_bytes = data.to_bytes()
        elif isinstance(data, list):
            for idx, val in enumerate(data):
                if not (0 <= val <= 0xFFFFFFFF):
                    raise ValueError(
                        f"扩展指令数据第{idx}项超出uint32_t范围（0~4294967295）"
                    )
            data_bytes = struct.pack(f"<{len(data)}I", *data)
        elif isinstance(data, bytes):
            data_bytes = data
        else:
            raise TypeError("data参数仅支持list（uint32_t）、bytes或FireAim类型")

        # 校验数据长度
        if len(data_bytes) != parameter_size:
            raise ValueError(
                f"扩展指令数据长度（{len(data_bytes)}字节）与parameter_size（{parameter_size}字节）不匹配"
            )

        return struct.pack("<III", command_id, parameter_size, cmd_type) + data_bytes


def unpack_q25_udp_cmd(data: bytes) -> tuple[CommandHead, Optional[Any]]:
    """
    解包天狼Q25机器人的UDP数据（支持控制指令和状态上报指令）

    :param data: 接收的UDP二进制数据包
    :return: 指令头部（CommandHead） + 数据实例（对应的数据类或None）
    :raises ValueError: 数据长度不足、指令未定义、解码失败
    """
    # 解包指令头部
    if len(data) < 12:
        raise ValueError("UDP数据长度不足，至少需要12字节指令头部")
    head = CommandHead.from_bytes(data[:12])

    # 查找指令类型
    try:
        command_type = CommandType(head.command_id)
    except ValueError:
        raise ValueError(f"未定义的指令码：0x{head.command_id:08X}")

    cmd_name, cmd_type, url = command_info_map[command_type]
    data_body = data[12 : 12 + head.parameter_size]

    # 根据指令类型解码数据体
    if cmd_type == 0:  # 基本指令：无数据体（parameter_size为指令值）
        if command_type == CommandType.BATTERY_LEVEL_REPORT:
            return head, BatteryLevel.from_bytes(head)
        elif command_type == CommandType.BATTERY_CHARGE_STATE_REPORT:
            return head, BatteryChargeState.from_bytes(head)
        elif command_type == CommandType.ERROR_CODE_REPORT:
            return head, ErrorCode.from_bytes(head)
        # elif command_type in (
        #     CommandType.CHARGE_RESPONSE,
        #     CommandType.CHARGE_QUERY_RESPONSE,
        # ):
        #     return head, ChargeResponse.from_bytes(head)
        else:
            return head, None  # 控制类基本指令无额外数据

    else:  # 扩展指令：解码数据体到对应类
        # 校验数据体长度
        if len(data_body) != head.parameter_size:
            raise ValueError(
                f"数据体长度（{len(data_body)}字节）与parameter_size（{head.parameter_size}字节）不匹配"
            )

        if command_type == CommandType.RUN_STATUS_REPORT:
            return head, RcsData.from_bytes(data_body)
        elif command_type == CommandType.MOTION_STATE_REPORT:
            return head, MotionStateData.from_bytes(data_body)
        elif command_type == CommandType.SENSOR_DATA_REPORT:
            return head, ControllerSensorData.from_bytes(data_body)
        elif command_type == CommandType.CONTROLLER_SAFE_DATA_REPORT:
            return head, Q20ControllerSafeData.from_bytes(data_body)
        elif command_type == CommandType.SET_AIM_POSE:
            return head, FireAim.from_bytes(data_body)
        else:
            return head, data_body  # 未定义的扩展指令返回原始数据体


def speed_to_cmd_value(
    axis_type: str,
    speed: float,
    v_max: float = None,  # 默认最大速度（可根据当前步态动态调整）
) -> int:
    """
    按天狼Q25文档3.1.5节公式，将速度值转换为轴指令值（-32767~32767）

    :param axis_type: 轴类型，可选值："x"（X轴速度）、"y"（Y轴速度）、"yaw"（Yaw角速度）
    :param speed: 目标速度值：
                  - X轴：线速度（m/s），正向前、负向后
                  - Y轴：线速度（m/s），正向右、负向左
                  - Yaw：角速度（rad/s），正向右、负向左
    :param v_max: 当前步态下的最大限制速度（m/s），仅X轴/Y轴需要
    :return: 轴指令值（-32767~32767）
    :raises ValueError: 轴类型无效或速度超出最大范围
    """
    if axis_type not in ["x", "y", "yaw"]:
        raise ValueError(f"轴类型无效，仅支持'x'/'y'/'yaw'，当前输入：{axis_type}")

    # 1. X轴速度 → 左摇杆Y轴指令值（公式1）
    if axis_type == "x":
        if speed > 0:
            # 向前运动：6553 ≤ y ≤ 32767
            cmd_value = (speed / v_max) * 26215 - 6553
        elif speed < 0:
            # 向后运动：-32767 ≤ y ≤ -6553
            cmd_value = (speed / v_max) * 26215 + 6553
        else:
            # 零速度（死区）：-6553 < y < 6553，取0
            cmd_value = 0

    # 2. Y轴速度 → 左摇杆X轴指令值（公式2）
    elif axis_type == "y":
        if speed > 0:
            cmd_value = -((speed / v_max) * 8554 + 24576)
        elif speed < 0:
            cmd_value = -((speed / v_max) * 8554 - 24576)
        else:
            cmd_value = 0

    # 3. Yaw角速度 → 右摇杆X轴指令值（公式3）
    elif axis_type == "yaw":
        cmd_value = -speed * 32768
    # 四舍五入为整数，并限制在[-32767, 32767]范围
    cmd_value = round(cmd_value)
    return max(-32767, min(32767, cmd_value))
