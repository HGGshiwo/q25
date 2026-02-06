#!/usr/bin/python3
# -*- coding: utf-8 -*-
import struct
import threading
import time
from typing import Optional

from event_callback.components.http.message_handler import MessageType
from controller_model import GaitModel, JoystickModel, PlatformHeightModel
from controller_ui import *
from utils.utils import *
from event_callback.components.socket import socketc, sockets
from event_callback import http
from event_callback import CallbackManager


class Controller(CallbackManager):
    def __init__(self, component_config=None):
        self.max_backward_vel = None
        self.max_forward_vel = None
        self.v_max_lock = threading.Lock()
        self.basic_state = None
        self.is_run = None  # 是否是跑步步态
        self.paltform_height = None  # 0 匍匐, 2 站立, 无法从上报状态读出
        super().__init__(component_config)

    def move_raw_old(
        self,
        x: Optional[float] = None,
        y: Optional[float] = None,
        yaw: Optional[float] = None,
    ):
        """虚拟摇杆的三轴运动控制（X/Y轴速度 + Yaw角速度），机体左手坐标系FRU
        :param x: 摇杆偏移机体X轴控制(对应摇杆Y)
        :param y: 摇杆偏移机体Y轴控制(对应摇杆X)
        :param yaw: Yaw摇杆
        """
        if x is not None:
            data = pack_q25_udp_cmd(CommandType.MOVE_X_AXIS, parameter_size=x)
            socketc.send_to_server(self, data)

        if y is not None:
            data = pack_q25_udp_cmd(CommandType.MOVE_Y_AXIS, parameter_size=y)
            socketc.send_to_server(self, data)

        if yaw is not None:
            data = pack_q25_udp_cmd(CommandType.MOVE_YAW_AXIS, parameter_size=yaw)
            socketc.send_to_server(self, data)

    def move_axis_no_dead_zone(
        self,
        left_x: int = 0,
        left_y: int = 0,
        right_x: int = 0,
        right_y: int = 0,
    ):
        """【新无死区】三轴运动控制（推荐使用），机体左手坐标系FRU，
        注意这个API的xy是相反的，和文档不一样

        :param left_x: X轴速度（-1000~1000，正向前）
        :param left_y: Y轴速度（-1000~1000，正向左）
        :param right_x: Yaw角速度（-1000~1000，正左转）
        :param right_y: 预留（固定为0）
        """
        # 校验参数范围
        left_x, left_y = left_y, left_x
        left_x = max(-1000, min(1000, left_x))
        left_y = max(-1000, min(1000, left_y))
        right_x = max(-1000, min(1000, right_x))
        right_y = 0  # 强制预留字段为0
        axis_data = AxisCommand(
            left_x=left_x, left_y=left_y, right_x=right_x, right_y=right_y
        )

        # 打包并发送指令（50Hz频率由调用方保证或新增线程控制）
        data = pack_q25_udp_cmd(
            command_type=CommandType.AXIS_COMMAND_NO_DEAD_ZONE,
            parameter_size=len(axis_data.to_bytes()),
            data=axis_data,
        )
        socketc.send_to_server(self, data)

    # ===================== 发送类指令（SOCKET监听）=====================

    @http.post("/takeoff")
    def takeoff(self):
        """起立"""
        if self.basic_state in [1, 2, 3, 0x10]:
            return {"status": "error", "msg": "已经处于起立状态或L模式"}
        data = pack_q25_udp_cmd(CommandType.TOGGLE_STAND_DOWN)
        socketc.send_to_server(self, data)
        return {"status": "success", "msg": "切换为起立状态"}

    @http.get("/prearms")
    def prearms(self):
        """检查起飞状态"""
        return {"status": "success", "msg": {"arm": True}}

    @http.post("/land")
    def land(self):
        """趴下"""
        if self.basic_state in [0, 5]:
            return {"status": "error", "msg": "已经处于趴下状态"}
        data = pack_q25_udp_cmd(CommandType.TOGGLE_STAND_DOWN)
        socketc.send_to_server(self, data)
        return {"status": "success", "msg": "切换为趴下状态"}

    @http.post("/toggle_stand_down")
    def pose_toggle_stand_down(self):
        """站立/趴下切换（可解除急停）"""
        data = pack_q25_udp_cmd(CommandType.TOGGLE_STAND_DOWN)
        socketc.send_to_server(self, data)
        return {"status": "success", "msg": "切换站立/趴下姿态"}

    @http.post("/emergecy_stop")
    def emergecy_stop(self):
        """触发软急停"""
        data = pack_q25_udp_cmd(CommandType.EMERGENCY_STOP)
        socketc.send_to_server(self, data)
        return {"status": "success", "msg": "已触发软急停"}

    @http.post("/l-mode")
    def control_l_mode(self, enter: bool = True):
        """控制L模式（强化学习模式）
        :param enter: True=进入，False=退出
        """
        cmd_type = CommandType.ENTER_L_MODE if enter else CommandType.EXIT_L_MODE
        data = pack_q25_udp_cmd(cmd_type)
        socketc.send_to_server(self, data)
        return {"status": "success", "msg": f"{'进入' if enter else '退出'}L模式"}

    @http.post("/motion-mode")
    def set_motion_mode(self, is_nav: bool = False):
        """切换运动模式
        :param is_nav: True=导航模式，False=手动模式
        """
        cmd_type = (
            CommandType.MOTION_MODE_NAVIGATION
            if is_nav
            else CommandType.MOTION_MODE_MANUAL
        )
        data = pack_q25_udp_cmd(cmd_type)
        socketc.send_to_server(self, data)
        return {"status": "success", "msg": f"切换到{'导航' if is_nav else '手动'}模式"}

    @http.post("/move/joystick_old")
    def move_joystick_old(self, item: JoystickModel):
        """摇杆控制"""
        x, y = item.x, item.y
        if x is not None:
            x = int(x * 32767)
        if y is not None:
            y = int(y * 32767)

        self.move_raw_old(y, x, None)
        http.ws_send(
            self,
            dict(info=f"{x} {y}"),
            MessageType.INFO,
        )
        return {"status": "success", "msg": "OK"}

    @http.post("/move/joystick")
    def move_joystick(self, item: JoystickModel):
        """摇杆控制（推荐使用）"""
        x, y = item.x, item.y
        # 映射到[-1000,1000]范围
        left_x = int(y * 1000)  # 左摇杆Y对应X轴速度
        left_y = int(x * 1000)  # 左摇杆X对应Y轴速度
        self.move_axis_no_dead_zone(left_x=left_x, left_y=left_y)
        http.ws_send(
            self,
            dict(info=f"无死区控制：left_x={left_x}, left_y={left_y}"),
            MessageType.INFO,
        )
        return {"status": "success", "msg": "OK"}

    @http.post("/move/velocity_old")
    def move_velocity_old(
        self,
        x: Optional[float] = None,
        y: Optional[float] = None,
        yaw: Optional[float] = None,
    ):
        """三轴速度控制"""
        with self.v_max_lock:
            max_backward_vel = self.max_backward_vel
            max_forward_vel = self.max_forward_vel

        if max_backward_vel is None or max_forward_vel is None:
            return {"status": "error", "msg": f"未收到motion_state数据，无最大速度"}

        _x, _y, _z = None, None, None
        if x is not None:
            v_max = max_forward_vel if x > 0 else max_backward_vel
            _x = speed_to_cmd_value("x", x, v_max)

        if y is not None:
            v_max = max_forward_vel if x > 0 else max_backward_vel
            _y = speed_to_cmd_value("y", y, v_max)

        if yaw is not None:
            _yaw = speed_to_cmd_value("yaw", yaw)

        self.move_raw_old(_x, _y, _yaw)
        return {"status": "success", "msg": f"运动控制：X={_x}, Y={_y}, Yaw={_yaw}"}

    @http.post("/move/velocity")
    def move_velocity(
        self,
        x: Optional[float] = None,
        y: Optional[float] = None,
        yaw: Optional[float] = None,
    ):
        """【新无死区】三轴速度控制（推荐使用）

        :param x: X轴速度(m/s)
        :param y: Y轴速度(m/s)
        :param yaw: Yaw角速度(rad/s)
        """

        with self.v_max_lock:
            max_backward_vel = self.max_backward_vel
            max_forward_vel = self.max_forward_vel

        if max_backward_vel is None or max_forward_vel is None:
            return {"status": "error", "msg": f"未收到motion_state数据，无最大速度"}

        # 线性映射到[-1000,1000]范围
        left_x = 0
        if x is not None:
            v_max = max_forward_vel if x > 0 else max_backward_vel
            left_x = int((x / v_max) * 1000) if v_max != 0 else 0

        left_y = 0
        if y is not None:
            v_max = max_forward_vel if y > 0 else max_backward_vel
            left_y = int((y / v_max) * 1000) if v_max != 0 else 0

        right_x = 0
        if yaw is not None:
            # Yaw角速度映射（假设最大角速度对应1000）
            right_x = int(yaw * 1000)

        self.move_axis_no_dead_zone(left_x=left_x, left_y=left_y, right_x=right_x)
        return {
            "status": "success",
            "msg": f"无死区运动控制：left_x={left_x}, left_y={left_y}, right_x={right_x}",
        }

    @http.get("/platform-height")
    def get_platform_height(self):
        return {"status": "success", "msg": {"height": str(self.paltform_height)}}

    @http.post("/platform-height")
    def set_platform_height(self, item: PlatformHeightModel):
        """切换平台高度
        :param height: 0=匍匐高度，2=正常高度（仅支持这两个值）
        """
        if item.height not in [0, 2]:
            return {"status": "error", "msg": "高度参数仅支持0（匍匐）或2（正常）"}
        data = pack_q25_udp_cmd(
            CommandType.SET_PLATFORM_HEIGHT, parameter_size=item.height
        )
        socketc.send_to_server(self, data)
        self.paltform_height = item.height 
        return {
            "status": "success",
            "msg": f"切换平台高度为{'匍匐' if item.height == 0 else '正常'}",
        }

    @http.get("/gait")
    def get_gait(self):
        """获取步态"""
        return {"status": "success", "msg": {"is_run": self.is_run}}

    @http.post("/gait")
    def set_gait(self, item: GaitModel):
        """切换步态
        :param is_run: True=跑步步态，False=行走步态
        """
        cmd_type = CommandType.GAIT_RUN if item.is_run else CommandType.GAIT_WALK
        data = pack_q25_udp_cmd(cmd_type)
        socketc.send_to_server(self, data)
        return {
            "status": "success",
            "msg": f"切换到{'跑步' if item.is_run else '行走'}步态",
        }

    @http.post("/speed-gear")
    def switch_speed_gear(self, is_high: bool = False):
        """切换速度档位（仅跑步步态有效）
        :param is_high: True=高速档，False=低速档
        """
        data = pack_q25_udp_cmd(
            CommandType.SWITCH_SPEED_GEAR, parameter_size=1 if is_high else 0
        )
        socketc.send_to_server(self, data)
        return {
            "status": "success",
            "msg": f"切换到{'高速' if is_high else '低速'}档位（仅跑步步态生效）",
        }

    @http.post("/camera")
    def control_camera(self, mode: int = 0):
        """控制相机开关
        :param mode: 0=关闭所有，1=仅主视开启，2=仅环视开启，3=全部开启
        """
        if mode not in [0, 1, 2, 3]:
            return {"status": "error", "msg": "相机模式仅支持0-3"}
        data = pack_q25_udp_cmd(CommandType.CAMERA_CONTROL, parameter_size=mode)
        socketc.send_to_server(self, data)
        mode_desc = {
            0: "关闭所有相机",
            1: "开启主视相机",
            2: "开启环视相机",
            3: "开启所有相机",
        }
        return {"status": "success", "msg": mode_desc[mode]}

    @http.post("/aim")
    def control_aim(
        self, reset: bool = False, roll: int = 0, pitch: int = 0, yaw: int = 0
    ):
        """对准姿态控制
        :param reset: True=重置对准，False=设置对准姿态
        :param roll: 翻滚角（-1000~1000，无效字段）
        :param pitch: 俯仰角（-1000~1000，对应-10°~17°）
        :param yaw: 偏航角（-1000~1000，对应-20°~20°）
        """
        if reset:
            # 重置对准指令
            data = pack_q25_udp_cmd(CommandType.RESET_AIM_POSE)
            socketc.send_to_server(self, data)
            return {"status": "success", "msg": "重置对准姿态"}
        else:
            # 校验角度范围
            if not (
                -1000 <= roll <= 1000
                and -1000 <= pitch <= 1000
                and -1000 <= yaw <= 1000
            ):
                return {"status": "error", "msg": "角度参数超出范围（-1000~1000）"}
            # 打包扩展指令（FireAim结构体）
            aim_data = FireAim(roll=roll, pitch=pitch, yaw=yaw)
            data = pack_q25_udp_cmd(
                command_type=CommandType.SET_AIM_POSE,
                parameter_size=len(aim_data.to_bytes()),
                data=aim_data,
            )
            socketc.send_to_server(self, data)
            return {
                "status": "success",
                "msg": f"设置对准姿态：roll={roll}, pitch={pitch}, yaw={yaw}",
            }

    @http.post("/upper-power")
    def control_upper_power(self, enable: bool = False):
        """高级设置：上装上下电控制（文档3.1.11）
        :param enable: True=上电，False=下电（默认）
        """
        data = pack_q25_udp_cmd(
            CommandType.UPPER_POWER_SWITCH, parameter_size=1 if enable else 0
        )
        socketc.send_to_server(self, data)
        return {
            "status": "success",
            "msg": f"上装{'上电' if enable else '下电'}成功（默认下电）",
        }

    @http.post("/data-report/switch")
    def set_data_report_switch(
        self, driver_joint_switch: int = 0, state_switch: int = 1
    ):
        """高级设置：数据上报开关控制（文档3.1.11）
        :param driver_joint_switch: 驱动器及关节采样数据开关（0=关闭，1=开启，默认0）
        :param state_switch: 状态数据开关（0=关闭，1=开启，默认1）
        """
        # 驱动器及关节采样数据开关（0x80110201）
        data1 = pack_q25_udp_cmd(
            CommandType.DRIVER_JOINT_DATA_SWITCH, parameter_size=driver_joint_switch
        )
        socketc.send_to_server(self, data1)
        # 状态数据开关（0x80110202）
        data2 = pack_q25_udp_cmd(
            CommandType.STATE_DATA_SWITCH, parameter_size=state_switch
        )
        socketc.send_to_server(self, data2)
        return {
            "status": "success",
            "msg": f"数据上报开关设置：驱动器关节采样={driver_joint_switch}，状态数据={state_switch}（需同时开启才上报传感器数据）",
        }

    # ===================== 接收类指令（SOCKET监听）=====================
    @sockets.recv(CommandType.MOTION_STATE_REPORT, frequency=1)
    def motion_state_report(self, data: bytes):
        """接收运动状态数据（200Hz）"""
        _, data_obj = unpack_q25_udp_cmd(data)
        if not isinstance(data_obj, MotionStateData):
            return
        with self.v_max_lock:
            self.max_forward_vel = data_obj.max_forward_vel
            self.max_backward_vel = data_obj.max_backward_vel
        # 修正gait_desc判断（文档：0x20=行走，0x23=跑步）
        gait_desc = "未知步态"
        if data_obj.gait_state == 0x20:
            gait_desc = "行走"
        elif data_obj.gait_state == 0x23:
            gait_desc = "跑步"

        _json_data = {
            "basic_state": data_obj.basic_state,
            "gait_state": data_obj.gait_state,
            "gait_desc": gait_desc,
            "max_forward_vel": round(data_obj.max_forward_vel, 2),
            "max_backward_vel": round(data_obj.max_backward_vel, 2),
            "position": [
                round(data_obj.pos_x, 3),
                round(data_obj.pos_y, 3),
                round(data_obj.pos_yaw, 3),
            ],
            "position_desc": f"{data_obj.pos_x:.1f}, {data_obj.pos_y:.1f}, {data_obj.pos_yaw:.1f}",
            "velocity": [
                round(data_obj.vel_x, 3),
                round(data_obj.vel_y, 3),
                round(data_obj.vel_yaw, 3),
            ],
            "run_distance": round(data_obj.robot_distance, 1),
            "charge_state": data_obj.auto_charge_state,
        }

        # 补充basic_state=0x10（L模式）映射
        state_map = {
            0: "趴下状态",
            1: "正在起立状态",
            2: "初始站立状态",
            3: "力控站立状态",
            4: "踏步状态",
            5: "正在趴下状态",
            6: "软急停/摔倒状态",
            0x10: "L模式",
        }
        basic_state_desc = state_map.get(
            data_obj.basic_state, f"未知状态({data_obj.basic_state})"
        )
        self.basic_state = data_obj.basic_state
        self.is_run = data_obj.gait_state == 0x23
        http.ws_send(
            self,
            dict(
                basic_state_desc=basic_state_desc,
                gait_desc=gait_desc,
                max_forward_vel=data_obj.max_forward_vel,
                max_backward_vel=data_obj.max_backward_vel,
            ),
            MessageType.STATE,
        )

    @sockets.recv(CommandType.RUN_STATUS_REPORT, frequency=1)
    def run_status_report(self, data: bytes):
        """接收运行状态数据（200Hz）"""
        _, data_obj = unpack_q25_udp_cmd(data)
        if not isinstance(data_obj, RcsData):
            return
        json_data = {
            "robot_name": data_obj.robot_name,
            "current_milege": data_obj.current_milege,
            "total_milege": data_obj.total_milege,
            "current_run_time": data_obj.current_run_time,
            "total_run_time": data_obj.total_run_time,
            "motion_mode": "导航" if data_obj.is_nav_mode == 1 else "手动",
            "joystick": {
                "lx": round(data_obj.joystick_lx, 3),
                "ly": round(data_obj.joystick_ly, 3),
                "rx": round(data_obj.joystick_rx, 3),
                "ry": round(data_obj.joystick_ry, 3),
            },
            "errors": {
                "imu_error": data_obj.imu_error,
                "wifi_error": data_obj.wifi_error,
                "driver_heat_warn": data_obj.driver_heat_warn,
                "driver_error": data_obj.driver_error,
                "motor_heat_warn": data_obj.motor_heat_warn,
                "battery_low_warn": data_obj.battery_low_warn,
            },
        }
        http.ws_send(self, json_data, MessageType.STATE)

    @sockets.recv(CommandType.SENSOR_DATA_REPORT, frequency=10)
    def sensor_data_report(self, data: bytes):
        """接收运动控制传感器数据（200Hz）"""
        _, data_obj = unpack_q25_udp_cmd(data)
        if not isinstance(data_obj, ControllerSensorData):
            return
        imu = data_obj.imu_data
        json_data = {
            "imu": {
                "timestamp": imu.timestamp,
                "angle": [round(imu.roll, 2), round(imu.pitch, 2), round(imu.yaw, 2)],
                "angular_vel": [
                    round(imu.omega_x, 3),
                    round(imu.omega_y, 3),
                    round(imu.omega_z, 3),
                ],
                "acceleration": [
                    round(imu.acc_x, 3),
                    round(imu.acc_y, 3),
                    round(imu.acc_z, 3),
                ],
            },
            "joint_pos": self._format_joint_data(data_obj.joint_pos),
            "joint_vel": self._format_joint_data(data_obj.joint_vel),
            "joint_torque": self._format_joint_data(data_obj.joint_tau),
        }
        # http.ws_send(self, json_data, MessageType.STATE)

    @sockets.recv(CommandType.CONTROLLER_SAFE_DATA_REPORT, frequency=1)
    def controller_safe_report(self, data: bytes):
        """接收运动控制系统数据（1Hz）"""
        _, data_obj = unpack_q25_udp_cmd(data)
        if not isinstance(data_obj, ControllerSafeData):
            return
        json_data = {
            "motor_temperatures": [
                round(temp, 1) for temp in data_obj.motor_temperatures
            ],
            "driver_temperatures": data_obj.driver_temperatures,
            "cpu": {
                "temperature": round(data_obj.cpu_info.temperature, 1),
                "frequency": round(data_obj.cpu_info.frequency, 0),
            },
        }
        http.ws_send(self, json_data, MessageType.STATE)

    @sockets.recv(CommandType.BATTERY_LEVEL_REPORT, frequency=10)
    def battery_level_report(self, data: bytes):
        """接收电池电量数据（0.5Hz）"""
        _, data_obj = unpack_q25_udp_cmd(data)
        if not isinstance(data_obj, BatteryLevel):
            return
        json_data = {"type": "battery_level", "level": data_obj.level}
        http.ws_send(self, json_data, MessageType.STATE)

    @sockets.recv(CommandType.BATTERY_CHARGE_STATE_REPORT, frequency=10)
    def battery_charge_report(self, data: bytes):
        """接收电池充电状态数据（0.5Hz）"""
        _, data_obj = unpack_q25_udp_cmd(data)
        if not isinstance(data_obj, BatteryChargeState):
            return
        json_data = {
            "level": data_obj.level,
            "is_charging": data_obj.is_charging,
            "charge_desc": "充电中" if data_obj.is_charging else "未充电",
        }
        http.ws_send(self, json_data, MessageType.STATE)

    @sockets.recv(CommandType.ERROR_CODE_REPORT, frequency=10)
    def error_code_report(self, data: bytes):
        """接收错误码数据"""
        _, data_obj = unpack_q25_udp_cmd(data)
        if not isinstance(data_obj, ErrorCode):
            return
        level_desc = {0: "通知", 1: "警告", 2: "错误"}
        json_data = {
            "error_level": data_obj.error_level,
            "level_desc": level_desc.get(data_obj.error_level, "未知"),
            "error_code": hex(data_obj.error_code),
            "error_msg": data_obj.error_msg,
        }
        http.ws_send(self, json_data, MessageType.STATE)

    # ===================== 辅助工具方法 =====================

    @staticmethod
    def _format_joint_data(joint_data: LegJointData) -> dict:
        """格式化关节数据（位置/速度/力矩）"""
        return {
            "fl": {  # 左前腿
                "hipx": round(joint_data.fl_hipx, 3),
                "hipy": round(joint_data.fl_hipy, 3),
                "knee": round(joint_data.fl_knee, 3),
            },
            "fr": {  # 右前腿
                "hipx": round(joint_data.fr_hipx, 3),
                "hipy": round(joint_data.fr_hipy, 3),
                "knee": round(joint_data.fr_knee, 3),
            },
            "hl": {  # 左后腿
                "hipx": round(joint_data.hl_hipx, 3),
                "hipy": round(joint_data.hl_hipy, 3),
                "knee": round(joint_data.hl_knee, 3),
            },
            "hr": {  # 右后腿
                "hipx": round(joint_data.hr_hipx, 3),
                "hipy": round(joint_data.hr_hipy, 3),
                "knee": round(joint_data.hr_knee, 3),
            },
        }

    # ===================== 心跳发送线程（2Hz）=====================
    def heartbeat_thread(self):
        """心跳指令发送线程（按文档要求2Hz频率）"""
        while True:
            try:
                # 打包手动模式心跳指令（指令值0，基本指令）
                data = pack_q25_udp_cmd(CommandType.MANUAL_HEARTBEAT, parameter_size=0)
                # 直接通过socketc发送
                socketc.send_to_server(self, data)
                time.sleep(0.5)  # 2Hz = 每0.5秒一次
            except Exception as e:
                print(f"心跳发送失败：{e}")
                time.sleep(0.5)


if __name__ == "__main__":
    # host = "localhost"
    host = "192.168.3.20"  # 实际机器人IP（无线接入）
    # host = "192.168.1.103"  # 有线接入IP
    # host = "192.168.2.103"  # 通讯接口IP
    # server_host = "192.168.3.157"
    server_host = "localhost"
    server_port = 43893

    def router(data: bytes):
        """路由函数：解析指令码并返回对应的CommandType"""
        try:
            command_id = struct.unpack("<I", data[0:4])[0]
            return CommandType(command_id)
        except Exception as e:
            print(f"指令解析失败！data: 0x{data.hex()}, error: {e}")
            return None

    # 组件配置
    config = [
        sockets.config(
            host="0.0.0.0",
            port=server_port,
            socket_type="udp",
            register=False,
            decode=False,
            router=router,
        ),
        socketc.config(
            host=host,
            port=43893,
            register=False,
            socket_type="udp",
            decode=False,
            router=router,
        ),
        http.config(port=8001),
    ]

    # 启动控制器
    controller = Controller(component_config=config)

    # 启动心跳发送线程（daemon=True：主进程退出时自动结束）
    heartbeat_t = threading.Thread(
        target=controller.heartbeat_thread,
        daemon=True,
        name="heartbeat-thread",
    )
    heartbeat_t.start()
    print("心跳线程已启动（2Hz）")

    # 主循环
    try:
        while True:
            time.sleep(0.1)
    except KeyboardInterrupt:
        print("\n程序正在退出...")
