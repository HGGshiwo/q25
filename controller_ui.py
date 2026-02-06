from dataclasses import dataclass
from event_callback.components.http.ui_config import (
    JoystickConfig,
    PrimaryButtonConfig,
    FormConfig,
    SwitchFormItemConfig,
    RadioFormItemConfig,
    SliderFormItemConfig,
    InputFormItemConfig,
    NumberFormItemConfig,
    SelectFormItemConfig,
    StatusConfig,
    ToastConfig,
    InnerButtonConfig,
)

# ===================== 核心状态显示配置 =====================
# 1. 机器人基础状态
StatusConfig(config_id="basic_state_desc", name="设备状态")
StatusConfig(config_id="gait_desc", name="当前步态")
StatusConfig(config_id="motion_mode", name="运动模式")

# 2. 电量与充电状态
StatusConfig(config_id="level", name="电池电量(%)")
StatusConfig(config_id="charge_desc", name="充电状态")

# 3. 运动性能参数
StatusConfig(config_id="max_forward_vel", name="最大前进速度(m/s)")
StatusConfig(config_id="max_backward_vel", name="最大后退速度(m/s)")

# 4. 错误信息提示
StatusConfig(config_id="level_desc", name="错误等级")
StatusConfig(config_id="error_msg", name="错误详情")

# ===================== 核心操作按钮配置 =====================
PrimaryButtonConfig(
    name="站立",
    target=ToastConfig(url="/takeoff", method="POST"),
)

PrimaryButtonConfig(
    name="趴下",
    target=ToastConfig(url="/land", method="POST"),
)

# 1. 基础姿态控制（站立/趴下）
PrimaryButtonConfig(
    name="站立/趴下",
    target=ToastConfig(url="/toggle_stand_down", method="POST"),
)

# 2. 紧急操作（软急停-危险按钮）
PrimaryButtonConfig(
    name="软急停",
    target=ToastConfig(url="/emergecy_stop", method="POST"),
)

# 3. L模式（强化学习模式）控制
PrimaryButtonConfig(
    name="L模式控制",
    target=FormConfig(
        title="L模式控制",
        url="/l-mode",
        method="POST",
        items={
            "enter": SwitchFormItemConfig(name="是否进入L模式", default=True),
        },
        submit=InnerButtonConfig(
            target=ToastConfig(url="/l-mode", method="POST"), name="确认执行"
        ),
    ),
)

# 5. 平台高度切换（匍匐/正常）
PrimaryButtonConfig(
    name="平台高度",
    target=FormConfig(
        title="平台高度",
        url="/platform-height",
        method="GET",
        items={
            "height": RadioFormItemConfig(
                name="平台高度",
                options={
                    "0": "匍匐高度",
                    "2": "正常高度",
                },
                default="2",
            ),
        },
        submit=InnerButtonConfig(
            target=ToastConfig(url="/platform-height", method="POST"), name="确认切换"
        ),
    ),
)

# 6. 步态切换（行走/跑步）
PrimaryButtonConfig(
    name="切换步态",
    target=FormConfig(
        title="切换步态",
        url="/gait",
        method="GET",
        items={
            "is_run": SwitchFormItemConfig(name="跑步步态", default=False),
        },
        submit=InnerButtonConfig(
            target=ToastConfig(url="/gait", method="POST"), name="确认切换"
        ),
    ),
)


# 8. 对准姿态控制
PrimaryButtonConfig(
    name="对准姿态",
    target=FormConfig(
        title="对准姿态",
        url="/aim",
        method="POST",
        items={
            "reset": SwitchFormItemConfig(name="是否重置对准姿态", default=True),
            "roll": NumberFormItemConfig(
                name="翻滚角(-1000~1000)", min=-1000, max=1000, default=0
            ),
            "pitch": NumberFormItemConfig(
                name="俯仰角(-1000~1000)", min=-1000, max=1000, default=0
            ),
            "yaw": NumberFormItemConfig(
                name="偏航角(-1000~1000)", min=-1000, max=1000, default=0
            ),
        },
        submit=InnerButtonConfig(
            target=ToastConfig(url="/aim", method="POST"), name="确认设置"
        ),
    ),
)

JoystickConfig(url="/move/joystick", method="POST")
