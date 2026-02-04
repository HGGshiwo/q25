from dataclasses import dataclass


from event_callback.components.http.ui_config import *


StatusConfig(config_id="gait_desc", name="步态")
StatusConfig(config_id="basic_state_desc", name="设备状态")
for i in range(30):
    StatusConfig(config_id=f"basic_state_desc{i}", name=f"设备状态{i}")

PrimaryButtonConfig(
    name="测试按钮",
    target=FormConfig(
        url="/test",
        method="GET",
        items={
            "test_switch": SwitchFormItemConfig(name="测试switch"),
            "test_slider": SliderFormItemConfig(
                name="测试slider", min=3, max=8, step=1
            ),
            "test_input": InputFormItemConfig(name="测试input"),
            "test_number": NumberFormItemConfig(name="测试number"),
            "test_select": SelectFormItemConfig(
                name="测试select",
                options={
                    "options1": "1",
                    "options2": "2",
                    "options3": "3",
                },
            ),
        },
        submit=InnerButtonConfig(target=ToastConfig(url="/test", method="POST")),
    ),
)

# ButtonConfig(
#     target=ToastConfig(value=dict(url=url1, method="GET"), format=lambda value: "xxx"),
# )

# ButtonConfig(
#     target=CopyConfig(value=dict(url=url1, method="GET")),
# )

# ButtonConfig(
#     target=FormCoinfg(
#         value=dict(url=url1, method="GET"),
#         items=[
#             SwitchFormItemConfig(key="test_switch", name="测试switch"),
#         ],
#         on_change=ButtonConfig(target=OnChangeConfig(url=xxx, method=xxx)),
#     ),
# )
