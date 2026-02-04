#!/usr/bin/python3
# -*- coding: utf-8 -*-
import threading
import time

from event_callback.components.http.message_handler import MessageType

from controller_test_model import TestModel
from controller_test_ui import *
from utils.utils import *
from event_callback import http
from event_callback import CallbackManager


class Controller(CallbackManager):
    def __init__(self, component_config=None):
        super().__init__(component_config)
        self.max_backward_vel = None
        self.max_forward_vel = None
        self.v_max_lock = threading.Lock()

        self.test_select = "123"
        self.test_input = "默认文字"
        self.test_number = 123
        self.test_slider = 2
        self.test_switch = False

    # ===================== 发送类指令（POST接口）=====================
    @http.get("/test")
    def test(self):
        return {
            "status": "success",
            "msg": {
                "test_switch": self.test_switch,
                "test_slider": self.test_slider,
                "test_input": self.test_input,
                "test_number": self.test_number,
                "test_select": self.test_select,
            },
        }

    @http.post("/test")
    def test(self, item: TestModel):
        self.test_switch = item.test_switch
        self.test_slider = item.test_slider
        self.test_input = item.test_input
        self.test_number = item.test_number
        self.test_select = item.test_select
        return {"status": "success", "msg": "OK"}


if __name__ == "__main__":
    host = "localhost"

    # 组件配置
    config = [
        http.config(port=8001),
    ]

    # 启动控制器
    controller = Controller(component_config=config)

    http.ws_send(controller, {"info": "测试info"}, MessageType.INFO)
    http.ws_send(controller, {"error": "测试error"}, MessageType.ERROR)
    http.ws_send(controller, {"basic_state_desc": "初始化中"}, MessageType.STATE)
    # 主循环
    try:
        while True:
            time.sleep(0.1)
    except KeyboardInterrupt:
        print("\n程序正在退出...")
