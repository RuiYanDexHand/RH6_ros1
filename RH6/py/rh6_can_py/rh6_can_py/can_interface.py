#!/usr/bin/env python3
"""
RH6 CAN communication library (Python)
Extracted from rh6_ctrl_py to a reusable package.
"""

import threading
import time
from typing import Dict, Any, Callable, Optional
import struct

# import can  # enable when using real socketcan

class CanMsg:
    def __init__(self, can_id: int = 0, data: bytes = b'', dlc: int = 0):
        self.ulId = can_id
        self.pucDat = data
        self.ucLen = dlc

class ServoData:
    def __init__(self):
        self.cmd = {'usTp': 0, 'usTv': 0, 'usTc': 0}
        self.info = {'ucStatus': 0, 'ub_P': 0, 'ub_V': 0, 'ub_I': 0, 'ub_F': 0}

class CanInterface:
    def __init__(self, interface: str = "can0"):
        self.interface = interface
        self.running = False
        self.rx_thread = None
        self.callbacks: Dict[int, Callable[[CanMsg], None]] = {}
        self.bus = None  # can.interface.Bus(channel=self.interface, bustype='socketcan')
        print(f"CAN 接口初始化: {interface}")

    def start(self):
        self.running = True
        self.rx_thread = threading.Thread(target=self._rx_thread_func, daemon=True)
        self.rx_thread.start()
        print("CAN 接收线程已启动")

    def stop(self):
        self.running = False
        if self.rx_thread and self.rx_thread.is_alive():
            self.rx_thread.join()
        print("CAN 接收线程已停止")

    def send_message(self, msg: CanMsg) -> bool:
        try:
            # can_msg = can.Message(arbitration_id=msg.ulId, data=msg.pucDat, is_extended_id=False)
            # self.bus.send(can_msg)
            print(f"发送 CAN 消息: ID=0x{msg.ulId:03X}, 数据={msg.pucDat.hex()}")
            return True
        except Exception as e:
            print(f"CAN 发送失败: {e}")
            return False

    def register_callback(self, can_id: int, callback: Callable[[CanMsg], None]):
        self.callbacks[can_id] = callback
        print(f"注册 CAN ID 0x{can_id:03X} 的回调函数")

    def _rx_thread_func(self):
        while self.running:
            time.sleep(0.01)
            self._simulate_received_messages()

    def _simulate_received_messages(self):
        for motor_id in range(1, 7):
            can_id = 0x180 + motor_id
            pos = 2048 + motor_id * 10
            vel = 0
            current = 0
            status = 0x37
            data = struct.pack('<HHHB', pos, vel, current, status)
            msg = CanMsg(can_id, data, len(data))
            self._handle_received_message(msg)

    def _handle_received_message(self, msg: CanMsg):
        cb = self.callbacks.get(msg.ulId)
        if cb:
            cb(msg)

class RyCanServoLib:
    def __init__(self):
        self.can_interface: Optional[CanInterface] = None
        self.servo_data: Dict[int, ServoData] = {}

    def init(self, interface: str = "can0") -> bool:
        self.can_interface = CanInterface(interface)
        self.can_interface.start()
        for motor_id in range(1, 7):
            can_id = 0x180 + motor_id
            self.can_interface.register_callback(can_id, self._motor_status_callback)
        print("CAN 伺服库初始化成功")
        return True

    def servo_move_mix(self, motor_id: int, pos: int, vel: int, current: int) -> bool:
        can_id = 0x200 + motor_id
        data = struct.pack('<HHH', pos, vel, current)
        msg = CanMsg(can_id, data, len(data))
        return self.can_interface.send_message(msg) if self.can_interface else False

    def _motor_status_callback(self, msg: CanMsg):
        motor_id = msg.ulId - 0x180
        if len(msg.pucDat) >= 7:
            pos, vel, current, status = struct.unpack('<HHHB', msg.pucDat[:7])
            sd = self.servo_data.setdefault(motor_id, ServoData())
            sd.info['ub_P'] = pos
            sd.info['ub_V'] = vel
            sd.info['ub_I'] = current
            sd.info['ucStatus'] = status

    def get_servo_data(self, motor_id: int) -> Optional[ServoData]:
        return self.servo_data.get(motor_id)

    def cleanup(self):
        if self.can_interface:
            self.can_interface.stop()

ry_can_servo_lib = RyCanServoLib()

def init_can_servo_lib(interface: str = "can0") -> bool:
    return ry_can_servo_lib.init(interface)

def servo_move_mix(motor_id: int, pos: int, vel: int, current: int) -> bool:
    return ry_can_servo_lib.servo_move_mix(motor_id, pos, vel, current)

def get_servo_data(motor_id: int) -> Optional[ServoData]:
    return ry_can_servo_lib.get_servo_data(motor_id)

def cleanup_can_servo_lib():
    ry_can_servo_lib.cleanup()
