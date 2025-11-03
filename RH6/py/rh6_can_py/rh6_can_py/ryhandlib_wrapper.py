#!/usr/bin/env python3
"""
RyHand CAN servo library Python wrapper (ctypes) with SocketCAN backend.

MVP goal: be able to send mixed position commands and query servo info,
matching the C++ implementation's core capabilities, so the Python ROS node
can control the hand in WSL/Ubuntu via a real CAN interface (e.g., can0).

This wrapper:
- Loads the vendor library (libRyhand*.so) appropriate for the CPU arch
- Creates a SocketCAN raw socket and binds to the given interface
- Provides a BusWrite callback to the C library (RyCanServoBusInit)
- Starts a RX thread to feed incoming CAN frames to RyCanServoLibRcvMsg
- Exposes minimal APIs used by the Python node: init, servo_move_mix,
  get_servo_data, cleanup

Notes:
- 同步/异步调用都需要持续把接收帧喂给库，否则超时；本封装已实现 RX 线程
- The vendor library expects a millisecond tick counter pointer; we provide a
  background thread to increment a 0..999 counter at 1ms (period=1000).
- You must bring up SocketCAN (e.g., `sudo ip link set can0 up type can bitrate 1000000`).
"""

from __future__ import annotations
import ctypes as C
import os
import socket
import fcntl
import struct
import threading
import time
import platform
from typing import Optional

# -----------------------------------------------------------------------------
# Paths and dynamic library loading
# -----------------------------------------------------------------------------

def _detect_lib_path() -> str:
    """
    Try multiple locations so it "just works" in both repo and catkin workspace:
    - <repo_root>/cpp/rh6_ctrl/lib
    - <workspace_root>/src/cpp/rh6_ctrl/lib
    - Package directory and its parent (where .so may be placed alongside the package)
    - <repo_root>/py/lib or <workspace_root>/src/py/lib (shared libs for py)
    - RYHAND_LIB_DIR (directory) or RYHAND_LIB (full path)
    """
    here = os.path.dirname(__file__)
    root = os.path.abspath(os.path.join(here, "../../.."))  # repo root or workspace root
    arch = platform.machine().lower()
    if arch in ("x86_64", "amd64"):
        names = ["libRyhand64.so", "libRyhand.so"]
    elif arch in ("aarch64", "arm64"):
        names = ["libRyhandArm64.so", "libRyhand.so"]
    elif arch.startswith("arm"):
        names = ["libRyhandArm.so", "libRyhand.so"]
    else:
        names = ["libRyhand.so"]

    # Candidate directories to scan (prioritize rh6_ctrl_py/lib as requested)
    search_dirs = [
        os.path.join(root, "py", "rh6_ctrl_py", "lib"),        # repo:   <repo_root>/py/rh6_ctrl_py/lib
        os.path.join(root, "src", "rh6_ctrl_py", "lib"),       # catkin: <ws_root>/src/rh6_ctrl_py/lib
        os.path.join(root, "cpp", "rh6_ctrl", "lib"),          # repo layout
        os.path.join(root, "src", "cpp", "rh6_ctrl", "lib"),  # catkin src layout
        here,                                                      # package dir (rh6_can_py/rh6_can_py)
        os.path.join(here, "lib"),                                # package lib subdir
        os.path.abspath(os.path.join(here, "..")),                # package parent (rh6_can_py)
        os.path.abspath(os.path.join(here, "..", "lib")),        # package parent lib subdir
        os.path.join(root, "py", "lib"),                         # shared py libs (repo)
        os.path.join(root, "src", "py", "lib"),                 # shared py libs (catkin)
    ]

    # Environment-provided directory
    env_dir = os.getenv("RYHAND_LIB_DIR")
    if env_dir:
        search_dirs.insert(0, env_dir)

    for d in search_dirs:
        if not d or not os.path.isdir(d):
            continue
        for n in names:
            p = os.path.join(d, n)
            if os.path.isfile(p):
                return p

    # Full path fallback
    env = os.getenv("RYHAND_LIB")
    if env and os.path.isfile(env):
        return env

    raise FileNotFoundError(
        "RyHand library not found; place libRyhand*.so in cpp/rh6_ctrl/lib or py/lib, "
        "or set RYHAND_LIB_DIR to a directory or RYHAND_LIB to a full path"
    )

# Load library
_LIB = None
_LIB_PATH = None
try:
    _LIB_PATH = _detect_lib_path()
    _LIB = C.CDLL(_LIB_PATH)
    print(f"[ryhandlib_wrapper] Loaded vendor library: {_LIB_PATH}")
except Exception as e:
    _LIB_LOAD_ERROR = e
else:
    _LIB_LOAD_ERROR = None

# -----------------------------------------------------------------------------
# C types mirror (subset used by the wrapper)
# -----------------------------------------------------------------------------

# Integer typedefs
# 注意：回调返回值需为“数值”，用 c_int8 而不是 c_char，避免 ctypes 要求 bytes 的错误
s8_t = C.c_int8
u8_t = C.c_uint8
u16_t = C.c_uint16
s16_t = C.c_int16
u32_t = C.c_uint32

class CanMsg_t(C.Structure):
    _fields_ = [
        ("ulId", u32_t),
        ("ucLen", u8_t),
        ("pucDat", u8_t * 64),
    ]

class ServoInfoBits(C.Structure):
    _pack_ = 1  # match #pragma pack(1) in vendor header
    _fields_ = [
        ("dummy_cmd", u8_t),
        ("ucStatus", u8_t),
        ("ub_P", u16_t),
        ("ub_V", u16_t),
        ("ub_I", u16_t),
        ("ub_F", u16_t),
    ]

class FingerServoCmd_t(C.Structure):
    _pack_ = 1  # match #pragma pack(1)
    _fields_ = [
        ("dummy_cmd", u8_t),
        ("usTp", u16_t),
        ("usTv", u16_t),
        ("usTc", u16_t),
        ("reserved", u8_t),
    ]

class ServoData_t(C.Union):
    _pack_ = 1  # match #pragma pack(1)
    _fields_ = [
        ("stuInfo", ServoInfoBits),
        ("stuCmd", FingerServoCmd_t),
        ("pucDat", u8_t * 64),
    ]

# Forward declaration for callback signature
class RyCanServoBus_t(C.Structure):
    pass

# BusWrite callback type: s8_t (*)(CanMsg_t)
BUSWRITE_CB = C.CFUNCTYPE(s8_t, CanMsg_t)
# Listen/Hook callback type: void (*)(CanMsg_t, void*)
LISTEN_CB = C.CFUNCTYPE(None, CanMsg_t, C.c_void_p)

# MsgHook_t and MsgListen_t for GetServoUpdateInfo
class MsgHook_t(C.Structure):
    _pack_ = 1
    _fields_ = [
        ("ucEn", u8_t),
        ("ucAlive", u8_t),
        ("pstuMsg", C.POINTER(CanMsg_t)),
        ("funCbk", LISTEN_CB),
    ]

class MsgListen_t(C.Structure):
    _pack_ = 1
    _fields_ = [
        ("stuListen", MsgHook_t),
        ("stuRet", ServoData_t),
        ("ucConfidence", u8_t),
    ]

# Define bus struct (only fields needed by the library; keep ordering)
# According to header, fields are:
#   volatile u16_t* pusTicksMs; u16_t usTicksPeriod; u16_t usHookNum; u16_t usListenNum;
#   MsgHook_t* pstuHook; MsgListen_t* pstuListen; BusWrite_t pfunWrite;
# We only need to pass pointers/values; we won't deref in Python.
RyCanServoBus_t._pack_ = 1
RyCanServoBus_t._fields_ = [
    ("pusTicksMs", C.POINTER(u16_t)),
    ("usTicksPeriod", u16_t),
    ("usHookNum", u16_t),
    ("usListenNum", u16_t),
    ("pstuHook", C.c_void_p),
    ("pstuListen", C.c_void_p),
    ("pfunWrite", BUSWRITE_CB),
]

# Bind library functions (subset)
if _LIB is not None:
    _LIB.RyCanServoBusInit.argtypes = [C.POINTER(RyCanServoBus_t), BUSWRITE_CB, C.POINTER(u16_t), u16_t]
    _LIB.RyCanServoBusInit.restype = u8_t

    _LIB.RyCanServoBusDeInit.argtypes = [C.POINTER(RyCanServoBus_t)]
    _LIB.RyCanServoBusDeInit.restype = None

    # Confirm exact signature per header: (RyCanServoBus_t*, u8_t, s16_t, u16_t, u16_t, ServoData_t*, u16_t)
    _LIB.RyMotion_ServoMove_Mix.argtypes = [C.POINTER(RyCanServoBus_t), u8_t, s16_t, u16_t, u16_t, C.POINTER(ServoData_t), u16_t]
    _LIB.RyMotion_ServoMove_Mix.restype = u8_t

    _LIB.RyFunc_GetServoInfo.argtypes = [C.POINTER(RyCanServoBus_t), u8_t, C.POINTER(ServoData_t), u16_t]
    _LIB.RyFunc_GetServoInfo.restype = u8_t

    # Optional APIs
    _LIB.RyParam_SetUpateRate = getattr(_LIB, 'RyParam_SetUpateRate', None)
    if _LIB.RyParam_SetUpateRate is not None:
        _LIB.RyParam_SetUpateRate.argtypes = [C.POINTER(RyCanServoBus_t), u8_t, u16_t, C.POINTER(ServoData_t), u16_t]
        _LIB.RyParam_SetUpateRate.restype = u8_t

    # 从监听缓存获取伺服上报信息
    _LIB.GetServoUpdateInfo = getattr(_LIB, 'GetServoUpdateInfo', None)
    if _LIB.GetServoUpdateInfo is not None:
        _LIB.GetServoUpdateInfo.argtypes = [C.POINTER(RyCanServoBus_t), u8_t, C.POINTER(MsgListen_t)]
        _LIB.GetServoUpdateInfo.restype = s8_t

    _LIB.AddHook = getattr(_LIB, 'AddHook', None)
    _LIB.AddListen = getattr(_LIB, 'AddListen', None)
    _LIB.DeleteHook = getattr(_LIB, 'DeleteHook', None)

    _LIB.RyCanServoLibRcvMsg = getattr(_LIB, 'RyCanServoLibRcvMsg', None)
    if _LIB.RyCanServoLibRcvMsg is not None:
        _LIB.RyCanServoLibRcvMsg.argtypes = [C.POINTER(RyCanServoBus_t), CanMsg_t]
        _LIB.RyCanServoLibRcvMsg.restype = s8_t

    # 便捷：轮询 GetServoInfo
    _LIB.RyFunc_GetVersion = getattr(_LIB, 'RyFunc_GetVersion', None)

    _LIB.RyParam_ClearFault.argtypes = [C.POINTER(RyCanServoBus_t), u8_t, u16_t]
    _LIB.RyParam_ClearFault.restype = u8_t

    # 设置伺服主动上报频率
    _LIB.RyParam_SetUpateRate = getattr(_LIB, 'RyParam_SetUpateRate', None)
    if _LIB.RyParam_SetUpateRate is not None:
        _LIB.RyParam_SetUpateRate.argtypes = [C.POINTER(RyCanServoBus_t), u8_t, u16_t, C.POINTER(ServoData_t), u16_t]
        _LIB.RyParam_SetUpateRate.restype = u8_t

    # 从监听缓存获取伺服上报信息
    _LIB.GetServoUpdateInfo = getattr(_LIB, 'GetServoUpdateInfo', None)
    if _LIB.GetServoUpdateInfo is not None:
        _LIB.GetServoUpdateInfo.argtypes = [C.POINTER(RyCanServoBus_t), u8_t, C.POINTER(MsgListen_t)]
        _LIB.GetServoUpdateInfo.restype = s8_t

    _LIB.AddHook = getattr(_LIB, 'AddHook', None)
    _LIB.AddListen = getattr(_LIB, 'AddListen', None)
    _LIB.DeleteHook = getattr(_LIB, 'DeleteHook', None)

    _LIB.RyCanServoLibRcvMsg = getattr(_LIB, 'RyCanServoLibRcvMsg', None)
    if _LIB.RyCanServoLibRcvMsg is not None:
        _LIB.RyCanServoLibRcvMsg.argtypes = [C.POINTER(RyCanServoBus_t), CanMsg_t]
        _LIB.RyCanServoLibRcvMsg.restype = s8_t

    # 便捷：轮询 GetServoInfo
    _LIB.RyFunc_GetVersion = getattr(_LIB, 'RyFunc_GetVersion', None)

# -----------------------------------------------------------------------------
# SocketCAN helpers
# -----------------------------------------------------------------------------

class _SocketCAN:
    def __init__(self, interface: str):
        self.interface = interface
        self.sock: Optional[socket.socket] = None

    def open(self) -> None:
        s = socket.socket(socket.PF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
        # non-blocking
        fl = fcntl.fcntl(s, fcntl.F_GETFL)
        fcntl.fcntl(s, fcntl.F_SETFL, fl | os.O_NONBLOCK)
        # Increase buffers to match C++ logs more closely
        try:
            s.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 229376)
            s.setsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF, 212992)
        except Exception:
            pass
        # bind to interface
        try:
            s.bind((self.interface,))  # name-based bind
        except OSError as e:
            s.close()
            raise RuntimeError(f"Failed to bind CAN socket on {self.interface}: {e}")
        # report buffers
        try:
            rcv = s.getsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF)
            snd = s.getsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF)
            print(f"CAN socket open ok on interface: {self.interface}")
            print(f"Send buffer size: {snd} bytes")
            print(f"Receive buffer size: {rcv} bytes")
        except Exception:
            print(f"CAN socket open ok on interface: {self.interface}")
        self.sock = s

    def close(self) -> None:
        if self.sock:
            try:
                self.sock.close()
            finally:
                self.sock = None

    def send_frame(self, can_id: int, data: bytes) -> bool:
        if self.sock is None:
            return False
        if len(data) > 8:
            data = data[:8]
        dlc = len(data)
        frame_fmt = 'IB3x8s'  # can_id (uint32), dlc (uint8), padding, data[8]
        frame = struct.pack(frame_fmt, can_id, dlc, data.ljust(8, b'\x00'))
        try:
            n = self.sock.send(frame)
            return n == 16
        except OSError:
            return False

    def recv_frame(self) -> Optional[tuple[int, bytes]]:
        if self.sock is None:
            return None
        try:
            raw = self.sock.recv(16)
            can_id, dlc, data = struct.unpack('IB3x8s', raw)
            return can_id, data[:dlc]
        except BlockingIOError:
            return None
        except OSError:
            return None

# -----------------------------------------------------------------------------
# Public wrapper API
# -----------------------------------------------------------------------------

class _ServoInfoObj:
    def __init__(self, info_dict: dict):
        self.info = info_dict

class RyCanServoLibWrapper:
    def __init__(self) -> None:
        if _LIB_LOAD_ERROR is not None:
            raise RuntimeError(f"Failed to load RyHand library: {_LIB_LOAD_ERROR}")
        self._can = None  # _SocketCAN
        self._bus = RyCanServoBus_t()
        self._tick_val = u16_t(0)
        self._tick_thread = None
        self._rx_thread = None
        self._running = False
        # Keep callback alive
        self._buswrite_cb = BUSWRITE_CB(self._buswrite)
        # config
        self._id_min = 1
        self._id_max = int(os.getenv('RYHAND_ID_MAX', '6'))
        self._report_period_ms = int(os.getenv('RYHAND_REPORT_MS', '50'))

    # BusWrite callback invoked by C library
    def _buswrite(self, msg: CanMsg_t) -> s8_t:
        if self._can is None:
            return -1  # return Python int; CFUNCTYPE will convert to c_int8
        # Use 11-bit standard ID; do not truncate to 8-bit (avoid 0x700->0x00)
        can_id = int(msg.ulId) & 0x7FF
        data = bytes(bytearray(msg.pucDat)[: msg.ucLen])
        # Debug print to verify outgoing frames
        try:
            print(f"[CAN TX] id=0x{can_id:X} len={msg.ucLen} data={data.hex(' ')}")
        except Exception:
            pass
        ok = self._can.send_frame(can_id, data)
        if not ok:
            try:
                print(f"[CAN TX ERROR] send failed for id=0x{can_id:X}")
            except Exception:
                pass
        return 0 if ok else -1

    def _tick_loop(self):
        # 1ms periodic tick 0..999
        while self._running:
            val = (self._tick_val.value + 1) % 1000
            self._tick_val.value = val
            time.sleep(0.001)

    def _rx_loop(self):
        if _LIB is None or _LIB.RyCanServoLibRcvMsg is None:
            return
        while self._running:
            frame = self._can.recv_frame() if self._can else None
            if frame is None:
                time.sleep(0.001)
                continue
            can_id, data = frame
            # Debug print to observe incoming frames
            try:
                print(f"[CAN RX] id=0x{can_id:X} len={len(data)} data={data.hex(' ')}")
            except Exception:
                pass
            msg = CanMsg_t()
            msg.ulId = can_id
            msg.ucLen = len(data)
            for i, b in enumerate(data):
                msg.pucDat[i] = b
            try:
                # 将帧喂给协议库做异步解析
                _LIB.RyCanServoLibRcvMsg(C.byref(self._bus), msg)
            except Exception as e:
                # Avoid crashing RX loop on occasional ctypes errors
                print(f"[ryhandlib_wrapper] RyCanServoLibRcvMsg error: {e}")
                time.sleep(0.001)
                continue

    def init(self, interface: str = "can0") -> bool:
        # Open CAN
        self._can = _SocketCAN(interface)
        self._can.open()
        # Prepare bus struct
        self._bus.pusTicksMs = C.pointer(self._tick_val)
        self._bus.usTicksPeriod = u16_t(1000)
        # 预留若干 listen 槽位（和 C++ 一样由库内部管理），Python 侧不直接分配内存
        self._bus.usHookNum = u16_t(0)
        self._bus.usListenNum = u16_t(0)
        self._bus.pstuHook = None
        self._bus.pstuListen = None
        self._bus.pfunWrite = self._buswrite_cb
        # Init C library first (before starting threads to avoid race conditions)
        ret = _LIB.RyCanServoBusInit(C.byref(self._bus), self._buswrite_cb, C.byref(self._tick_val), u16_t(1000))
        if ret != 0:
            self.cleanup()
            raise RuntimeError(f"RyCanServoBusInit failed, ret={ret}")
        # Now start background threads
        self._running = True
        self._tick_thread = threading.Thread(target=self._tick_loop, daemon=True)
        self._tick_thread.start()
        self._rx_thread = threading.Thread(target=self._rx_loop, daemon=True)
        self._rx_thread.start()
        # Clear faults (broadcast id 0)
        try:
            _LIB.RyParam_ClearFault(C.byref(self._bus), u8_t(0), u16_t(1))
        except Exception:
            pass
        # 配置主动上报频率，并扫描有效 ID（只调用一次）
        try:
            self._scan_and_config_ids()
        except Exception as e:
            print(f"[ryhandlib_wrapper] scan/config skipped due to error: {e}")
        return True
    def _scan_and_config_ids(self):
        ok_ids = []
        # 配置主动上报频率（若库支持）
        if hasattr(_LIB, 'RyParam_SetUpateRate'):
            for mid in range(self._id_min, self._id_max + 1):
                try:
                    sd = ServoData_t()
                    _LIB.RyParam_SetUpateRate(C.byref(self._bus), u8_t(mid), u16_t(self._report_period_ms), C.byref(sd), u16_t(50))
                except Exception:
                    pass
        # 扫描响应的 ID（增加超时与重试）
        for mid in range(self._id_min, self._id_max + 1):
            try:
                found = False
                for _ in range(3):
                    sd = ServoData_t()
                    r = _LIB.RyFunc_GetServoInfo(C.byref(self._bus), u8_t(mid), C.byref(sd), u16_t(100))
                    if r in (0, 1):
                        ok_ids.append(mid)
                        found = True
                        break
                    time.sleep(0.01)
                if not found:
                    try:
                        print(f"[scan] id={mid} no response")
                    except Exception:
                        pass
            except Exception:
                pass
        if ok_ids:
            self._active_ids = ok_ids
            print(f"[ryhandlib_wrapper] Active servo IDs: {ok_ids}; report={self._report_period_ms}ms")
        else:
            self._active_ids = []
            print("[ryhandlib_wrapper] No servo responded in scan; check IDs, power, and cabling")

    def servo_move_mix(self, motor_id: int, pos: int, vel: int, cur: int) -> bool:
        sd = ServoData_t()
        ret = _LIB.RyMotion_ServoMove_Mix(C.byref(self._bus), u8_t(motor_id & 0xFF), s16_t(pos & 0xFFFF), u16_t(vel & 0xFFFF), u16_t(cur & 0xFFFF), C.byref(sd), u16_t(1))
        return bool(ret == 0 or ret == 1)

    def get_servo_data(self, motor_id: int) -> Optional[dict]:
        sd = ServoData_t()
        # 确保 motor_id 映射到 8 位无符号数
        mid = int(motor_id) & 0xFF
        # 提高等待时间，避免超时导致无数据
        ret = _LIB.RyFunc_GetServoInfo(C.byref(self._bus), u8_t(mid), C.byref(sd), u16_t(100))
        if ret in (0, 1):
            # 直接从原始字节解析（与 candump 观察到的帧一致：AA | posLo posHi | velLo velHi | curLo curHi）
            raw = bytes(bytearray(sd.pucDat)[:7])
            if len(raw) < 7 or raw[0] != 0xAA:
                # 回退到结构体字段（某些固件可能用结构体填充）
                info = sd.stuInfo
                v = int(info.ub_V if info.ub_V <= 2047 else info.ub_V - 4096)
                i = int(info.ub_I if info.ub_I <= 2047 else info.ub_I - 4096)
                return {
                    'ub_P': int(info.ub_P),
                    'ub_V': v,
                    'ub_I': i,
                    'ucStatus': int(info.ucStatus),
                    'ub_F': int(info.ub_F),
                }
            import struct
            pos, vel, cur = struct.unpack('<HHH', raw[1:7])
            # 速度/电流是 12bit 有符号，按你原先逻辑做 0..4095 转换
            if vel > 2047:
                vel -= 4096
            if cur > 2047:
                cur -= 4096
            return {
                'ub_P': int(pos),
                'ub_V': int(vel),
                'ub_I': int(cur),
                'ucStatus': int(raw[6]),
                'ub_F': 0,
            }
        return None

    def cleanup(self) -> None:
        # Stop background threads first to avoid calling into the C library while deinitializing
        self._running = False
        # Join RX first (it may still be calling into the C library), then the tick thread
        for th in (self._rx_thread, self._tick_thread):
            if th and th.is_alive():
                try:
                    th.join(timeout=1.0)
                except Exception:
                    pass
        # Now it's safe to deinit the C library
        try:
            if _LIB is not None:
                _LIB.RyCanServoBusDeInit(C.byref(self._bus))
        except Exception:
            pass
        # Clear references and close socket
        self._tick_thread = None
        self._rx_thread = None
        if self._can:
            try:
                self._can.close()
            finally:
                self._can = None

# Convenience global instance API (compatible with existing Python node import)
_wrapper: Optional[RyCanServoLibWrapper] = None

def init_can_servo_lib(interface: str = "can0") -> bool:
    global _wrapper
    if _wrapper is None:
        _wrapper = RyCanServoLibWrapper()
    return _wrapper.init(interface)


def servo_move_mix(motor_id: int, pos: int, vel: int, current: int) -> bool:
    if _wrapper is None:
        raise RuntimeError("RyHand wrapper not initialized")
    return _wrapper.servo_move_mix(motor_id, pos, vel, current)


def get_servo_data(motor_id: int) -> Optional[object]:
    if _wrapper is None:
        return None
    d = _wrapper.get_servo_data(motor_id)
    return _ServoInfoObj(d) if d is not None else None


def cleanup_can_servo_lib() -> None:
    global _wrapper
    if _wrapper is not None:
        _wrapper.cleanup()
        _wrapper = None


def get_loaded_lib_path() -> Optional[str]:
    return _LIB_PATH

