"""
GCU 云台 MCP Server

基于 GCU 私有通信协议 V2.0.6
支持串口通信控制云台旋转
设备默认连接: COM8 @ 115200bps
"""

import serial
import struct
import time
import threading
import asyncio
from typing import Optional, Tuple, List, Dict
from dataclasses import dataclass
from mcp.server.fastmcp import FastMCP

# Initialize MCP Server
mcp = FastMCP("gcu-gimbal")


@dataclass
class GimbalState:
    """云台状态数据"""
    timestamp: str
    work_mode: int
    camera_abs_roll: float
    camera_abs_pitch: float
    camera_abs_yaw: float
    camera_rel_x: float
    camera_rel_y: float
    camera_rel_z: float
    camera_vel_x: float
    camera_vel_y: float
    camera_vel_z: float


class GCUGimbalBridge:
    """
    GCU 云台串口通信桥接

    协议细节:
    - 帧头: 0xA8 0xE5 (发送) / 0x8A 0x5E (接收)
    - 波特率: 115200
    - 校验: CRC16
    - 字节序: Little Endian (数据), Big Endian (CRC)

    设备配置: COM8 @ 115200bps
    """

    # 协议常量
    PROTOCOL_HEADER_SEND = bytes([0xA8, 0xE5])
    PROTOCOL_HEADER_RECV = bytes([0x8A, 0x5E])
    PROTOCOL_VERSION = 0x01

    # 工作模式
    MODE_ANGLE_CONTROL = 0x10
    MODE_POINTING_LOCK = 0x11
    MODE_POINTING_FOLLOW = 0x12
    MODE_TOP_DOWN = 0x13
    MODE_EULER_ANGLE = 0x14
    MODE_GEO_STARE = 0x15
    MODE_TARGET_LOCK = 0x16
    MODE_TRACKING = 0x17
    MODE_POINTING_MOVE = 0x1A
    MODE_FPV = 0x1C

    # 相机命令
    CMD_CALIBRATION = 0x01
    CMD_RESET = 0x03
    CMD_PHOTO = 0x20
    CMD_RECORD = 0x21
    CMD_ZOOM_IN = 0x22
    CMD_ZOOM_OUT = 0x23
    CMD_ZOOM_STOP = 0x24
    CMD_ZOOM_SET = 0x25
    CMD_FOCUS = 0x26
    CMD_NIGHT_VISION = 0x2B
    CMD_OSD = 0x73
    CMD_ILLUMINATION = 0x80
    CMD_RANGING = 0x81

    # 状态标志位
    FLAG_CONTROL_VALID = 0x04
    FLAG_IMU_VALID = 0x01

    def __init__(self, port: str = 'COM8', baudrate: int = 115200):
        """
        初始化云台桥接

        Args:
            port: 串口号，默认 COM8
            baudrate: 波特率，默认 115200
        """
        self.port = port
        self.baudrate = baudrate
        self.serial: Optional[serial.Serial] = None
        self._running = False
        self._send_thread: Optional[threading.Thread] = None
        self._recv_thread: Optional[threading.Thread] = None
        self._lock = threading.Lock()

        # 控制状态
        self._roll_control = 0
        self._pitch_control = 0
        self._yaw_control = 0
        self._control_valid = True
        self._imu_valid = True
        self._current_mode = self.MODE_ANGLE_CONTROL

        # 载机数据
        self._aircraft_roll = 0
        self._aircraft_pitch = 0
        self._aircraft_yaw = 0
        self._accel_north = 0
        self._accel_east = 0
        self._accel_up = 0
        self._vel_north = 0
        self._vel_east = 0
        self._vel_up = 0

        # 接收数据
        self._recv_buffer = bytearray()
        self._latest_status: Optional[Dict] = None

        # 安全限位
        self.safety_limits = {
            "roll_control": 18000,      # ±18000 (±180°)
            "pitch_control": 18000,     # ±18000 (±180°)
            "yaw_control": 18000,       # ±18000 (±180°)
            "pitch_speed": 1500,        # ±1500 (±150°/s)
            "yaw_speed": 1500,          # ±1500 (±150°/s)
        }

    def connect(self) -> bool:
        """连接串口"""
        try:
            self.serial = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=0.01
            )
            return True
        except serial.SerialException as e:
            print(f"串口连接失败: {e}")
            return False

    def disconnect(self):
        """断开串口连接"""
        self.stop_sending()
        self.stop_receiving()
        if self.serial and self.serial.is_open:
            self.serial.close()

    def _calculate_crc16(self, data: bytes) -> int:
        """计算 CRC16 校验值"""
        crc_ta = [
            0x0000, 0x1021, 0x2042, 0x3063, 0x4084, 0x50a5, 0x60c6, 0x70e7,
            0x8108, 0x9129, 0xa14a, 0xb16b, 0xc18c, 0xd1ad, 0xe1ce, 0xf1ef
        ]
        crc = 0
        for byte in data:
            da = (crc >> 12) & 0x0F
            crc = (crc << 4) & 0xFFFF
            crc ^= crc_ta[da ^ ((byte >> 4) & 0x0F)]
            da = (crc >> 12) & 0x0F
            crc = (crc << 4) & 0xFFFF
            crc ^= crc_ta[da ^ (byte & 0x0F)]
        return crc

    def _build_control_packet(self, command: int = 0x00, params: bytes = b'') -> bytes:
        """构建控制数据包"""
        base_length = 72
        packet_length = base_length + len(params)

        packet = bytearray()
        packet.extend(self.PROTOCOL_HEADER_SEND)
        packet.extend(struct.pack('<H', packet_length))
        packet.append(self.PROTOCOL_VERSION)

        # 数据主帧 (32字节)
        packet.extend(struct.pack('<h', self._roll_control))
        packet.extend(struct.pack('<h', self._pitch_control))
        packet.extend(struct.pack('<h', self._yaw_control))

        status_flag = 0
        if self._control_valid:
            status_flag |= self.FLAG_CONTROL_VALID
        if self._imu_valid:
            status_flag |= self.FLAG_IMU_VALID
        packet.append(status_flag)

        packet.extend(struct.pack('<h', self._aircraft_roll))
        packet.extend(struct.pack('<h', self._aircraft_pitch))
        packet.extend(struct.pack('<H', self._aircraft_yaw % 36000))
        packet.extend(struct.pack('<h', self._accel_north))
        packet.extend(struct.pack('<h', self._accel_east))
        packet.extend(struct.pack('<h', self._accel_up))
        packet.extend(struct.pack('<h', self._vel_north))
        packet.extend(struct.pack('<h', self._vel_east))
        packet.extend(struct.pack('<h', self._vel_up))
        packet.append(0x01)
        packet.extend(bytes(6))

        # 数据副帧 (32字节)
        packet.append(0x01)
        packet.extend(bytes(31))

        # 指令和参数
        packet.append(command)
        packet.extend(params)

        # CRC校验 (大端序)
        crc = self._calculate_crc16(packet)
        packet.extend(struct.pack('>H', crc))

        return bytes(packet)

    def send_packet(self, command: int = 0x00, params: bytes = b'') -> bool:
        """发送控制数据包"""
        if not self.serial or not self.serial.is_open:
            return False

        try:
            packet = self._build_control_packet(command, params)
            self.serial.write(packet)
            return True
        except serial.SerialException:
            return False

    def start_sending(self, frequency: float = 30.0):
        """启动持续发送线程"""
        if self._running:
            return

        self._running = True
        self._send_thread = threading.Thread(target=self._send_loop, args=(frequency,))
        self._send_thread.daemon = True
        self._send_thread.start()

    def stop_sending(self):
        """停止持续发送"""
        self._running = False
        if self._send_thread:
            self._send_thread.join(timeout=1.0)

    def _send_loop(self, frequency: float):
        """发送循环"""
        interval = 1.0 / frequency
        while self._running:
            with self._lock:
                self.send_packet()
            time.sleep(interval)

    def start_receiving(self):
        """启动接收线程"""
        self._recv_thread = threading.Thread(target=self._recv_loop)
        self._recv_thread.daemon = True
        self._recv_thread.start()

    def stop_receiving(self):
        """停止接收"""
        if self._recv_thread:
            self._recv_thread.join(timeout=1.0)

    def _recv_loop(self):
        """接收循环"""
        while self.serial and self.serial.is_open:
            try:
                if self.serial.in_waiting > 0:
                    data = self.serial.read(self.serial.in_waiting)
                    self._recv_buffer.extend(data)
                    self._parse_buffer()
            except serial.SerialException:
                break
            time.sleep(0.001)

    def _parse_buffer(self):
        """解析接收缓冲区"""
        while len(self._recv_buffer) >= 72:
            header_idx = self._recv_buffer.find(self.PROTOCOL_HEADER_RECV)
            if header_idx == -1:
                self._recv_buffer.clear()
                return

            if header_idx > 0:
                self._recv_buffer = self._recv_buffer[header_idx:]

            if len(self._recv_buffer) < 4:
                return

            packet_length = struct.unpack('<H', self._recv_buffer[2:4])[0]

            if len(self._recv_buffer) < packet_length:
                return

            packet = bytes(self._recv_buffer[:packet_length])
            self._recv_buffer = self._recv_buffer[packet_length:]

            self._parse_packet(packet)

    def _parse_packet(self, packet: bytes):
        """解析GCU返回数据包"""
        try:
            if len(packet) < 72:
                return

            crc_received = struct.unpack('<H', packet[-2:])[0]
            crc_calculated = self._calculate_crc16(packet[:-2])
            if crc_received != crc_calculated:
                return

            self._latest_status = {
                'work_mode': packet[5],
                'camera_abs_roll': struct.unpack('<h', packet[18:20])[0] / 100.0,
                'camera_abs_pitch': struct.unpack('<h', packet[20:22])[0] / 100.0,
                'camera_abs_yaw': struct.unpack('<H', packet[22:24])[0] / 100.0,
                'camera_rel_x': struct.unpack('<h', packet[12:14])[0] / 100.0,
                'camera_rel_y': struct.unpack('<h', packet[14:16])[0] / 100.0,
                'camera_rel_z': struct.unpack('<h', packet[16:18])[0] / 100.0,
                'camera_vel_x': struct.unpack('<h', packet[24:26])[0] / 10.0,
                'camera_vel_y': struct.unpack('<h', packet[26:28])[0] / 10.0,
                'camera_vel_z': struct.unpack('<h', packet[28:30])[0] / 10.0,
            }

        except Exception:
            pass

    def get_latest_status(self) -> Optional[Dict]:
        """获取最新状态"""
        return self._latest_status

    def set_control_values(self, roll: int = 0, pitch: int = 0, yaw: int = 0, valid: bool = True):
        """设置控制量"""
        with self._lock:
            self._roll_control = roll
            self._pitch_control = pitch
            self._yaw_control = yaw
            self._control_valid = valid

    def set_aircraft_attitude(self, roll: float = 0, pitch: float = 0, yaw: float = 0):
        """设置载机姿态"""
        with self._lock:
            self._aircraft_roll = int(roll * 100)
            self._aircraft_pitch = int(pitch * 100)
            self._aircraft_yaw = int(yaw * 100) % 36000

    def _validate_safety(self, **kwargs) -> Tuple[bool, str]:
        """安全限位检查"""
        for limit_name, limit_value in self.safety_limits.items():
            if limit_name in kwargs:
                val = abs(kwargs[limit_name])
                if val > limit_value:
                    return False, f"{limit_name} 超出安全限位: {limit_value}"
        return True, "OK"


# Global bridge instance
_bridge: Optional[GCUGimbalBridge] = None


@mcp.tool()
async def connect_gimbal(port: str = "COM8", baudrate: int = 115200) -> str:
    """
    连接 GCU 云台设备

    Args:
        port: 串口号，默认 COM8
        baudrate: 波特率，默认 115200
    """
    global _bridge

    if _bridge is None:
        _bridge = GCUGimbalBridge(port=port, baudrate=baudrate)

    if _bridge.connect():
        _bridge.start_receiving()
        return f"✓ 已连接到 GCU 云台: {port} @ {baudrate}bps"
    else:
        return f"✗ 连接失败: {port}"


@mcp.tool()
async def disconnect_gimbal() -> str:
    """断开云台连接"""
    global _bridge

    if _bridge:
        _bridge.disconnect()
        _bridge = None
        return "✓ 已断开连接"

    return "未连接设备"


@mcp.tool()
async def set_mode(mode: str) -> str:
    """
    设置云台工作模式

    Args:
        mode: 模式名称 (angle_lock/follow/euler/fpv/top_down)
            - angle_lock: 指向锁定模式
            - follow: 指向跟随模式
            - euler: 欧拉角控制模式
            - fpv: FPV模式
            - top_down: 俯拍模式
    """
    global _bridge

    if _bridge is None:
        return "错误: 未连接设备"

    mode_map = {
        "angle_lock": _bridge.MODE_POINTING_LOCK,
        "follow": _bridge.MODE_POINTING_FOLLOW,
        "euler": _bridge.MODE_EULER_ANGLE,
        "fpv": _bridge.MODE_FPV,
        "top_down": _bridge.MODE_TOP_DOWN,
    }

    if mode not in mode_map:
        return f"错误: 未知模式 '{mode}'，可选: {list(mode_map.keys())}"

    mode_code = mode_map[mode]
    _bridge._current_mode = mode_code
    _bridge.send_packet(mode_code)

    return f"✓ 已切换到 {mode} 模式"


@mcp.tool()
async def rotate(pitch_speed: float = 0, yaw_speed: float = 0, duration: float = 1.0) -> str:
    """
    旋转云台（角速度模式）

    Args:
        pitch_speed: 俯仰速度 (°/s)，范围 [-150, 150]
        yaw_speed: 偏航速度 (°/s)，范围 [-150, 150]
        duration: 持续时间（秒）
    """
    global _bridge

    if _bridge is None:
        return "错误: 未连接设备"

    # 转换为协议单位 (0.1°/s)
    pitch_val = int(pitch_speed * 10)
    yaw_val = int(yaw_speed * 10)

    # 安全限位检查
    valid, msg = _bridge._validate_safety(pitch_speed=abs(pitch_val), yaw_speed=abs(yaw_val))
    if not valid:
        return f"安全限位检查失败: {msg}"

    _bridge.set_control_values(roll=0, pitch=pitch_val, yaw=yaw_val, valid=True)
    _bridge.start_sending(frequency=30)

    await asyncio.sleep(duration)

    _bridge.stop_sending()
    _bridge.set_control_values(roll=0, pitch=0, yaw=0, valid=False)

    return f"✓ 旋转完成: 俯仰={pitch_speed}°/s, 偏航={yaw_speed}°/s, 持续={duration}s"


@mcp.tool()
async def set_euler_angles(roll: float = 0, pitch: float = 0, yaw: float = 0) -> str:
    """
    设置云台欧拉角（需在 euler 模式下）

    Args:
        roll: 滚转角（度），范围 [-180, 180]
        pitch: 俯仰角（度），范围 [-180, 180]
        yaw: 偏航角（度），范围 [-180, 180]
    """
    global _bridge

    if _bridge is None:
        return "错误: 未连接设备"

    if _bridge._current_mode != _bridge.MODE_EULER_ANGLE:
        return "错误: 请先切换到 euler 模式"

    roll_val = int(roll * 100)
    pitch_val = int(pitch * 100)
    yaw_val = int(yaw * 100)

    _bridge.set_control_values(roll=roll_val, pitch=pitch_val, yaw=yaw_val, valid=True)
    _bridge.send_packet()

    return f"✓ 欧拉角设置: 滚转={roll}°, 俯仰={pitch}°, 偏航={yaw}°"


@mcp.tool()
async def reset_gimbal() -> str:
    """云台回中"""
    global _bridge

    if _bridge is None:
        return "错误: 未连接设备"

    _bridge.send_packet(_bridge.CMD_RESET)
    return "✓ 云台已回中"


@mcp.tool()
async def calibrate_gimbal() -> str:
    """云台校准（校准时请保持静止）"""
    global _bridge

    if _bridge is None:
        return "错误: 未连接设备"

    _bridge.send_packet(_bridge.CMD_CALIBRATION)
    return "✓ 校准命令已发送，请保持云台静止数秒"


@mcp.tool()
async def take_photo(camera: int = 1) -> str:
    """
    拍照

    Args:
        camera: 相机序号，1=可见光相机，2=热成像相机
    """
    global _bridge

    if _bridge is None:
        return "错误: 未连接设备"

    camera_code = 0x01 if camera == 1 else 0x02
    _bridge.send_packet(_bridge.CMD_PHOTO, bytes([0x01, camera_code]))

    return f"✓ 已触发拍照 (相机 {camera})"


@mcp.tool()
async def toggle_record(camera: int = 1) -> str:
    """
    开始/停止录像

    Args:
        camera: 相机序号，1=可见光相机，2=热成像相机
    """
    global _bridge

    if _bridge is None:
        return "错误: 未连接设备"

    camera_code = 0x01 if camera == 1 else 0x02
    _bridge.send_packet(_bridge.CMD_RECORD, bytes([0x01, camera_code]))

    return f"✓ 已切换录像状态 (相机 {camera})"


@mcp.tool()
async def zoom(direction: str, camera: int = 1) -> str:
    """
    变焦控制

    Args:
        direction: 变焦方向 (in/out/stop)
        camera: 相机序号
    """
    global _bridge

    if _bridge is None:
        return "错误: 未连接设备"

    camera_code = 0x01 if camera == 1 else 0x02

    if direction == "in":
        _bridge.send_packet(_bridge.CMD_ZOOM_IN, bytes([camera_code]))
    elif direction == "out":
        _bridge.send_packet(_bridge.CMD_ZOOM_OUT, bytes([camera_code]))
    elif direction == "stop":
        _bridge.send_packet(_bridge.CMD_ZOOM_STOP, bytes([camera_code]))
    else:
        return "错误: direction 必须是 in/out/stop"

    return f"✓ 变焦 {direction}"


@mcp.tool()
async def set_zoom_level(level: float, camera: int = 1) -> str:
    """
    设置变焦倍率

    Args:
        level: 倍率值，负值表示期望倍率(如 -50 = 5.0x)
        camera: 相机序号
    """
    global _bridge

    if _bridge is None:
        return "错误: 未连接设备"

    camera_code = 0x01 if camera == 1 else 0x02
    zoom_val = int(level * -10)

    _bridge.send_packet(_bridge.CMD_ZOOM_SET, bytes([camera_code]) + struct.pack('<h', zoom_val))

    return f"✓ 倍率设置为 {level}x"


@mcp.tool()
async def focus(camera: int = 1) -> str:
    """
    聚焦

    Args:
        camera: 相机序号
    """
    global _bridge

    if _bridge is None:
        return "错误: 未连接设备"

    camera_code = 0x01 if camera == 1 else 0x02
    _bridge.send_packet(_bridge.CMD_FOCUS, bytes([0x01, camera_code]))

    return "✓ 聚焦命令已发送"


@mcp.tool()
async def set_night_vision(enabled: bool) -> str:
    """
    设置夜视模式

    Args:
        enabled: 是否开启
    """
    global _bridge

    if _bridge is None:
        return "错误: 未连接设备"

    mode = 0x01 if enabled else 0x00
    _bridge.send_packet(_bridge.CMD_NIGHT_VISION, bytes([0x01, mode]))

    return f"✓ 夜视模式: {'开启' if enabled else '关闭'}"


@mcp.tool()
async def set_osd(enabled: bool) -> str:
    """
    设置 OSD 显示

    Args:
        enabled: 是否显示 OSD
    """
    global _bridge

    if _bridge is None:
        return "错误: 未连接设备"

    _bridge.send_packet(_bridge.CMD_OSD, bytes([0x01 if enabled else 0x00]))

    return f"✓ OSD: {'开启' if enabled else '关闭'}"


@mcp.tool()
async def set_illumination(brightness: int) -> str:
    """
    设置补光亮度

    Args:
        brightness: 亮度值，范围 [0, 255]
    """
    global _bridge

    if _bridge is None:
        return "错误: 未连接设备"

    if not (0 <= brightness <= 255):
        return "错误: brightness 必须在 [0, 255] 范围内"

    _bridge.send_packet(_bridge.CMD_ILLUMINATION, bytes([brightness]))

    return f"✓ 补光亮度设置为 {brightness}"


@mcp.tool()
async def set_ranging(enabled: bool) -> str:
    """
    设置连续测距

    Args:
        enabled: 是否开启测距
    """
    global _bridge

    if _bridge is None:
        return "错误: 未连接设备"

    mode = 0x02 if enabled else 0x00
    _bridge.send_packet(_bridge.CMD_RANGING, bytes([mode]))

    return f"✓ 连续测距: {'开启' if enabled else '关闭'}"


@mcp.tool()
async def stop_rotation() -> str:
    """停止旋转"""
    global _bridge

    if _bridge is None:
        return "错误: 未连接设备"

    _bridge.stop_sending()
    _bridge.set_control_values(roll=0, pitch=0, yaw=0, valid=False)
    _bridge.send_packet()

    return "✓ 已停止旋转"


@mcp.resource("gimbal://status")
async def get_gimbal_status() -> str:
    """
    获取云台当前状态

    返回:
        云台角度、速度、工作模式等信息
    """
    global _bridge

    if _bridge is None:
        return "未连接设备"

    status = _bridge.get_latest_status()
    if status is None:
        return "暂无状态数据"

    mode_names = {
        0x10: '角度控制',
        0x11: '指向锁定',
        0x12: '指向跟随',
        0x13: '俯拍',
        0x14: '欧拉角控制',
        0x15: '凝视(地理)',
        0x16: '凝视(目标)',
        0x17: '跟踪',
        0x1A: '指点平移',
        0x1C: 'FPV',
    }

    mode = mode_names.get(status['work_mode'], f"未知(0x{status['work_mode']:02X})")

    return f"""
云台状态:
  工作模式: {mode}
  相机绝对角度: 滚转={status['camera_abs_roll']:.2f}°, 俯仰={status['camera_abs_pitch']:.2f}°, 偏航={status['camera_abs_yaw']:.2f}°
  相机相对角度: X={status['camera_rel_x']:.2f}°, Y={status['camera_rel_y']:.2f}°, Z={status['camera_rel_z']:.2f}°
  相机角速度: X={status['camera_vel_x']:.1f}°/s, Y={status['camera_vel_y']:.1f}°/s, Z={status['camera_vel_z']:.1f}°/s
"""


@mcp.resource("gimbal://connection")
async def get_connection_status() -> str:
    """获取连接状态"""
    global _bridge

    if _bridge and _bridge.serial and _bridge.serial.is_open:
        return f"已连接: {_bridge.port} @ {_bridge.baudrate}bps"
    else:
        return "未连接"


@mcp.tool()
async def demo_scan() -> str:
    """
    执行扫描演示动作
    云台上下左右扫描，然后回中
    """
    global _bridge

    if _bridge is None:
        return "错误: 未连接设备"

    # 切换到指向锁定模式
    await set_mode("angle_lock")
    await asyncio.sleep(0.5)

    # 向上扫描
    await rotate(pitch_speed=30, yaw_speed=0, duration=2)
    await asyncio.sleep(0.5)

    # 向下扫描
    await rotate(pitch_speed=-30, yaw_speed=0, duration=2)
    await asyncio.sleep(0.5)

    # 向左扫描
    await rotate(pitch_speed=0, yaw_speed=-30, duration=2)
    await asyncio.sleep(0.5)

    # 向右扫描
    await rotate(pitch_speed=0, yaw_speed=30, duration=2)
    await asyncio.sleep(0.5)

    # 回中
    await reset_gimbal()

    return "✓ 扫描演示完成"


if __name__ == "__main__":
    mcp.run(transport="stdio")
