"""
ROSClaw Gimbal MCP Server

GCU (Gimbal Control Unit) MCP Server using Serial protocol.
Part of the ROSClaw Embodied Intelligence Operating System.

Features:
- Serial communication (COM8 @ 115200bps)
- Gimbal actions: rotate, set_mode, zoom, photo, record
- Safety guards for rotation limits
- State monitoring (angles, velocities)
- Complete protocol implementation (CRC16)

Hardware: GCU Gimbal (可见光+热成像双光吊舱)
Protocol: Proprietary Serial Protocol V2.0.6
"""

import serial
import struct
import time
import threading
import asyncio
from typing import Optional, Tuple, Dict
from dataclasses import dataclass
from mcp.server.fastmcp import FastMCP

# Initialize MCP Server
mcp = FastMCP("rosclaw-gimbal")


@dataclass
class GimbalState:
    """Gimbal state data"""
    timestamp: str
    work_mode: int

    # Camera angles (degrees)
    camera_abs_roll: float
    camera_abs_pitch: float
    camera_abs_yaw: float
    camera_rel_x: float
    camera_rel_y: float
    camera_rel_z: float

    # Camera velocities (deg/s)
    camera_vel_x: float
    camera_vel_y: float
    camera_vel_z: float

    # Extended info
    zoom1: float = 0.0  # Visible camera zoom
    zoom2: float = 0.0  # Thermal camera zoom
    target_distance: float = 0.0  # Laser rangefinder


class GimbalSerialBridge:
    """
    GCU Gimbal Serial Communication Bridge

    Protocol: GCU Private Communication Protocol V2.0.6
    Serial Config: COM8 @ 115200bps, 8N1
    Frame Structure:
    - Header: 0xA8 0xE5 (send) / 0x8A 0x5E (receive)
    - Length: 2 bytes (little-endian)
    - Version: 1 byte
    - Data: Variable
    - CRC16: 2 bytes (big-endian)
    """

    # Protocol constants
    PROTOCOL_HEADER_SEND = bytes([0xA8, 0xE5])
    PROTOCOL_HEADER_RECV = bytes([0x8A, 0x5E])
    PROTOCOL_VERSION = 0x01

    # Work modes
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

    # Camera commands
    CMD_CALIBRATION = 0x01
    CMD_RESET = 0x03
    CMD_PHOTO = 0x20
    CMD_RECORD = 0x21
    CMD_ZOOM_IN = 0x22
    CMD_ZOOM_OUT = 0x23
    CMD_ZOOM_STOP = 0x24
    CMD_ZOOM_SET = 0x25
    CMD_FOCUS = 0x26
    CMD_PALETTE = 0x2A
    CMD_NIGHT_VISION = 0x2B
    CMD_OSD = 0x73
    CMD_PIP = 0x74
    CMD_TARGET_DETECT = 0x75
    CMD_DIGITAL_ZOOM = 0x76
    CMD_ILLUMINATION = 0x80
    CMD_RANGING = 0x81

    # Status flags
    FLAG_CONTROL_VALID = 0x04
    FLAG_IMU_VALID = 0x01

    # Safety limits
    SAFETY_LIMITS = {
        "pitch_speed": 1500,  # ±150°/s (in 0.1°/s units)
        "yaw_speed": 1500,
        "pitch_angle": 18000,  # ±180° (in 0.01° units)
        "yaw_angle": 18000,
        "roll_angle": 18000,
    }

    def __init__(self, port: str = "COM8", baudrate: int = 115200):
        self.port = port
        self.baudrate = baudrate
        self.serial: Optional[serial.Serial] = None

        self._running = False
        self._send_thread: Optional[threading.Thread] = None
        self._recv_thread: Optional[threading.Thread] = None
        self._lock = threading.Lock()

        # Control state
        self._roll_control = 0
        self._pitch_control = 0
        self._yaw_control = 0
        self._control_valid = True
        self._imu_valid = True
        self._current_mode = self.MODE_ANGLE_CONTROL

        # Aircraft data (for enhanced control)
        self._aircraft_roll = 0
        self._aircraft_pitch = 0
        self._aircraft_yaw = 0
        self._accel_north = 0
        self._accel_east = 0
        self._accel_up = 0
        self._vel_north = 0
        self._vel_east = 0
        self._vel_up = 0

        # Receive buffer
        self._recv_buffer = bytearray()
        self._latest_status: Optional[GimbalState] = None

    def connect(self) -> bool:
        """Connect to gimbal via serial port"""
        try:
            self.serial = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=0.01
            )

            # Start receive thread
            self._recv_thread = threading.Thread(target=self._recv_loop, daemon=True)
            self._recv_thread.start()

            return True

        except serial.SerialException as e:
            print(f"Serial connection failed: {e}")
            return False

    def disconnect(self):
        """Disconnect from gimbal"""
        self._running = False

        if self._send_thread and self._send_thread.is_alive():
            self._send_thread.join(timeout=1.0)

        if self._recv_thread and self._recv_thread.is_alive():
            self._recv_thread.join(timeout=1.0)

        if self.serial and self.serial.is_open:
            self.serial.close()

    def _calculate_crc16(self, data: bytes) -> int:
        """Calculate CRC16 checksum"""
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

    def _build_control_packet(self, command: int = 0x00, params: bytes = b"") -> bytes:
        """Build control packet according to protocol spec"""
        base_length = 72
        packet_length = base_length + len(params)

        packet = bytearray()

        # Header
        packet.extend(self.PROTOCOL_HEADER_SEND)
        packet.extend(struct.pack("<H", packet_length))
        packet.append(self.PROTOCOL_VERSION)

        # Data frame (32 bytes)
        packet.extend(struct.pack("<h", self._roll_control))
        packet.extend(struct.pack("<h", self._pitch_control))
        packet.extend(struct.pack("<h", self._yaw_control))

        status_flag = 0
        if self._control_valid:
            status_flag |= self.FLAG_CONTROL_VALID
        if self._imu_valid:
            status_flag |= self.FLAG_IMU_VALID
        packet.append(status_flag)

        # Aircraft attitude (for enhanced control)
        packet.extend(struct.pack("<h", self._aircraft_roll))
        packet.extend(struct.pack("<h", self._aircraft_pitch))
        packet.extend(struct.pack("<H", self._aircraft_yaw % 36000))
        packet.extend(struct.pack("<h", self._accel_north))
        packet.extend(struct.pack("<h", self._accel_east))
        packet.extend(struct.pack("<h", self._accel_up))
        packet.extend(struct.pack("<h", self._vel_north))
        packet.extend(struct.pack("<h", self._vel_east))
        packet.extend(struct.pack("<h", self._vel_up))
        packet.append(0x01)  # Request sub-frame
        packet.extend(bytes(6))

        # Sub-frame (32 bytes)
        packet.append(0x01)
        packet.extend(bytes(31))

        # Command and parameters
        packet.append(command)
        packet.extend(params)

        # CRC (big-endian)
        crc = self._calculate_crc16(packet)
        packet.extend(struct.pack(">H", crc))

        return bytes(packet)

    def send_packet(self, command: int = 0x00, params: bytes = b"") -> bool:
        """Send control packet to gimbal"""
        if not self.serial or not self.serial.is_open:
            return False

        try:
            packet = self._build_control_packet(command, params)
            self.serial.write(packet)
            return True
        except serial.SerialException:
            return False

    def start_sending(self, frequency: float = 30.0):
        """Start continuous sending thread"""
        if self._running:
            return

        self._running = True
        self._send_thread = threading.Thread(
            target=self._send_loop,
            args=(frequency,),
            daemon=True
        )
        self._send_thread.start()

    def stop_sending(self):
        """Stop continuous sending"""
        self._running = False
        if self._send_thread:
            self._send_thread.join(timeout=1.0)

    def _send_loop(self, frequency: float):
        """Send loop for continuous control"""
        interval = 1.0 / frequency
        while self._running:
            with self._lock:
                self.send_packet()
            time.sleep(interval)

    def _recv_loop(self):
        """Receive loop for state updates"""
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
        """Parse receive buffer for complete packets"""
        while len(self._recv_buffer) >= 72:
            header_idx = self._recv_buffer.find(self.PROTOCOL_HEADER_RECV)
            if header_idx == -1:
                self._recv_buffer.clear()
                return

            if header_idx > 0:
                self._recv_buffer = self._recv_buffer[header_idx:]

            if len(self._recv_buffer) < 4:
                return

            packet_length = struct.unpack("<H", self._recv_buffer[2:4])[0]

            if len(self._recv_buffer) < packet_length:
                return

            packet = bytes(self._recv_buffer[:packet_length])
            self._recv_buffer = self._recv_buffer[packet_length:]

            self._parse_packet(packet)

    def _parse_packet(self, packet: bytes):
        """Parse gimbal status packet"""
        try:
            if len(packet) < 72:
                return

            crc_received = struct.unpack("<H", packet[-2:])[0]
            crc_calculated = self._calculate_crc16(packet[:-2])
            if crc_received != crc_calculated:
                return

            self._latest_status = GimbalState(
                timestamp=time.strftime("%H:%M:%S"),
                work_mode=packet[5],
                camera_abs_roll=struct.unpack("<h", packet[18:20])[0] / 100.0,
                camera_abs_pitch=struct.unpack("<h", packet[20:22])[0] / 100.0,
                camera_abs_yaw=struct.unpack("<H", packet[22:24])[0] / 100.0,
                camera_rel_x=struct.unpack("<h", packet[12:14])[0] / 100.0,
                camera_rel_y=struct.unpack("<h", packet[14:16])[0] / 100.0,
                camera_rel_z=struct.unpack("<h", packet[16:18])[0] / 100.0,
                camera_vel_x=struct.unpack("<h", packet[24:26])[0] / 10.0,
                camera_vel_y=struct.unpack("<h", packet[26:28])[0] / 10.0,
                camera_vel_z=struct.unpack("<h", packet[28:30])[0] / 10.0,
            )

            # Parse extended info if available
            if len(packet) >= 73 and packet[37] == 0x01:
                self._latest_status.zoom1 = struct.unpack("<H", packet[59:61])[0] / 10.0
                self._latest_status.zoom2 = struct.unpack("<H", packet[61:63])[0] / 10.0
                self._latest_status.target_distance = struct.unpack("<i", packet[43:47])[0] / 10.0

        except Exception:
            pass

    def set_control_values(self, roll: int = 0, pitch: int = 0, yaw: int = 0, valid: bool = True):
        """Set control values"""
        with self._lock:
            self._roll_control = roll
            self._pitch_control = pitch
            self._yaw_control = yaw
            self._control_valid = valid

    def get_latest_status(self) -> Optional[GimbalState]:
        """Get latest gimbal status"""
        return self._latest_status

    def _validate_safety(self, **kwargs) -> Tuple[bool, str]:
        """Validate parameters against safety limits"""
        for limit_name, limit_value in self.SAFETY_LIMITS.items():
            if limit_name in kwargs:
                val = abs(kwargs[limit_name])
                if val > limit_value:
                    return False, f"{limit_name} exceeds safety limit: {limit_value}"
        return True, "OK"


# Global bridge instance
_bridge: Optional[GimbalSerialBridge] = None


# ============ MCP Tools ============

@mcp.tool()
async def connect_gimbal(port: str = "COM8", baudrate: int = 115200) -> str:
    """
    Connect to GCU gimbal via serial port

    Args:
        port: Serial port (default: COM8)
        baudrate: Baud rate (default: 115200)
    """
    global _bridge

    if _bridge is None:
        _bridge = GimbalSerialBridge(port=port, baudrate=baudrate)

    if _bridge.connect():
        return f"✓ Connected to gimbal on {port} @ {baudrate}bps"
    else:
        return f"✗ Failed to connect to {port}"


@mcp.tool()
async def disconnect_gimbal() -> str:
    """Disconnect from gimbal"""
    global _bridge

    if _bridge:
        _bridge.disconnect()
        _bridge = None
        return "✓ Disconnected from gimbal"

    return "Not connected"


@mcp.tool()
async def set_mode(mode: str) -> str:
    """
    Set gimbal work mode

    Args:
        mode: Mode name
            - angle_lock: Pointing lock mode (角速度控制)
            - follow: Pointing follow mode
            - euler: Euler angle control mode
            - fpv: FPV mode
            - top_down: Top-down shooting mode
    """
    global _bridge

    if _bridge is None:
        return "Error: Not connected"

    mode_map = {
        "angle_lock": _bridge.MODE_POINTING_LOCK,
        "follow": _bridge.MODE_POINTING_FOLLOW,
        "euler": _bridge.MODE_EULER_ANGLE,
        "fpv": _bridge.MODE_FPV,
        "top_down": _bridge.MODE_TOP_DOWN,
    }

    if mode not in mode_map:
        return f"Error: Unknown mode '{mode}'. Valid: {list(mode_map.keys())}"

    mode_code = mode_map[mode]
    _bridge._current_mode = mode_code
    _bridge.send_packet(mode_code)

    return f"✓ Set mode to {mode}"


@mcp.tool()
async def rotate(pitch_speed: float = 0.0, yaw_speed: float = 0.0, duration: float = 1.0) -> str:
    """
    Rotate gimbal at specified angular velocity

    Args:
        pitch_speed: Pitch speed (°/s), range [-150, 150]
        yaw_speed: Yaw speed (°/s), range [-150, 150]
        duration: Rotation duration (seconds)
    """
    global _bridge

    if _bridge is None:
        return "Error: Not connected"

    # Convert to protocol units (0.1°/s)
    pitch_val = int(pitch_speed * 10)
    yaw_val = int(yaw_speed * 10)

    # Safety check
    valid, msg = _bridge._validate_safety(pitch_speed=abs(pitch_val), yaw_speed=abs(yaw_val))
    if not valid:
        return f"Safety check failed: {msg}"

    _bridge.set_control_values(roll=0, pitch=pitch_val, yaw=yaw_val, valid=True)
    _bridge.start_sending(frequency=30)

    await asyncio.sleep(duration)

    _bridge.stop_sending()
    _bridge.set_control_values(roll=0, pitch=0, yaw=0, valid=False)

    return f"✓ Rotated: pitch={pitch_speed}°/s, yaw={yaw_speed}°/s, duration={duration}s"


@mcp.tool()
async def set_euler_angles(roll: float = 0.0, pitch: float = 0.0, yaw: float = 0.0) -> str:
    """
    Set gimbal to specified Euler angles (in euler mode)

    Args:
        roll: Roll angle (degrees)
        pitch: Pitch angle (degrees)
        yaw: Yaw angle (degrees)
    """
    global _bridge

    if _bridge is None:
        return "Error: Not connected"

    if _bridge._current_mode != _bridge.MODE_EULER_ANGLE:
        return "Error: Please switch to euler mode first"

    roll_val = int(roll * 100)
    pitch_val = int(pitch * 100)
    yaw_val = int(yaw * 100)

    _bridge.set_control_values(roll=roll_val, pitch=pitch_val, yaw=yaw_val, valid=True)
    _bridge.send_packet()

    return f"✓ Set angles: roll={roll}°, pitch={pitch}°, yaw={yaw}°"


@mcp.tool()
async def reset_gimbal() -> str:
    """Reset gimbal to center position"""
    global _bridge

    if _bridge is None:
        return "Error: Not connected"

    _bridge.send_packet(_bridge.CMD_RESET)
    return "✓ Reset command sent"


@mcp.tool()
async def calibrate() -> str:
    """Calibrate gimbal (keep stationary during calibration)"""
    global _bridge

    if _bridge is None:
        return "Error: Not connected"

    _bridge.send_packet(_bridge.CMD_CALIBRATION)
    return "✓ Calibration command sent. Keep gimbal stationary."


@mcp.tool()
async def take_photo(camera: int = 1) -> str:
    """
    Take photo

    Args:
        camera: Camera number (1=visible, 2=thermal)
    """
    global _bridge

    if _bridge is None:
        return "Error: Not connected"

    camera_code = 0x01 if camera == 1 else 0x02
    _bridge.send_packet(_bridge.CMD_PHOTO, bytes([0x01, camera_code]))

    return f"✓ Photo triggered (camera {camera})"


@mcp.tool()
async def toggle_record(camera: int = 1) -> str:
    """
    Toggle video recording

    Args:
        camera: Camera number (1=visible, 2=thermal)
    """
    global _bridge

    if _bridge is None:
        return "Error: Not connected"

    camera_code = 0x01 if camera == 1 else 0x02
    _bridge.send_packet(_bridge.CMD_RECORD, bytes([0x01, camera_code]))

    return f"✓ Recording toggled (camera {camera})"


@mcp.tool()
async def zoom(direction: str, camera: int = 1) -> str:
    """
    Control zoom

    Args:
        direction: "in" (zoom in), "out" (zoom out), "stop"
        camera: Camera number
    """
    global _bridge

    if _bridge is None:
        return "Error: Not connected"

    camera_code = 0x01 if camera == 1 else 0x02

    if direction == "in":
        _bridge.send_packet(_bridge.CMD_ZOOM_IN, bytes([camera_code]))
    elif direction == "out":
        _bridge.send_packet(_bridge.CMD_ZOOM_OUT, bytes([camera_code]))
    elif direction == "stop":
        _bridge.send_packet(_bridge.CMD_ZOOM_STOP, bytes([camera_code]))
    else:
        return "Error: direction must be 'in', 'out', or 'stop'"

    return f"✓ Zoom {direction}"


@mcp.tool()
async def set_zoom_level(level: float, camera: int = 1) -> str:
    """
    Set zoom level

    Args:
        level: Zoom level (e.g., 5.0 for 5x)
        camera: Camera number
    """
    global _bridge

    if _bridge is None:
        return "Error: Not connected"

    camera_code = 0x01 if camera == 1 else 0x02
    zoom_val = int(level * -10)  # Negative value for absolute zoom

    _bridge.send_packet(
        _bridge.CMD_ZOOM_SET,
        bytes([camera_code]) + struct.pack("<h", zoom_val)
    )

    return f"✓ Zoom level set to {level}x"


@mcp.tool()
async def focus(camera: int = 1) -> str:
    """
    Trigger autofocus

    Args:
        camera: Camera number
    """
    global _bridge

    if _bridge is None:
        return "Error: Not connected"

    camera_code = 0x01 if camera == 1 else 0x02
    _bridge.send_packet(_bridge.CMD_FOCUS, bytes([0x01, camera_code]))

    return "✓ Focus triggered"


@mcp.tool()
async def set_night_vision(enabled: bool) -> str:
    """
    Enable/disable night vision

    Args:
        enabled: True to enable, False to disable
    """
    global _bridge

    if _bridge is None:
        return "Error: Not connected"

    mode = 0x01 if enabled else 0x00
    _bridge.send_packet(_bridge.CMD_NIGHT_VISION, bytes([0x01, mode]))

    return f"✓ Night vision {'enabled' if enabled else 'disabled'}"


@mcp.tool()
async def set_osd(enabled: bool) -> str:
    """
    Enable/disable OSD (on-screen display)

    Args:
        enabled: True to enable, False to disable
    """
    global _bridge

    if _bridge is None:
        return "Error: Not connected"

    _bridge.send_packet(_bridge.CMD_OSD, bytes([0x01 if enabled else 0x00]))

    return f"✓ OSD {'enabled' if enabled else 'disabled'}"


@mcp.tool()
async def set_illumination(brightness: int) -> str:
    """
    Set illumination brightness

    Args:
        brightness: Brightness level (0-255)
    """
    global _bridge

    if _bridge is None:
        return "Error: Not connected"

    if not (0 <= brightness <= 255):
        return "Error: Brightness must be 0-255"

    _bridge.send_packet(_bridge.CMD_ILLUMINATION, bytes([brightness]))

    return f"✓ Illumination set to {brightness}"


@mcp.tool()
async def set_ranging(enabled: bool) -> str:
    """
    Enable/disable continuous laser ranging

    Args:
        enabled: True to enable, False to disable
    """
    global _bridge

    if _bridge is None:
        return "Error: Not connected"

    mode = 0x02 if enabled else 0x00
    _bridge.send_packet(_bridge.CMD_RANGING, bytes([mode]))

    return f"✓ Laser ranging {'enabled' if enabled else 'disabled'}"


@mcp.tool()
async def stop_rotation() -> str:
    """Stop gimbal rotation"""
    global _bridge

    if _bridge is None:
        return "Error: Not connected"

    _bridge.stop_sending()
    _bridge.set_control_values(roll=0, pitch=0, yaw=0, valid=False)
    _bridge.send_packet()

    return "✓ Rotation stopped"


@mcp.tool()
async def demo_scan() -> str:
    """
    Perform a scan demonstration

    Scans up, down, left, right, then returns to center.
    """
    global _bridge

    if _bridge is None:
        return "Error: Not connected"

    # Set mode
    await set_mode("angle_lock")
    await asyncio.sleep(0.5)

    # Scan up
    await rotate(pitch_speed=30, yaw_speed=0, duration=2)
    await asyncio.sleep(0.5)

    # Scan down
    await rotate(pitch_speed=-30, yaw_speed=0, duration=2)
    await asyncio.sleep(0.5)

    # Scan left
    await rotate(pitch_speed=0, yaw_speed=-30, duration=2)
    await asyncio.sleep(0.5)

    # Scan right
    await rotate(pitch_speed=0, yaw_speed=30, duration=2)
    await asyncio.sleep(0.5)

    # Reset
    await reset_gimbal()

    return "✓ Scan demonstration complete"


# ============ MCP Resources ============

@mcp.resource("gimbal://status")
async def get_gimbal_status() -> str:
    """
    Get gimbal current status

    Returns work mode, camera angles, and velocities
    """
    global _bridge

    if _bridge is None:
        return "Not connected"

    status = _bridge.get_latest_status()
    if status is None:
        return "No status data available"

    mode_names = {
        0x10: 'Angle Control',
        0x11: 'Pointing Lock',
        0x12: 'Pointing Follow',
        0x13: 'Top-Down',
        0x14: 'Euler Angle',
        0x15: 'Geo Stare',
        0x16: 'Target Lock',
        0x17: 'Tracking',
        0x1A: 'Pointing Move',
        0x1C: 'FPV',
    }

    mode = mode_names.get(status.work_mode, f"Unknown(0x{status.work_mode:02X})")

    return f"""
Gimbal Status:
  Work Mode: {mode}

  Camera Absolute Angles:
    Roll:  {status.camera_abs_roll:7.2f}°
    Pitch: {status.camera_abs_pitch:7.2f}°
    Yaw:   {status.camera_abs_yaw:7.2f}°

  Camera Relative Angles:
    X: {status.camera_rel_x:7.2f}°
    Y: {status.camera_rel_y:7.2f}°
    Z: {status.camera_rel_z:7.2f}°

  Camera Velocities:
    X: {status.camera_vel_x:6.1f}°/s
    Y: {status.camera_vel_y:6.1f}°/s
    Z: {status.camera_vel_z:6.1f}°/s

  Zoom: Visible={status.zoom1:.1f}x, Thermal={status.zoom2:.1f}x
  Target Distance: {status.target_distance:.1f}m
"""


@mcp.resource("gimbal://connection")
async def get_connection_status() -> str:
    """Get serial connection status"""
    global _bridge

    if _bridge and _bridge.serial and _bridge.serial.is_open:
        return f"Connected: {_bridge.port} @ {_bridge.baudrate}bps"
    else:
        return "Disconnected"


if __name__ == "__main__":
    mcp.run(transport="stdio")
