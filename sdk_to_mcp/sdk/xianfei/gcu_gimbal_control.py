#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
GCU 云台控制脚本
基于 GCU 私有通信协议 V2.0.6 (协议版本 V0.2)
支持串口通信控制云台旋转
"""

import serial
import struct
import time
import threading
from typing import Optional, Tuple, List, Dict
from datetime import datetime


class GCUGimbalController:
    """GCU 云台控制器"""
    
    # 协议常量
    PROTOCOL_HEADER_SEND = bytes([0xA8, 0xE5])
    PROTOCOL_HEADER_RECV = bytes([0x8A, 0x5E])
    PROTOCOL_VERSION = 0x01  # V0.1 (根据实际测试)
    
    # 工作模式
    MODE_ANGLE_CONTROL = 0x10       # 角度控制模式
    MODE_POINTING_LOCK = 0x11       # 指向锁定模式
    MODE_POINTING_FOLLOW = 0x12     # 指向跟随模式
    MODE_TOP_DOWN = 0x13            # 俯拍模式
    MODE_EULER_ANGLE = 0x14         # 欧拉角控制模式
    MODE_GEO_STARE = 0x15           # 凝视模式（地理坐标引导）
    MODE_TARGET_LOCK = 0x16         # 凝视模式（地理目标锁定）
    MODE_TRACKING = 0x17            # 跟踪模式
    MODE_POINTING_MOVE = 0x1A       # 指点平移模式
    MODE_FPV = 0x1C                 # FPV模式
    
    # 相机命令
    CMD_CALIBRATION = 0x01          # 吊舱校准
    CMD_RESET = 0x03                # 回中
    CMD_PHOTO = 0x20                # 拍照
    CMD_RECORD = 0x21               # 录像开始/停止
    CMD_ZOOM_IN = 0x22              # 连续放大
    CMD_ZOOM_OUT = 0x23             # 连续缩小
    CMD_ZOOM_STOP = 0x24            # 停止变倍
    CMD_ZOOM_SET = 0x25             # 指定倍率
    CMD_FOCUS = 0x26                # 聚焦
    CMD_PALETTE = 0x2A              # 调色盘
    CMD_NIGHT_VISION = 0x2B         # 夜视
    CMD_OSD = 0x73                  # OSD开关
    CMD_PIP = 0x74                  # 画中画
    CMD_TARGET_DETECT = 0x75        # 目标识别
    CMD_DIGITAL_ZOOM = 0x76         # 数字变焦
    CMD_ILLUMINATION = 0x80         # 补光
    CMD_RANGING = 0x81              # 连续测距
    
    # 状态标志位
    FLAG_CONTROL_VALID = 0x04       # B2: 控制量有效
    FLAG_IMU_VALID = 0x01           # B0: 载机惯导数据有效
    
    def __init__(self, port: str = 'COM4', baudrate: int = 115200):
        """
        初始化云台控制器
        
        Args:
            port: 串口号，如 'COM4'
            baudrate: 波特率，支持 115200, 250000, 500000, 1000000
        """
        self.port = port
        self.baudrate = baudrate
        self.serial: Optional[serial.Serial] = None
        self._running = False
        self._send_thread: Optional[threading.Thread] = None
        self._recv_thread: Optional[threading.Thread] = None
        self._lock = threading.Lock()
        
        # 控制状态
        self._roll_control = 0          # 滚转控制量
        self._pitch_control = 0         # 俯仰控制量
        self._yaw_control = 0           # 偏航控制量
        self._control_valid = True      # 控制量是否有效
        self._imu_valid = True          # 惯导数据是否有效（默认有效）
        self._current_mode = self.MODE_ANGLE_CONTROL
        
        # 载机数据（惯导数据 - 非常重要！）
        self._aircraft_roll = 0         # 载机滚转角 (0.01°)
        self._aircraft_pitch = 0        # 载机俯仰角 (0.01°)
        self._aircraft_yaw = 0          # 载机偏航角 (0.01°)
        self._accel_north = 0           # 北向加速度 (0.01m/s²)
        self._accel_east = 0            # 东向加速度 (0.01m/s²)
        self._accel_up = 0              # 天向加速度 (0.01m/s²)
        self._vel_north = 0             # 北向速度 (0.01m/s)
        self._vel_east = 0              # 东向速度 (0.01m/s)
        self._vel_up = 0                # 天向速度 (0.01m/s)
        
        # 接收数据缓存
        self._recv_buffer = bytearray()
        self._latest_status: Optional[Dict] = None
        self._status_callback = None
        
    def connect(self) -> bool:
        """
        连接串口
        
        Returns:
            是否连接成功
        """
        try:
            self.serial = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=0.01  # 非阻塞读取
            )
            print(f"✓ 串口连接成功: {self.port} @ {self.baudrate}bps")
            return True
        except serial.SerialException as e:
            print(f"✗ 串口连接失败: {e}")
            return False
    
    def disconnect(self):
        """断开串口连接"""
        self.stop_sending()
        self.stop_receiving()
        if self.serial and self.serial.is_open:
            self.serial.close()
            print("✓ 串口已断开")
    
    def _calculate_crc16(self, data: bytes) -> int:
        """
        计算 CRC16 校验值
        
        Args:
            data: 待校验数据
            
        Returns:
            CRC16 校验值
        """
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
        """
        构建控制数据包
        
        Args:
            command: 指令字节
            params: 参数字节
            
        Returns:
            完整的数据包
        """
        # 计算包长度
        base_length = 72  # 基础长度（不含参数）
        packet_length = base_length + len(params)
        
        # 构建数据包
        packet = bytearray()
        
        # 协议头
        packet.extend(self.PROTOCOL_HEADER_SEND)
        
        # 包长度（小端序）
        packet.extend(struct.pack('<H', packet_length))
        
        # 版本号
        packet.append(self.PROTOCOL_VERSION)
        
        # ========== 数据主帧 (32字节) ==========
        # 字节 5-6: 滚转控制量 (S16)
        packet.extend(struct.pack('<h', self._roll_control))
        # 字节 7-8: 俯仰控制量 (S16)
        packet.extend(struct.pack('<h', self._pitch_control))
        # 字节 9-10: 偏航控制量 (S16)
        packet.extend(struct.pack('<h', self._yaw_control))
        # 字节 11: 状态标志
        status_flag = 0
        if self._control_valid:
            status_flag |= self.FLAG_CONTROL_VALID
        if self._imu_valid:
            status_flag |= self.FLAG_IMU_VALID
        packet.append(status_flag)
        
        # 字节 12-13: 载机绝对滚转角 (S16, 0.01deg)
        packet.extend(struct.pack('<h', self._aircraft_roll))
        # 字节 14-15: 载机绝对俯仰角 (S16, 0.01deg)
        packet.extend(struct.pack('<h', self._aircraft_pitch))
        # 字节 16-17: 载机绝对偏航角 (U16, 0.01deg)
        packet.extend(struct.pack('<H', self._aircraft_yaw % 36000))
        
        # 字节 18-19: 载机北向加速度 (S16, 0.01m/s²)
        packet.extend(struct.pack('<h', self._accel_north))
        # 字节 20-21: 载机东向加速度 (S16, 0.01m/s²)
        packet.extend(struct.pack('<h', self._accel_east))
        # 字节 22-23: 载机天向加速度 (S16, 0.01m/s²)
        packet.extend(struct.pack('<h', self._accel_up))
        
        # 字节 24-25: 载机北向速度 (S16, 0.01m/s)
        packet.extend(struct.pack('<h', self._vel_north))
        # 字节 26-27: 载机东向速度 (S16, 0.01m/s)
        packet.extend(struct.pack('<h', self._vel_east))
        # 字节 28-29: 载机天向速度 (S16, 0.01m/s)
        packet.extend(struct.pack('<h', self._vel_up))
        
        # 字节 30: GCU返回数据副帧请求码
        packet.append(0x01)  # 请求副帧
        # 字节 31-36: 预留
        packet.extend(bytes(6))
        
        # ========== 数据副帧 (32字节) ==========
        # 字节 37: 副帧帧头 (0x01表示有副帧数据)
        packet.append(0x01)
        # 字节 38-68: 载机GNSS数据（填0）
        packet.extend(bytes(31))
        
        # 指令
        packet.append(command)
        
        # 参数
        packet.extend(params)
        
        # CRC校验（除CRC本身外的所有数据）- 注意：CRC是大端序！
        crc = self._calculate_crc16(packet)
        packet.extend(struct.pack('>H', crc))  # 大端序
        
        return bytes(packet)
    
    def send_packet(self, command: int = 0x00, params: bytes = b'', debug: bool = False) -> bool:
        """
        发送控制数据包
        
        Args:
            command: 指令字节
            params: 参数字节
            debug: 是否打印调试信息
            
        Returns:
            是否发送成功
        """
        if not self.serial or not self.serial.is_open:
            print("✗ 串口未连接")
            return False
        
        try:
            packet = self._build_control_packet(command, params)
            if debug:
                self._print_packet_debug(packet)
            self.serial.write(packet)
            return True
        except serial.SerialException as e:
            print(f"✗ 发送失败: {e}")
            return False
    
    def _print_packet_debug(self, packet: bytes):
        """打印数据包调试信息"""
        print(f"\n=== 发送数据包 ({len(packet)} 字节) ===")
        print(f"协议头:     {packet[0]:02X} {packet[1]:02X}")
        print(f"包长度:     {int.from_bytes(packet[2:4], 'little')} 字节")
        print(f"版本号:     {packet[4]:02X}")
        print(f"滚转控制量: {int.from_bytes(packet[5:7], 'little', signed=True)}")
        print(f"俯仰控制量: {int.from_bytes(packet[7:9], 'little', signed=True)}")
        print(f"偏航控制量: {int.from_bytes(packet[9:11], 'little', signed=True)}")
        print(f"状态标志:   {packet[11]:02X} (控制量有效: {bool(packet[11] & 0x04)}, IMU有效: {bool(packet[11] & 0x01)})")
        print(f"指令:       {packet[69]:02X}")
        print(f"CRC:        {packet[-2]:02X} {packet[-1]:02X}")
        print(f"完整数据:   {packet.hex()}")
        print("=" * 40)
    
    def start_sending(self, frequency: float = 30.0):
        """
        启动持续发送线程
        
        Args:
            frequency: 发送频率（Hz），建议 30-50Hz
        """
        if self._running:
            return
        
        self._running = True
        self._send_thread = threading.Thread(target=self._send_loop, args=(frequency,))
        self._send_thread.daemon = True
        self._send_thread.start()
        print(f"✓ 开始持续发送（{frequency}Hz）")
    
    def stop_sending(self):
        """停止持续发送"""
        self._running = False
        if self._send_thread:
            self._send_thread.join(timeout=1.0)
            print("✓ 停止持续发送")
    
    def _send_loop(self, frequency: float):
        """发送循环"""
        interval = 1.0 / frequency
        while self._running:
            with self._lock:
                self.send_packet()
            time.sleep(interval)
    
    def start_receiving(self, callback=None):
        """
        启动接收线程
        
        Args:
            callback: 状态更新回调函数，接收解析后的状态字典
        """
        self._status_callback = callback
        self._recv_thread = threading.Thread(target=self._recv_loop)
        self._recv_thread.daemon = True
        self._recv_thread.start()
        print("✓ 开始接收数据")
    
    def stop_receiving(self):
        """停止接收"""
        if self._recv_thread:
            self._recv_thread.join(timeout=1.0)
            print("✓ 停止接收数据")
    
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
        while len(self._recv_buffer) >= 72:  # 最小包长度
            # 查找协议头
            header_idx = self._recv_buffer.find(self.PROTOCOL_HEADER_RECV)
            if header_idx == -1:
                self._recv_buffer.clear()
                return
            
            # 丢弃头部之前的数据
            if header_idx > 0:
                self._recv_buffer = self._recv_buffer[header_idx:]
            
            # 检查是否有足够的数据
            if len(self._recv_buffer) < 4:
                return
            
            # 获取包长度
            packet_length = struct.unpack('<H', self._recv_buffer[2:4])[0]
            
            # 检查是否有完整的数据包
            if len(self._recv_buffer) < packet_length:
                return
            
            # 提取数据包
            packet = bytes(self._recv_buffer[:packet_length])
            self._recv_buffer = self._recv_buffer[packet_length:]
            
            # 解析数据包
            self._parse_packet(packet)
    
    def _parse_packet(self, packet: bytes):
        """解析GCU返回数据包"""
        try:
            if len(packet) < 72:
                return
            
            # 验证CRC
            crc_received = struct.unpack('<H', packet[-2:])[0]
            crc_calculated = self._calculate_crc16(packet[:-2])
            if crc_received != crc_calculated:
                return  # CRC校验失败
            
            # 解析主帧
            status = {
                'timestamp': datetime.now().strftime('%H:%M:%S.%f')[:-3],
                'work_mode': packet[5],
                'status_flags': struct.unpack('<H', packet[6:8])[0],
                'tracking_offset_h': struct.unpack('<h', packet[8:10])[0] / 10.0,  # 水平脱靶量
                'tracking_offset_v': struct.unpack('<h', packet[10:12])[0] / 10.0,  # 垂直脱靶量
                'camera_rel_x': struct.unpack('<h', packet[12:14])[0] / 100.0,  # 相机X轴相对角度
                'camera_rel_y': struct.unpack('<h', packet[14:16])[0] / 100.0,  # 相机Y轴相对角度
                'camera_rel_z': struct.unpack('<h', packet[16:18])[0] / 100.0,  # 相机Z轴相对角度
                'camera_abs_roll': struct.unpack('<h', packet[18:20])[0] / 100.0,  # 相机绝对滚转角
                'camera_abs_pitch': struct.unpack('<h', packet[20:22])[0] / 100.0,  # 相机绝对俯仰角
                'camera_abs_yaw': struct.unpack('<H', packet[22:24])[0] / 100.0,  # 相机绝对偏航角
                'camera_vel_x': struct.unpack('<h', packet[24:26])[0] / 10.0,  # 相机X轴角速度
                'camera_vel_y': struct.unpack('<h', packet[26:28])[0] / 10.0,  # 相机Y轴角速度
                'camera_vel_z': struct.unpack('<h', packet[28:30])[0] / 10.0,  # 相机Z轴角速度
            }
            
            # 解析副帧（如果有）
            if len(packet) >= 73 and packet[37] == 0x01:
                status.update({
                    'hardware_ver': packet[38],
                    'firmware_ver': packet[39],
                    'gimbal_code': packet[40],
                    'error_code': struct.unpack('<H', packet[41:43])[0],
                    'target_distance': struct.unpack('<i', packet[43:47])[0] / 10.0,
                    'target_lon': struct.unpack('<i', packet[47:51])[0] / 1e7,
                    'target_lat': struct.unpack('<i', packet[51:55])[0] / 1e7,
                    'target_alt': struct.unpack('<i', packet[55:59])[0] / 1000.0,
                    'zoom1': struct.unpack('<H', packet[59:61])[0] / 10.0,
                    'zoom2': struct.unpack('<H', packet[61:63])[0] / 10.0,
                })
            
            self._latest_status = status
            
            # 调用回调
            if self._status_callback:
                self._status_callback(status)
            
        except Exception as e:
            pass  # 解析失败，忽略
    
    def get_latest_status(self) -> Optional[Dict]:
        """获取最新状态"""
        return self._latest_status
    
    def print_status(self, status: Dict = None):
        """打印当前状态"""
        if status is None:
            status = self._latest_status
        if status is None:
            print("暂无状态数据")
            return
        
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
        
        print(f"\n[{status['timestamp']}] 云台状态:")
        print(f"  工作模式: {mode}")
        print(f"  相机绝对角度: 滚转={status['camera_abs_roll']:7.2f}°, 俯仰={status['camera_abs_pitch']:7.2f}°, 偏航={status['camera_abs_yaw']:7.2f}°")
        print(f"  相机相对角度: X={status['camera_rel_x']:7.2f}°, Y={status['camera_rel_y']:7.2f}°, Z={status['camera_rel_z']:7.2f}°")
        print(f"  相机角速度: X={status['camera_vel_x']:6.1f}°/s, Y={status['camera_vel_y']:6.1f}°/s, Z={status['camera_vel_z']:6.1f}°/s")
        
        if 'zoom1' in status:
            print(f"  相机倍率: 可见光={status['zoom1']:.1f}x, 热成像={status['zoom2']:.1f}x")
        if 'target_distance' in status and status['target_distance'] > 0:
            print(f"  目标距离: {status['target_distance']:.1f}m")
    
    def set_control_values(self, roll: int = 0, pitch: int = 0, yaw: int = 0, valid: bool = True):
        """
        设置控制量
        
        Args:
            roll: 滚转控制量
                  - 期望角速率模式: [-1500, 1500]，分辨率 0.1°/s
                  - 期望欧拉角模式: [-18000, 18000]，分辨率 0.01°
                  - 期望相对角度模式: [-18000, 18000]，分辨率 0.01°
            pitch: 俯仰控制量
            yaw: 偏航控制量
            valid: 控制量是否有效
        """
        with self._lock:
            self._roll_control = roll
            self._pitch_control = pitch
            self._yaw_control = yaw
            self._control_valid = valid
    
    def set_aircraft_attitude(self, roll: float = 0, pitch: float = 0, yaw: float = 0):
        """
        设置载机姿态（用于提高控制精度）
        
        Args:
            roll: 载机滚转角（度），范围 [-180, 180]
            pitch: 载机俯仰角（度），范围 [-90, 90]
            yaw: 载机偏航角（度），范围 [0, 360)
        """
        with self._lock:
            self._aircraft_roll = int(roll * 100)
            self._aircraft_pitch = int(pitch * 100)
            self._aircraft_yaw = int(yaw * 100) % 36000
    
    def set_aircraft_motion(self, accel_north: float = 0, accel_east: float = 0, accel_up: float = 0,
                           vel_north: float = 0, vel_east: float = 0, vel_up: float = 0):
        """
        设置载机运动状态
        
        Args:
            accel_north: 北向加速度 (m/s²)
            accel_east: 东向加速度 (m/s²)
            accel_up: 天向加速度 (m/s²)
            vel_north: 北向速度 (m/s)
            vel_east: 东向速度 (m/s)
            vel_up: 天向速度 (m/s)
        """
        with self._lock:
            self._accel_north = int(accel_north * 100)
            self._accel_east = int(accel_east * 100)
            self._accel_up = int(accel_up * 100)
            self._vel_north = int(vel_north * 100)
            self._vel_east = int(vel_east * 100)
            self._vel_up = int(vel_up * 100)
    
    # ==================== 模式切换命令 ====================
    
    def set_mode_angle_control(self):
        """切换到角度控制模式
        
        控制量定义：
        - 滚转：期望欧拉角
        - 俯仰：期望欧拉角
        - 偏航：期望相对角度
        """
        self._current_mode = self.MODE_ANGLE_CONTROL
        return self.send_packet(self.MODE_ANGLE_CONTROL)
    
    def set_mode_pointing_lock(self):
        """切换到指向锁定模式
        
        控制量定义：
        - 滚转：无效
        - 俯仰：期望角速度
        - 偏航：期望角速度
        """
        self._current_mode = self.MODE_POINTING_LOCK
        return self.send_packet(self.MODE_POINTING_LOCK)
    
    def set_mode_pointing_follow(self):
        """切换到指向跟随模式
        
        控制量定义：
        - 滚转：无效
        - 俯仰：期望角速度
        - 偏航：期望角速度（为0或无效时跟随载机）
        """
        self._current_mode = self.MODE_POINTING_FOLLOW
        return self.send_packet(self.MODE_POINTING_FOLLOW)
    
    def set_mode_euler_angle(self):
        """切换到欧拉角控制模式
        
        控制量定义：
        - 滚转：期望欧拉角
        - 俯仰：期望欧拉角
        - 偏航：期望欧拉角
        """
        self._current_mode = self.MODE_EULER_ANGLE
        return self.send_packet(self.MODE_EULER_ANGLE)
    
    def set_mode_fpv(self):
        """切换到FPV模式
        
        控制量定义：
        - 滚转：期望相对角度
        - 俯仰：期望相对角度
        - 偏航：期望相对角度
        """
        self._current_mode = self.MODE_FPV
        return self.send_packet(self.MODE_FPV)
    
    def set_mode_top_down(self):
        """切换到俯拍模式"""
        return self.send_packet(self.MODE_TOP_DOWN)
    
    # ==================== 运动控制 ====================
    
    def rotate_pitch(self, speed: int):
        """
        俯仰旋转（角速度模式）
        
        Args:
            speed: 旋转速度，范围 [-1500, 1500]，单位 0.1°/s
                   正值向上，负值向下
        """
        self.set_control_values(roll=0, pitch=speed, yaw=0, valid=True)
    
    def rotate_yaw(self, speed: int):
        """
        偏航旋转（角速度模式）
        
        Args:
            speed: 旋转速度，范围 [-1500, 1500]，单位 0.1°/s
                   正值向右，负值向左
        """
        self.set_control_values(roll=0, pitch=0, yaw=speed, valid=True)
    
    def rotate(self, pitch_speed: int = 0, yaw_speed: int = 0):
        """
        同时控制俯仰和偏航旋转
        
        Args:
            pitch_speed: 俯仰旋转速度
            yaw_speed: 偏航旋转速度
        """
        self.set_control_values(roll=0, pitch=pitch_speed, yaw=yaw_speed, valid=True)
    
    def stop_rotation(self):
        """停止旋转（控制量置为无效）"""
        self.set_control_values(roll=0, pitch=0, yaw=0, valid=False)
    
    def set_euler_angles(self, roll: float, pitch: float, yaw: float):
        """
        设置期望欧拉角（用于欧拉角控制模式）
        
        Args:
            roll: 滚转角（度），范围 [-180, 180]
            pitch: 俯仰角（度），范围 [-180, 180]
            yaw: 偏航角（度），范围 [-180, 180]
        """
        roll_val = int(roll * 100)
        pitch_val = int(pitch * 100)
        yaw_val = int(yaw * 100)
        self.set_control_values(roll=roll_val, pitch=pitch_val, yaw=yaw_val, valid=True)
    
    def set_relative_angles(self, roll: float, pitch: float, yaw: float):
        """
        设置期望相对角度（用于角度控制模式或FPV模式）
        
        Args:
            roll: 相对滚转角（度），范围 [-180, 180]
            pitch: 相对俯仰角（度），范围 [-180, 180]
            yaw: 相对偏航角（度），范围 [-180, 180]
        """
        roll_val = int(roll * 100)
        pitch_val = int(pitch * 100)
        yaw_val = int(yaw * 100)
        self.set_control_values(roll=roll_val, pitch=pitch_val, yaw=yaw_val, valid=True)
    
    # ==================== 常用功能命令 ====================
    
    def reset_gimbal(self):
        """回中"""
        return self.send_packet(self.CMD_RESET)
    
    def calibrate(self):
        """吊舱校准（校准时需保持吊舱静止，持续数秒）"""
        return self.send_packet(self.CMD_CALIBRATION)
    
    def take_photo(self, camera: int = 0x01):
        """
        拍照
        
        Args:
            camera: 相机序号，0x01=1号相机，0xFF=所有相机
        """
        return self.send_packet(self.CMD_PHOTO, bytes([0x01, camera]))
    
    def toggle_record(self, camera: int = 0x01):
        """
        开始/停止录像
        
        Args:
            camera: 相机序号
        """
        return self.send_packet(self.CMD_RECORD, bytes([0x01, camera]))
    
    def zoom_in(self, camera: int = 0x01):
        """
        连续放大
        
        Args:
            camera: 相机序号
        """
        return self.send_packet(self.CMD_ZOOM_IN, bytes([camera]))
    
    def zoom_out(self, camera: int = 0x01):
        """
        连续缩小
        
        Args:
            camera: 相机序号
        """
        return self.send_packet(self.CMD_ZOOM_OUT, bytes([camera]))
    
    def zoom_stop(self, camera: int = 0x01):
        """
        停止变倍
        
        Args:
            camera: 相机序号
        """
        return self.send_packet(self.CMD_ZOOM_STOP, bytes([camera]))
    
    def set_zoom(self, zoom_value: float, camera: int = 0x01):
        """
        设置指定倍率
        
        Args:
            zoom_value: 倍率值
                       - 负值区 [-32768, -10]: 期望倍率，分辨率 0.1x
                       - 正值区 [1, 10000]: 倍率比例，1对应最小倍率，10000对应最大倍率
            camera: 相机序号
        """
        if zoom_value < 0:
            zoom_val = int(zoom_value)
        else:
            zoom_val = int(zoom_value)
        return self.send_packet(self.CMD_ZOOM_SET, bytes([camera]) + struct.pack('<h', zoom_val))
    
    def focus(self, camera: int = 0x01):
        """
        聚焦
        
        Args:
            camera: 相机序号
        """
        return self.send_packet(self.CMD_FOCUS, bytes([0x01, camera]))
    
    def set_night_vision(self, enabled: bool):
        """
        设置夜视模式
        
        Args:
            enabled: 是否开启
        """
        mode = 0x01 if enabled else 0x00
        return self.send_packet(self.CMD_NIGHT_VISION, bytes([0x01, mode]))
    
    def set_osd(self, enabled: bool):
        """
        设置OSD显示
        
        Args:
            enabled: 是否开启
        """
        return self.send_packet(self.CMD_OSD, bytes([0x01 if enabled else 0x00]))
    
    def set_illumination(self, brightness: int):
        """
        设置补光亮度
        
        Args:
            brightness: 亮度值，范围 [0, 255]
        """
        return self.send_packet(self.CMD_ILLUMINATION, bytes([brightness]))
    
    def set_ranging(self, enabled: bool):
        """
        设置连续测距
        
        Args:
            enabled: 是否开启
        """
        mode = 0x02 if enabled else 0x00
        return self.send_packet(self.CMD_RANGING, bytes([mode]))


def demo_rotation():
    """云台旋转控制演示"""
    
    # 创建控制器实例
    gimbal = GCUGimbalController(port='COM4', baudrate=115200)
    
    # 连接串口
    if not gimbal.connect():
        return
    
    # 启动接收线程，打印状态
    def on_status(status):
        gimbal.print_status(status)
    
    gimbal.start_receiving(callback=on_status)
    
    try:
        print("\n" + "="*60)
        print("云台旋转控制演示")
        print("="*60)
        
        # 设置载机姿态（假设载机水平静止）
        print("\n[初始化] 设置载机姿态（水平静止）")
        gimbal.set_aircraft_attitude(roll=0, pitch=0, yaw=0)
        gimbal.set_aircraft_motion(accel_north=0, accel_east=0, accel_up=0,
                                   vel_north=0, vel_east=0, vel_up=0)
        
        # 1. 切换到指向锁定模式
        print("\n[1] 切换到指向锁定模式")
        gimbal.set_mode_pointing_lock()
        time.sleep(0.5)
        
        # 发送空命令隔开（协议要求）
        gimbal.send_packet(0x00)
        time.sleep(0.1)
        
        # 启动持续发送（30Hz）
        gimbal.start_sending(frequency=30)
        
        # 2. 向上旋转（俯仰 +50°/s）
        print("\n[2] 向上旋转（俯仰 +50°/s，持续2秒）")
        gimbal.rotate_pitch(500)  # 500 = 50.0°/s
        time.sleep(2)
        
        # 3. 向下旋转（俯仰 -50°/s）
        print("\n[3] 向下旋转（俯仰 -50°/s，持续2秒）")
        gimbal.rotate_pitch(-500)
        time.sleep(2)
        
        # 4. 向右旋转（偏航 +30°/s）
        print("\n[4] 向右旋转（偏航 +30°/s，持续2秒）")
        gimbal.rotate_yaw(300)  # 300 = 30.0°/s
        time.sleep(2)
        
        # 5. 向左旋转（偏航 -30°/s）
        print("\n[5] 向左旋转（偏航 -30°/s，持续2秒）")
        gimbal.rotate_yaw(-300)
        time.sleep(2)
        
        # 6. 同时俯仰和偏航旋转
        print("\n[6] 同时俯仰和偏航旋转（俯仰30°/s，偏航20°/s，持续2秒）")
        gimbal.rotate(pitch_speed=300, yaw_speed=200)
        time.sleep(2)
        
        # 7. 停止旋转
        print("\n[7] 停止旋转")
        gimbal.stop_rotation()
        time.sleep(1)
        
        # 停止持续发送
        gimbal.stop_sending()
        
        # 8. 切换到欧拉角控制模式，设置固定角度
        print("\n[8] 切换到欧拉角控制模式，设置俯仰45°")
        gimbal.set_mode_euler_angle()
        time.sleep(0.5)
        gimbal.set_euler_angles(roll=0, pitch=45, yaw=0)
        gimbal.start_sending(frequency=30)
        time.sleep(3)
        
        # 9. 回中
        print("\n[9] 回中")
        gimbal.stop_sending()
        gimbal.reset_gimbal()
        
        print("\n" + "="*60)
        print("演示完成！")
        print("="*60)
        
    except KeyboardInterrupt:
        print("\n\n用户中断")
    finally:
        gimbal.stop_sending()
        gimbal.stop_receiving()
        gimbal.disconnect()


def demo_camera_control():
    """相机控制演示"""
    
    gimbal = GCUGimbalController(port='COM4', baudrate=115200)
    
    if not gimbal.connect():
        return
    
    # 启动接收线程
    gimbal.start_receiving()
    
    try:
        print("\n" + "="*60)
        print("相机控制演示")
        print("="*60)
        
        # 设置载机姿态
        gimbal.set_aircraft_attitude(roll=0, pitch=0, yaw=0)
        gimbal.set_aircraft_motion(0, 0, 0, 0, 0, 0)
        
        # 拍照
        print("\n[1] 拍照")
        gimbal.take_photo()
        time.sleep(1)
        
        # 开始录像
        print("\n[2] 开始录像")
        gimbal.toggle_record()
        time.sleep(3)
        
        # 停止录像
        print("\n[3] 停止录像")
        gimbal.toggle_record()
        time.sleep(1)
        
        # 放大
        print("\n[4] 放大")
        gimbal.zoom_in()
        time.sleep(2)
        
        # 停止变倍
        print("\n[5] 停止变倍")
        gimbal.zoom_stop()
        time.sleep(1)
        
        # 设置倍率为5倍
        print("\n[6] 设置倍率为5倍")
        gimbal.set_zoom(-50)  # -50 = 5.0x
        time.sleep(1)
        
        # 聚焦
        print("\n[7] 聚焦")
        gimbal.focus()
        
        # 打印当前状态
        time.sleep(0.5)
        gimbal.print_status()
        
        print("\n" + "="*60)
        print("演示完成！")
        print("="*60)
        
    except KeyboardInterrupt:
        print("\n\n用户中断")
    finally:
        gimbal.stop_receiving()
        gimbal.disconnect()


def interactive_control():
    """交互式控制"""
    
    gimbal = GCUGimbalController(port='COM4', baudrate=115200)
    
    if not gimbal.connect():
        return
    
    # 启动接收线程
    gimbal.start_receiving()
    
    # 设置默认载机姿态
    gimbal.set_aircraft_attitude(roll=0, pitch=0, yaw=0)
    gimbal.set_aircraft_motion(0, 0, 0, 0, 0, 0)
    
    print("\n" + "="*60)
    print("交互式云台控制")
    print("="*60)
    print("命令列表:")
    print("  mode [angle|lock|follow|euler|fpv] - 切换模式")
    print("  pitch [speed]   - 设置俯仰速度 (°/s)")
    print("  yaw [speed]     - 设置偏航速度 (°/s)")
    print("  rotate [p] [y]  - 同时设置俯仰和偏航速度")
    print("  angle [r] [p] [y] - 设置欧拉角 (度)")
    print("  stop            - 停止旋转")
    print("  reset           - 回中")
    print("  photo           - 拍照")
    print("  record          - 开始/停止录像")
    print("  zoom [in|out|stop|value] - 变焦控制")
    print("  status          - 打印当前状态")
    print("  start           - 开始持续发送")
    print("  stop_send       - 停止持续发送")
    print("  q/quit          - 退出")
    print("="*60)
    
    gimbal.set_mode_pointing_lock()
    
    try:
        while True:
            cmd = input("\n> ").strip().lower().split()
            if not cmd:
                continue
            
            if cmd[0] == 'q' or cmd[0] == 'quit':
                break
            
            elif cmd[0] == 'mode':
                if len(cmd) < 2:
                    print("用法: mode [angle|lock|follow|euler|fpv]")
                    continue
                mode = cmd[1]
                if mode == 'angle':
                    gimbal.set_mode_angle_control()
                elif mode == 'lock':
                    gimbal.set_mode_pointing_lock()
                elif mode == 'follow':
                    gimbal.set_mode_pointing_follow()
                elif mode == 'euler':
                    gimbal.set_mode_euler_angle()
                elif mode == 'fpv':
                    gimbal.set_mode_fpv()
                else:
                    print(f"未知模式: {mode}")
                    continue
                print(f"✓ 已切换到 {mode} 模式")
            
            elif cmd[0] == 'pitch':
                if len(cmd) < 2:
                    print("用法: pitch [speed]")
                    continue
                speed = int(float(cmd[1]) * 10)
                gimbal.rotate_pitch(speed)
                gimbal.send_packet()
                print(f"✓ 俯仰速度设置为 {cmd[1]}°/s")
            
            elif cmd[0] == 'yaw':
                if len(cmd) < 2:
                    print("用法: yaw [speed]")
                    continue
                speed = int(float(cmd[1]) * 10)
                gimbal.rotate_yaw(speed)
                gimbal.send_packet()
                print(f"✓ 偏航速度设置为 {cmd[1]}°/s")
            
            elif cmd[0] == 'rotate':
                if len(cmd) < 3:
                    print("用法: rotate [pitch_speed] [yaw_speed]")
                    continue
                p_speed = int(float(cmd[1]) * 10)
                y_speed = int(float(cmd[2]) * 10)
                gimbal.rotate(p_speed, y_speed)
                gimbal.send_packet()
                print(f"✓ 旋转速度: 俯仰={cmd[1]}°/s, 偏航={cmd[2]}°/s")
            
            elif cmd[0] == 'angle':
                if len(cmd) < 4:
                    print("用法: angle [roll] [pitch] [yaw]")
                    continue
                roll = float(cmd[1])
                pitch = float(cmd[2])
                yaw = float(cmd[3])
                gimbal.set_euler_angles(roll, pitch, yaw)
                gimbal.send_packet()
                print(f"✓ 欧拉角: 滚转={roll}°, 俯仰={pitch}°, 偏航={yaw}°")
            
            elif cmd[0] == 'stop':
                gimbal.stop_rotation()
                gimbal.send_packet()
                print("✓ 已停止旋转")
            
            elif cmd[0] == 'reset':
                gimbal.reset_gimbal()
                print("✓ 已回中")
            
            elif cmd[0] == 'photo':
                gimbal.take_photo()
                print("✓ 已拍照")
            
            elif cmd[0] == 'record':
                gimbal.toggle_record()
                print("✓ 已切换录像状态")
            
            elif cmd[0] == 'zoom':
                if len(cmd) < 2:
                    print("用法: zoom [in|out|stop|value]")
                    continue
                action = cmd[1]
                if action == 'in':
                    gimbal.zoom_in()
                elif action == 'out':
                    gimbal.zoom_out()
                elif action == 'stop':
                    gimbal.zoom_stop()
                else:
                    try:
                        value = float(action)
                        gimbal.set_zoom(int(value * -10))
                    except ValueError:
                        print("无效的变焦值")
                        continue
                print(f"✓ 变焦: {action}")
            
            elif cmd[0] == 'status':
                gimbal.print_status()
            
            elif cmd[0] == 'start':
                gimbal.start_sending(frequency=30)
            
            elif cmd[0] == 'stop_send':
                gimbal.stop_sending()
            
            else:
                print(f"未知命令: {cmd[0]}")
    
    except KeyboardInterrupt:
        print("\n\n用户中断")
    finally:
        gimbal.stop_sending()
        gimbal.stop_receiving()
        gimbal.disconnect()


def test_example_packet():
    """测试发送生成的数据包"""
    
    gimbal = GCUGimbalController(port='COM4', baudrate=115200)
    
    if not gimbal.connect():
        return
    
    # 启动接收线程
    gimbal.start_receiving()
    
    try:
        print("\n" + "="*60)
        print("测试生成的数据包")
        print("="*60)
        
        # 1. 切换到指向锁定模式
        print("\n[1] 切换到指向锁定模式")
        gimbal.set_mode_pointing_lock()
        time.sleep(0.5)
        
        # 2. 俯仰控制（+10°/s）
        print("\n[2] 俯仰 +10°/s（持续2秒）")
        gimbal.rotate_pitch(100)  # 100 = 10.0°/s
        gimbal.send_packet(debug=True)
        time.sleep(2)
        
        # 3. 俯仰控制（-10°/s）
        print("\n[3] 俯仰 -10°/s（持续2秒）")
        gimbal.rotate_pitch(-100)
        gimbal.send_packet(debug=True)
        time.sleep(2)
        
        # 4. 停止控制
        print("\n[4] 停止控制")
        gimbal.stop_rotation()
        gimbal.send_packet(debug=True)
        
        # 5. 回中
        print("\n[5] 回中")
        gimbal.reset_gimbal()
        
        print("\n" + "="*60)
        print("测试完成！")
        print("="*60)
        
    except KeyboardInterrupt:
        print("\n\n用户中断")
    finally:
        gimbal.stop_receiving()
        gimbal.disconnect()


if __name__ == '__main__':
    import sys
    
    if len(sys.argv) > 1:
        if sys.argv[1] == 'demo':
            demo_rotation()
        elif sys.argv[1] == 'camera':
            demo_camera_control()
        elif sys.argv[1] == 'interactive':
            interactive_control()
        elif sys.argv[1] == 'test':
            test_example_packet()
        else:
            print(f"用法: python {sys.argv[0]} [demo|camera|interactive|test]")
            print("  demo       - 运行旋转控制演示")
            print("  camera     - 运行相机控制演示")
            print("  interactive - 交互式控制")
            print("  test       - 测试示例数据包")
    else:
        # 默认运行旋转演示
        demo_rotation()
