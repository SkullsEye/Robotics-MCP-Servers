"""
Robotiq Gripper control module (direct TCP on port 63352).
Based on robotiq_gripper.py from ur_rtde SDK.
Communicates directly with the Robotiq_grippers UR Cap port.
"""

from __future__ import annotations

import socket
import threading
import time
from enum import Enum
from typing import Union, Tuple, OrderedDict, Optional


class RobotiqGripper:
    """
    Communicates with the Robotiq gripper directly via socket with string commands.
    Uses the Robotiq_grippers UR Cap port (default: 63352).
    """

    # WRITE VARIABLES (CAN ALSO READ)
    ACT = 'ACT'  # act : activate (1 while activated, can be reset to clear fault status)
    GTO = 'GTO'  # gto : go to (will perform go to with the actions set in pos, for, spe)
    ATR = 'ATR'  # atr : auto-release (emergency slow move)
    ADR = 'ADR'  # adr : auto-release direction (open(1) or close(0) during auto-release)
    FOR = 'FOR'  # for : force (0-255)
    SPE = 'SPE'  # spe : speed (0-255)
    POS = 'POS'  # pos : position (0-255), 0 = open

    # READ VARIABLES
    STA = 'STA'  # status (0 = is reset, 1 = activating, 3 = active)
    PRE = 'PRE'  # position request (echo of last commanded position)
    OBJ = 'OBJ'  # object detection (0 = moving, 1 = outer grip, 2 = inner grip, 3 = no object at rest)
    FLT = 'FLT'  # fault (0=ok, see manual for errors if not zero)

    ENCODING = 'UTF-8'

    class GripperStatus(Enum):
        """Gripper status reported by the gripper."""
        RESET = 0
        ACTIVATING = 1
        ACTIVE = 3

    class ObjectStatus(Enum):
        """Object status reported by the gripper."""
        MOVING = 0
        STOPPED_OUTER_OBJECT = 1
        STOPPED_INNER_OBJECT = 2
        AT_DEST = 3

    def __init__(self) -> None:
        """Constructor."""
        self.socket: Optional[socket.socket] = None
        self.command_lock = threading.Lock()
        self._min_position = 0
        self._max_position = 255
        self._min_speed = 0
        self._max_speed = 255
        self._min_force = 0
        self._max_force = 255
        self._hostname: str = ""
        self._port: int = 63352

    # -----------------------------------------------------------------------
    # Connection
    # -----------------------------------------------------------------------

    def connect(self, hostname: str, port: int = 63352, socket_timeout: float = 2.0) -> None:
        """Connects to a gripper at the given address.

        :param hostname: Hostname or IP of the robot (gripper uses same IP, different port).
        :param port: Port (default 63352 for Robotiq UR Cap).
        :param socket_timeout: Timeout for blocking socket operations.
        """
        self._hostname = hostname
        self._port = port
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.socket.connect((hostname, port))
        self.socket.settimeout(socket_timeout)

    def disconnect(self) -> None:
        """Closes the connection with the gripper."""
        if self.socket:
            self.socket.close()
            self.socket = None

    def is_connected(self) -> bool:
        """Returns True if socket is connected."""
        if self.socket is None:
            return False
        try:
            # Check if socket is still usable by getting peer name
            self.socket.getpeername()
            return True
        except (socket.error, OSError):
            return False

    # -----------------------------------------------------------------------
    # Low-level communication
    # -----------------------------------------------------------------------

    def _set_vars(self, var_dict: OrderedDict[str, Union[int, float]]) -> bool:
        """Sends command to set multiple variables, waits for 'ack' response."""
        cmd = "SET"
        for variable, value in var_dict.items():
            cmd += f" {variable} {str(value)}"
        cmd += '\n'
        with self.command_lock:
            self.socket.sendall(cmd.encode(self.ENCODING))
            data = self.socket.recv(1024)
        return self._is_ack(data)

    def _set_var(self, variable: str, value: Union[int, float]) -> bool:
        """Sends command to set a single variable."""
        return self._set_vars(OrderedDict([(variable, value)]))

    def _get_var(self, variable: str) -> int:
        """Retrieves the value of a variable from the gripper."""
        with self.command_lock:
            cmd = f"GET {variable}\n"
            self.socket.sendall(cmd.encode(self.ENCODING))
            data = self.socket.recv(1024)
        var_name, value_str = data.decode(self.ENCODING).split()
        if var_name != variable:
            raise ValueError(f"Unexpected response {data}: does not match '{variable}'")
        return int(value_str)

    @staticmethod
    def _is_ack(data: bytes) -> bool:
        return data == b'ack'

    # -----------------------------------------------------------------------
    # Activation and calibration
    # -----------------------------------------------------------------------

    def _reset(self) -> None:
        """Reset the gripper."""
        self._set_var(self.ACT, 0)
        self._set_var(self.ATR, 0)
        while (not self._get_var(self.ACT) == 0 or not self._get_var(self.STA) == 0):
            self._set_var(self.ACT, 0)
            self._set_var(self.ATR, 0)
        time.sleep(0.5)

    def activate(self, auto_calibrate: bool = True) -> None:
        """Activates the gripper. Resets activation flag and sets it back to one.

        :param auto_calibrate: Whether to calibrate min/max positions.
        """
        if not self.is_active():
            self._reset()
            while (not self._get_var(self.ACT) == 0 or not self._get_var(self.STA) == 0):
                time.sleep(0.01)
            self._set_var(self.ACT, 1)
            time.sleep(1.0)
            while (not self._get_var(self.ACT) == 1 or not self._get_var(self.STA) == 3):
                time.sleep(0.01)

        if auto_calibrate:
            self.auto_calibrate()

    def is_active(self) -> bool:
        """Returns whether the gripper is active."""
        status = self._get_var(self.STA)
        return RobotiqGripper.GripperStatus(status) == RobotiqGripper.GripperStatus.ACTIVE

    def auto_calibrate(self, log: bool = True) -> None:
        """Calibrates open and closed positions by moving gripper."""
        # First try to open
        position, status = self.move_and_wait_for_pos(self.get_open_position(), 64, 1)
        if RobotiqGripper.ObjectStatus(status) != RobotiqGripper.ObjectStatus.AT_DEST:
            raise RuntimeError(f"Calibration failed opening: {str(status)}")

        # Close as far as possible
        position, status = self.move_and_wait_for_pos(self.get_closed_position(), 64, 1)
        if RobotiqGripper.ObjectStatus(status) != RobotiqGripper.ObjectStatus.AT_DEST:
            raise RuntimeError(f"Calibration failed closing: {str(status)}")
        self._max_position = position

        # Open as far as possible
        position, status = self.move_and_wait_for_pos(self.get_open_position(), 64, 1)
        if RobotiqGripper.ObjectStatus(status) != RobotiqGripper.ObjectStatus.AT_DEST:
            raise RuntimeError(f"Calibration failed opening: {str(status)}")
        self._min_position = position

        if log:
            print(f"Gripper auto-calibrated to [{self.get_min_position()}, {self.get_max_position()}]")

    # -----------------------------------------------------------------------
    # Position queries
    # -----------------------------------------------------------------------

    def get_min_position(self) -> int:
        """Returns minimum position (open)."""
        return self._min_position

    def get_max_position(self) -> int:
        """Returns maximum position (closed)."""
        return self._max_position

    def get_open_position(self) -> int:
        """Returns open position (minimum value)."""
        return self.get_min_position()

    def get_closed_position(self) -> int:
        """Returns closed position (maximum value)."""
        return self.get_max_position()

    def get_current_position(self) -> int:
        """Returns current position from hardware (0-255)."""
        return self._get_var(self.POS)

    def is_open(self) -> bool:
        """Returns True if gripper is considered fully open."""
        return self.get_current_position() <= self.get_open_position()

    def is_closed(self) -> bool:
        """Returns True if gripper is considered fully closed."""
        return self.get_current_position() >= self.get_closed_position()

    # -----------------------------------------------------------------------
    # Motion
    # -----------------------------------------------------------------------

    def move(self, position: int, speed: int, force: int) -> Tuple[bool, int]:
        """Start moving towards position with specified speed and force.

        :param position: Position to move to [min_position, max_position].
        :param speed: Speed [0-255].
        :param force: Force [0-255].
        :return: (success, actual_position_commanded).
        """
        def clip_val(min_val, val, max_val):
            return max(min_val, min(val, max_val))

        clip_pos = clip_val(self._min_position, position, self._max_position)
        clip_spe = clip_val(self._min_speed, speed, self._max_speed)
        clip_for = clip_val(self._min_force, force, self._max_force)

        var_dict = OrderedDict([(self.POS, clip_pos), (self.SPE, clip_spe), (self.FOR, clip_for), (self.GTO, 1)])
        return self._set_vars(var_dict), clip_pos

    def move_and_wait_for_pos(self, position: int, speed: int, force: int) -> Tuple[int, ObjectStatus]:
        """Move and wait for completion.

        :return: (final_position, object_status).
        """
        set_ok, cmd_pos = self.move(position, speed, force)
        if not set_ok:
            raise RuntimeError("Failed to set variables for move.")

        while self._get_var(self.PRE) != cmd_pos:
            time.sleep(0.001)

        cur_obj = self._get_var(self.OBJ)
        while RobotiqGripper.ObjectStatus(cur_obj) == RobotiqGripper.ObjectStatus.MOVING:
            cur_obj = self._get_var(self.OBJ)

        final_pos = self._get_var(self.POS)
        return final_pos, RobotiqGripper.ObjectStatus(cur_obj)

    def open_gripper(self, speed: int = 255, force: int = 255) -> Tuple[int, ObjectStatus]:
        """Open gripper fully."""
        return self.move_and_wait_for_pos(self.get_open_position(), speed, force)

    def close_gripper(self, speed: int = 255, force: int = 255) -> Tuple[int, ObjectStatus]:
        """Close gripper fully."""
        return self.move_and_wait_for_pos(self.get_closed_position(), speed, force)

    # -----------------------------------------------------------------------
    # Status
    # -----------------------------------------------------------------------

    def get_status(self) -> dict:
        """Returns full gripper status as dict."""
        pos = self._get_var(self.POS)
        sta = self._get_var(self.STA)
        obj = self._get_var(self.OBJ)
        pre = self._get_var(self.PRE)
        flt = self._get_var(self.FLT)

        status_names = {0: "RESET", 1: "ACTIVATING", 3: "ACTIVE"}
        obj_names = {0: "MOVING", 1: "OUTER_OBJECT", 2: "INNER_OBJECT", 3: "AT_DEST"}

        return {
            "position": pos,
            "status": status_names.get(sta, f"UNKNOWN({sta})"),
            "status_code": sta,
            "object_status": obj_names.get(obj, f"UNKNOWN({obj})"),
            "object_code": obj,
            "requested_position": pre,
            "fault": flt,
            "is_open": pos <= self._min_position,
            "is_closed": pos >= self._max_position,
            "calibrated_range": [self._min_position, self._max_position],
        }
