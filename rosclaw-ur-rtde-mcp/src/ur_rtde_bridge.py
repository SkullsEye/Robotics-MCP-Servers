"""
ur_rtde hardware bridge — thread-safe wrapper around RTDEControlInterface,
RTDEReceiveInterface, RTDEIOInterface, DashboardClient, and RobotiqGripper.
"""

from __future__ import annotations

import json
import threading
from dataclasses import dataclass, field
from typing import Optional

from robotiq_gripper import RobotiqGripper

# ---------------------------------------------------------------------------
# ur_rtde import (graceful degradation when not installed)
# ---------------------------------------------------------------------------
try:
    from ur_rtde import RTDEControlInterface, RTDEReceiveInterface, RTDEIOInterface
    from ur_rtde import DashboardClient
    _HAS_UR_RTDE = True
except ImportError:
    try:
        from rtde_control import RTDEControlInterface   # type: ignore[no-redef]
        from rtde_receive import RTDEReceiveInterface   # type: ignore[no-redef]
        from rtde_io import RTDEIOInterface             # type: ignore[no-redef]
        from dashboard_client import DashboardClient   # type: ignore[no-redef]
        _HAS_UR_RTDE = True
    except ImportError:
        _HAS_UR_RTDE = False

# ---------------------------------------------------------------------------
# Safety constants (from RTDEControlInterface header)
# ---------------------------------------------------------------------------
JOINT_VEL_MAX = 3.14        # rad/s
JOINT_ACC_MAX = 40.0        # rad/s²
TOOL_VEL_MAX = 3.0          # m/s
TOOL_ACC_MAX = 150.0        # m/s²
SERVO_LOOKAHEAD_MIN = 0.03  # s
SERVO_LOOKAHEAD_MAX = 0.2   # s
SERVO_GAIN_MIN = 100
SERVO_GAIN_MAX = 2000

# Human-readable robot mode names (getRobotMode)
ROBOT_MODE_NAMES: dict[int, str] = {
    -1: "NO_CONTROLLER", 0: "DISCONNECTED", 1: "CONFIRM_SAFETY",
    2: "BOOTING", 3: "POWER_OFF", 4: "POWER_ON", 5: "IDLE",
    6: "BACKDRIVE", 7: "RUNNING", 8: "UPDATING_FIRMWARE",
}

SAFETY_MODE_NAMES: dict[int, str] = {
    0: "NORMAL", 1: "REDUCED", 2: "PROTECTIVE_STOP",
    3: "RECOVERY", 4: "SAFEGUARD_STOP", 5: "SYS_EMERGENCY",
    6: "ROBOT_EMERGENCY", 7: "EMERGENCY", 8: "VIOLATION",
    9: "FAULT", 10: "STOPPED_BY_SAFETY",
}


@dataclass
class URRTDEState:
    """Snapshot of UR robot state from RTDEReceiveInterface."""
    timestamp: float = 0.0
    actual_q: list[float] = field(default_factory=lambda: [0.0] * 6)
    actual_qd: list[float] = field(default_factory=lambda: [0.0] * 6)
    actual_tcp_pose: list[float] = field(default_factory=lambda: [0.0] * 6)
    actual_tcp_speed: list[float] = field(default_factory=lambda: [0.0] * 6)
    actual_tcp_force: list[float] = field(default_factory=lambda: [0.0] * 6)
    robot_mode: int = -1
    safety_mode: int = -1
    is_protective_stopped: bool = False
    is_emergency_stopped: bool = False
    runtime_state: int = 0
    joint_temperatures: list[float] = field(default_factory=lambda: [0.0] * 6)
    actual_main_voltage: float = 0.0
    actual_robot_current: float = 0.0
    speed_scaling_combined: float = 1.0


class URRTDEBridge:
    """Thread-safe wrapper around the three ur_rtde client interfaces and Robotiq gripper."""

    # Robotiq gripper default port (via UR Cap)
    GRIPPER_PORT = 63352

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._rtde_c: Optional[RTDEControlInterface] = None   # type: ignore[name-defined]
        self._rtde_r: Optional[RTDEReceiveInterface] = None   # type: ignore[name-defined]
        self._rtde_io: Optional[RTDEIOInterface] = None       # type: ignore[name-defined]
        self._dash: Optional[DashboardClient] = None          # type: ignore[name-defined]
        self._gripper: Optional[RobotiqGripper] = None
        self.hostname: str = ""
        self._connected: bool = False

    # ---- lifecycle --------------------------------------------------------

    def connect(self, hostname: str, frequency: float = -1.0, connect_gripper: bool = False) -> None:
        """Connect all three RTDE interfaces and the Dashboard client.

        Args:
            hostname: Robot IP address.
            frequency: RTDE frequency (-1 = robot default).
            connect_gripper: Also connect to Robotiq gripper on port 63352.
        """
        if not _HAS_UR_RTDE:
            raise RuntimeError("ur_rtde not installed. Run: pip install ur-rtde")
        with self._lock:
            self.hostname = hostname
            self._rtde_r = RTDEReceiveInterface(hostname, frequency)
            self._rtde_io = RTDEIOInterface(hostname)
            self._rtde_c = RTDEControlInterface(hostname, frequency)
            self._dash = DashboardClient(hostname)
            self._dash.connect()

            # Connect to Robotiq gripper if requested
            if connect_gripper:
                self._gripper = RobotiqGripper()
                self._gripper.connect(hostname, self.GRIPPER_PORT)

            self._connected = True

    def disconnect(self) -> None:
        """Gracefully disconnect all interfaces."""
        with self._lock:
            for client, stop_fn in [
                (self._rtde_c, lambda c: (c.stopScript(), c.disconnect())),
                (self._rtde_r, lambda c: c.disconnect()),
                (self._rtde_io, lambda c: c.disconnect()),
                (self._dash, lambda c: c.disconnect()),
                (self._gripper, lambda c: c.disconnect()),
            ]:
                if client:
                    try:
                        stop_fn(client)
                    except Exception:
                        pass
            self._gripper = None
            self._connected = False

    def is_connected(self) -> bool:
        if not self._connected or not self._rtde_r:
            return False
        try:
            return self._rtde_r.isConnected()
        except Exception:
            return False

    # ---- gripper helpers --------------------------------------------------

    def connect_gripper(self) -> None:
        """Connect to Robotiq gripper (port 63352). Call after connect()."""
        if not self.hostname:
            raise RuntimeError("Robot hostname not set. Call connect() first.")
        with self._lock:
            if self._gripper is None:
                self._gripper = RobotiqGripper()
            self._gripper.connect(self.hostname, self.GRIPPER_PORT)

    def disconnect_gripper(self) -> None:
        """Disconnect gripper only."""
        with self._lock:
            if self._gripper:
                try:
                    self._gripper.disconnect()
                except Exception:
                    pass
                self._gripper = None

    def is_gripper_connected(self) -> bool:
        """Returns True if gripper is connected."""
        if self._gripper is None:
            return False
        return self._gripper.is_connected()

    # ---- safety helpers ---------------------------------------------------

    def require_connected(self) -> None:
        if not self._connected:
            raise RuntimeError("Not connected. Call connect_robot first.")

    def check_safe_to_move(self) -> None:
        """Raise if robot is in protective or emergency stop."""
        if not self._rtde_r:
            return
        if self._rtde_r.isEmergencyStopped():
            raise RuntimeError("Robot is in EMERGENCY STOP. Cannot move.")
        if self._rtde_r.isProtectiveStopped():
            raise RuntimeError("Robot is in PROTECTIVE STOP. Call unlock_protective_stop first.")

    @staticmethod
    def clamp(val: float, lo: float, hi: float, name: str) -> float:
        if not lo <= val <= hi:
            raise ValueError(f"{name}={val} out of range [{lo}, {hi}]")
        return val

    # ---- state snapshot ---------------------------------------------------

    def get_state(self) -> URRTDEState:
        self.require_connected()
        with self._lock:
            r = self._rtde_r
            return URRTDEState(
                timestamp=r.getTimestamp(),
                actual_q=r.getActualQ(),
                actual_qd=r.getActualQd(),
                actual_tcp_pose=r.getActualTCPPose(),
                actual_tcp_speed=r.getActualTCPSpeed(),
                actual_tcp_force=r.getActualTCPForce(),
                robot_mode=r.getRobotMode(),
                safety_mode=r.getSafetyMode(),
                is_protective_stopped=r.isProtectiveStopped(),
                is_emergency_stopped=r.isEmergencyStopped(),
                runtime_state=r.getRuntimeState(),
                joint_temperatures=r.getJointTemperatures(),
                actual_main_voltage=r.getActualMainVoltage(),
                actual_robot_current=r.getActualRobotCurrent(),
                speed_scaling_combined=r.getSpeedScalingCombined(),
            )

    # ---- motion commands ------------------------------------------------

    def moveJ_IK(self, pose: list[float], speed: float = 1.05, acceleration: float = 1.4,
                 asynchronous: bool = False) -> bool:
        """
        Position control: move to Cartesian pose using IK (moveJ_IK)

        Combines get_inverse_kin() with moveJ() for fast, intuitive positioning.
        Faster than moveL, more intuitive than pure moveJ.

        Args:
            pose: [x, y, z, rx, ry, rz] in meters + rotation vector
            speed: joint speed (rad/s), max 3.14
            acceleration: joint acceleration (rad/s²), max 40.0
            asynchronous: if True, returns immediately without waiting
        """
        self.require_connected()
        self.check_safe_to_move()

        with self._lock:
            ok = self._rtde_c.moveJ_IK(pose, speed, acceleration, asynchronous)
            return ok

    def format_state_json(self) -> str:
        """Return get_state() as a formatted JSON string."""
        s = self.get_state()
        return json.dumps({
            "timestamp_s": round(s.timestamp, 3),
            "robot_mode": ROBOT_MODE_NAMES.get(s.robot_mode, str(s.robot_mode)),
            "safety_mode": SAFETY_MODE_NAMES.get(s.safety_mode, str(s.safety_mode)),
            "is_protective_stopped": s.is_protective_stopped,
            "is_emergency_stopped": s.is_emergency_stopped,
            "joint_positions_rad": [round(v, 6) for v in s.actual_q],
            "joint_velocities_rad_s": [round(v, 6) for v in s.actual_qd],
            "tcp_pose_m_rad": [round(v, 6) for v in s.actual_tcp_pose],
            "tcp_speed_m_s": [round(v, 6) for v in s.actual_tcp_speed],
            "tcp_force_N_Nm": [round(v, 4) for v in s.actual_tcp_force],
            "joint_temperatures_C": [round(v, 1) for v in s.joint_temperatures],
            "main_voltage_V": round(s.actual_main_voltage, 2),
            "robot_current_A": round(s.actual_robot_current, 3),
            "speed_scaling": round(s.speed_scaling_combined, 3),
        }, indent=2)
