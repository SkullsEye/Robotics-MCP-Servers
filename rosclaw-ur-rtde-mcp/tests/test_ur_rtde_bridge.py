"""
Unit tests for URRTDEBridge (no hardware required).
"""

from __future__ import annotations

import json
import sys
import os
import pytest
from unittest.mock import MagicMock, patch

sys.path.insert(0, str(os.path.join(os.path.dirname(__file__), "..", "src")))

from ur_rtde_bridge import URRTDEBridge, URRTDEState, JOINT_VEL_MAX, TOOL_VEL_MAX


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def make_bridge_with_mocks():
    """Return a URRTDEBridge with all SDK clients mocked."""
    bridge = URRTDEBridge()
    bridge._rtde_r = MagicMock()
    bridge._rtde_c = MagicMock()
    bridge._rtde_io = MagicMock()
    bridge._dash = MagicMock()
    bridge._connected = True
    bridge.hostname = "192.168.1.100"
    return bridge


# ---------------------------------------------------------------------------
# Safety helpers
# ---------------------------------------------------------------------------

class TestClamp:
    def test_within_bounds(self):
        assert URRTDEBridge.clamp(1.0, 0.0, 2.0, "x") == 1.0

    def test_at_lower_bound(self):
        assert URRTDEBridge.clamp(0.0, 0.0, 1.0, "x") == 0.0

    def test_at_upper_bound(self):
        assert URRTDEBridge.clamp(3.14, 0.0, 3.14, "x") == 3.14

    def test_below_lower_raises(self):
        with pytest.raises(ValueError, match="speed="):
            URRTDEBridge.clamp(-0.1, 0.0, 3.14, "speed")

    def test_above_upper_raises(self):
        with pytest.raises(ValueError, match="speed="):
            URRTDEBridge.clamp(9.0, 0.0, JOINT_VEL_MAX, "speed")


class TestSafetyChecks:
    def test_check_safe_when_ok(self):
        bridge = make_bridge_with_mocks()
        bridge._rtde_r.isEmergencyStopped.return_value = False
        bridge._rtde_r.isProtectiveStopped.return_value = False
        bridge.check_safe_to_move()  # should not raise

    def test_check_safe_emergency_stopped(self):
        bridge = make_bridge_with_mocks()
        bridge._rtde_r.isEmergencyStopped.return_value = True
        bridge._rtde_r.isProtectiveStopped.return_value = False
        with pytest.raises(RuntimeError, match="EMERGENCY STOP"):
            bridge.check_safe_to_move()

    def test_check_safe_protective_stopped(self):
        bridge = make_bridge_with_mocks()
        bridge._rtde_r.isEmergencyStopped.return_value = False
        bridge._rtde_r.isProtectiveStopped.return_value = True
        with pytest.raises(RuntimeError, match="PROTECTIVE STOP"):
            bridge.check_safe_to_move()

    def test_require_connected_raises_when_not_connected(self):
        bridge = URRTDEBridge()
        with pytest.raises(RuntimeError, match="Not connected"):
            bridge.require_connected()

    def test_require_connected_ok_when_connected(self):
        bridge = make_bridge_with_mocks()
        bridge.require_connected()  # should not raise


# ---------------------------------------------------------------------------
# State snapshot
# ---------------------------------------------------------------------------

class TestGetState:
    def test_get_state_returns_correct_values(self):
        bridge = make_bridge_with_mocks()
        r = bridge._rtde_r
        r.getTimestamp.return_value = 123.45
        r.getActualQ.return_value = [0.1, 0.2, 0.3, 0.4, 0.5, 0.6]
        r.getActualQd.return_value = [0.0] * 6
        r.getActualTCPPose.return_value = [0.3, 0.0, 0.5, 0.0, 3.14, 0.0]
        r.getActualTCPSpeed.return_value = [0.0] * 6
        r.getActualTCPForce.return_value = [1.0, 2.0, 3.0, 0.1, 0.2, 0.3]
        r.getRobotMode.return_value = 7
        r.getSafetyMode.return_value = 0
        r.isProtectiveStopped.return_value = False
        r.isEmergencyStopped.return_value = False
        r.getRuntimeState.return_value = 2
        r.getJointTemperatures.return_value = [30.0] * 6
        r.getActualMainVoltage.return_value = 48.0
        r.getActualRobotCurrent.return_value = 2.5
        r.getSpeedScalingCombined.return_value = 1.0

        state = bridge.get_state()
        assert state.timestamp == 123.45
        assert state.robot_mode == 7
        assert state.safety_mode == 0
        assert not state.is_protective_stopped
        assert state.actual_q[0] == pytest.approx(0.1)
        assert state.actual_tcp_force[0] == pytest.approx(1.0)

    def test_format_state_json_valid(self):
        bridge = make_bridge_with_mocks()
        r = bridge._rtde_r
        r.getTimestamp.return_value = 0.0
        r.getActualQ.return_value = [0.0] * 6
        r.getActualQd.return_value = [0.0] * 6
        r.getActualTCPPose.return_value = [0.0] * 6
        r.getActualTCPSpeed.return_value = [0.0] * 6
        r.getActualTCPForce.return_value = [0.0] * 6
        r.getRobotMode.return_value = 7
        r.getSafetyMode.return_value = 0
        r.isProtectiveStopped.return_value = False
        r.isEmergencyStopped.return_value = False
        r.getRuntimeState.return_value = 2
        r.getJointTemperatures.return_value = [25.0] * 6
        r.getActualMainVoltage.return_value = 48.0
        r.getActualRobotCurrent.return_value = 1.0
        r.getSpeedScalingCombined.return_value = 1.0

        result = json.loads(bridge.format_state_json())
        assert result["robot_mode"] == "RUNNING"
        assert result["safety_mode"] == "NORMAL"
        assert len(result["joint_positions_rad"]) == 6
        assert result["speed_scaling"] == 1.0


# ---------------------------------------------------------------------------
# is_connected
# ---------------------------------------------------------------------------

class TestIsConnected:
    def test_is_connected_when_bridge_initialized(self):
        bridge = make_bridge_with_mocks()
        bridge._rtde_r.isConnected.return_value = True
        assert bridge.is_connected() is True

    def test_not_connected_when_flag_false(self):
        bridge = URRTDEBridge()
        assert bridge.is_connected() is False

    def test_not_connected_when_rtde_r_raises(self):
        bridge = make_bridge_with_mocks()
        bridge._rtde_r.isConnected.side_effect = Exception("socket closed")
        assert bridge.is_connected() is False
