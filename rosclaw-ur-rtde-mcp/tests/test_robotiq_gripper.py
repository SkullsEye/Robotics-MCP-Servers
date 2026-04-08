"""
Unit tests for RobotiqGripper (no hardware required).
"""

from __future__ import annotations

import sys
import os
import pytest
from unittest.mock import MagicMock, patch, call
from collections import OrderedDict

sys.path.insert(0, str(os.path.join(os.path.dirname(__file__), "..", "src")))

from robotiq_gripper import RobotiqGripper


class TestRobotiqGripperInitialization:
    def test_default_init(self):
        g = RobotiqGripper()
        assert g._min_position == 0
        assert g._max_position == 255
        assert g._min_speed == 0
        assert g._max_speed == 255
        assert g._min_force == 0
        assert g._max_force == 255
        assert g.socket is None


class TestRobotiqGripperConnection:
    @patch("robotiq_gripper.socket.socket")
    def test_connect_sets_socket(self, mock_socket_class):
        mock_socket = MagicMock()
        mock_socket_class.return_value = mock_socket

        g = RobotiqGripper()
        g.connect("192.168.1.100", 63352)

        assert g._hostname == "192.168.1.100"
        assert g._port == 63352
        mock_socket.connect.assert_called_once_with(("192.168.1.100", 63352))
        mock_socket.settimeout.assert_called_once_with(2.0)

    @patch("robotiq_gripper.socket.socket")
    def test_is_connected_true(self, mock_socket_class):
        mock_socket = MagicMock()
        mock_socket.getpeername.return_value = ("192.168.1.100", 63352)
        mock_socket_class.return_value = mock_socket

        g = RobotiqGripper()
        g.connect("192.168.1.100")
        assert g.is_connected() is True

    def test_is_connected_no_socket(self):
        g = RobotiqGripper()
        assert g.is_connected() is False

    @patch("robotiq_gripper.socket.socket")
    def test_disconnect(self, mock_socket_class):
        mock_socket = MagicMock()
        mock_socket_class.return_value = mock_socket

        g = RobotiqGripper()
        g.connect("192.168.1.100")
        g.disconnect()

        mock_socket.close.assert_called_once()
        assert g.socket is None


class TestRobotiqGripperLowLevelCommands:
    @patch("robotiq_gripper.socket.socket")
    def test_set_var_sends_correct_command(self, mock_socket_class):
        mock_socket = MagicMock()
        mock_socket.recv.return_value = b"ack"
        mock_socket_class.return_value = mock_socket

        g = RobotiqGripper()
        g.connect("192.168.1.100")
        result = g._set_var(g.POS, 128)

        mock_socket.sendall.assert_called_once_with(b"SET POS 128\n")
        assert result is True

    @patch("robotiq_gripper.socket.socket")
    def test_get_var_parses_response(self, mock_socket_class):
        mock_socket = MagicMock()
        mock_socket.recv.return_value = b"POS 128"
        mock_socket_class.return_value = mock_socket

        g = RobotiqGripper()
        g.connect("192.168.1.100")
        result = g._get_var(g.POS)

        mock_socket.sendall.assert_called_once_with(b"GET POS\n")
        assert result == 128

    @patch("robotiq_gripper.socket.socket")
    def test_get_var_wrong_variable_raises(self, mock_socket_class):
        mock_socket = MagicMock()
        mock_socket.recv.return_value = b"STA 3"  # Wrong variable returned
        mock_socket_class.return_value = mock_socket

        g = RobotiqGripper()
        g.connect("192.168.1.100")
        with pytest.raises(ValueError, match="Unexpected response"):
            g._get_var(g.POS)


class TestRobotiqGripperPositionQueries:
    @patch("robotiq_gripper.socket.socket")
    def test_get_current_position(self, mock_socket_class):
        mock_socket = MagicMock()
        mock_socket.recv.return_value = b"POS 100"
        mock_socket_class.return_value = mock_socket

        g = RobotiqGripper()
        g.connect("192.168.1.100")
        result = g.get_current_position()

        assert result == 100

    def test_is_open_true(self):
        g = RobotiqGripper()
        g._min_position = 0
        with patch.object(g, "get_current_position", return_value=0):
            assert g.is_open() is True

    def test_is_open_false(self):
        g = RobotiqGripper()
        g._min_position = 0
        with patch.object(g, "get_current_position", return_value=128):
            assert g.is_open() is False

    def test_is_closed_true(self):
        g = RobotiqGripper()
        g._max_position = 255
        with patch.object(g, "get_current_position", return_value=255):
            assert g.is_closed() is True


class TestRobotiqGripperMove:
    @patch("robotiq_gripper.socket.socket")
    def test_move_clips_values(self, mock_socket_class):
        mock_socket = MagicMock()
        mock_socket.recv.return_value = b"ack"
        mock_socket_class.return_value = mock_socket

        g = RobotiqGripper()
        g.connect("192.168.1.100")

        # Set calibrated range
        g._min_position = 10
        g._max_position = 240

        # Request outside range should be clipped
        result, cmd_pos = g.move(300, 500, -50)
        assert cmd_pos == 240  # Clipped to max

    @patch("robotiq_gripper.socket.socket")
    def test_move_and_wait(self, mock_socket_class):
        mock_socket = MagicMock()
        # First ack for move, then PRE values, then OBJ, then POS
        mock_socket.recv.side_effect = [
            b"ack",       # set_vars response
            b"PRE 128",   # PRE check
            b"OBJ 3",     # OBJ check (AT_DEST)
            b"POS 127",   # Final position
        ]
        mock_socket_class.return_value = mock_socket

        g = RobotiqGripper()
        g.connect("192.168.1.100")

        final_pos, status = g.move_and_wait_for_pos(128, 255, 255)

        assert final_pos == 127
        assert status == RobotiqGripper.ObjectStatus.AT_DEST


class TestRobotiqGripperStatus:
    @patch("robotiq_gripper.socket.socket")
    def test_get_status(self, mock_socket_class):
        mock_socket = MagicMock()
        mock_socket.recv.side_effect = [
            b"POS 128",
            b"STA 3",
            b"OBJ 3",
            b"PRE 128",
            b"FLT 0",
        ]
        mock_socket_class.return_value = mock_socket

        g = RobotiqGripper()
        g.connect("192.168.1.100")
        g._min_position = 0
        g._max_position = 255

        status = g.get_status()

        assert status["position"] == 128
        assert status["status"] == "ACTIVE"
        assert status["object_status"] == "AT_DEST"
        assert status["fault"] == 0
