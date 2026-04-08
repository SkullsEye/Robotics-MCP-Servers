# rosclaw-ur-rtde-mcp

ROSClaw MCP Server for Universal Robots — using **ur_rtde** (RTDE protocol, direct TCP, no ROS2 required). **With Robotiq Gripper support** (port 63352 via UR Cap).

Part of the [ROSClaw](https://github.com/ros-claw) Embodied Intelligence Operating System.

## Features

| Category | Tools |
|----------|-------|
| Connection | `connect_robot`, `disconnect_robot` |
| Robot Lifecycle | `get_robot_info`, `robot_power_control`, `unlock_protective_stop`, `restart_safety` |
| Motion | `move_joint`, `move_linear`, **`move_joint_ik`** (NEW), `stop_motion`, `servo_joint`, `speed_joint` |
| Force Control | `force_mode`, `force_mode_stop`, `zero_ft_sensor` |
| Teaching | `teach_mode`, `jog` |
| Kinematics | `get_inverse_kinematics` |
| Payload | `set_payload` |
| I/O | `set_digital_output`, `set_speed_slider`, `set_analog_output`, `get_digital_io_state` |
| State | `get_robot_state`, `get_tcp_pose`, `get_joint_positions`, `get_tcp_force` |
| **Robotiq Gripper** | `connect_gripper`, `disconnect_gripper`, `gripper_activate`, `gripper_open`, `gripper_close`, `gripper_move`, `gripper_get_status` |

**MCP Resources**: `robot://status`, `robot://connection`

## Hardware

Universal Robots UR3/UR5/UR10/UR16/UR20 (CB3 and e-Series)
Protocol: RTDE over TCP port 30004
Robotiq Gripper: TCP port 63352 (via Robotiq_grippers UR Cap)

## Quick Start

```bash
# Install dependencies
pip install ur-rtde mcp

# Run MCP server
python src/ur_rtde_mcp_server.py

# Or with uv
uv venv --python 3.11
source .venv/bin/activate
uv pip install ur-rtde mcp
python src/ur_rtde_mcp_server.py
```

## Claude Desktop Configuration

Add to `claude_desktop_config.json`:

```json
{
  "mcpServers": {
    "rosclaw-ur-rtde": {
      "command": "python",
      "args": ["/path/to/rosclaw-ur-rtde-mcp/src/ur_rtde_mcp_server.py"],
      "transportType": "stdio"
    }
  }
}
```

## LLM Usage Example — Robot Arm

```
User: Connect to the robot at 192.168.1.100, move to home position, then check TCP force.

LLM calls:
1. connect_robot("192.168.1.100")
2. robot_power_control("brake_release")   # if needed
3. move_joint([0, -1.57, 0, -1.57, 0, 0])  # home position
4. get_tcp_force()
```

## LLM Usage Example — Robotiq Gripper

```
User: Connect to the robot and gripper, then pick up an object.

LLM calls:
1. connect_robot("192.168.1.100")
2. connect_gripper()                     # Connects to same IP, port 63352
3. gripper_activate()                    # Activates + auto-calibrates
4. gripper_open()                        # Open before picking
5. move_joint([...])                     # Move above object
6. gripper_close(speed=128, force=100)   # Close with controlled force
7. move_joint([...])                     # Move to drop position
8. gripper_open()                        # Release object
```

### moveJ_IK — Cartesian Pose Control (NEW)

The `move_joint_ik` tool moves the robot to a Cartesian pose using inverse kinematics (faster than moveL, more intuitive than joint angles).

```python
# Move to specific Cartesian pose [x, y, z, rx, ry, rz]
move_joint_ik(
    pose=[-0.212, 0.319, 0.41, -3.1416, 0, 0],  # meters + rotation vector
    speed=0.5,                                    # rad/s (conservative)
    acceleration=0.8                              # rad/s²
)
```

**Use Case**: When you know the desired TCP position in Cartesian space but don't want to calculate joint angles manually.

## Compatibility

| PolyScope Version | Compatibility | Notes |
|-------------------|---------------|-------|
| 5.6.0+ | ✅ Full | All features supported |
| 3.x - 5.5.x | ✅ Compatible | `get_serial_number` and `is_remote_control` return "N/A" |
| CB3 Series | ✅ Compatible | Tested on PolyScope 3.15.8 |
| e-Series | ✅ Compatible | Recommended for best performance |

## Hardware Requirements

- **Universal Robots**: UR3, UR5, UR10, UR16, UR20 (CB3 or e-Series)
- **Communication**: RTDE over TCP port 30004
- **Optional - Robotiq Gripper**: TCP port 63352 (via Robotiq_grippers UR Cap)
- **Optional - F/T Sensor**: Required for `force_mode` and `zero_ft_sensor`

| Position | Description |
|----------|-------------|
| 0 | Fully open |
| 255 | Fully closed |
| 128 | Halfway |

Use `gripper_move(position=64, speed=255, force=100)` for precise positioning.

## Safety

- All motion commands validate against UR hardware limits before sending:
  - Joint velocity: max 3.14 rad/s
  - Tool velocity: max 3.0 m/s
  - Joint acceleration: max 40 rad/s²
- `check_safe_to_move()` blocks commands during protective/emergency stop
- Protective stop can be unlocked via `unlock_protective_stop()` (after 5 s)
- Gripper auto-calibration opens/closes to detect limits (call after `activate()`)

## Architecture

```
LLM (Claude)
    │ MCP tools (semantic level)
    ▼
ur_rtde_mcp_server.py   (FastMCP, stdio)
    │
ur_rtde_bridge.py       (thread-safe wrapper, threading.Lock)
    ├───────────────────────────────────────┐
    │                                       │
ur_rtde Python bindings              robotiq_gripper.py
    │ TCP port 30004                       │ TCP port 63352
    ▼                                       ▼
UR Robot Controller (RTDE)         Robotiq Gripper (UR Cap)
```

## File Structure

```
rosclaw-ur-rtde-mcp/
├── src/
│   ├── ur_rtde_mcp_server.py   # MCP server with FastMCP (~600 lines)
│   ├── ur_rtde_bridge.py       # Thread-safe bridge for RTDE/IO/Dashboard (~270 lines)
│   └── robotiq_gripper.py      # Robotiq gripper direct TCP control (297 lines)
├── tests/
│   ├── test_ur_rtde_bridge.py  # Unit tests for bridge (15 passed)
│   ├── test_robotiq_gripper.py # Unit tests for gripper (15 passed)
│   ├── system_test_moveJ_IK.py # System test for moveJ_IK feature (hardware required)
│   ├── full_function_test.py   # Complete system test for all MCP tools (hardware required)
│   ├── diagnose_failures.py    # Diagnostic tool for troubleshooting
│   └── failure_analysis_report.md # Analysis of test failures
├── config/
│   └── mcp_config.json         # MCP client configuration
├── pyproject.toml
├── README.md
└── LICENSE
```

## Testing

### Unit Tests (No Hardware Required)

```bash
# Run all unit tests (30 total, no hardware required)
pytest tests/test_ur_rtde_bridge.py tests/test_robotiq_gripper.py -v

# Run specific test file
pytest tests/test_ur_rtde_bridge.py -v
pytest tests/test_robotiq_gripper.py -v
```

### System Tests (Hardware Required)

⚠️ **Warning**: These tests require a real UR robot. Ensure safety before running.

```bash
# Test moveJ_IK feature (requires robot at ROBOT_IP in the script)
python tests/system_test_moveJ_IK.py

# Full functional test of all MCP tools
python tests/full_function_test.py

# Diagnostic tool for troubleshooting
python tests/diagnose_failures.py
```

### Test Reports

System tests generate timestamped reports:
- `ur5_system_test_report_YYYYMMDD_HHMMSS.txt`
- `ur5_full_test_report_YYYYMMDD_HHMMSS.txt`

## Changelog

### v0.2.0 (2025-03-25)
- ✨ **NEW**: Added `moveJ_IK` method to `URRTDEBridge` for Cartesian pose control via IK
- ✨ **NEW**: Added comprehensive system tests for hardware validation
- ✨ **NEW**: Added diagnostic tools for troubleshooting
- 🐛 **FIX**: `get_robot_info` now handles PolyScope < 5.6.0 gracefully
- 📚 **DOC**: Updated README with compatibility matrix and new features

### v0.1.0 (Initial Release)
- Initial MCP server implementation
- Support for UR robots via RTDE protocol
- Robotiq gripper support
- Thread-safe bridge implementation

## Related ROSClaw Servers

- [rosclaw-ur-ros2-mcp](https://github.com/ros-claw/rosclaw-ur-ros2-mcp) — UR5 via ROS2/MoveIt
- [rosclaw-g1-dds-mcp](https://github.com/ros-claw/rosclaw-g1-dds-mcp) — Unitree G1 humanoid
- [rosclaw-gimbal-mcp](https://github.com/ros-claw/rosclaw-gimbal-mcp) — GCU gimbal
- [rosclaw-vision-mcp](https://github.com/ros-claw/rosclaw-vision-mcp) — RealSense RGB-D
