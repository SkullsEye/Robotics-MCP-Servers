# rosclaw-ur-ros2-mcp

ROSClaw MCP Server for **Universal Robots UR5/UR5e Robotic Arm** via ROS2.

Part of the [ROSClaw](https://github.com/ros-claw) Embodied Intelligence Operating System.

## Overview

This MCP server enables LLM agents to control a UR5/UR5e robotic arm through the Model Context Protocol. It integrates with ROS2 (Robot Operating System 2) and supports MoveIt 2 for motion planning.

```
LLM Agent  ──MCP──►  rosclaw-ur-ros2-mcp  ──ROS2──►  UR5/UR5e
```

## Features

- **6-DOF joint control**: Direct joint position commands
- **Cartesian control**: End-effector pose (with MoveIt IK)
- **Gripper control**: Open/close with force control
- **Pick & Place**: High-level pick and place sequences
- **Safety guards**: Joint limits and workspace limits enforced
- **ROS2 integration**: Native rclpy node, subscribes to `/joint_states`
- **Async design**: Background ROS2 spin thread

## Hardware

| Field | Value |
|-------|-------|
| Robot | Universal Robots UR5 / UR5e |
| Protocol | ROS2 (Robot Operating System 2) |
| ROS2 Version | Humble / Jazzy |
| Joint Topics | `/joint_states`, `/joint_group_position_controller/commands` |
| Motion | MoveIt 2 (IK/trajectory planning) |

## Installation

```bash
# Requires ROS2 Humble or Jazzy
source /opt/ros/humble/setup.bash

# Clone
git clone https://github.com/ros-claw/rosclaw-ur-ros2-mcp.git
cd rosclaw-ur-ros2-mcp

# Install with uv (recommended)
uv venv --python /usr/bin/python3.10
source .venv/bin/activate
uv pip install -e .

# Or with pip
pip install -e .
```

## Quick Start

### Prerequisites

```bash
# Install UR ROS2 driver
sudo apt install ros-humble-ur

# Launch UR5 driver (real robot)
ros2 launch ur_robot_driver ur_control.launch.py \
  ur_type:=ur5 robot_ip:=192.168.1.102

# Or simulation
ros2 launch ur_simulation_gazebo ur_sim_control.launch.py
```

### Run as MCP Server

```bash
source /opt/ros/humble/setup.bash
python src/ur_mcp_server.py
```

### Claude Desktop Configuration

```json
{
  "mcpServers": {
    "rosclaw-ur": {
      "command": "bash",
      "args": ["-c", "source /opt/ros/humble/setup.bash && python /path/to/rosclaw-ur-ros2-mcp/src/ur_mcp_server.py"],
      "transportType": "stdio"
    }
  }
}
```

## Available Tools

| Tool | Description |
|------|-------------|
| `connect_ur5` | Connect to UR5 via ROS2 |
| `disconnect_ur5` | Disconnect from UR5 |
| `move_to_home` | Move to safe home position |
| `move_joint` | Move a single joint |
| `move_joints` | Move multiple joints simultaneously |
| `move_ee_to_pose` | Move end-effector to Cartesian pose |
| `close_gripper` | Close gripper with force control |
| `open_gripper` | Open gripper |
| `pick_object` | Pick object at specified position |
| `place_object` | Place object at specified position |
| `stop_motion` | Stop all robot motion |

## Available Resources

| Resource | Description |
|----------|-------------|
| `ur5://status` | Joint positions, velocities, EE pose |
| `ur5://joints` | Joint limits and velocity limits |
| `ur5://workspace` | Workspace (cylinder) limits |
| `ur5://connection` | ROS2 connection status |

## Joint Reference

| Joint | Range | Max Velocity |
|-------|-------|-------------|
| shoulder_pan_joint | ±360° | 3.15 rad/s |
| shoulder_lift_joint | ±360° | 3.15 rad/s |
| elbow_joint | ±180° | 3.15 rad/s |
| wrist_1_joint | ±360° | 6.28 rad/s |
| wrist_2_joint | ±360° | 6.28 rad/s |
| wrist_3_joint | ±360° | 6.28 rad/s |

## Workspace

```
Cylindrical workspace:
  Radius: 0.0 → 0.85 m
  Height: 0.05 → 1.2 m
```

## Home Position

```python
HOME = {
    "shoulder_pan_joint":  0.0,
    "shoulder_lift_joint": -1.57,  # -90°
    "elbow_joint":         0.0,
    "wrist_1_joint":       -1.57,
    "wrist_2_joint":       0.0,
    "wrist_3_joint":       0.0,
}
```

## Dependencies

- Python 3.10+
- ROS2 Humble or Jazzy
- `mcp[fastmcp]` — MCP framework
- `rclpy` — ROS2 Python client library
- `geometry_msgs`, `sensor_msgs`, `std_msgs` — ROS2 message types
- `moveit_msgs` — MoveIt 2 (for IK planning)

## Architecture

```
ur_mcp_server.py
├── UR5State         — Robot state dataclass
├── UR5ROS2Bridge    — ROS2 Node (extends rclpy.Node)
│   ├── _joint_state_callback() — /joint_states subscriber
│   ├── publish_joint_command() — /joint_group_position_controller publisher
│   ├── _validate_joint_positions() — Safety check
│   └── _validate_cartesian_pose() — Workspace check
└── MCP Tools        — FastMCP tool definitions
```

## License

MIT License — See [LICENSE](LICENSE)

## Part of ROSClaw

- [rosclaw-g1-dds-mcp](https://github.com/ros-claw/rosclaw-g1-dds-mcp) — Unitree G1 (DDS)
- [rosclaw-ur-ros2-mcp](https://github.com/ros-claw/rosclaw-ur-ros2-mcp) — UR5 arm (ROS2)
- [rosclaw-gimbal-mcp](https://github.com/ros-claw/rosclaw-gimbal-mcp) — GCU Gimbal (Serial)
