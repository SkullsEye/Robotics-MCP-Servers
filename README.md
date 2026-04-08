# Robotics MCP Servers

A comprehensive collection of **Model Context Protocol (MCP)** servers for multi-robot control and integration. Provides a unified abstraction layer for controlling diverse robotic systems through natural language interfaces.

## Supported Robots

| Server | Robot/Device | Protocol |
|--------|-------------|----------|
| rosclaw-g1-dds-mcp | Unitree G1 Humanoid | DDS |
| rosclaw-ur-ros2-mcp | Universal Robots UR5 | ROS2 |
| rosclaw-ur-rtde-mcp | Universal Robots (all) | RTDE |
| rosclaw-gimbal-mcp | Gimbal Camera System | Serial/USB |
| rosclaw-vision-mcp | Intel RealSense Camera | librealsense |
| ros-mcp-server | Any ROS2 Robot | ROS2 Topics/Services |
| nav2_mcp_server | Nav2 Navigation Stack | ROS2 Actions |

## Features

- Unified MCP interface for heterogeneous robot systems
- Real-time control via DDS, ROS2, and RTDE protocols
- Vision system integration with Intel RealSense
- Navigation and path planning through Nav2
- Gimbal camera control for active perception

## Architecture

```
AI Agent / Natural Language
        |
        v
    MCP Protocol Layer
        |
    +---+---+---+---+---+
    |   |   |   |   |   |
    v   v   v   v   v   v
   G1  UR5 Gimbal Vision Nav2 ...
  (DDS)(RTDE)(Serial)(RS)(ROS2)
```

## Project Structure

```
rosclaw/                    # Core robotics control framework
rosclaw-g1-dds-mcp/         # G1 humanoid DDS interface
rosclaw-ur-ros2-mcp/        # UR5 ROS2 interface
rosclaw-ur-rtde-mcp/        # UR RTDE real-time interface
rosclaw-gimbal-mcp/         # Gimbal camera control
rosclaw-vision-mcp/         # RealSense vision system
rosclaw-examples/           # Usage examples
ros-mcp-server/             # Generic ROS2 MCP bridge
nav2_mcp_server/            # Nav2 navigation MCP
sdk_to_mcp/                 # SDK to MCP converter utility
```

## Prerequisites

- Python 3.10+
- ROS2 Humble (for ROS2-based servers)
- Robot-specific SDKs as needed

## Installation

```bash
git clone https://github.com/SkullsEye/Robotics-MCP-Servers.git
cd Robotics-MCP-Servers

# Install a specific server
cd rosclaw-g1-dds-mcp && pip install -r requirements.txt
```

## Author

**Umar Bin Muzzafar**
B.Tech in Artificial Intelligence and Robotics, Dayananda Sagar University, Bangalore

## License

MIT License. See [LICENSE](LICENSE) for details.
