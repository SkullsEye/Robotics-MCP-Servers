# rosclaw-gimbal-mcp

ROSClaw MCP Server for **GCU Camera Gimbal** via Serial protocol.

Part of the [ROSClaw](https://github.com/ros-claw) Embodied Intelligence Operating System.

## Overview

This MCP server enables LLM agents to control a GCU (Gimbal Control Unit) camera gimbal through the Model Context Protocol. It communicates via RS-232/USB serial using the GCU binary protocol with CRC-16 error checking.

```
LLM Agent  ──MCP──►  rosclaw-gimbal-mcp  ──Serial──►  GCU Gimbal
```

## Features

- **3-axis control**: Yaw, pitch, roll with degree precision
- **Multiple modes**: Stabilization, follow, FPV, lock
- **Camera control**: Photo capture, video record, zoom, focus
- **Night vision**: IR mode with multiple sensitivity levels
- **Real-time feedback**: Position, temperature, velocity at 50Hz
- **CRC-16 checksum**: Reliable serial communication
- **Auto-reconnect**: Handles serial disconnections gracefully

## Hardware

| Field | Value |
|-------|-------|
| Device | GCU Camera Gimbal |
| Protocol | Binary Serial (RS-232 / USB-Serial) |
| Baud Rate | 115200 bps |
| Data Bits | 8 |
| Stop Bits | 1 |
| Parity | None |
| Header | `0xA8 0xE5` (send) / `0xA8 0xE4` (receive) |
| Checksum | CRC-16 |

## Installation

```bash
# Clone
git clone https://github.com/ros-claw/rosclaw-gimbal-mcp.git
cd rosclaw-gimbal-mcp

# Install with uv (recommended)
uv venv --python python3.10
source .venv/bin/activate
uv pip install -e .

# Or with pip
pip install -e .
```

### Serial Port Permissions (Linux)

```bash
sudo usermod -a -G dialout $USER
# Log out and back in
```

## Quick Start

### Run as MCP Server

```bash
# stdio transport
python src/gimbal_mcp_server.py
```

### Claude Desktop Configuration

```json
{
  "mcpServers": {
    "rosclaw-gimbal": {
      "command": "python",
      "args": ["/path/to/rosclaw-gimbal-mcp/src/gimbal_mcp_server.py"],
      "transportType": "stdio"
    }
  }
}
```

### MCP Inspector (Testing)

```bash
mcp dev src/gimbal_mcp_server.py
```

## Available Tools

| Tool | Description |
|------|-------------|
| `connect_gimbal` | Connect to gimbal on serial port |
| `disconnect_gimbal` | Disconnect from gimbal |
| `set_gimbal_mode` | Set control mode (stabilize/follow/fpv/lock) |
| `rotate_gimbal` | Rotate with angular velocity |
| `set_euler_angles` | Go to absolute yaw/pitch/roll |
| `center_gimbal` | Return to center position |
| `take_photo` | Trigger camera shutter |
| `start_recording` | Start video recording |
| `stop_recording` | Stop video recording |
| `set_zoom` | Set optical/digital zoom level |
| `set_focus` | Set focus mode (auto/manual) |
| `set_night_vision` | Enable IR/night vision mode |
| `get_gimbal_status` | Get current angles and state |
| `set_stabilization` | Enable/disable stabilization |

## Available Resources

| Resource | Description |
|----------|-------------|
| `gimbal://status` | Current angles, mode, temperature |
| `gimbal://capabilities` | Hardware specs, limits |
| `gimbal://connection` | Serial connection status |

## Angle Limits

| Axis | Range |
|------|-------|
| Yaw | -170° to +170° |
| Pitch | -90° to +30° |
| Roll | -45° to +45° |

## Modes

| Mode | Description |
|------|-------------|
| `stabilize` | Hold absolute orientation |
| `follow` | Follow vehicle heading |
| `fpv` | First-person view (locked to vehicle) |
| `lock` | Lock to current position |

## Serial Protocol

The GCU protocol uses binary frames:

```
[0xA8][0xE5][LEN][CMD][PARAMS...][CRC16_L][CRC16_H]
```

Commands implemented:
- `0x01` — Control mode
- `0x02` — Euler angle target
- `0x03` — Angular velocity
- `0x04` — Camera shutter
- `0x05` — Video record
- `0x06` — Zoom control
- `0x07` — Focus control
- `0x08` — Night vision / IR
- `0x09` — Center gimbal
- `0x10` — Query state

## Finding Your Serial Port

```bash
# Linux
ls /dev/ttyUSB* /dev/ttyACM*

# Windows
# Use Device Manager → Ports (COM & LPT)
# Common: COM3, COM4, COM8

# macOS
ls /dev/tty.usbserial-*
```

## Dependencies

- Python 3.10+
- `mcp[fastmcp]` — MCP framework
- `pyserial` — Serial communication

## Architecture

```
gimbal_mcp_server.py
├── GimbalState      — Gimbal state dataclass
├── GimbalSerialBridge — Serial communication bridge
│   ├── connect()    — Open serial port
│   ├── _calculate_crc16() — CRC-16 checksum
│   ├── _build_control_packet() — Build binary frame
│   ├── _send_command() — Thread-safe send
│   └── _read_thread() — Background 50Hz reader
└── MCP Tools        — FastMCP tool definitions
```

## License

MIT License — See [LICENSE](LICENSE)

## Part of ROSClaw

- [rosclaw-g1-dds-mcp](https://github.com/ros-claw/rosclaw-g1-dds-mcp) — Unitree G1 (DDS)
- [rosclaw-ur-ros2-mcp](https://github.com/ros-claw/rosclaw-ur-ros2-mcp) — UR5 arm (ROS2)
- [rosclaw-gimbal-mcp](https://github.com/ros-claw/rosclaw-gimbal-mcp) — GCU Gimbal (Serial)
