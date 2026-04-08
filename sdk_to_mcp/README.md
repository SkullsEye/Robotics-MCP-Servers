# SDK-to-MCP: Hardware SDK to MCP Server Converter

<div align="center">

[![Python](https://img.shields.io/badge/Python-3.10+-blue.svg)](https://www.python.org/)
[![MCP](https://img.shields.io/badge/MCP-Compatible-green.svg)](https://modelcontextprotocol.io/)
[![License](https://img.shields.io/badge/License-Apache%202.0-yellow.svg)](LICENSE)

**Transform any hardware SDK or protocol document into a standardized MCP Server**

</div>

## 🎯 Overview

**sdk-to-mcp** is an intelligent automation pipeline that converts heterogeneous hardware interfaces (PDF protocols, C++/Python SDKs, ROS interfaces) into standardized Model Context Protocol (MCP) servers. This enables AI assistants like Claude to control physical robots and devices through natural language.

### Key Features

- 🔌 **Multi-Protocol Support**: Serial, DDS, ROS/ROS2, HTTP/REST
- 🤖 **AI-Powered Analysis**: LLM-driven protocol extraction from PDFs and code
- 🛡️ **Built-in Safety**: Automatic safety guards and limit validation
- ⚡ **Code Generation**: Jinja2-based template engine for MCP server generation
- ✅ **Auto-Validation**: Automatic syntax checking and error correction
- 🔧 **Production Ready**: Full type hints, async support, comprehensive error handling

## 📁 Repository Structure

```
sdk_to_mcp/
├── skills/sdk_to_mcp/          # Core SDK-to-MCP Skill
│   ├── main.py                 # MCP Server entry point
│   ├── SKILL.md                # Skill definition
│   ├── core/                   # Core modules
│   │   ├── analyzer.py         # Protocol analyzer (PDF/Code → JSON)
│   │   ├── generator.py        # Code generator (JSON → Python)
│   │   └── validator.py        # Auto-validation & repair
│   ├── templates/              # Jinja2 templates
│   │   ├── serial_mcp.jinja2   # Serial/Byte-stream protocols
│   │   ├── dds_mcp.jinja2      # DDS distributed communication
│   │   ├── ros_mcp.jinja2      # ROS/ROS2 interfaces
│   │   ├── http_mcp.jinja2     # HTTP/REST APIs
│   │   └── base_mcp.jinja2     # Generic base template
│   └── test_runner.py          # Test suite
│
├── example/                    # Example MCP servers
│   ├── ros-mcp-server/         # Full-featured ROS MCP server
│   ├── ros2-mcp-server/        # ROS 2 native MCP server
│   ├── unitree-go2-mcp-server/ # Unitree Go2 robot MCP
│   └── robot_MCP/              # SO-ARM100/LeKiwi robot MCP
│
├── sdk/                        # Hardware SDKs
│   ├── xianfei/                # GCU Gimbal SDK (Chinese)
│   ├── unitree_sdk2/           # Unitree G1/G02/H1 SDK
│   └── Universal_Robots_ROS2_Driver/  # UR5 arm driver
│
├── generated/                  # Generated MCP servers
│   ├── gcu_gimbal_mcp_server.py  # GCU Gimbal MCP Server
│   └── README.md               # Usage guide
│
└── sdk_to_mcp_core.py          # Legacy core transformation logic
```

## 🚀 Quick Start

### Installation

```bash
# Clone the repository
git clone https://github.com/ros-claw/sdk_to_mcp.git
cd sdk_to_mcp

# Install dependencies (for skill development)
cd skills/sdk_to_mcp
pip install -r requirements.txt

# Or install with optional dependencies
pip install -e ".[dev,llm,serial]"
```

### Using the Skill

```bash
# Run the MCP server
python main.py

# Or with HTTP transport
python main.py --transport http --port 8000
```

### Converting a Hardware SDK

```python
# Through MCP tool
generate_mcp_from_sdk(
    source_path="/path/to/hardware_protocol.pdf",
    hardware_type="serial",  # Options: serial, dds, ros, http
    target_name="my-hardware",
    output_dir="./generated_mcp_servers"
)
```

## 💡 Examples

### Example 1: GCU Gimbal (Serial Protocol)

The GCU gimbal uses a proprietary serial protocol with CRC16 checksum:

```python
generate_mcp_from_sdk(
    source_path="./sdk/xianfei/GCU私有通信协议-XF(A5)V2.0.6.pdf",
    hardware_type="serial",
    target_name="gcu-gimbal"
)
```

Generated MCP Server features:
- Serial connection management (COM8 @ 115200bps)
- Multiple control modes (angle lock, follow, FPV)
- Camera controls (photo, record, zoom, focus)
- Real-time status monitoring
- Automatic safety limits (±150°/s rotation speed)

### Example 2: Unitree Go2 (DDS Protocol)

```python
generate_mcp_from_sdk(
    source_path="./sdk/unitree_sdk2/",
    hardware_type="dds",
    target_name="unitree-go2"
)
```

### Example 3: UR5 Robot (ROS2)

```python
generate_mcp_from_sdk(
    source_path="./sdk/Universal_Robots_ROS2_Driver/",
    hardware_type="ros",
    target_name="ur5-arm"
)
```

## 🏗️ Architecture

The SDK-to-MCP pipeline consists of 4 phases:

```
┌─────────────────────────────────────────────────────────────┐
│  Phase 1: Input                                             │
│  - PDF Protocol Documents                                   │
│  - C++/Python SDK Source Code                               │
│  - ROS Interface Definitions                                │
└──────────────────────┬──────────────────────────────────────┘
                       │
                       ▼
┌─────────────────────────────────────────────────────────────┐
│  Phase 2: Analyze (analyzer.py)                             │
│  - PDF text extraction (pypdf)                              │
│  - Code directory scanning                                  │
│  - LLM-powered semantic extraction (litellm/claude)         │
│  - Output: Structured JSON Schema                           │
└──────────────────────┬──────────────────────────────────────┘
                       │
                       ▼
┌─────────────────────────────────────────────────────────────┐
│  Phase 3: Generate (generator.py)                           │
│  - Jinja2 template selection (serial/dds/ros/http)          │
│  - Code rendering with safety guards                        │
│  - struct.pack/unpack for binary protocols                  │
│  - Output: MCP Server Python Code                           │
└──────────────────────┬──────────────────────────────────────┘
                       │
                       ▼
┌─────────────────────────────────────────────────────────────┐
│  Phase 4: Validate (validator.py)                           │
│  - Syntax checking (py_compile)                             │
│  - Import validation                                        │
│  - Auto-repair (max 3 retries)                              │
│  - OpenClaw config registration                             │
└─────────────────────────────────────────────────────────────┘
```

## 📋 Supported Hardware Types

| Type | Description | Key Features | Example Devices |
|------|-------------|--------------|-----------------|
| `serial` | Serial/Byte-stream protocols | struct.pack/unpack, CRC16/32, baudrate config | GCU Gimbal, Serial robots |
| `dds` | DDS distributed communication | Background daemon threads, state buffering | Unitree G1/Go2, DDS-based robots |
| `ros` | ROS/ROS2 interfaces | Topic pub/sub, Service calls, Action clients | UR5, TurtleBot3, ROS robots |
| `http` | HTTP/REST APIs | aiohttp async requests, JSON handling | Cloud APIs, HTTP-based devices |

## 🛠️ Development

### Running Tests

```bash
cd skills/sdk_to_mcp
python test_runner.py
```

### Adding a New Template

1. Create `templates/my_protocol.jinja2`
2. Implement protocol-specific logic
3. Add template selection in `generator.py`
4. Run tests to validate

### Project Structure for Generated Servers

```python
generated_mcp_servers/
└── {target_name}_server.py
    ├── {TargetName}Bridge          # Hardware communication bridge
    │   ├── connect()/disconnect()   # Connection management
    │   ├── _validate_safety()       # Safety limit checks
    │   └── protocol-specific methods
    │
    ├── @mcp.tool() functions       # MCP Tools (actions)
    ├── @mcp.resource() functions   # MCP Resources (states)
    └── main()                       # Entry point
```

## 🔒 Safety Features

All generated MCP servers include:

- **Safety Guards**: Hard limits on velocity, angle, position
- **Connection Validation**: Prevents commands when disconnected
- **CRC/Checksum**: Automatic verification for binary protocols
- **Exception Handling**: Graceful error messages to LLM
- **Async Design**: Non-blocking state reads

## 📝 Configuration

### MCP Client Configuration (Claude Desktop)

```json
{
  "mcpServers": {
    "gcu-gimbal": {
      "command": "python",
      "args": ["C:/path/to/gcu_gimbal_mcp_server.py"],
      "transportType": "stdio"
    }
  }
}
```

Configuration file locations:
- **Windows**: `%APPDATA%/Claude/claude_desktop_config.json`
- **Mac**: `~/Library/Application Support/Claude/claude_desktop_config.json`
- **Linux**: `~/.config/Claude/claude_desktop_config.json`

## 🔧 Troubleshooting

### Common Issues

1. **Serial port not found**
   - Check COM port number in Device Manager
   - Verify port not used by other applications

2. **Permission denied**
   - Run with administrator/root privileges
   - Add user to `dialout` group (Linux)

3. **Import errors**
   - Install missing dependencies: `pip install mcp pyserial`

4. **Protocol mismatch**
   - Verify protocol version matches hardware
   - Check baudrate settings

## 🤝 Contributing

We welcome contributions! Please see our [Contributing Guide](CONTRIBUTING.md) for details.

### Areas for Contribution

- New protocol templates (CAN, Modbus, MQTT, etc.)
- Additional hardware examples
- Improved LLM prompting strategies
- Better error handling and recovery
- Documentation improvements

## 📄 License

This project is licensed under the Apache License 2.0 - see the [LICENSE](LICENSE) file for details.

## 🙏 Acknowledgments

- [Model Context Protocol](https://modelcontextprotocol.io/) - The MCP standard
- [FastMCP](https://github.com/jlowin/fastmcp) - Python MCP framework
- [ROS](https://ros.org/) - Robot Operating System
- [Unitree](https://unitree.com/) - Robot hardware platforms

## 📞 Support

- **Issues**: [GitHub Issues](https://github.com/ros-claw/sdk_to_mcp/issues)
- **Discussions**: [GitHub Discussions](https://github.com/ros-claw/sdk_to_mcp/discussions)

---

<div align="center">

**Made with ❤️ for the ROS and AI communities**

</div>
