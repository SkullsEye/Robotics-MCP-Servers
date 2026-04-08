# GCU 云台 MCP Server 使用说明

## 设备配置

- **串口号**: COM8
- **波特率**: 115200
- **协议**: GCU 私有通信协议 V2.0.6

## 快速开始

### 1. 安装依赖

```bash
pip install mcp pyserial
```

### 2. 配置 MCP Client

在 Claude Desktop 或 Cline 的 MCP 配置中添加：

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

### 3. 启动服务器

```bash
python gcu_gimbal_mcp_server.py
```

## 可用的 MCP Tools

### 连接管理

| Tool | 描述 |
|------|------|
| `connect_gimbal(port="COM8")` | 连接云台设备 |
| `disconnect_gimbal()` | 断开连接 |

### 工作模式

| Tool | 描述 |
|------|------|
| `set_mode(mode)` | 设置工作模式 (angle_lock/follow/euler/fpv/top_down) |

### 运动控制

| Tool | 描述 |
|------|------|
| `rotate(pitch_speed, yaw_speed, duration)` | 旋转云台（角速度模式） |
| `set_euler_angles(roll, pitch, yaw)` | 设置欧拉角 |
| `stop_rotation()` | 停止旋转 |
| `reset_gimbal()` | 云台回中 |
| `calibrate_gimbal()` | 云台校准 |

### 相机控制

| Tool | 描述 |
|------|------|
| `take_photo(camera=1)` | 拍照 |
| `toggle_record(camera=1)` | 开始/停止录像 |
| `zoom(direction, camera=1)` | 变焦 (in/out/stop) |
| `set_zoom_level(level, camera=1)` | 设置变焦倍率 |
| `focus(camera=1)` | 聚焦 |

### 高级功能

| Tool | 描述 |
|------|------|
| `set_night_vision(enabled)` | 夜视模式 |
| `set_osd(enabled)` | OSD 显示 |
| `set_illumination(brightness)` | 补光亮度 [0-255] |
| `set_ranging(enabled)` | 连续测距 |
| `demo_scan()` | 扫描演示 |

## 可用的 MCP Resources

| Resource | 描述 |
|----------|------|
| `gimbal://status` | 获取云台当前状态 |
| `gimbal://connection` | 获取连接状态 |

## 使用示例

### 1. 连接设备

```
请连接 GCU 云台设备
```

### 2. 设置工作模式并旋转

```
将云台设置为指向锁定模式，然后向上旋转30度/秒，持续2秒
```

### 3. 拍照并变焦

```
拍摄一张照片，然后将可见光相机放大到5倍
```

### 4. 执行扫描演示

```
执行云台扫描演示动作
```

### 5. 获取状态

```
获取当前云台状态和角度信息
```

## 安全限位

- 俯仰速度: ±150°/s
- 偏航速度: ±150°/s
- 欧拉角: ±180°

所有运动命令都会自动检查安全限位，超出范围的命令会被拒绝。

## 故障排除

### 无法连接设备
- 检查 COM8 是否被其他程序占用
- 确认设备已正确连接
- 检查波特率设置

### 命令无响应
- 确认设备已通电
- 检查协议版本是否匹配
- 尝试重新连接

### 旋转不停止
- 使用 `stop_rotation()` 工具停止
- 或使用 `reset_gimbal()` 回中
