# GCU 云台控制脚本

基于先飞机器人 GCU 私有通信协议 V2.0.6 的 Python 控制脚本。

## 功能特性

- 串口通信控制云台旋转
- 支持多种控制模式（角度控制、指向锁定、指向跟随、欧拉角控制、FPV模式）
- 相机控制（拍照、录像、变焦、聚焦等）
- 支持持续发送（30-50Hz）
- 提供演示模式和交互式控制

## 硬件要求

- 串口：COM4（可修改）
- 波特率：115200（支持 115200/250000/500000/1000000）
- 数据位：8
- 停止位：1
- 校验位：无

## 快速开始

### 1. 安装依赖

```bash
pip install pyserial
```

### 2. 运行演示

```bash
# 运行旋转控制演示
python gcu_gimbal_control.py demo

# 运行相机控制演示
python gcu_gimbal_control.py camera

# 交互式控制
python gcu_gimbal_control.py interactive
```

## 使用示例

### 基础控制

```python
from gcu_gimbal_control import GCUGimbalController

# 创建控制器
gimbal = GCUGimbalController(port='COM4', baudrate=115200)

# 连接串口
gimbal.connect()

# 切换到指向锁定模式
gimbal.set_mode_pointing_lock()

# 向上旋转（俯仰 +50°/s）
gimbal.rotate_pitch(500)  # 500 = 50.0°/s
gimbal.send_packet()
time.sleep(2)

# 向右旋转（偏航 +30°/s）
gimbal.rotate_yaw(300)  # 300 = 30.0°/s
gimbal.send_packet()
time.sleep(2)

# 停止旋转
gimbal.stop_rotation()
gimbal.send_packet()

# 回中
gimbal.reset_gimbal()

# 断开连接
gimbal.disconnect()
```

### 持续发送模式（推荐）

```python
# 切换到指向锁定模式
gimbal.set_mode_pointing_lock()

# 启动持续发送（30Hz）
gimbal.start_sending(frequency=30)

# 控制旋转（会自动持续发送）
gimbal.rotate_pitch(500)   # 向上旋转
time.sleep(2)

gimbal.rotate_yaw(300)     # 向右旋转
time.sleep(2)

gimbal.stop_rotation()     # 停止旋转

# 停止持续发送
gimbal.stop_sending()
```

### 欧拉角控制模式

```python
# 切换到欧拉角控制模式
gimbal.set_mode_euler_angle()

# 启动持续发送
gimbal.start_sending(frequency=30)

# 设置期望角度（滚转0°，俯仰45°，偏航0°）
gimbal.set_euler_angles(roll=0, pitch=45, yaw=0)
```

## 控制量说明

### 角速度模式（指向锁定/指向跟随模式）

| 控制量 | 范围 | 分辨率 | 说明 |
|--------|------|--------|------|
| 俯仰 | [-1500, 1500] | 0.1°/s | 正值向上，负值向下 |
| 偏航 | [-1500, 1500] | 0.1°/s | 正值向右，负值向左 |

### 欧拉角模式

| 控制量 | 范围 | 分辨率 | 说明 |
|--------|------|--------|------|
| 滚转 | [-18000, 18000] | 0.01° | 期望滚转角 |
| 俯仰 | [-18000, 18000] | 0.01° | 期望俯仰角 |
| 偏航 | [-18000, 18000] | 0.01° | 期望偏航角 |

## 交互式命令

运行 `python gcu_gimbal_control.py interactive` 进入交互模式：

```
> mode lock          # 切换到指向锁定模式
> pitch 50           # 俯仰速度 50°/s（向上）
> yaw 30             # 偏航速度 30°/s（向右）
> rotate 30 20       # 同时设置俯仰30°/s，偏航20°/s
> stop               # 停止旋转
> reset              # 回中
> photo              # 拍照
> record             # 开始/停止录像
> zoom in            # 放大
> zoom out           # 缩小
> zoom 5             # 设置5倍变焦
> q                  # 退出
```

## 相机控制命令

| 命令 | 说明 |
|------|------|
| `take_photo(camera)` | 拍照 |
| `toggle_record(camera)` | 开始/停止录像 |
| `zoom_in(camera)` | 连续放大 |
| `zoom_out(camera)` | 连续缩小 |
| `zoom_stop(camera)` | 停止变倍 |
| `set_zoom(value, camera)` | 设置指定倍率 |
| `focus(camera)` | 聚焦 |
| `set_night_vision(enabled)` | 夜视开关 |
| `set_osd(enabled)` | OSD显示开关 |
| `set_illumination(brightness)` | 补光亮度 [0-255] |
| `set_ranging(enabled)` | 连续测距开关 |

## 注意事项

1. **通信频率**：建议 30-50Hz，频率过低或停发会导致云台失控
2. **控制量有效性**：状态标志 B2 位为 1 时控制量有效，为 0 时期望角速率为 0
3. **模式切换**：不同模式下控制量的定义不同，请参考协议文档
4. **回中命令**：在角度控制、欧拉角控制、凝视模式、跟踪模式及 FPV 模式下不响应回中命令
