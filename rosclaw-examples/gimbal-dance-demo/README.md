# 10-Gimbal Dance Demo

> **ROSClaw 杀手级 Demo：10台云台交响舞队，由大模型现场听歌编舞指挥**

Part of [ROSClaw Examples](https://github.com/ros-claw/rosclaw-examples).

## 架构：大脑-小脑解耦

```
用户 → OpenClaw (LLM) ──────────────────────────────── 大脑层
          │
          ├─ analyze_music_file("fade.mp3")
          │     → BPM=128, Drop@30s, Vibe=Energetic EDM
          │
          ├─ choreograph_dance(music_json)
          │     → 编舞剧本 JSON
          │
          └─ run_choreography(script)  ─── 并发下发 ──→ 小脑层
                                              │
              ┌───────────────────────────────┼────────────────────────┐
              ▼                               ▼                        ▼
       🥁 Drummers (1-3)            🎤 Vocals (4-7)          ✨ Spotlight (8-10)
       headbang @ BPM               wave + phase              strobe_center
       50Hz serial → GCU            50Hz serial → GCU         50Hz serial → GCU
```

**大模型只下发语义指令**（headbang @ 128 BPM），**从不触碰50Hz原始角度流**。
Rhythm Engine（小脑）用正弦波/方波 + 时间戳，以50Hz高频写串口。

## 为什么这个 Demo 震撼业界

| 传统方案 | ROSClaw 方案 |
|---------|------------|
| 程序员手动K帧 | **大模型现场听歌，现场编排** |
| 换首歌 = 重写代码 | **换首歌 = 一条指令** |
| 单机控制 | **多智能体并发，3个Agent帮派** |
| LLM+串口 = 延迟崩溃 | **语义-硬件解耦，50Hz本地执行** |

## 快速开始

### 仿真模式（无需硬件）

```bash
cd gimbal-dance-demo
uv venv --python 3.10 && source .venv/bin/activate
uv pip install -e ".[music]"

# 启动 MCP Server（仿真模式）
python src/dance_mcp_server.py --sim
```

然后在 OpenClaw / MCP Inspector 中：

```
connect_dance_floor()
analyze_music_file("demo")          # 无需真实音频
choreograph_dance(music_json)
run_choreography(script_json)
```

### 真实硬件

```bash
# 1. 配置串口（编辑 config/gimbal_config.yaml）
#    将 COM3..COM12 改为你的实际串口

# 2. 启动 MCP Server（硬件模式）
python src/dance_mcp_server.py
```

### Claude Desktop 配置

```json
{
  "mcpServers": {
    "rosclaw-dance": {
      "command": "python",
      "args": ["/path/to/gimbal-dance-demo/src/dance_mcp_server.py", "--sim"],
      "transportType": "stdio"
    }
  }
}
```

### 音频依赖（真实音乐分析）

```bash
uv pip install librosa numpy
```

无 librosa 时，`analyze_music_file("demo")` 返回一段合成的 128 BPM EDM 结构，
Demo 仍然可以完整运行。

## 云台分组与"灵魂"

| 组 | IDs | 灵魂 | 默认行为 |
|----|-----|------|---------|
| 🥁 drummers | 1, 2, 3 | 激进金属鼓手，爆发式点头 | `headbang` |
| 🎤 vocals   | 4, 5, 6, 7 | 优雅主唱，左右丝滑扫射 | `wave` |
| ✨ spotlight | 8, 9, 10 | 神级追光，高潮时全闪聚焦 | `strobe_center` |

## 律动行为库

| 行为 | 轴 | 波形 | 适合场景 |
|------|---|------|---------|
| `headbang` | Tilt | 方波 | 鼓点/节拍强烈段落 |
| `sweep` | Pan | 正弦 | 副歌，流畅扫射 |
| `wave` | Pan+phase | 正弦+相位 | 多云台海浪效果 |
| `heartbeat` | Tilt | 双脉冲 | 强调音/snare hit |
| `figure8` | Pan+Tilt | Lissajous | 追光器，8字形 |
| `strobe_center` | Both | 随机抖动→归零 | Drop爆发瞬间 |
| `slow_circle` | Both | 慢速正弦 | 前奏/间奏氛围 |
| `freeze` | - | 归零保持 | 静止戏剧效果 |

`phase_wave=True` 时，组内每台云台相位依次偏移 `i/n × 2π`，产生涟漪波浪视觉。

## 编舞工作流（LLM视角）

```python
# Step 1: 分析音乐
result = analyze_music_file("cyberpunk_2077_theme.mp3")
# → BPM=105, Drop@45s, Vibe=Dark Industrial

# Step 2: 生成编舞剧本（LLM根据vibe决策）
script = choreograph_dance(music_json=result["json"], style="dramatic")

# Step 3: 并发执行（OpenClaw自动并发调用）
run_choreography(script_json=script)

# 紧急停止
stop_all_gimbals()
```

## Demo 视频分镜

| 时间 | 画面 | 后台日志 |
|------|------|---------|
| 0:00 | 暗室，10台云台低头休眠 | - |
| 0:05 | 用户输入：`Let's dance to Cyberpunk 2077` | LLM开始分析 |
| 0:10 | 终端滚动：`BPM: 105. Drop at 0:45.` | 编舞生成 |
| 0:12 | 10束激光笔同时亮起，齐刷刷抬头 | connect + 初始化 |
| 0:15 | Vocals组开始左右丝滑扫射 | `wave @ 52.5 BPM` |
| 0:25 | Drummers开始heartbeat，节奏渐强 | `heartbeat @ 105 BPM` |
| 0:45 | 🔥 DROP！Drummers疯狂点头，Spotlight全频闪 | `headbang + strobe_center @ 105 BPM` |
| 1:30 | 音乐结束，10台瞬间归零，低头休眠 | `freeze` → disconnect |

## 文件结构

```
gimbal-dance-demo/
├── src/
│   ├── dance_mcp_server.py   # MCP Server (大脑接口层)
│   ├── rhythm_engine.py      # 小脑：50Hz波形生成 + 串口控制
│   └── music_analyzer.py     # 音频分析：BPM / 段落 / 情感
├── config/
│   └── gimbal_config.yaml    # 10台云台串口映射
├── pyproject.toml
└── README.md
```

## 依赖

- `mcp[fastmcp]>=1.0.0` — MCP 框架
- `pyserial>=3.5` — 串口通信
- `pyyaml>=6.0` — 配置文件
- `librosa>=0.10.0` *(可选)* — 真实音频分析
- `numpy>=1.24` *(可选)* — librosa 依赖

## License

MIT — Part of [ROSClaw Examples](https://github.com/ros-claw/rosclaw-examples)
