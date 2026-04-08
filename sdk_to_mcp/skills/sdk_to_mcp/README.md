# SDK-to-MCP Skill

将硬件 SDK/协议文档自动转换为标准化 MCP Server 的 OpenClaw Skill。

## 项目概述

本 Skill 实现了一个自动化流水线，能够：
1. 分析异构硬件接口（PDF 协议、C++/Python SDK、ROS 接口）
2. 使用 LLM 提取结构化的协议描述
3. 基于 Jinja2 模板生成可直接运行的 MCP Server 代码
4. 自动验证并修复代码错误（最多 3 次重试）
5. 自动注册到 OpenClaw 配置

## 目录结构

```
sdk_to_mcp/
├── main.py                  # Skill 主入口（MCP Server）
├── SKILL.md                 # Skill 定义文档
├── requirements.txt         # 依赖清单
├── pyproject.toml           # 项目配置
├── test_runner.py           # 测试运行器
├── core/                    # 核心模块
│   ├── __init__.py
│   ├── analyzer.py          # 协议分析器（Phase 2）
│   ├── generator.py         # 代码生成器（Phase 3）
│   └── validator.py         # 验证器（Phase 4）
└── templates/               # Jinja2 模板
    ├── base_mcp.jinja2      # 基础模板
    ├── serial_mcp.jinja2    # 串口协议模板（含 CRC 校验）
    ├── dds_mcp.jinja2       # DDS 协议模板（含守护线程）
    ├── ros_mcp.jinja2       # ROS/ROS2 模板
    └── http_mcp.jinja2      # HTTP API 模板
```

## 开发阶段

### ✅ Phase 1: 基础设施与入口设计
- [x] SKILL.md 技能定义
- [x] main.py FastMCP 入口
- [x] requirements.txt 依赖清单
- [x] core/__init__.py 模块初始化

### ✅ Phase 2: 智能解析引擎
- [x] analyzer.py 协议分析器
  - PDF 文档读取（pypdf）
  - 代码目录扫描
  - LLM 驱动的语义提取（litellm）
  - 规则分析回退
  - 结构化 JSON 输出

### ✅ Phase 3: 代码生成引擎
- [x] generator.py 代码生成器
  - Jinja2 模板引擎
  - 5 种硬件类型模板
  - SafetyGuard 安全限位
  - struct.pack/unpack 支持
  - DDS 后台守护线程

### ✅ Phase 4: 自动验证与修复
- [x] validator.py 验证器
  - subprocess 隔离验证
  - SyntaxError/ImportError 捕获
  - 自动修复（最多 3 次）
  - OpenClaw 配置注册

## 使用方法

### 1. 安装依赖

```bash
cd sdk_to_mcp
pip install -r requirements.txt
# 或使用 uv
uv pip install -r requirements.txt
```

### 2. 运行 MCP Server

```bash
# stdio 传输（默认）
python main.py

# HTTP 传输
python main.py --transport http --port 8000

# SSE 传输
python main.py --transport sse --port 8000
```

### 3. 使用 MCP Tool 生成硬件服务

```python
# 通过 MCP 调用
generate_mcp_from_sdk(
    source_path="/path/to/GCU协议.pdf",
    hardware_type="serial",
    target_name="gcu-gimbal",
    output_dir="./generated_mcp_servers"
)
```

### 4. 运行测试

```bash
python test_runner.py
```

## 支持的硬件类型

| 类型 | 描述 | 关键特性 |
|------|------|----------|
| `serial` | 串口/字节流协议 | struct.pack/unpack, CRC 校验 |
| `dds` | DDS 分布式通信 | 后台守护线程，状态缓冲区 |
| `ros` | ROS/ROS2 接口 | ROS2 节点封装，Topic/Service/Action |
| `http` | HTTP/REST API | aiohttp 异步请求 |

## 生成的 MCP Server 特性

1. **安全限位 (Safety Guard)**
   - 所有动作命令都包含安全限位检查
   - 参数范围验证
   - 可配置的安全边界

2. **异步设计**
   - FastMCP async 工具
   - 非阻塞状态读取
   - DDS 后台守护线程

3. **连接管理**
   - 连接/断开工具
   - 状态监控
   - 错误处理

## 关键文件说明

### main.py
主入口文件，暴露以下 MCP Tools：
- `generate_mcp_from_sdk`: 完整转换流水线
- `analyze_hardware_protocol`: 仅分析协议
- `validate_mcp_server`: 验证已有 Server

### core/analyzer.py
协议分析器，支持：
- PDF 文档读取
- C++/Python 代码扫描
- LLM API 调用（claude-3-5-sonnet）
- 规则分析回退

### core/generator.py
代码生成器，基于 Jinja2：
- 动态模板选择
- PascalCase 类名转换
- 参数类型映射

### core/validator.py
验证器功能：
- 语法检查（py_compile）
- 导入检查
- 自动修复（import 缺失、语法错误）
- OpenClaw 配置更新

### templates/
Jinja2 模板文件：
- `serial_mcp.jinja2`: 串口协议，含 struct 和 CRC
- `dds_mcp.jinja2`: DDS 协议，含后台线程
- `ros_mcp.jinja2`: ROS2 节点封装
- `http_mcp.jinja2`: HTTP API 封装
- `base_mcp.jinja2`: 通用基础模板

## 测试

运行 `test_runner.py` 验证：
1. 虚拟协议文档分析
2. 代码生成
3. 语法验证
4. DDS 模板检查

## 下一步

使用此 Skill 转换 xianfei/GCU 协议：

```python
# 读取 GCU 协议 PDF
generate_mcp_from_sdk(
    source_path="../sdk/xianfei/GCU私有通信协议-XF(A5)V2.0.6.pdf",
    hardware_type="serial",
    target_name="gcu-gimbal",
    output_dir="../generated_mcp_servers"
)
```

## 注意事项

1. **litellm**: 如需使用 LLM 分析，需配置 API 密钥
2. **ROS2**: 如需生成 ROS 模板，需安装 rclpy
3. **串口**: 如需实际串口通信，需安装 pyserial
4. **安全**: 所有生成的代码都包含 SafetyGuard，请根据实际硬件调整限位参数
