# 任务指令：开发 OpenClaw Skill - `sdk-to-mcp`

## 1. 项目概述 (Project Overview)
你现在的任务是为 OpenClaw 或 Claude code 框架开发一个名为 `sdk-to-mcp` 的核心 Skill。 你可以参考你自己的 mcp-build 和 skill-creator 这些skill！
以下只是一些建议，具体实施你来，需要交付完整工程给我！
还有目录 sdk_to_mcp/sdk/xianfei/ 是一个具体例子，我需要你用你创建的 skill 将 云台 GCU协议 sdk 转成一个 MCP 服务（可以先支持串口通信）！ 

**目标：** 构建一个自动化流水线工具，能够接收异构硬件的接口文档（如 PDF 协议、C++/Python SDK 源码、ROS 接口），自动解析其通信语义，并使用 Python 的 `FastMCP` 框架生成可直接运行的标准化 MCP Server 代码，最后自动验证并注册到系统。

## 2. 目录架构设计 (Directory Structure)
请在 OpenClaw 的 `skills/` 目录下创建 `sdk_to_mcp/` 文件夹，并严格按照以下结构生成文件：

```text
skills/sdk_to_mcp/
├── SKILL.md                 # 供 OpenClaw Agent 读取的技能说明书（思维链定义）
├── main.py                  # Skill 的入口程序（对外暴露的 MCP Tool）
├── requirements.txt         # 依赖清单 (mcp, pypdf, jinja2, litellm 等)
├── core/
│   ├── __init__.py
│   ├── analyzer.py          # 文档解析与语义提取器 (LLM 驱动)
│   ├── generator.py         # MCP 代码生成器 (基于 Jinja2 模板)
│   └── validator.py         # 自动测试与修复沙盒
└── templates/
    ├── serial_mcp.jinja2    # 针对基于字节流/串口协议的模板 (如 GCU)
    ├── dds_mcp.jinja2       # 针对基于 DDS 通信的模板 (如 宇树 G1)
    └── ros_mcp.jinja2       # 针对 ROS/Action 接口的模板 (如 UR 机械臂)
```

## 3. 分阶段开发实施计划 (Phased Implementation Plan)

请严格按照以下 4 个阶段逐步完成开发，每完成一个阶段请向我汇报，确认无误后再进行下一阶段：

### Phase 1: 基础设施与入口设计 (Foundation)
- **任务:** 编写 `SKILL.md` 和 `main.py`。
- **细节要求:**
  - `SKILL.md` 需定义清晰的系统提示词，说明该技能的作用是“将硬件接口转化为 MCP Server”。
  - `main.py` 需要使用 `FastMCP` 暴露一个核心 Tool：`generate_mcp_from_sdk(source_path: str, hardware_type: str, target_name: str)`。
  - `hardware_type` 支持枚举：`['serial', 'dds', 'ros', 'http']`。

### Phase 2: 智能解析引擎开发 (Analyzer Engine)
- **任务:** 实现 `core/analyzer.py`。
- **细节要求:**
  - 实现一个能够读取 PDF 文档（针对 GCU 等私有协议）和遍历代码目录（针对 SDK）的解析类。
  - 核心逻辑：调用 LLM API（可使用 `litellm` 库，模型指向 `claude-3-5-sonnet`），将读取到的原始文本转化为结构化的 JSON Schema。
  - **强制输出 JSON 格式要求:**
    ```json
    {
      "actions": [{"name": "move", "params": {"vx": "float", "vy": "float"}, "description": "移动机器狗"}],
      "states": [{"name": "battery", "type": "int", "description": "电量百分比"}],
      "protocol_details": {"endian": "little", "header": "0xA8 0xE5", "checksum": "crc16"} 
    }
    ```

### Phase 3: 代码生成引擎开发 (Code Generator Engine)
- **任务:** 实现 `core/generator.py` 和 `templates/` 下的模板。
- **细节要求:**
  - 使用 `Jinja2` 模板引擎，将 Phase 2 提取出的 JSON Schema 渲染成 Python 代码。
  - **模板关键约束:**
    - 针对 `serial_mcp.jinja2`：必须在生成的代码中自动包含 `struct.pack/unpack` 逻辑和 CRC 校验函数骨架。
    - 针对 `dds_mcp.jinja2`：必须在生成的代码中设计后台异步守护线程（Daemon Thread）来监听 DDS 状态，防止阻塞 MCP 的主事件循环。
    - 所有生成的代码必须包含 **安全护栏 (Safety Guards)** 函数拦截器，限制异常参数输入。

### Phase 4: 自动验证与自修复闭环 (Validator Engine)
- **任务:** 实现 `core/validator.py`。
- **细节要求:**
  - 使用 `subprocess` 模块，将生成的 `mcp_server.py` 在隔离的子进程中尝试运行（`python mcp_server.py --test`）。
  - 捕获 `stderr` 的输出。如果检测到 `SyntaxError`、`ImportError` 等运行时错误，将错误日志反馈给 LLM（调用 Analyzer），重新生成修正后的代码。
  - 最大重试次数设置为 3 次。成功运行后，自动在 OpenClaw 的全局 `config.yaml` 中追加该 MCP Server 的配置路径。

## 4. 关键交付内容清单 (Deliverables)
在你（Claude Code）完成所有阶段后，需要确保以下交付物可用且健壮：
1. **完整可运行的代码库**：如上述目录结构所示。
2. **错误处理机制**：特别是能够处理 PDF 读取乱码、C++ SDK 缺少头文件等常见边缘情况的异常捕获。
3. **测试用例脚本 `test_runner.py`**：模拟传入一个虚拟的硬件文档，验证整个流水线能否正常输出一份包含 `@mcp.tool()` 的 Python 文件。

## 5. 开发原则与约束 (Constraints)
- **代码质量**：所有 Python 代码必须包含完整的 Type Hints (类型注解) 和 Docstrings。
- **解耦**：解析逻辑和生成逻辑必须严格分离。
- **安全性**：禁止使用 `eval()` 执行不可信的代码。运行生成的 MCP Server 必须使用 `subprocess.run` 并设置合理的超时时间（Timeout）。

---
**开始指令：** 请首先确认你已理解上述需求，并立即开始执行 **Phase 1** 的目录创建与入口文件编写工作。完成 Phase 1 后展示代码并等待下一步指示。
```

---

### 给你的执行建议：
1. **环境准备**：在运行 Claude Code 之前，确保你的开发机上安装了 `uv` 或 `pip`，并且已经建好了 `skills` 根目录。
2. **提供真实数据喂养**：当 Claude Code 写好这个系统并运行到 Phase 2 测试阶段时，你就可以把之前提供的 `GCU私有通信协议-XF(A5)V2.0.6.pdf` 和宇树的 GitHub 链接直接扔给这个 Skill 的入口工具，它就会开始自动“吐出”代码了。