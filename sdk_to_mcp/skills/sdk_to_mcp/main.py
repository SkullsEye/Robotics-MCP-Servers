"""
sdk-to-mcp Skill 主入口

将异构硬件接口（PDF协议、C++/Python SDK、ROS接口）自动转换为标准化 MCP Server。
"""

import os
import sys
import argparse
from pathlib import Path
from typing import Literal, Optional
from mcp.server.fastmcp import FastMCP

# 将 core 模块加入路径
sys.path.insert(0, str(Path(__file__).parent))

from core.analyzer import SDKAnalyzer
from core.generator import MCPGenerator
from core.validator import MCPValidator

# 创建 MCP Server 实例
mcp = FastMCP("sdk-to-mcp-factory")


@mcp.tool()
async def generate_mcp_from_sdk(
    source_path: str,
    hardware_type: Literal['serial', 'dds', 'ros', 'http'],
    target_name: str,
    output_dir: str = "./generated_mcp_servers"
) -> str:
    """
    将硬件 SDK/协议文档转换为标准化 MCP Server。

    Args:
        source_path: SDK 文件夹路径或 PDF 协议文档路径
        hardware_type: 硬件通信类型 ('serial', 'dds', 'ros', 'http')
        target_name: 目标 MCP Server 名称（如 "gcu-gimbal", "unitree-g1"）
        output_dir: 输出目录（默认：./generated_mcp_servers）

    Returns:
        生成的 MCP Server 文件路径或错误信息

    Example:
        generate_mcp_from_sdk(
            source_path="/path/to/GCU协议.pdf",
            hardware_type="serial",
            target_name="gcu-gimbal"
        )
    """
    try:
        # 验证输入路径
        if not os.path.exists(source_path):
            return f"❌ 错误：源路径不存在: {source_path}"

        # 验证 hardware_type
        valid_types = ['serial', 'dds', 'ros', 'http']
        if hardware_type not in valid_types:
            return f"❌ 错误：不支持的 hardware_type '{hardware_type}'。支持的类型: {valid_types}"

        # 创建输出目录
        output_path = Path(output_dir)
        output_path.mkdir(parents=True, exist_ok=True)

        print(f"🚀 开始转换: {target_name}")
        print(f"   源路径: {source_path}")
        print(f"   硬件类型: {hardware_type}")
        print(f"   输出目录: {output_dir}")

        # Phase 1: 协议分析
        print("\n📋 Phase 1: 分析硬件接口文档...")
        analyzer = SDKAnalyzer()
        protocol_schema = await analyzer.analyze(source_path, hardware_type)

        if not protocol_schema:
            return "❌ 错误：无法解析硬件接口文档"

        print(f"   ✓ 识别到 {len(protocol_schema.get('actions', []))} 个动作")
        print(f"   ✓ 识别到 {len(protocol_schema.get('states', []))} 个状态")

        # Phase 2: 代码生成
        print("\n🔨 Phase 2: 生成 MCP Server 代码...")
        generator = MCPGenerator()
        server_code = await generator.generate(
            schema=protocol_schema,
            hardware_type=hardware_type,
            target_name=target_name
        )

        # 保存生成的代码
        server_file = output_path / f"{target_name}_server.py"
        server_file.write_text(server_code, encoding='utf-8')
        print(f"   ✓ 代码已保存: {server_file}")

        # Phase 3: 验证与修复
        print("\n🔍 Phase 3: 验证生成的代码...")
        validator = MCPValidator()
        success = await validator.validate(
            server_file=str(server_file),
            max_retries=3
        )

        if not success:
            return f"❌ 错误：代码验证失败，请检查生成的文件: {server_file}"

        print(f"   ✓ 代码验证通过")

        # Phase 4: 注册到 OpenClaw
        print("\n📦 Phase 4: 注册到 OpenClaw...")
        config_updated = await validator.register_to_openclaw(
            server_file=str(server_file),
            target_name=target_name
        )

        if config_updated:
            print(f"   ✓ 已更新 OpenClaw 配置")
        else:
            print(f"   ⚠ 跳过配置更新（未找到 OpenClaw 配置）")

        print(f"\n✅ 转换完成！MCP Server 已生成: {server_file}")
        return str(server_file)

    except Exception as e:
        return f"❌ 转换过程中发生错误: {type(e).__name__}: {str(e)}"


@mcp.tool()
async def analyze_hardware_protocol(
    source_path: str,
    hardware_type: Literal['serial', 'dds', 'ros', 'http']
) -> dict:
    """
    仅分析硬件协议，返回结构化的协议描述（调试用）。

    Args:
        source_path: SDK 文件夹路径或 PDF 协议文档路径
        hardware_type: 硬件通信类型

    Returns:
        协议描述的 JSON 结构
    """
    analyzer = SDKAnalyzer()
    return await analyzer.analyze(source_path, hardware_type)


@mcp.tool()
async def validate_mcp_server(server_file: str, timeout: int = 30) -> bool:
    """
    验证已有的 MCP Server 文件是否能正常运行。

    Args:
        server_file: MCP Server Python 文件路径
        timeout: 验证超时时间（秒）

    Returns:
        验证是否通过
    """
    validator = MCPValidator()
    return await validator.validate(server_file, timeout=timeout)


def main():
    """命令行入口"""
    parser = argparse.ArgumentParser(
        description="SDK-to-MCP Skill - 将硬件SDK转换为MCP Server"
    )
    parser.add_argument(
        "--transport",
        choices=["stdio", "sse", "http"],
        default="stdio",
        help="MCP 传输协议"
    )
    parser.add_argument(
        "--port",
        type=int,
        default=8000,
        help="HTTP/SSE 传输的端口"
    )

    args = parser.parse_args()

    if args.transport == "stdio":
        mcp.run(transport="stdio")
    elif args.transport in ["sse", "http"]:
        mcp.run(transport=args.transport, port=args.port)


if __name__ == "__main__":
    main()
