"""
测试运行器 (Test Runner)

验证整个 sdk-to-mcp 流水线是否能正常工作
"""

import asyncio
import tempfile
import sys
from pathlib import Path

# 添加 parent 到路径
sys.path.insert(0, str(Path(__file__).parent))

from core.analyzer import SDKAnalyzer
from core.generator import MCPGenerator
from core.validator import MCPValidator


async def test_full_pipeline():
    """测试完整的转换流水线"""

    print("=" * 60)
    print("SDK-to-MCP 流水线测试")
    print("=" * 60)

    # 创建临时目录
    with tempfile.TemporaryDirectory() as tmpdir:
        output_dir = Path(tmpdir) / "generated"
        output_dir.mkdir()

        # 测试用例 1: 虚拟协议文档测试
        print("\n📋 测试用例 1: 虚拟串口协议")
        print("-" * 40)

        # 创建一个虚拟的协议描述文件
        protocol_desc = """
# 虚拟机械臂协议 V1.0

## 通信参数
- 波特率: 115200
- 帧头: 0xAA 0x55
- 校验: CRC16
- 字节序: Little Endian

## 控制命令

### move_joint(joint_id: int, angle: float)
移动关节到指定角度
- joint_id: 关节编号 (0-5)
- angle: 目标角度 (-180.0 到 180.0 度)
- 命令码: 0x01

### set_speed(speed: float)
设置运动速度
- speed: 速度百分比 (0.0 到 100.0)
- 命令码: 0x02

### gripper_open()
打开夹爪
- 命令码: 0x03

### gripper_close()
关闭夹爪
- 命令码: 0x04

## 状态反馈

### joint_positions
当前关节角度 (float array, 6个值)
单位: 度

### battery_level
电池电量 (int)
单位: %

### temperature
电机温度 (float)
单位: 摄氏度

## 安全限位
- max_angle: 180.0
- max_speed: 100.0
- max_temperature: 80.0
"""

        # 保存虚拟协议文档
        protocol_file = Path(tmpdir) / "test_protocol.txt"
        protocol_file.write_text(protocol_desc, encoding='utf-8')

        # Phase 1: 分析
        print("\n🔍 Phase 1: 分析协议文档...")
        analyzer = SDKAnalyzer()

        try:
            schema = await analyzer.analyze(
                source_path=str(protocol_file),
                hardware_type='serial'
            )

            print(f"   ✓ 识别到 {len(schema.get('actions', []))} 个动作")
            print(f"   ✓ 识别到 {len(schema.get('states', []))} 个状态")

            # 显示提取的内容
            for action in schema.get('actions', []):
                print(f"     - 动作: {action.get('name')} - {action.get('description', '无描述')}")
            for state in schema.get('states', []):
                print(f"     - 状态: {state.get('name')} - {state.get('description', '无描述')}")

        except Exception as e:
            print(f"   ✗ 分析失败: {e}")
            return False

        # Phase 2: 生成代码
        print("\n🔨 Phase 2: 生成 MCP Server 代码...")
        generator = MCPGenerator()

        try:
            server_code = await generator.generate(
                schema=schema,
                hardware_type='serial',
                target_name='test_robot_arm'
            )

            # 保存生成的代码
            server_file = output_dir / "test_robot_arm_server.py"
            server_file.write_text(server_code, encoding='utf-8')
            print(f"   ✓ 代码已保存: {server_file}")

            # 显示代码预览
            lines = server_code.split('\n')[:30]
            print("\n   代码预览 (前30行):")
            for i, line in enumerate(lines, 1):
                print(f"   {i:3d}: {line}")
            print(f"   ... ({len(server_code.split(chr(10)))} 行 total)")

        except Exception as e:
            print(f"   ✗ 生成失败: {e}")
            import traceback
            traceback.print_exc()
            return False

        # Phase 3: 验证
        print("\n🔍 Phase 3: 验证生成的代码...")
        validator = MCPValidator()

        try:
            success = await validator.validate(
                server_file=str(server_file),
                max_retries=3,
                timeout=30
            )

            if success:
                print("   ✓ 代码验证通过")
            else:
                print("   ✗ 代码验证失败")
                return False

        except Exception as e:
            print(f"   ✗ 验证过程出错: {e}")
            return False

    # 测试用例 2: DDS 模板测试
    print("\n\n📋 测试用例 2: DDS 协议模板")
    print("-" * 40)

    dds_schema = {
        "actions": [
            {"name": "walk", "params": {"vx": "float", "vy": "float"}, "description": "行走控制"},
            {"name": "stand_up", "params": {}, "description": "起立"},
            {"name": "sit_down", "params": {}, "description": "坐下"}
        ],
        "states": [
            {"name": "battery", "type": "int", "description": "电池电量", "unit": "%"},
            {"name": "imu", "type": "dict", "description": "IMU 数据", "unit": ""}
        ],
        "protocol_details": {"endian": "little", "frequency": 100},
        "hardware_info": {"name": "Unitree G1", "safety_limits": {"max_velocity": 3.7}}
    }

    try:
        server_code = await generator.generate(
            schema=dds_schema,
            hardware_type='dds',
            target_name='test_unitree_g1'
        )

        # 检查是否包含关键元素
        checks = [
            ("DDSListenerDaemon" in server_code, "守护线程"),
            ("StateBuffer" in server_code, "状态缓冲区"),
            ("safety_limits" in server_code, "安全限位"),
            ("@mcp.tool()" in server_code, "MCP 工具装饰器"),
        ]

        for passed, name in checks:
            status = "✓" if passed else "✗"
            print(f"   {status} 检查 {name}: {'通过' if passed else '失败'}")

        # 保存 DDS 代码
        dds_file = output_dir / "test_unitree_g1_server.py"
        dds_file.write_text(server_code, encoding='utf-8')

        # 验证
        success = await validator.validate(str(dds_file), max_retries=1, timeout=10)
        print(f"   {'✓' if success else '✗'} DDS 代码验证")

    except Exception as e:
        print(f"   ✗ DDS 测试失败: {e}")

    # 测试完成
    print("\n" + "=" * 60)
    print("✅ 所有测试完成!")
    print("=" * 60)

    return True


if __name__ == "__main__":
    success = asyncio.run(test_full_pipeline())
    sys.exit(0 if success else 1)
