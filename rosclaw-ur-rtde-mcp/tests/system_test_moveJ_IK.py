#!/usr/bin/env python3
"""
UR5 RTDE MCP 系统测试脚本
测试 moveJ_IK 功能及整体系统稳定性

安全约束:
- 工作区间: x ∈ [-0.5, 0.1], y ∈ [0.2, 1.0], z ∈ [0.00, 0.6]
- 只进行小范围浮动移动
- 先查询状态再移动

作者: 系统测试
日期: 2025-03-25
"""

import sys
import os
import time
import json
import numpy as np
from datetime import datetime

# 添加 src 到路径
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'src'))

from ur_rtde_bridge import URRTDEBridge, JOINT_VEL_MAX, JOINT_ACC_MAX

# ============ 配置参数 ============
ROBOT_IP = "192.168.123.6"
HOME_POSE = [-0.2118, 0.3192, 0.41, -np.pi, 0, 0]

# 安全工作区间 [min, max]
WORKSPACE_LIMITS = {
    'x': [-0.5, 0.1],
    'y': [0.2, 1.0],
    'z': [0.00, 0.6]
}

# 测试参数
TEST_SPEED = 0.5        # rad/s (保守速度)
TEST_ACCEL = 0.8        # rad/s² (保守加速度)
SMALL_MOVE = 0.03       # 小浮动移动 3cm

# ============ 测试报告 ============
class TestReport:
    def __init__(self):
        self.results = []
        self.start_time = datetime.now()
        
    def add_result(self, test_name: str, status: str, details: str = ""):
        result = {
            'test': test_name,
            'status': status,
            'details': details,
            'timestamp': datetime.now().isoformat()
        }
        self.results.append(result)
        status_icon = "✅" if status == "PASS" else "❌" if status == "FAIL" else "⚠️"
        print(f"{status_icon} {test_name}: {status}")
        if details:
            print(f"   详情: {details}")
        
    def generate_summary(self) -> str:
        passed = sum(1 for r in self.results if r['status'] == 'PASS')
        failed = sum(1 for r in self.results if r['status'] == 'FAIL')
        warnings = sum(1 for r in self.results if r['status'] == 'WARN')
        
        duration = (datetime.now() - self.start_time).total_seconds()
        
        summary = f"""
{'='*60}
                    UR5 RTDE MCP 系统测试报告
{'='*60}
测试时间: {self.start_time.strftime('%Y-%m-%d %H:%M:%S')}
测试时长: {duration:.1f} 秒
机器人IP: {ROBOT_IP}
{'='*60}
测试结果统计:
  ✅ 通过: {passed}
  ❌ 失败: {failed}
  ⚠️  警告: {warnings}
  总计: {len(self.results)}
{'='*60}
详细结果:
"""
        for r in self.results:
            icon = "✅" if r['status'] == "PASS" else "❌" if r['status'] == "FAIL" else "⚠️"
            summary += f"\n{icon} {r['test']}\n   状态: {r['status']}\n"
            if r['details']:
                summary += f"   详情: {r['details']}\n"
        
        summary += f"\n{'='*60}\n"
        
        if failed == 0:
            summary += "结论: 所有测试通过！系统运行正常。\n"
        else:
            summary += f"结论: 存在 {failed} 个失败项，请检查。\n"
        
        summary += f"{'='*60}\n"
        return summary

# ============ 辅助函数 ============
def check_pose_in_workspace(pose: list) -> tuple[bool, str]:
    """检查位姿是否在安全工作区间内"""
    x, y, z = pose[0], pose[1], pose[2]
    
    if not (WORKSPACE_LIMITS['x'][0] <= x <= WORKSPACE_LIMITS['x'][1]):
        return False, f"X坐标 {x:.4f} 超出范围 {WORKSPACE_LIMITS['x']}"
    if not (WORKSPACE_LIMITS['y'][0] <= y <= WORKSPACE_LIMITS['y'][1]):
        return False, f"Y坐标 {y:.4f} 超出范围 {WORKSPACE_LIMITS['y']}"
    if not (WORKSPACE_LIMITS['z'][0] <= z <= WORKSPACE_LIMITS['z'][1]):
        return False, f"Z坐标 {z:.4f} 超出范围 {WORKSPACE_LIMITS['z']}"
    
    return True, "位姿在安全工作区间内"

def print_state(bridge: URRTDEBridge, label: str = ""):
    """打印机器人状态"""
    state = bridge.get_state()
    print(f"\n{'-'*40}")
    if label:
        print(f"状态 [{label}]")
    print(f"  TCP位姿: {[round(v, 4) for v in state.actual_tcp_pose]}")
    print(f"  关节角: {[round(v, 4) for v in state.actual_q]}")
    print(f"  机器人模式: {state.robot_mode}")
    print(f"  安全模式: {state.safety_mode}")
    print(f"  保护停止: {state.is_protective_stopped}")
    print(f"  紧急停止: {state.is_emergency_stopped}")
    print(f"{'-'*40}")
    return state

# ============ 测试函数 ============
def test_connection(bridge: URRTDEBridge, report: TestReport) -> bool:
    """测试连接功能"""
    print("\n[测试1] 连接功能测试")
    try:
        print(f"  正在连接到 {ROBOT_IP}...")
        bridge.connect(ROBOT_IP)
        
        if bridge.is_connected():
            report.add_result("连接功能", "PASS", f"成功连接到 {ROBOT_IP}")
            return True
        else:
            report.add_result("连接功能", "FAIL", "连接后状态显示未连接")
            return False
    except Exception as e:
        report.add_result("连接功能", "FAIL", str(e))
        return False

def test_get_state(bridge: URRTDEBridge, report: TestReport) -> bool:
    """测试获取状态功能"""
    print("\n[测试2] 获取状态测试")
    try:
        state = bridge.get_state()
        print(f"  TCP位姿: {[round(v, 4) for v in state.actual_tcp_pose]}")
        print(f"  关节位置: {[round(v, 4) for v in state.actual_q]}")
        
        # 验证关键字段
        assert len(state.actual_tcp_pose) == 6, "TCP位姿应为6维"
        assert len(state.actual_q) == 6, "关节位置应为6维"
        
        report.add_result("获取状态", "PASS", 
                         f"TCP={ [round(v, 3) for v in state.actual_tcp_pose[:3]]}")
        return True
    except Exception as e:
        report.add_result("获取状态", "FAIL", str(e))
        return False

def test_workspace_safety(bridge: URRTDEBridge, report: TestReport) -> bool:
    """测试工作区间安全检查"""
    print("\n[测试3] 工作区间安全检查")
    try:
        state = bridge.get_state()
        pose = state.actual_tcp_pose
        
        in_workspace, msg = check_pose_in_workspace(pose)
        print(f"  当前位姿: {[round(v, 4) for v in pose]}")
        print(f"  检查结果: {msg}")
        
        if in_workspace:
            report.add_result("工作区间检查", "PASS", msg)
        else:
            report.add_result("工作区间检查", "WARN", 
                            f"当前位置不在安全工作区内: {msg}")
        return in_workspace
    except Exception as e:
        report.add_result("工作区间检查", "FAIL", str(e))
        return False

def test_moveJ_IK_small_movement(bridge: URRTDEBridge, report: TestReport) -> bool:
    """测试 moveJ_IK 小范围移动"""
    print("\n[测试4] moveJ_IK 小范围移动测试")
    print("  ⚠️  注意: 即将移动机械臂！")
    print(f"  将进行 {SMALL_MOVE*100:.0f}cm 的小范围浮动移动")
    
    try:
        # 获取当前状态
        state = print_state(bridge, "移动前")
        current_pose = state.actual_tcp_pose.copy()
        
        # 验证当前位置在工作区间内
        in_ws, msg = check_pose_in_workspace(current_pose)
        if not in_ws:
            report.add_result("moveJ_IK小范围移动", "FAIL", 
                            f"当前位置不在安全工作区: {msg}")
            return False
        
        # 计算目标位置 (Z轴向下移动 SMALL_MOVE)
        target_pose = current_pose.copy()
        target_pose[2] -= SMALL_MOVE  # Z向下移动
        
        # 验证目标位置
        in_ws, msg = check_pose_in_workspace(target_pose)
        if not in_ws:
            report.add_result("moveJ_IK小范围移动", "FAIL", 
                            f"目标位置超出安全工作区: {msg}")
            return False
        
        print(f"\n  当前位置: {[round(v, 4) for v in current_pose]}")
        print(f"  目标位置: {[round(v, 4) for v in target_pose]}")
        print(f"  移动距离: {SMALL_MOVE*100:.0f}cm (Z轴向下)")
        print(f"  速度: {TEST_SPEED} rad/s, 加速度: {TEST_ACCEL} rad/s²")
        
        # 执行移动 (给用户3秒取消时间)
        print("\n  3秒后开始移动...")
        for i in range(3, 0, -1):
            print(f"    {i}...")
            time.sleep(1)
        
        print("  执行 moveJ_IK...")
        ok = bridge.moveJ_IK(target_pose, TEST_SPEED, TEST_ACCEL, asynchronous=False)
        
        if not ok:
            report.add_result("moveJ_IK小范围移动", "FAIL", "moveJ_IK 返回 False")
            return False
        
        # 等待稳定
        time.sleep(0.5)
        
        # 验证移动结果
        state_after = print_state(bridge, "移动后")
        final_pose = state_after.actual_tcp_pose
        
        # 计算实际移动距离
        diff = np.array(final_pose[:3]) - np.array(current_pose[:3])
        distance = np.linalg.norm(diff)
        
        print(f"\n  实际位移: {[round(v, 4) for v in diff]}")
        print(f"  移动距离: {distance*100:.2f}cm")
        
        # 验证移动成功 (允许 5mm 误差)
        if distance >= SMALL_MOVE - 0.005:
            report.add_result("moveJ_IK小范围移动", "PASS",
                            f"移动成功，距离={distance*100:.1f}cm")
            return True
        else:
            report.add_result("moveJ_IK小范围移动", "WARN",
                            f"移动距离偏小: {distance*100:.1f}cm (期望 {SMALL_MOVE*100:.1f}cm)")
            return True
            
    except Exception as e:
        report.add_result("moveJ_IK小范围移动", "FAIL", str(e))
        return False

def test_moveJ_IK_return_home(bridge: URRTDEBridge, report: TestReport) -> bool:
    """测试 moveJ_IK 返回 Home 位置"""
    print("\n[测试5] moveJ_IK 返回 Home 位置测试")
    
    try:
        # 验证 Home 位置在工作区间内
        in_ws, msg = check_pose_in_workspace(HOME_POSE)
        if not in_ws:
            report.add_result("moveJ_IK返回Home", "FAIL",
                            f"Home位置不在安全工作区: {msg}")
            return False
        
        state = bridge.get_state()
        print(f"  当前位置: {[round(v, 4) for v in state.actual_tcp_pose]}")
        print(f"  Home位置: {[round(v, 4) for v in HOME_POSE]}")
        
        # 检查是否已经在 Home (允许 1cm 误差)
        diff = np.array(state.actual_tcp_pose[:3]) - np.array(HOME_POSE[:3])
        distance = np.linalg.norm(diff)
        
        if distance < 0.01:
            print("  已经在 Home 位置，跳过移动")
            report.add_result("moveJ_IK返回Home", "PASS", "已在Home位置")
            return True
        
        print(f"\n  距离 Home: {distance*100:.1f}cm")
        print(f"  速度: {TEST_SPEED} rad/s")
        print("\n  3秒后开始返回 Home...")
        for i in range(3, 0, -1):
            print(f"    {i}...")
            time.sleep(1)
        
        print("  执行 moveJ_IK to Home...")
        ok = bridge.moveJ_IK(HOME_POSE, TEST_SPEED, TEST_ACCEL, asynchronous=False)
        
        if not ok:
            report.add_result("moveJ_IK返回Home", "FAIL", "moveJ_IK 返回 False")
            return False
        
        time.sleep(0.5)
        state_after = bridge.get_state()
        final_pose = state_after.actual_tcp_pose
        
        diff = np.array(final_pose[:3]) - np.array(HOME_POSE[:3])
        distance = np.linalg.norm(diff)
        
        print(f"  最终位置: {[round(v, 4) for v in final_pose]}")
        print(f"  与Home偏差: {distance*100:.2f}cm")
        
        if distance < 0.01:  # 1cm 误差
            report.add_result("moveJ_IK返回Home", "PASS",
                            f"成功返回Home，偏差={distance*100:.1f}cm")
            return True
        else:
            report.add_result("moveJ_IK返回Home", "WARN",
                            f"返回Home偏差较大: {distance*100:.1f}cm")
            return True
            
    except Exception as e:
        report.add_result("moveJ_IK返回Home", "FAIL", str(e))
        return False

def test_disconnect(bridge: URRTDEBridge, report: TestReport) -> bool:
    """测试断开连接"""
    print("\n[测试6] 断开连接测试")
    try:
        bridge.disconnect()
        if not bridge.is_connected():
            report.add_result("断开连接", "PASS", "成功断开")
            return True
        else:
            report.add_result("断开连接", "FAIL", "断开后状态仍为连接")
            return False
    except Exception as e:
        report.add_result("断开连接", "FAIL", str(e))
        return False

# ============ 主测试流程 ============
def run_system_test():
    """执行完整的系统测试"""
    print("\n" + "="*60)
    print("     UR5 RTDE MCP 系统测试")
    print("="*60)
    print(f"机器人IP: {ROBOT_IP}")
    print(f"安全工作区: {WORKSPACE_LIMITS}")
    print(f"Home位置: {HOME_POSE}")
    print("="*60)
    
    bridge = URRTDEBridge()
    report = TestReport()
    
    try:
        # 1. 连接测试
        if not test_connection(bridge, report):
            print("\n❌ 连接失败，终止测试")
            report.add_result("系统测试", "FAIL", "连接失败，无法继续")
            return report
        
        # 2. 获取状态测试
        test_get_state(bridge, report)
        
        # 3. 工作区间安全检查
        in_workspace = test_workspace_safety(bridge, report)
        
        # 4. moveJ_IK 小范围移动测试 (仅当在安全工作区内时)
        if in_workspace:
            test_moveJ_IK_small_movement(bridge, report)
        else:
            print("\n⚠️  跳过移动测试: 当前位置不在安全工作区内")
            report.add_result("moveJ_IK小范围移动", "SKIP", "不在安全工作区")
        
        # 5. 返回 Home 测试
        test_moveJ_IK_return_home(bridge, report)
        
        # 6. 断开连接测试
        test_disconnect(bridge, report)
        
    except KeyboardInterrupt:
        print("\n\n用户中断测试")
        report.add_result("系统测试", "FAIL", "用户中断")
    except Exception as e:
        print(f"\n\n测试异常: {e}")
        report.add_result("系统测试", "FAIL", f"异常: {e}")
    finally:
        # 确保断开连接
        try:
            if bridge.is_connected():
                bridge.disconnect()
                print("\n已断开与机器人的连接")
        except:
            pass
    
    return report

if __name__ == "__main__":
    print("""
╔══════════════════════════════════════════════════════════════╗
║                    UR5 机械臂系统测试                         ║
║                                                               ║
║  ⚠️  警告: 此测试将控制真实机械臂运动！                        ║
║     请确保:                                                   ║
║     1. 机械臂周围无人员                                       ║
║     2. 工作区域无障碍物                                       ║
║     3. 紧急停止按钮触手可及                                   ║
║                                                               ║
║  安全工作区间:                                                ║
║    X: [-0.5, 0.1] m                                          ║
║    Y: [0.2, 1.0] m                                           ║
║    Z: [0.0, 0.6] m                                           ║
║                                                               ║
║  按 Ctrl+C 取消测试                                           ║
╚══════════════════════════════════════════════════════════════╝
""")
    
    input("按 Enter 开始测试...")
    
    report = run_system_test()
    
    # 输出报告
    summary = report.generate_summary()
    print(summary)
    
    # 保存报告到文件
    report_file = f"ur5_system_test_report_{datetime.now().strftime('%Y%m%d_%H%M%S')}.txt"
    with open(report_file, 'w', encoding='utf-8') as f:
        f.write(summary)
    print(f"\n测试报告已保存到: {report_file}")
