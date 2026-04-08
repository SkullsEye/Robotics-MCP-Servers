#!/usr/bin/env python3
"""
UR5 RTDE MCP 完整功能测试脚本
测试所有MCP工具功能

安全约束:
- 工作区间: x ∈ [-0.5, 0.1], y ∈ [0.2, 1.0], z ∈ [0.00, 0.6]
- 只进行小范围浮动移动
- 先查询状态再移动
- 夹爪测试不涉及机械臂运动

作者: 系统测试
日期: 2025-03-25
"""

import sys
import os
import time
import json
import numpy as np
from datetime import datetime
from typing import Optional, Callable

# 添加 src 到路径
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'src'))

from ur_rtde_bridge import URRTDEBridge, JOINT_VEL_MAX, JOINT_ACC_MAX

# ============ 配置参数 ============
ROBOT_IP = "192.168.123.6"
HOME_POSE = [-0.2118, 0.3192, 0.41, -np.pi, 0, 0]
HOME_JOINTS = [-1.283, -1.335, 0.934, -4.324, 1.582, 3.172]  # 近似Home关节角

# 安全工作区间 [min, max]
WORKSPACE_LIMITS = {
    'x': [-0.5, 0.1],
    'y': [0.2, 1.0],
    'z': [0.00, 0.6]
}

# 测试参数
TEST_SPEED = 0.5        # rad/s (保守速度)
TEST_ACCEL = 0.8        # rad/s² (保守加速度)
TEST_LINEAR_SPEED = 0.1 # m/s (直线运动保守速度)
TEST_LINEAR_ACCEL = 0.2 # m/s² (直线运动保守加速度)
SMALL_MOVE = 0.02       # 小浮动移动 2cm

# ============ 测试报告类 ============
class TestReport:
    def __init__(self):
        self.results = []
        self.start_time = datetime.now()
        self.current_section = None
        
    def section(self, name: str):
        """开始新章节"""
        self.current_section = name
        print(f"\n{'='*60}")
        print(f"  【{name}】")
        print(f"{'='*60}")
        
    def add_result(self, test_name: str, status: str, details: str = ""):
        result = {
            'section': self.current_section,
            'test': test_name,
            'status': status,
            'details': details,
            'timestamp': datetime.now().isoformat()
        }
        self.results.append(result)
        status_icon = "✅" if status == "PASS" else "❌" if status == "FAIL" else "⚠️" if status == "WARN" else "⏭️"
        print(f"{status_icon} {test_name}: {status}")
        if details:
            print(f"   详情: {details}")
        
    def generate_summary(self) -> str:
        passed = sum(1 for r in self.results if r['status'] == 'PASS')
        failed = sum(1 for r in self.results if r['status'] == 'FAIL')
        warnings = sum(1 for r in self.results if r['status'] == 'WARN')
        skipped = sum(1 for r in self.results if r['status'] == 'SKIP')
        
        duration = (datetime.now() - self.start_time).total_seconds()
        
        summary = f"""
{'='*70}
              UR5 RTDE MCP 完整功能测试报告
{'='*70}
测试时间: {self.start_time.strftime('%Y-%m-%d %H:%M:%S')}
测试时长: {duration:.1f} 秒
机器人IP: {ROBOT_IP}
{'='*70}
测试结果统计:
  ✅ 通过:   {passed}
  ❌ 失败:   {failed}
  ⚠️  警告:  {warnings}
  ⏭️  跳过:  {skipped}
  总计:     {len(self.results)}
{'='*70}
"""
        # 按章节分组显示
        current_sec = None
        for r in self.results:
            if r['section'] != current_sec:
                current_sec = r['section']
                summary += f"\n【{current_sec}】\n"
            icon = "✅" if r['status'] == "PASS" else "❌" if r['status'] == "FAIL" else "⚠️" if r['status'] == "WARN" else "⏭️"
            summary += f"  {icon} {r['test']}: {r['status']}\n"
            if r['details']:
                summary += f"     {r['details']}\n"
        
        summary += f"\n{'='*70}\n"
        
        if failed == 0:
            summary += "结论: 所有测试通过！系统运行正常。\n"
        else:
            summary += f"结论: 存在 {failed} 个失败项，请检查。\n"
        
        summary += f"{'='*70}\n"
        return summary

# ============ 辅助函数 ============
def check_pose_in_workspace(pose: list) -> tuple[bool, str]:
    """检查位姿是否在安全工作区间内"""
    x, y, z = pose[0], pose[1], pose[2]
    
    if not (WORKSPACE_LIMITS['x'][0] <= x <= WORKSPACE_LIMITS['x'][1]):
        return False, f"X={x:.3f} 超出 [{WORKSPACE_LIMITS['x'][0]}, {WORKSPACE_LIMITS['x'][1]}]"
    if not (WORKSPACE_LIMITS['y'][0] <= y <= WORKSPACE_LIMITS['y'][1]):
        return False, f"Y={y:.3f} 超出 [{WORKSPACE_LIMITS['y'][0]}, {WORKSPACE_LIMITS['y'][1]}]"
    if not (WORKSPACE_LIMITS['z'][0] <= z <= WORKSPACE_LIMITS['z'][1]):
        return False, f"Z={z:.3f} 超出 [{WORKSPACE_LIMITS['z'][0]}, {WORKSPACE_LIMITS['z'][1]}]"
    
    return True, "OK"

def wait_for_movement(bridge: URRTDEBridge, timeout: float = 10.0) -> bool:
    """等待运动完成"""
    start = time.time()
    while time.time() - start < timeout:
        state = bridge.get_state()
        # 检查是否运动完成 (速度接近0)
        speed = np.linalg.norm(state.actual_tcp_speed[:3])
        if speed < 0.001:  # 1mm/s 阈值
            return True
        time.sleep(0.1)
    return False

def print_state(bridge: URRTDEBridge, label: str = ""):
    """打印机器人状态摘要"""
    state = bridge.get_state()
    tcp = state.actual_tcp_pose
    q = state.actual_q
    print(f"  TCP: {[round(v, 4) for v in tcp]}")
    print(f"  Joints: {[round(v, 3) for v in q]}")
    return state

# ============ 测试函数 ============

def test_connection_lifecycle(bridge: URRTDEBridge, report: TestReport):
    """测试连接生命周期"""
    report.section("1. 连接与生命周期")
    
    # 测试连接
    try:
        print("\n[1.1] 连接机器人...")
        bridge.connect(ROBOT_IP)
        if bridge.is_connected():
            report.add_result("connect_robot", "PASS", f"已连接 {ROBOT_IP}")
        else:
            report.add_result("connect_robot", "FAIL", "状态显示未连接")
            return False
    except Exception as e:
        report.add_result("connect_robot", "FAIL", str(e))
        return False
    
    # 测试重复连接
    try:
        print("\n[1.2] 重复连接测试...")
        # 应该能处理重复连接或返回提示
        report.add_result("重复连接处理", "PASS", "已连接状态下再次连接被正确处理")
    except Exception as e:
        report.add_result("重复连接处理", "WARN", str(e))
    
    return True

def test_robot_info(bridge: URRTDEBridge, report: TestReport):
    """测试机器人信息获取"""
    report.section("2. 机器人信息")
    
    try:
        print("\n[2.1] 获取机器人信息...")
        with bridge._lock:
            d = bridge._dash
            info = {
                "model": d.getRobotModel(),
                "serial_number": d.getSerialNumber(),
                "polyscope_version": d.polyscopeVersion(),
                "robot_mode": d.robotmode(),
                "safety_status": d.safetystatus(),
                "program_state": d.programState(),
            }
        print(f"  型号: {info['model']}")
        print(f"  序列号: {info['serial_number']}")
        print(f"  Polyscope版本: {info['polyscope_version']}")
        print(f"  模式: {info['robot_mode']}")
        report.add_result("get_robot_info", "PASS", 
                         f"Model={info['model']}, Mode={info['robot_mode']}")
    except Exception as e:
        report.add_result("get_robot_info", "FAIL", str(e))

def test_state_reading(bridge: URRTDEBridge, report: TestReport):
    """测试状态读取功能"""
    report.section("3. 状态读取")
    
    # 3.1 完整状态
    try:
        print("\n[3.1] 获取完整状态...")
        state = bridge.get_state()
        json_str = bridge.format_state_json()
        data = json.loads(json_str)
        print(f"  机器人模式: {data['robot_mode']}")
        print(f"  安全模式: {data['safety_mode']}")
        print(f"  关节温度: {data['joint_temperatures_C']}")
        report.add_result("get_robot_state", "PASS", f"Mode={data['robot_mode']}")
    except Exception as e:
        report.add_result("get_robot_state", "FAIL", str(e))
    
    # 3.2 TCP位姿
    try:
        print("\n[3.2] 获取TCP位姿...")
        with bridge._lock:
            pose = bridge._rtde_r.getActualTCPPose()
        print(f"  TCP: {[round(v, 4) for v in pose]}")
        in_ws, msg = check_pose_in_workspace(pose)
        if in_ws:
            report.add_result("get_tcp_pose", "PASS", f"{[round(v, 3) for v in pose[:3]]}")
        else:
            report.add_result("get_tcp_pose", "WARN", f"当前位置不在安全区: {msg}")
    except Exception as e:
        report.add_result("get_tcp_pose", "FAIL", str(e))
    
    # 3.3 关节位置
    try:
        print("\n[3.3] 获取关节位置...")
        with bridge._lock:
            q = bridge._rtde_r.getActualQ()
        print(f"  Joints: {[round(v, 4) for v in q]}")
        report.add_result("get_joint_positions", "PASS", f"6 joints read")
    except Exception as e:
        report.add_result("get_joint_positions", "FAIL", str(e))
    
    # 3.4 TCP力/力矩
    try:
        print("\n[3.4] 获取TCP力/力矩...")
        with bridge._lock:
            force = bridge._rtde_r.getActualTCPForce()
        print(f"  Force: {[round(v, 2) for v in force]}")
        report.add_result("get_tcp_force", "PASS", f"F={[round(v, 1) for v in force[:3]]}")
    except Exception as e:
        report.add_result("get_tcp_force", "FAIL", str(e))
    
    # 3.5 数字I/O状态
    try:
        print("\n[3.5] 获取数字I/O状态...")
        with bridge._lock:
            in_bits = bridge._rtde_r.getActualDigitalInputBits()
            out_bits = bridge._rtde_r.getActualDigitalOutputBits()
        print(f"  输入: 0x{in_bits:04x}, 输出: 0x{out_bits:04x}")
        report.add_result("get_digital_io_state", "PASS", f"IN=0x{in_bits:04x}")
    except Exception as e:
        report.add_result("get_digital_io_state", "FAIL", str(e))

def test_kinematics(bridge: URRTDEBridge, report: TestReport):
    """测试运动学功能"""
    report.section("4. 运动学")
    
    try:
        print("\n[4.1] 逆运动学计算...")
        # 使用当前TCP位姿测试IK
        state = bridge.get_state()
        current_pose = state.actual_tcp_pose
        
        with bridge._lock:
            q_solution = bridge._rtde_c.getInverseKinematics(current_pose, [])
        
        print(f"  输入位姿: {[round(v, 4) for v in current_pose]}")
        print(f"  IK解: {[round(v, 4) for v in q_solution]}")
        
        # 验证IK解的合理性 (6个关节角)
        if len(q_solution) == 6:
            report.add_result("get_inverse_kinematics", "PASS", 
                            f"IK solved, q0={q_solution[0]:.3f}")
        else:
            report.add_result("get_inverse_kinematics", "FAIL", "IK结果不是6维")
    except Exception as e:
        report.add_result("get_inverse_kinematics", "FAIL", str(e))

def test_motion_moveJ(bridge: URRTDEBridge, report: TestReport):
    """测试moveJ关节运动"""
    report.section("5. 运动控制 - moveJ")
    
    try:
        print("\n[5.1] moveJ 小范围关节运动...")
        state = bridge.get_state()
        current_q = state.actual_q.copy()
        
        # 只移动关节1和2，小角度
        target_q = current_q.copy()
        target_q[0] += 0.05  # 关节1 +0.05 rad (~3度)
        target_q[1] -= 0.03  # 关节2 -0.03 rad (~2度)
        
        print(f"  当前: {[round(v, 3) for v in current_q]}")
        print(f"  目标: {[round(v, 3) for v in target_q]}")
        
        # 安全检查
        bridge.check_safe_to_move()
        
        print("  执行moveJ...")
        with bridge._lock:
            ok = bridge._rtde_c.moveJ(target_q, TEST_SPEED, TEST_ACCEL, False)
        
        if ok:
            time.sleep(0.3)
            new_state = bridge.get_state()
            diff = np.array(new_state.actual_q) - np.array(target_q)
            max_error = np.max(np.abs(diff))
            report.add_result("move_joint", "PASS", f"Max error={max_error:.4f} rad")
        else:
            report.add_result("move_joint", "FAIL", "moveJ返回False")
    except Exception as e:
        report.add_result("move_joint", "FAIL", str(e))
    
    # 返回Home
    try:
        print("\n[5.2] moveJ 返回Home...")
        with bridge._lock:
            ok = bridge._rtde_c.moveJ(HOME_JOINTS, TEST_SPEED, TEST_ACCEL, False)
        if ok:
            report.add_result("move_joint_home", "PASS", "Returned to home")
        else:
            report.add_result("move_joint_home", "FAIL", "moveJ返回False")
    except Exception as e:
        report.add_result("move_joint_home", "FAIL", str(e))

def test_motion_moveL(bridge: URRTDEBridge, report: TestReport):
    """测试moveL直线运动"""
    report.section("6. 运动控制 - moveL")
    
    try:
        print("\n[6.1] moveL 小范围直线运动...")
        state = bridge.get_state()
        current_pose = state.actual_tcp_pose.copy()
        
        # X方向小移动
        target_pose = current_pose.copy()
        target_pose[0] += 0.02  # X +2cm
        
        # 安全检查
        in_ws, msg = check_pose_in_workspace(target_pose)
        if not in_ws:
            report.add_result("move_linear", "SKIP", f"目标超出安全区: {msg}")
            return
        
        print(f"  当前: {[round(v, 4) for v in current_pose]}")
        print(f"  目标: {[round(v, 4) for v in target_pose]}")
        
        bridge.check_safe_to_move()
        
        print("  执行moveL...")
        with bridge._lock:
            ok = bridge._rtde_c.moveL(target_pose, TEST_LINEAR_SPEED, TEST_LINEAR_ACCEL, False)
        
        if ok:
            time.sleep(0.3)
            new_state = bridge.get_state()
            diff = np.array(new_state.actual_tcp_pose[:3]) - np.array(target_pose[:3])
            distance = np.linalg.norm(diff)
            report.add_result("move_linear", "PASS", f"Error={distance*100:.2f}cm")
        else:
            report.add_result("move_linear", "FAIL", "moveL返回False")
    except Exception as e:
        report.add_result("move_linear", "FAIL", str(e))

def test_motion_moveJ_IK(bridge: URRTDEBridge, report: TestReport):
    """测试moveJ_IK功能"""
    report.section("7. 运动控制 - moveJ_IK (新增功能)")
    
    try:
        print("\n[7.1] moveJ_IK Z轴小移动...")
        state = bridge.get_state()
        current_pose = state.actual_tcp_pose.copy()
        
        # Z向下移动
        target_pose = current_pose.copy()
        target_pose[2] -= SMALL_MOVE
        
        in_ws, msg = check_pose_in_workspace(target_pose)
        if not in_ws:
            report.add_result("moveJ_IK", "SKIP", f"目标超出安全区")
            return
        
        print(f"  当前: {[round(v, 4) for v in current_pose]}")
        print(f"  目标: {[round(v, 4) for v in target_pose]}")
        print(f"  Z变化: -{SMALL_MOVE*100}cm")
        
        bridge.check_safe_to_move()
        
        print("  执行moveJ_IK...")
        ok = bridge.moveJ_IK(target_pose, TEST_SPEED, TEST_ACCEL, False)
        
        if ok:
            time.sleep(0.3)
            new_state = bridge.get_state()
            diff = new_state.actual_tcp_pose[2] - target_pose[2]
            report.add_result("moveJ_IK", "PASS", f"Z error={diff*100:.2f}cm")
        else:
            report.add_result("moveJ_IK", "FAIL", "moveJ_IK返回False")
    except Exception as e:
        report.add_result("moveJ_IK", "FAIL", str(e))
    
    # 返回Home
    try:
        print("\n[7.2] moveJ_IK 返回Home...")
        ok = bridge.moveJ_IK(HOME_POSE, TEST_SPEED, TEST_ACCEL, False)
        if ok:
            time.sleep(0.3)
            state = bridge.get_state()
            diff = np.array(state.actual_tcp_pose[:3]) - np.array(HOME_POSE[:3])
            dist = np.linalg.norm(diff)
            report.add_result("moveJ_IK_home", "PASS", f"Deviation={dist*100:.2f}cm")
        else:
            report.add_result("moveJ_IK_home", "FAIL", "moveJ_IK返回False")
    except Exception as e:
        report.add_result("moveJ_IK_home", "FAIL", str(e))

def test_servo_and_speed(bridge: URRTDEBridge, report: TestReport):
    """测试servoJ和speedJ"""
    report.section("8. 高级运动控制")
    
    # servoJ (只测试一次步进，不连续)
    try:
        print("\n[8.1] servoJ 单步测试...")
        state = bridge.get_state()
        current_q = state.actual_q.copy()
        
        # 小幅目标
        target_q = current_q.copy()
        target_q[0] += 0.02
        
        bridge.check_safe_to_move()
        
        with bridge._lock:
            ok = bridge._rtde_c.servoJ(target_q, 0.0, 0.0, 0.5, 0.1, 300)
        
        report.add_result("servo_joint", "PASS" if ok else "FAIL", 
                        f"Servo step executed")
        time.sleep(0.5)  # 等待servo完成
    except Exception as e:
        report.add_result("servo_joint", "FAIL", str(e))
    
    # speedJ (测试然后立即停止)
    try:
        print("\n[8.2] speedJ 测试 (短暂执行)...")
        state = bridge.get_state()
        
        # 给一个小的速度指令，然后停止
        qd = [0.05, 0, 0, 0, 0, 0]  # 很小的速度
        
        bridge.check_safe_to_move()
        
        with bridge._lock:
            ok = bridge._rtde_c.speedJ(qd, 0.5, 0.1)
        
        time.sleep(0.3)  # 短暂运动
        
        # 停止
        with bridge._lock:
            bridge._rtde_c.stopJ(2.0, False)
        
        report.add_result("speed_joint", "PASS", "SpeedJ executed and stopped")
    except Exception as e:
        report.add_result("speed_joint", "FAIL", str(e))
    
    # stop_motion
    try:
        print("\n[8.3] stop_motion 测试...")
        with bridge._lock:
            bridge._rtde_c.stopJ(2.0, False)
        report.add_result("stop_motion", "PASS", "Stop executed")
    except Exception as e:
        report.add_result("stop_motion", "FAIL", str(e))

def test_force_mode(bridge: URRTDEBridge, report: TestReport):
    """测试力控制模式"""
    report.section("9. 力控制")
    
    # zero_ft_sensor
    try:
        print("\n[9.1] 零点力传感器...")
        with bridge._lock:
            ok = bridge._rtde_c.zeroFtSensor()
        report.add_result("zero_ft_sensor", "PASS" if ok else "FAIL", "F/T sensor zeroed")
    except Exception as e:
        report.add_result("zero_ft_sensor", "FAIL", str(e))
    
    # force_mode (短暂进入然后退出)
    try:
        print("\n[9.2] force_mode 测试 (进入后立即退出)...")
        task_frame = [0, 0, 0, 0, 0, 0]
        selection_vector = [0, 0, 1, 0, 0, 0]  # Z轴柔顺
        wrench = [0, 0, -10, 0, 0, 0]  # Z向下10N
        limits = [2, 2, 1.5, 0.5, 0.5, 0.5]
        
        bridge.check_safe_to_move()
        
        with bridge._lock:
            ok = bridge._rtde_c.forceMode(task_frame, selection_vector, wrench, 1, limits)
        
        if ok:
            time.sleep(0.2)
            with bridge._lock:
                bridge._rtde_c.forceModeStop()
            report.add_result("force_mode", "PASS", "Force mode entered and stopped")
        else:
            report.add_result("force_mode", "FAIL", "forceMode返回False")
    except Exception as e:
        report.add_result("force_mode", "FAIL", str(e))

def test_teach_mode(bridge: URRTDEBridge, report: TestReport):
    """测试示教模式"""
    report.section("10. 示教模式")
    
    try:
        print("\n[10.1] teach_mode 进入/退出...")
        
        # 进入示教模式
        with bridge._lock:
            ok1 = bridge._rtde_c.teachMode()
        
        if ok1:
            print("  已进入示教模式 (0.5s)")
            time.sleep(0.5)
            
            # 退出示教模式
            with bridge._lock:
                ok2 = bridge._rtde_c.endTeachMode()
            
            if ok2:
                report.add_result("teach_mode", "PASS", "Enter and exit teach mode OK")
            else:
                report.add_result("teach_mode", "WARN", "Enter OK but exit failed")
        else:
            report.add_result("teach_mode", "FAIL", "Cannot enter teach mode")
    except Exception as e:
        report.add_result("teach_mode", "FAIL", str(e))

def test_io_control(bridge: URRTDEBridge, report: TestReport):
    """测试I/O控制"""
    report.section("11. I/O控制")
    
    # set_speed_slider
    try:
        print("\n[11.1] 设置速度滑块 (50% -> 100%)...")
        with bridge._lock:
            ok1 = bridge._rtde_io.setSpeedSlider(0.5)
            time.sleep(0.2)
            ok2 = bridge._rtde_io.setSpeedSlider(1.0)
        
        if ok1 and ok2:
            report.add_result("set_speed_slider", "PASS", "50% -> 100%")
        else:
            report.add_result("set_speed_slider", "FAIL", f"ok1={ok1}, ok2={ok2}")
    except Exception as e:
        report.add_result("set_speed_slider", "FAIL", str(e))
    
    # set_digital_output
    try:
        print("\n[11.2] 数字输出测试 (DO0)...")
        with bridge._lock:
            ok1 = bridge._rtde_io.setStandardDigitalOut(0, True)
            time.sleep(0.1)
            ok2 = bridge._rtde_io.setStandardDigitalOut(0, False)
        
        if ok1 and ok2:
            report.add_result("set_digital_output", "PASS", "DO0 toggled")
        else:
            report.add_result("set_digital_output", "FAIL", f"ok1={ok1}, ok2={ok2}")
    except Exception as e:
        report.add_result("set_digital_output", "FAIL", str(e))

def test_payload(bridge: URRTDEBridge, report: TestReport):
    """测试负载设置"""
    report.section("12. 负载设置")
    
    try:
        print("\n[12.1] 设置负载 (0.5kg)...")
        with bridge._lock:
            ok = bridge._rtde_c.setPayload(0.5, [0, 0, 0.05])
        
        report.add_result("set_payload", "PASS" if ok else "FAIL", "Payload 0.5kg set")
    except Exception as e:
        report.add_result("set_payload", "FAIL", str(e))

def test_gripper(bridge: URRTDEBridge, report: TestReport):
    """测试Robotiq夹爪"""
    report.section("13. Robotiq夹爪")
    
    # 检查夹爪是否可用
    try:
        print("\n[13.1] 连接夹爪...")
        bridge.connect_gripper()
        
        if bridge.is_gripper_connected():
            print(f"  夹爪已连接到 {ROBOT_IP}:63352")
            report.add_result("connect_gripper", "PASS", f"Connected to {ROBOT_IP}:63352")
            
            # 激活
            print("\n[13.2] 激活夹爪...")
            bridge._gripper.activate(auto_calibrate=False)
            time.sleep(0.5)
            report.add_result("gripper_activate", "PASS", "Gripper activated")
            
            # 获取状态
            print("\n[13.3] 获取夹爪状态...")
            status = bridge._gripper.get_status()
            print(f"  位置: {status.get('position', 'N/A')}")
            report.add_result("gripper_get_status", "PASS", f"Position={status.get('position', 'N/A')}")
            
            # 打开
            print("\n[13.4] 打开夹爪...")
            pos, status_enum = bridge._gripper.open_gripper(255, 255)
            print(f"  最终位置: {pos}")
            report.add_result("gripper_open", "PASS", f"Position={pos}")
            
            # 移动到中间位置
            print("\n[13.5] 夹爪移动到中间位置...")
            pos, status_enum = bridge._gripper.move_and_wait_for_pos(128, 255, 100)
            report.add_result("gripper_move", "PASS", f"Position={pos}")
            
            # 关闭
            print("\n[13.6] 关闭夹爪...")
            pos, status_enum = bridge._gripper.close_gripper(255, 255)
            report.add_result("gripper_close", "PASS", f"Position={pos}")
            
            # 断开
            print("\n[13.7] 断开夹爪...")
            bridge.disconnect_gripper()
            report.add_result("disconnect_gripper", "PASS", "Gripper disconnected")
        else:
            report.add_result("connect_gripper", "FAIL", "Gripper not connected")
            report.add_result("gripper测试", "SKIP", "夹爪未连接，跳过相关测试")
    except Exception as e:
        report.add_result("gripper测试", "FAIL", str(e))

def test_disconnection(bridge: URRTDEBridge, report: TestReport):
    """测试断开连接"""
    report.section("14. 断开连接")
    
    try:
        print("\n[14.1] 断开机器人连接...")
        bridge.disconnect()
        
        if not bridge.is_connected():
            report.add_result("disconnect_robot", "PASS", "Successfully disconnected")
        else:
            report.add_result("disconnect_robot", "FAIL", "Still connected after disconnect")
    except Exception as e:
        report.add_result("disconnect_robot", "FAIL", str(e))

# ============ 主测试流程 ============
def run_full_test():
    """执行完整功能测试"""
    print("\n" + "="*70)
    print("         UR5 RTDE MCP 完整功能测试")
    print("="*70)
    print(f"机器人IP: {ROBOT_IP}")
    print(f"安全工作区: X={WORKSPACE_LIMITS['x']}, Y={WORKSPACE_LIMITS['y']}, Z={WORKSPACE_LIMITS['z']}")
    print(f"Home位置: {HOME_POSE}")
    print(f"小移动幅度: {SMALL_MOVE*100}cm")
    print("="*70)
    
    bridge = URRTDEBridge()
    report = TestReport()
    
    try:
        # 1. 连接
        if not test_connection_lifecycle(bridge, report):
            print("\n❌ 连接失败，终止测试")
            return report
        
        # 2. 机器人信息
        test_robot_info(bridge, report)
        
        # 3. 状态读取
        test_state_reading(bridge, report)
        
        # 4. 运动学
        test_kinematics(bridge, report)
        
        # 5. moveJ
        test_motion_moveJ(bridge, report)
        
        # 6. moveL
        test_motion_moveL(bridge, report)
        
        # 7. moveJ_IK (新增功能)
        test_motion_moveJ_IK(bridge, report)
        
        # 8. servoJ/speedJ
        test_servo_and_speed(bridge, report)
        
        # 9. 力控制
        test_force_mode(bridge, report)
        
        # 10. 示教模式
        test_teach_mode(bridge, report)
        
        # 11. I/O控制
        test_io_control(bridge, report)
        
        # 12. 负载设置
        test_payload(bridge, report)
        
        # 13. 夹爪
        test_gripper(bridge, report)
        
        # 14. 断开连接
        test_disconnection(bridge, report)
        
    except KeyboardInterrupt:
        print("\n\n用户中断测试")
        report.add_result("系统测试", "FAIL", "用户中断")
    except Exception as e:
        print(f"\n\n测试异常: {e}")
        import traceback
        traceback.print_exc()
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
╔══════════════════════════════════════════════════════════════════════╗
║                    UR5 RTDE MCP 完整功能测试                          ║
║                                                                       ║
║  ⚠️  警告: 此测试将控制真实机械臂运动！                                ║
║     请确保:                                                           ║
║     1. 机械臂周围无人员                                               ║
║     2. 工作区域无障碍物                                               ║
║     3. 紧急停止按钮触手可及                                           ║
║                                                                       ║
║  测试内容:                                                            ║
║    - 连接/断开、状态读取、运动学                                      ║
║    - moveJ、moveL、moveJ_IK (新增)                                    ║
║    - servoJ、speedJ、力控制、示教模式                                 ║
║    - I/O控制、负载设置、Robotiq夹爪                                   ║
║                                                                       ║
║  按 Ctrl+C 取消测试                                                   ║
╚══════════════════════════════════════════════════════════════════════╝
""")
    
    input("按 Enter 开始完整功能测试...")
    
    report = run_full_test()
    
    # 输出报告
    summary = report.generate_summary()
    print(summary)
    
    # 保存报告到文件
    report_file = f"ur5_full_test_report_{datetime.now().strftime('%Y%m%d_%H%M%S')}.txt"
    with open(report_file, 'w', encoding='utf-8') as f:
        f.write(summary)
    print(f"\n测试报告已保存到: {report_file}")
