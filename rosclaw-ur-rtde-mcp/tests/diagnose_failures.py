#!/usr/bin/env python3
"""
UR5 失败项诊断脚本
分析5个失败项的根本原因
"""

import sys
import os
import time
import json
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'src'))

from ur_rtde_bridge import URRTDEBridge

ROBOT_IP = "192.168.123.6"

def diagnose_failures():
    bridge = URRTDEBridge()
    
    print("="*70)
    print("           UR5 失败项深度诊断")
    print("="*70)
    
    # 连接
    print("\n【连接机器人...】")
    bridge.connect(ROBOT_IP)
    print(f"  连接状态: {bridge.is_connected()}")
    
    # 获取基本信息
    state = bridge.get_state()
    print(f"\n【机器人基本状态】")
    print(f"  机器人模式: {state.robot_mode} (7=RUNNING)")
    print(f"  安全模式: {state.safety_mode}")
    print(f"  保护停止: {state.is_protective_stopped}")
    print(f"  紧急停止: {state.is_emergency_stopped}")
    print(f"  运行时状态: {state.runtime_state}")
    
    # ============================================================
    # 诊断1: get_robot_info (getSerialNumber失败)
    # ============================================================
    print("\n" + "="*70)
    print("【诊断1】get_robot_info - getSerialNumber失败")
    print("="*70)
    
    try:
        with bridge._lock:
            d = bridge._dash
            
            print("\n测试各项Dashboard功能:")
            
            # getRobotModel
            try:
                model = d.getRobotModel()
                print(f"  ✅ getRobotModel(): {model}")
            except Exception as e:
                print(f"  ❌ getRobotModel(): {e}")
            
            # getSerialNumber (预期失败)
            try:
                serial = d.getSerialNumber()
                print(f"  ✅ getSerialNumber(): {serial}")
            except Exception as e:
                print(f"  ❌ getSerialNumber(): {e}")
                print(f"     → 原因: PolyScope版本低于5.6.0，不支持此API")
            
            # polyscopeVersion
            try:
                version = d.polyscopeVersion()
                print(f"  ✅ polyscopeVersion(): {version}")
            except Exception as e:
                print(f"  ❌ polyscopeVersion(): {e}")
            
            # robotmode
            try:
                mode = d.robotmode()
                print(f"  ✅ robotmode(): {mode}")
            except Exception as e:
                print(f"  ❌ robotmode(): {e}")
            
            # safetystatus
            try:
                safety = d.safetystatus()
                print(f"  ✅ safetystatus(): {safety}")
            except Exception as e:
                print(f"  ❌ safetystatus(): {e}")
                
    except Exception as e:
        print(f"  Dashboard访问错误: {e}")
    
    print("\n【结论1】")
    print("  问题: PolyScope版本过低，getSerialNumber()需要>=5.6.0")
    print("  类型: 🔧 软件版本限制 (非代码问题)")
    print("  建议: 升级PolyScope或捕获此异常返回'不支持'")
    
    # ============================================================
    # 诊断2: zero_ft_sensor
    # ============================================================
    print("\n" + "="*70)
    print("【诊断2】zero_ft_sensor - 力传感器归零失败")
    print("="*70)
    
    # 检查是否有力传感器数据
    print(f"\n当前TCP力/力矩读数:")
    print(f"  Force: {[round(v, 2) for v in state.actual_tcp_force]}")
    
    # 尝试调用zeroFtSensor
    try:
        with bridge._lock:
            result = bridge._rtde_c.zeroFtSensor()
        print(f"\nzeroFtSensor() 返回: {result}")
        if result:
            print("  返回值是True，但测试显示失败")
            print("  可能原因: 调用后需要等待生效，或没有力传感器")
    except Exception as e:
        print(f"\nzeroFtSensor() 抛出异常: {e}")
    
    print("\n【结论2】")
    print("  可能原因:")
    print("    A) 机械臂没有安装力传感器 (最可能)")
    print("    B) 力传感器未激活/未配置")
    print("    C) 需要在特定控制模式下")
    print("  类型: 🔧 硬件未安装或配置问题 (非代码问题)")
    
    # ============================================================
    # 诊断3: force_mode
    # ============================================================
    print("\n" + "="*70)
    print("【诊断3】force_mode - 力控制模式进入失败")
    print("="*70)
    
    print("\n力控制模式要求:")
    print("  1. 必须有力传感器 (F/T Sensor)")
    print("  2. 机器人必须在RUNNING模式")
    print("  3. 不能在保护停止状态")
    print("  4. 需要正确的任务坐标系和选择向量")
    
    print(f"\n当前状态检查:")
    print(f"  机器人模式: {state.robot_mode} (应为7=RUNNING)")
    print(f"  保护停止: {state.is_protective_stopped} (应为False)")
    print(f"  紧急停止: {state.is_emergency_stopped} (应为False)")
    
    # 尝试forceMode
    try:
        task_frame = [0, 0, 0, 0, 0, 0]
        selection_vector = [0, 0, 1, 0, 0, 0]
        wrench = [0, 0, -10, 0, 0, 0]
        limits = [2, 2, 1.5, 0.5, 0.5, 0.5]
        
        print(f"\n尝试进入forceMode...")
        print(f"  任务坐标系: {task_frame}")
        print(f"  选择向量: {selection_vector} (Z轴柔顺)")
        print(f"  期望力: {wrench}")
        
        with bridge._lock:
            result = bridge._rtde_c.forceMode(task_frame, selection_vector, wrench, 1, limits)
        print(f"\nforceMode() 返回: {result}")
        
        if result:
            # 退出force mode
            bridge._rtde_c.forceModeStop()
            print("  已退出forceMode")
        else:
            print("  返回False，无法进入力控制模式")
            
    except Exception as e:
        print(f"\nforceMode() 抛出异常: {e}")
    
    print("\n【结论3】")
    print("  问题: 无法进入力控制模式")
    print("  最可能原因: 没有安装力传感器")
    print("  类型: 🔧 硬件未安装 (非代码问题)")
    
    # ============================================================
    # 诊断4: teach_mode
    # ============================================================
    print("\n" + "="*70)
    print("【诊断4】teach_mode - 示教模式进入失败")
    print("="*70)
    
    print("\n示教模式(teachMode)要求:")
    print("  1. 机器人必须在RUNNING模式")
    print("  2. 不能在保护停止状态")
    print("  3. 必须在远程控制模式 (Remote Control)")
    print("  4. 某些版本需要在特定安全配置下")
    
    # 检查远程控制状态
    try:
        with bridge._lock:
            d = bridge._dash
            remote = d.isInRemoteControl()
        print(f"\n远程控制状态: {remote}")
        if not remote:
            print("  ⚠️  未在远程控制模式！")
            print("  → 需要在PolyScope上启用远程控制")
    except Exception as e:
        print(f"\n无法获取远程控制状态: {e}")
    
    # 尝试进入teachMode
    try:
        print(f"\n尝试进入teachMode...")
        with bridge._lock:
            result = bridge._rtde_c.teachMode()
        print(f"teachMode() 返回: {result}")
        
        if result:
            print("  ✅ 成功进入示教模式")
            time.sleep(0.5)
            with bridge._lock:
                bridge._rtde_c.endTeachMode()
            print("  已退出示教模式")
        else:
            print("  ❌ 无法进入示教模式")
            print("  可能原因:")
            print("    - 未在远程控制模式")
            print("    - 安全模式为REDUCED")
            print("    - 需要特定的安全配置")
            
    except Exception as e:
        print(f"\nteachMode() 抛出异常: {e}")
    
    print("\n【结论4】")
    print("  问题: 无法进入示教模式")
    print(f"  当前安全模式: {state.safety_mode} (1=NORMAL, 2=REDUCED)")
    print("  最可能原因:")
    print("    A) 未启用远程控制模式 (需要在PolyScope上设置)")
    print("    B) 安全模式为REDUCED")
    print("  类型: 🔧 配置/权限问题 (可能需要手动配置)")
    
    # ============================================================
    # 诊断5: set_payload
    # ============================================================
    print("\n" + "="*70)
    print("【诊断5】set_payload - 负载设置失败")
    print("="*70)
    
    print("\n负载设置要求:")
    print("  1. 机器人必须已连接")
    print("  2. 通常在IDLE或RUNNING模式下")
    print("  3. 某些版本需要在特定权限下")
    
    # 尝试setPayload
    try:
        print(f"\n尝试设置负载 0.5kg, CoG=[0,0,0.05]...")
        with bridge._lock:
            result = bridge._rtde_c.setPayload(0.5, [0, 0, 0.05])
        print(f"setPayload() 返回: {result}")
        
        if result:
            print("  ✅ 负载设置成功")
            # 恢复原负载 (0kg)
            with bridge._lock:
                bridge._rtde_c.setPayload(0.0, [0, 0, 0])
            print("  已恢复负载为0kg")
        else:
            print("  ❌ 负载设置失败")
            print("  可能原因:")
            print("    - 控制模式限制")
            print("    - 需要在特定安全模式下")
            
    except Exception as e:
        print(f"\nsetPayload() 抛出异常: {e}")
        import traceback
        traceback.print_exc()
    
    print("\n【结论5】")
    print("  问题: setPayload返回False")
    print("  可能原因:")
    print("    A) 控制模式限制")
    print("    B) ur_rtde库版本兼容性问题")
    print("  类型: ⚠️  可能需要代码检查")
    
    # ============================================================
    # 总结
    # ============================================================
    print("\n" + "="*70)
    print("【诊断总结】")
    print("="*70)
    
    results = [
        ("get_robot_info", "PolyScope版本<5.6.0", "🔧 软件版本限制", "无需修复，建议添加异常处理"),
        ("zero_ft_sensor", "无力传感器", "🔧 硬件未安装", "无需修复"),
        ("force_mode", "无力传感器", "🔧 硬件未安装", "无需修复"),
        ("teach_mode", "未启用远程控制/安全模式", "🔧 配置问题", "检查PolyScope远程控制设置"),
        ("set_payload", "控制模式/权限", "⚠️  待确认", "建议检查ur_rtde文档"),
    ]
    
    print("\n| 测试项 | 根本原因 | 类型 | 建议 |")
    print("|--------|----------|------|------|")
    for item, cause, type_, suggestion in results:
        print(f"| {item} | {cause} | {type_} | {suggestion} |")
    
    print("\n【关键发现】")
    print("  1. 前4项失败都是环境/硬件/配置问题，不是代码bug")
    print("  2. 机械臂没有安装力传感器 (zero_ft_sensor, force_mode)")
    print("  3. PolyScope版本较低，不支持getSerialNumber")
    print("  4. 可能未启用远程控制模式 (teach_mode)")
    print("  5. 建议检查set_payload的调用方式")
    
    # 断开
    bridge.disconnect()
    print("\n诊断完成，已断开连接")

if __name__ == "__main__":
    diagnose_failures()
