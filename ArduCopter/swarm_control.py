#!/usr/bin/env python3
from pymavlink import mavutil
import time
import sys

# 默认配置：5 架飞机的端口 (ID0=14550, ID1=14560, 每个实例端口间隔 10)
# 可以通过命令行参数覆盖
DEFAULT_BASE_PORT = 14550
DEFAULT_PORT_STEP = 10
DEFAULT_NUM_VEHICLES = 5

# 全局变量，稍后由 main() 设置
BASE_PORT = DEFAULT_BASE_PORT
PORT_STEP = DEFAULT_PORT_STEP
NUM_VEHICLES = DEFAULT_NUM_VEHICLES

def connect_vehicles():
    vehicles = []
    print(f"正在连接 {NUM_VEHICLES} 架无人机...")
    
    for i in range(NUM_VEHICLES):
        port = BASE_PORT + i * PORT_STEP
        sysid = i + 1
        print(f"  -> 连接车辆 {sysid} (端口 {port})...", end=' ')
        try:
            # 尝试多种连接 URI（UDP / TCP / udpin），提高与不同 sim/MAVProxy 配置的兼容性
            tried = []
            conn = None
            per_vehicle_timeout = 10.0
            t0 = time.time()
            # 循环尝试直到超时
            while time.time() - t0 < per_vehicle_timeout and conn is None:
                for uri in (f'udp:127.0.0.1:{port}', f'tcp:127.0.0.1:{port}', f'udpin:{port}', f'udp:0.0.0.0:{port}'):
                    if time.time() - t0 >= per_vehicle_timeout:
                        break
                    tried.append(uri)
                    try:
                        conn = mavutil.mavlink_connection(uri)
                        # 等待心跳，若成功则跳出
                        conn.wait_heartbeat(timeout=2)
                        break
                    except Exception:
                        conn = None
                if conn is None:
                    # 睡一小会再重试全部 URI
                    time.sleep(0.5)
            if conn is None:
                raise RuntimeError(f"无法连接，尝试过: {list(dict.fromkeys(tried))}")
            # 打印连接到的系统/组件信息，便于调试
            print("成功!", end=' ')
            try:
                print(f"(sys={conn.target_system} comp={conn.target_component})")
            except Exception:
                print("")
            vehicles.append(conn)
        except Exception as e:
            print(f"失败: {e}")
            vehicles.append(None)
            
    return vehicles

def arm_vehicle(vehicle):
    """手动发送解锁指令"""
    vehicle.mav.command_long_send(
        vehicle.target_system, vehicle.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0, 1, 0, 0, 0, 0, 0, 0, 0
    )

def takeoff_vehicle(vehicle, altitude):
    """手动发送起飞指令，altitude 为相对于 home 的高度米数"""
    # note: command_long_send takes confirmation + 7 params (p1..p7)
    # the takeoff altitude is p7
    vehicle.mav.command_long_send(
        vehicle.target_system, vehicle.target_component,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
        0,              # confirmation
        0, 0, 0, 0, 0, 0,
        altitude        # param7 = altitude
    )

def set_mode_vehicle(vehicle, mode_id):
    """手动发送切换模式指令"""
    # ArduPilot 推荐使用 MAV_CMD_DO_SET_MODE
    vehicle.mav.command_long_send(
        vehicle.target_system, vehicle.target_component,
        mavutil.mavlink.MAV_CMD_DO_SET_MODE,
        0,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, # Base mode
        mode_id, # Custom mode (例如 29)
        0, 0, 0, 0, 0
    )


def wait_for_command_ack(vehicle, command, timeout=5):
    """等待 COMMAND_ACK 来确认命令是否被接受（返回 True/False）。"""
    start = time.time()
    while time.time() - start < timeout:
        try:
            msg = vehicle.recv_match(type='COMMAND_ACK', blocking=False)
        except Exception:
            msg = None
        if msg:
            try:
                if msg.command == command:
                    return int(msg.result) == 0
            except Exception:
                pass
        time.sleep(0.05)
    return False


def wait_for_mode(vehicle, desired_custom_mode, timeout=5):
    """等待 HEARTBEAT 中的 custom_mode 变为期望值。"""
    start = time.time()
    while time.time() - start < timeout:
        hb = vehicle.recv_match(type='HEARTBEAT', blocking=True, timeout=timeout)
        if not hb:
            return False
        try:
            if int(hb.custom_mode) == int(desired_custom_mode):
                return True
        except Exception:
            pass
    return False


def wait_for_armed(vehicle, desired=True, timeout=5):
    """等待 HEARTBEAT 显示 armed/ disarmed 状态。"""
    start = time.time()
    flag = mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
    while time.time() - start < timeout:
        try:
            hb = vehicle.recv_match(type='HEARTBEAT', blocking=False)
        except Exception:
            hb = None
        if hb:
            try:
                armed = (int(hb.base_mode) & flag) != 0
                if armed == desired:
                    return True
            except Exception:
                pass
        time.sleep(0.05)
    return False


def wait_for_alt_increase(vehicle, base_alt, delta=1.0, timeout=10):
    """等待 VFR_HUD.alt 超过 base_alt + delta。"""
    start = time.time()
    while time.time() - start < timeout:
        try:
            hud = vehicle.recv_match(type='VFR_HUD', blocking=False)
        except Exception:
            hud = None
        if hud:
            try:
                if float(hud.alt) >= base_alt + delta:
                    return True
            except Exception:
                pass
        time.sleep(0.05)
    return False

def main():
    global BASE_PORT, PORT_STEP, NUM_VEHICLES

    # 命令行参数支持，方便与不同 SITL 启动命令配合
    import argparse
    parser = argparse.ArgumentParser(description='控制多架 SITL 飞机的简易脚本')
    parser.add_argument('--base-port', type=int, default=DEFAULT_BASE_PORT,
                        help='第一个飞机的 MAVLink 端口 (默认 14550)')
    parser.add_argument('--step', type=int, default=DEFAULT_PORT_STEP,
                        help='每架飞机端口的步长 (默认 10)')
    parser.add_argument('--count', type=int, default=DEFAULT_NUM_VEHICLES,
                        help='要控制的飞机数量 (默认 5)')
    args = parser.parse_args()

    BASE_PORT = args.base_port
    PORT_STEP = args.step
    NUM_VEHICLES = args.count

    vehicles = connect_vehicles()
    
    # 过滤掉连接失败的车辆
    active_vehicles = [v for v in vehicles if v is not None]
    
    if not active_vehicles:
        print("没有连接任何车辆，退出。")
        return

    print(f"\n成功连接 {len(active_vehicles)} 架车辆。")

    GUIDED_MODE = 4
    AUTODRAW_MODE = 29

    # 1. 先把所有飞机切换到 GUIDED
    print(">>> 切换所有飞机到 GUIDED 模式...")
    for i, v in enumerate(active_vehicles):
        set_mode_vehicle(v, GUIDED_MODE)
        ok = wait_for_command_ack(v, mavutil.mavlink.MAV_CMD_DO_SET_MODE, timeout=3)
        print(f"  [车辆 {i+1}] 发送 GUIDED 切换 {'成功' if ok else '无 ACK'}")
        if ok:
            if wait_for_mode(v, GUIDED_MODE, timeout=5):
                print(f"  [车辆 {i+1}] 心跳显示 GUIDED")
            else:
                print(f"  [车辆 {i+1}] 未在心跳中检测到 GUIDED")
    
    # 2. 依次解锁并在短间隔内起飞（避免自动重新上锁）
    print(">>> 解锁并尽快起飞（GUIDED 下）...")
    initial_alts = [None] * len(active_vehicles)
    sysids = [getattr(v, 'target_system', None) for v in active_vehicles]
    # 依次发送起飞命令后，采用短窗口轮询每个连接以获取初始 VFR_HUD
    for i, v in enumerate(active_vehicles):
        arm_vehicle(v)
        ok = wait_for_armed(v, desired=True, timeout=5)
        print(f"  [车辆 {i+1}] 解锁 {'ACK' if ok else '无 ACK'}")
        # 快速起飞，间隔短（0.5s）以避免被重新锁定
        time.sleep(0.5)
        takeoff_vehicle(v, 10)
        print(f"  [车辆 {i+1}] 已发送起飞命令")
        # 可选：短期内等待高度变化以确认起飞已开始
        # 我们随后会在统一的初始高度收集中检测 VFR_HUD
        print(f"    [车辆 {i+1}] 起飞命令 已发送")
    # 非阻塞收集初始高度：在 small_window 秒内轮询各连接
    small_window = 6
    window_start = time.time()
    while time.time() - window_start < small_window and any(x is None for x in initial_alts):
        for idx, v in enumerate(active_vehicles):
            if initial_alts[idx] is not None:
                continue
            try:
                hud = v.recv_match(type='VFR_HUD', blocking=False)
            except Exception:
                hud = None
            if hud:
                try:
                    initial_alts[idx] = float(hud.alt)
                    print(f"    [车辆 {idx+1}] 初始高度基准: {hud.alt}")
                except Exception:
                    initial_alts[idx] = 0.0
        # 略微休眠以避免 busy loop
        time.sleep(0.1)
    # 对仍未获取到基准的设置为 0
    for idx in range(len(initial_alts)):
        if initial_alts[idx] is None:
            initial_alts[idx] = 0.0
            print(f"    [车辆 {idx+1}] 未收到初始 VFR_HUD，基准设为 0")

    # 3. 等待所有飞机达到起飞高度 (10m 相对高度)
    print(">>> 等待所有飞机爬升到 +10m（相对）...")
    target_reached = [False] * len(active_vehicles)
    timeout_s = 60
    start_time = time.time()
    # 更稳健的非阻塞轮询：收集所有 VFR_HUD，并根据索引更新状态
    while time.time() - start_time < timeout_s and not all(target_reached):
        for idx, v in enumerate(active_vehicles):
            if target_reached[idx]:
                continue
            try:
                hud = v.recv_match(type='VFR_HUD', blocking=False)
            except Exception:
                hud = None
            if not hud:
                continue
            try:
                current_alt = float(hud.alt)
                if current_alt >= initial_alts[idx] + 10.0:
                    target_reached[idx] = True
                    print(f"  [车辆 {idx+1}] 达到目标高度: {current_alt} (基准 {initial_alts[idx]})")
            except Exception:
                pass
        time.sleep(0.05)
    for idx, ok in enumerate(target_reached):
        if not ok:
            print(f"  [车辆 {idx+1}] 未在超时内达到 +10m")

    # 4. 同时切换到 AUTODRAW (Mode 29)
    print(">>> 同时切换到 Mode 29 (AutoDraw)...")
    for i, v in enumerate(active_vehicles):
        set_mode_vehicle(v, AUTODRAW_MODE)
    # 之后检查 ACK 与心跳
    for i, v in enumerate(active_vehicles):
        # 以心跳为准判断模式切换成功；若心跳检测失败，再尝试读取 COMMAND_ACK
        if wait_for_mode(v, AUTODRAW_MODE, timeout=5):
            print(f"  [车辆 {i+1}] 心跳显示 custom_mode=29")
        else:
            ok = wait_for_command_ack(v, mavutil.mavlink.MAV_CMD_DO_SET_MODE, timeout=2)
            if ok:
                print(f"  [车辆 {i+1}] 收到切换 ACK，但心跳未及时更新")
            else:
                print(f"  [车辆 {i+1}] 切换到 29 未收到 ACK 或心跳未更新")

    print("\n>>> 集群任务启动完成，请在地图或日志中观察行为。")
    
    # 保持脚本运行，以便查看后续日志（可选）
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print("\n脚本停止。")

if __name__ == "__main__":
    main()
