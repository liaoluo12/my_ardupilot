#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
cluster_control.py: ArduPilot 无人机集群控制脚本

该脚本使用 Pymavlink 库连接到多个正在运行的 ArduPilot 仿真 (SITL) 实例，
并以并行方式对它们执行一系列基本指令，例如切换模式、解锁、起飞和移动。

主要功能:
- 通过 UDP 端口连接到多个无人机。
- 并行执行命令，并等待所有无人机完成。
- 监控每个无人机的状态（模式、解锁状态、高度）。
- 执行一个预设的任务序列：切换到 GUIDED 模式 -> 解锁 -> 起飞 -> 向前移动 -> 切换到自定义模式 29。
"""

import argparse
import sys
import time
import math

from pymavlink import mavutil


class VehicleLink:
    """
    代表与单个无人机的 MAVLink 连接。
    这个类封装了连接、发送指令和轮询状态所需的所有功能。
    """
    def __init__(self, port, timeout):
        """
        初始化一个无人机连接。

        Args:
            port (int): 连接无人机的 UDP 端口号。
            timeout (float): 等待心跳包的超时时间（秒）。
        """
        self.port = port
        print(f"正在连接到端口 {port} 上的无人机...")
        try:
            # 建立 MAVLink 连接，使用 UDP-in 模式
            self.mav = mavutil.mavlink_connection(f"udpin:127.0.0.1:{port}")
            # 等待心跳包以确认连接成功
            if self.mav.wait_heartbeat(timeout=timeout) is None:
                raise RuntimeError(f"在端口 {port} 上等待心跳包超时")
            print(f"已连接到无人机 {self.mav.target_system} (端口: {port})")
        except Exception as e:
            # 如果是心跳包超时，给出针对 WSL2 用户的特别提示
            if "Heartbeat timeout" in str(e):
                print(f"错误: 在端口 {port} 上等待心跳包超时。")
                print("提示: 如果在 WSL2 中运行，MAVProxy 可能正在将数据包转发到 Windows 主机。")
                print("      请尝试使用 --no-wsl2-network 参数运行 sim_vehicle.py 以启用本地连接。")
            raise
        self.sysid = self.mav.target_system
        self.compid = self.mav.target_component
        self.last_heartbeat = None
        self.last_position = None

    def set_mode(self, mode):
        """
        设置无人机的飞行模式。

        Args:
            mode (str or int): 目标模式的字符串（如 "GUIDED"）或数字。
        """
        self.mav.set_mode(mode)

    def ensure_mode(self, target_mode, timeout=10):
        """
        确保无人机已切换到目标模式，会重试直到成功或超时。

        Args:
            target_mode (str): 目标飞行模式。
            timeout (int): 超时时间（秒）。

        Returns:
            bool: 如果成功切换则返回 True，否则返回 False。
        """
        print(f"无人机 {self.sysid}: 正在切换到 {target_mode} 模式...")
        start = time.time()
        while time.time() - start < timeout:
            self.set_mode(target_mode)
            self.poll()  # 更新状态
            current_mode, _, _ = self.status()
            if current_mode == target_mode:
                print(f"无人机 {self.sysid}: 已确认进入 {target_mode} 模式")
                return True
            time.sleep(1)
        print(f"无人机 {self.sysid}: 切换到 {target_mode} 模式失败！")
        return False

    def arm(self):
        """发送解锁指令。"""
        self.mav.mav.command_long_send(
            self.sysid,
            self.compid,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,  # 确认参数
            1,  # 1 表示解锁, 0 表示上锁
            0, 0, 0, 0, 0, 0  # 未使用的参数
        )

    def ensure_armed(self, timeout=20):
        """
        确保无人机已解锁，会重试直到成功或超时。

        Args:
            timeout (int): 超时时间（秒）。

        Returns:
            bool: 如果成功解锁则返回 True，否则返回 False。
        """
        print(f"无人机 {self.sysid}: 正在解锁...")
        start = time.time()
        while time.time() - start < timeout:
            self.arm()
            self.poll()
            _, armed_status, _ = self.status()
            if armed_status == "ARMED":
                print(f"无人机 {self.sysid}: 已解锁")
                return True
            time.sleep(1)
        print(f"无人机 {self.sysid}: 解锁失败！")
        return False

    def takeoff(self, alt):
        """
        发送起飞指令。

        Args:
            alt (float): 目标起飞高度（米）。
        """
        self.mav.mav.command_long_send(
            self.sysid,
            self.compid,
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
            0,  # 确认参数
            0, 0, 0, 0,  # 未使用的参数
            0, 0,        # Yaw, Latitude, Longitude (未使用)
            alt          # 起飞高度
        )

    def land(self):
        """发送降落指令。"""
        self.mav.mav.command_long_send(
            self.sysid,
            self.compid,
            mavutil.mavlink.MAV_CMD_NAV_LAND,
            0,
            0, 0, 0, 0, 0, 0, 0 # 未使用的参数
        )

    def move_body(self, x, y, z):
        """
        在机体坐标系下移动无人机。

        Args:
            x (float): 向前移动的距离（米）。
            y (float):向右移动的距离（米）。
            z (float): 向下移动的距离（米）。
        """
        # type_mask 用于指定哪些字段是有效的。
        # 我们只关心位置 (x, y, z)，忽略速度、加速度、偏航角和偏航速率。
        # 0b110111111000 = 3576
        # 位掩码的含义 (从右到左，第0位开始):
        # 位 0-2 (X, Y, Z): 0 表示有效
        # 位 3-5 (VX, VY, VZ): 1 表示忽略
        # 位 6-8 (AX, AY, AZ): 1 表示忽略
        # 位 9 (Force Set): 未使用
        # 位 10 (Yaw): 1 表示忽略
        # 位 11 (Yaw Rate): 1 表示忽略
        self.mav.mav.set_position_target_local_ned_send(
            0,  # time_boot_ms (未使用, 设为0)
            self.sysid, self.compid,
            mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED, # 坐标系：相对于机体当前位置的NED坐标系
            0b110111111000, # type_mask
            x, y, z,  # 位置 (m)
            0, 0, 0,  # 速度 (m/s)
            0, 0, 0,  # 加速度 (m/s^2)
            0, 0      # 偏航角, 偏航速率
        )

    def poll(self):
        """
        轮询并处理来自无人机的 MAVLink 消息。
        只处理心跳包和全局位置信息，以更新无人机的最新状态。
        """
        msg = self.mav.recv_match(type=["HEARTBEAT", "GLOBAL_POSITION_INT"], blocking=False)
        while msg is not None:
            if msg.get_type() == "HEARTBEAT":
                self.last_heartbeat = msg
            elif msg.get_type() == "GLOBAL_POSITION_INT":
                self.last_position = msg
            msg = self.mav.recv_match(type=["HEARTBEAT", "GLOBAL_POSITION_INT"], blocking=False)

    def status(self):
        """
        获取无人机的当前状态摘要。

        Returns:
            tuple: (模式字符串, 解锁状态字符串, 高度字符串)
        """
        mode = "UNKNOWN"
        armed = "UNKNOWN"
        if self.last_heartbeat is not None:
            mode = mavutil.mode_string_v10(self.last_heartbeat)
            armed = "ARMED" if self.last_heartbeat.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED else "DISARMED"
        
        alt = "N/A"
        if self.last_position is not None:
            # relative_alt 单位是毫米，转换为米
            alt = f"{self.last_position.relative_alt / 1000.0:.1f}m"
        
        return mode, armed, alt

    def is_armed(self):
        """检查无人机是否已解锁。"""
        if self.last_heartbeat is None:
            return False
        return bool(self.last_heartbeat.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)

    def get_mode(self):
        """获取当前飞行模式字符串。"""
        if self.last_heartbeat is None:
            return None
        return mavutil.mode_string_v10(self.last_heartbeat)

    def get_custom_mode(self):
        """获取当前自定义模式编号。"""
        if self.last_heartbeat is None:
            return None
        return self.last_heartbeat.custom_mode

    def get_alt(self):
        """获取当前相对高度（米）。"""
        if self.last_position is None:
            return None
        return self.last_position.relative_alt / 1000.0

    def get_location(self):
        """获取当前的 GLOBAL_POSITION_INT 消息对象。"""
        return self.last_position


def get_distance_metres(loc1, loc2):
    """
    计算两个 GLOBAL_POSITION_INT 位置之间的近似水平距离。
    这是一个简化的计算，适用于小距离。
    """
    if loc1 is None or loc2 is None:
        return 0.0
    # 纬度差和经度差（单位是度 * 1e7）
    dlat = loc1.lat - loc2.lat
    dlon = loc1.lon - loc2.lon
    # 将经纬度差转换为米
    return math.sqrt(dlat * dlat + dlon * dlon) * 1.113195e-2


def parse_args():
    """解析命令行参数。"""
    parser = argparse.ArgumentParser(description="ArduPilot 无人机集群控制脚本")
    parser.add_argument("--ports", default="14550,14560,14570", help="逗号分隔的无人机 UDP 端口列表。")
    parser.add_argument("--alt", type=float, default=20.0, help="起飞的目标高度（米）。")
    parser.add_argument("--dist", type=float, default=50.0, help="向前移动的目标距离（米）。")
    parser.add_argument("--timeout", type=float, default=30.0, help="连接和操作的默认超时时间（秒）。")
    return parser.parse_args()


def execute_step(vehicles, action_func, check_func, timeout, label):
    """
    在所有无人机上并行执行一个步骤，并带有重试机制。

    Args:
        vehicles (list): VehicleLink 对象列表。
        action_func (function): 对单个无人机执行操作的函数 (例如 v.set_mode)。如果检查失败，会周期性调用。
        check_func (function): 检查无人机是否准备就绪的函数。返回 (bool, status_string)。
        timeout (int): 此步骤的超时时间（秒）。
        label (str): 用于在控制台输出中标识此步骤的标签。

    Returns:
        bool: 如果所有无人机都成功完成步骤，则返回 True。
    """
    print(f"\n=== {label} ===")
    start = time.time()
    last_action_time = 0
    
    while time.time() - start < timeout:
        # 每隔 1 秒对尚未准备好的无人机重试操作
        if time.time() - last_action_time > 1.0:
            for v in vehicles:
                v.poll()
                is_ready, _ = check_func(v)
                if not is_ready:
                    if action_func:
                        action_func(v)
            last_action_time = time.time()

        # 检查所有无人机的状态
        all_ok = True
        status_list = []
        for v in vehicles:
            v.poll()
            ok, status_str = check_func(v)
            if not ok:
                all_ok = False
            status_list.append(f"V{v.sysid}:{status_str}")
        
        # 清除当前行并打印最新状态，实现动态刷新效果
        print(f"\r状态: " + " | ".join(status_list) + "   ", end="")
        sys.stdout.flush()
        
        if all_ok:
            print(f"\n{label} 完成。")
            return True
        time.sleep(0.1)
    
    print(f"\n{label} 失败 (超时)。")
    return False


def main():
    """主执行函数"""
    args = parse_args()
    ports = [int(p.strip()) for p in args.ports.split(",") if p.strip()]
    if len(ports) == 0:
        raise RuntimeError("未提供任何端口")

    # --- 步骤 0: 连接到所有无人机 ---
    vehicles = [VehicleLink(p, args.timeout) for p in ports]

    # --- 步骤 1: 切换到 GUIDED 模式 ---
    def check_guided(v):
        m = v.get_mode()
        return m == "GUIDED", m if m else "无心跳"

    if not execute_step(vehicles, 
                        lambda v: v.set_mode("GUIDED"), 
                        check_guided, 
                        timeout=15, 
                        label="切换到 GUIDED 模式"):
        print("任务中止: 并非所有无人机都进入了 GUIDED 模式。")
        return

    # --- 步骤 2: 解锁 ---
    def check_armed(v):
        armed = v.is_armed()
        return armed, "已解锁" if armed else "未解锁"

    # 增加解锁的超时时间到 60 秒，以允许 EKF 和 GPS 初始化
    if not execute_step(vehicles, 
                        lambda v: v.arm(), 
                        check_armed, 
                        timeout=60, 
                        label="解锁无人机"):
        print("任务中止: 并非所有无人机都已解锁。")
        return

    # 等待电机旋转和状态稳定
    print("等待状态稳定...")
    time.sleep(5)

    # --- 步骤 3: 起飞 ---
    def check_altitude(v):
        alt = v.get_alt()
        if alt is None:
            return False, "无GPS"
        # 检查高度是否达到目标高度的 90%
        return alt >= args.alt * 0.9, f"{alt:.1f}m"

    if not execute_step(vehicles, 
                        lambda v: v.takeoff(args.alt), 
                        check_altitude, 
                        timeout=60, 
                        label=f"起飞至 {args.alt}m"):
        print("任务中止: 起飞失败。")
        return

    # --- 步骤 4: 向前移动 ---
    start_locs = {}
    for v in vehicles:
        # 确保在移动前获取到初始位置
        while v.get_location() is None:
            v.poll()
            time.sleep(0.1)
        start_locs[v] = v.get_location()

    def check_distance(v):
        curr = v.get_location()
        if curr is None:
            return False, "无位置"
        dist = get_distance_metres(start_locs[v], curr)
        # 检查移动距离是否达到目标距离的容差范围内 (这里是 -2.0 米)
        return dist >= args.dist - 2.0, f"{dist:.1f}m"

    # 发送移动指令
    if not execute_step(vehicles, 
                        lambda v: v.move_body(args.dist, 0, 0), 
                        check_distance, 
                        timeout=60, 
                        label=f"向前移动 {args.dist}m"):
        print("警告: 移动超时。")
        # 即使超时，我们仍然继续执行下一步

    # --- 步骤 5: 切换到自定义模式 29 ---
    # 这是一个示例步骤，用于演示如何切换到非标准模式
    def check_mode_29(v):
        m = v.get_custom_mode()
        return m == 29, f"{m}" if m is not None else "无心跳"

    if not execute_step(vehicles, 
                        lambda v: v.set_mode(29), 
                        check_mode_29, 
                        timeout=30, 
                        label="切换到模式 29"):
        print("警告: 切换到模式 29 超时。")

    print("\n任务完成。")


if __name__ == "__main__":
    main()