import tkinter as tk
from tkinter import ttk, filedialog, messagebox
import threading
import time
import os
import sys
import argparse
import concurrent.futures
from pymavlink import mavutil
# FTP not needed in this simplified GCS
HAS_FTP = False

DEFAULT_WAYPACK_PATH = "/home/liaoluo/ardupilot/Tools/autotest/waypack.txt"

def load_default_config_from_file():
    paths_to_check = ["waypack.txt", DEFAULT_WAYPACK_PATH]
    found_path = None
    for p in paths_to_check:
        if os.path.exists(p):
            found_path = p
            break
    if not found_path:
        return {}, None
    new_configs = {}
    with open(found_path, 'r') as f:
        for line in f:
            parts = line.strip().split(',')
            if parts:
                try:
                    sysid = int(parts[0])
                    port = 14550 + (sysid - 1) * 10
                    new_configs[sysid] = f"udp:127.0.0.1:{port}"
                except ValueError:
                    pass
    return new_configs, found_path

def format_log(message):
    timestamp = time.strftime("%H:%M:%S", time.localtime())
    return f"[{timestamp}] {message}"

def _connect_one_drone(sysid, conn_str, log):
    """Helper function to connect to a single drone."""
    log(f"正在连接无人机 {sysid} ({conn_str})...")
    drone = DroneInstance(sysid, conn_str)
    if drone.connect():
        log(f"无人机 {sysid} 连接成功。")
        return sysid, drone
    else:
        log(f"无人机 {sysid} 连接失败或未收到心跳。")
        return sysid, None

def connect_drones(drone_configs, log, max_workers=10):
    drones = {}
    with concurrent.futures.ThreadPoolExecutor(max_workers=max_workers) as executor:
        future_to_sysid = {executor.submit(_connect_one_drone, sysid, conn_str, log): sysid for sysid, conn_str in drone_configs.items()}
        
        for future in concurrent.futures.as_completed(future_to_sysid):
            sysid, drone = future.result()
            if drone:
                drones[sysid] = drone
                
    return drones




class DroneInstance:
    def __init__(self, sysid, connection_string):
        self.sysid = sysid
        self.connection_string = connection_string
        self.master = None
        self.ftp_master = None
        self.ftp = None
        self.connected = False
        self.lock = threading.Lock()
        self.data = {
            "mode": "未知",
            "global_pos": "N/A",
            "local_pos": "N/A",
            "alt": "N/A",
            "bat": "N/A",
            "last_heartbeat": 0,
            "group_name": "初始",
            "group_no": "初始"
        }

    def connect(self, timeout=5):
        try:
            self.master = mavutil.mavlink_connection(self.connection_string, source_system=255)
            hb = None
            deadline = time.time() + timeout
            while time.time() < deadline:
                msg = self.master.recv_match(type="HEARTBEAT", blocking=True, timeout=1)
                if msg and msg.get_srcSystem() == self.sysid:
                    hb = msg
                    break
            if hb is None:
                self.connected = False
                return False
            self.master.target_system = self.sysid
            self.master.target_component = 1
            self.data["last_heartbeat"] = time.time()
            self.connected = True
            return True
        except Exception as e:
            print(f"Error connecting to drone {self.sysid}: {e}")
            return False


class NetworkConfigDialog:
    def __init__(self, parent, drone_configs, on_save):
        self.window = tk.Toplevel(parent)
        self.window.title("无人机网络连接配置")
        self.window.geometry("500x400")
        self.drone_configs = drone_configs.copy() # copy for editing
        self.on_save = on_save
        
        self.setup_ui()

    def setup_ui(self):
        # 列表区域
        self.tree_frame = ttk.Frame(self.window)
        self.tree_frame.pack(fill='both', expand=True, padx=10, pady=10)
        
        columns = ("ID", "连接字符串 (Connection String)")
        self.tree = ttk.Treeview(self.tree_frame, columns=columns, show='headings')
        self.tree.heading("ID", text="ID")
        self.tree.column("ID", width=50, anchor='center')
        self.tree.heading("连接字符串 (Connection String)", text="连接字符串")
        self.tree.column("连接字符串 (Connection String)", width=350)
        self.tree.pack(side='left', fill='both', expand=True)
        
        scrollbar = ttk.Scrollbar(self.tree_frame, orient="vertical", command=self.tree.yview)
        scrollbar.pack(side='right', fill='y')
        self.tree.configure(yscrollcommand=scrollbar.set)
        
        self.tree.bind("<Double-1>", self.on_double_click)
        
        # 按钮区域
        btn_frame = ttk.Frame(self.window)
        btn_frame.pack(fill='x', padx=10, pady=10)
        
        ttk.Button(btn_frame, text="添加/修改", command=self.edit_entry).pack(side='left', padx=5)
        ttk.Button(btn_frame, text="删除选中", command=self.delete_entry).pack(side='left', padx=5)
        ttk.Button(btn_frame, text="保存并应用", command=self.save).pack(side='right', padx=5)
        
        self.refresh_list()

    def refresh_list(self):
        for item in self.tree.get_children():
            self.tree.delete(item)
        for sysid in sorted(self.drone_configs.keys()):
            self.tree.insert('', 'end', values=(sysid, self.drone_configs[sysid]))

    def on_double_click(self, event):
        self.edit_entry()

    def edit_entry(self):
        # 简易弹窗输入
        selected = self.tree.selection()
        initial_id = ""
        initial_str = "udp:127.0.0.1:14550"
        
        if selected:
            values = self.tree.item(selected[0], 'values')
            initial_id = values[0]
            initial_str = values[1]

        # 创建编辑弹窗
        edit_win = tk.Toplevel(self.window)
        edit_win.title("编辑连接")
        
        ttk.Label(edit_win, text="ID:").grid(row=0, column=0, padx=5, pady=5)
        id_entry = ttk.Entry(edit_win)
        id_entry.insert(0, initial_id)
        id_entry.grid(row=0, column=1, padx=5, pady=5)
        
        ttk.Label(edit_win, text="连接串:").grid(row=1, column=0, padx=5, pady=5)
        str_entry = ttk.Entry(edit_win, width=30)
        str_entry.insert(0, initial_str)
        str_entry.grid(row=1, column=1, padx=5, pady=5)
        
        def confirm():
            try:
                sysid = int(id_entry.get())
                conn_str = str_entry.get()
                self.drone_configs[sysid] = conn_str
                self.refresh_list()
                edit_win.destroy()
            except ValueError:
                messagebox.showerror("错误", "ID必须是数字")

        ttk.Button(edit_win, text="确定", command=confirm).grid(row=2, column=0, columnspan=2, pady=10)

    def delete_entry(self):
        selected = self.tree.selection()
        if selected:
            sysid = int(self.tree.item(selected[0], 'values')[0])
            del self.drone_configs[sysid]
            self.refresh_list()

    def save(self):
        self.on_save(self.drone_configs)
        self.window.destroy()

class SwarmGCSApp:
    def __init__(self, root):
        self.root = root
        self.root.title("集群无人机地面站 (基于MAVProxy)")
        self.root.geometry("1100x750")

        self.drones = {}  # sysid -> DroneInstance
        self.drone_configs = {} # sysid -> connection_string
        self.running = True
        
        self.setup_ui()
        self.start_mavlink_thread()

    def setup_ui(self):
        self.notebook = ttk.Notebook(self.root)
        self.notebook.pack(fill='both', expand=True, padx=5, pady=5)

        # 只保留调试标签页
        self.tab_debug = ttk.Frame(self.notebook)
        self.notebook.add(self.tab_debug, text="集群调试工具")
        self.setup_debug_tab()

    def setup_debug_tab(self):
        # 1. 网络连接配置
        config_frame = ttk.LabelFrame(self.tab_debug, text="网络连接配置")
        config_frame.pack(fill='x', padx=5, pady=5)
        
        ttk.Button(config_frame, text="读取默认配置 (waypack.txt)", command=self.load_default_config).pack(side='left', padx=5, pady=5)
        ttk.Button(config_frame, text="网络设置 (查看/修改)", command=self.open_network_settings).pack(side='left', padx=5)
        self.lbl_drone_count = ttk.Label(config_frame, text="配置数量: 0")
        self.lbl_drone_count.pack(side='left', padx=5)
        
        ttk.Button(config_frame, text="连接所有配置的无人机", command=self.connect_all).pack(side='left', padx=5)

        # 2. 实时状态显示
        status_frame = ttk.LabelFrame(self.tab_debug, text="实时状态显示")
        status_frame.pack(fill='both', expand=True, padx=5, pady=5)
        
        control_sub_frame = ttk.Frame(status_frame)
        control_sub_frame.pack(fill='x', padx=5, pady=2)
        ttk.Label(control_sub_frame, text="位置显示格式: ").pack(side='left')
        self.pos_display_var = tk.StringVar(value="Global")
        ttk.Radiobutton(control_sub_frame, text="全局 (经纬度)", variable=self.pos_display_var, value="Global").pack(side='left', padx=5)
        ttk.Radiobutton(control_sub_frame, text="局部 (NED)", variable=self.pos_display_var, value="Local").pack(side='left', padx=5)
        
        columns = ("ID", "组名", "编号", "连接状态", "位置", "高度 (m)", "电量 (%)", "飞行模式")
        self.tree = ttk.Treeview(status_frame, columns=columns, show='headings')
        for col in columns:
            self.tree.heading(col, text=col)
            self.tree.column(col, width=120, anchor='center')
        
        scrollbar = ttk.Scrollbar(status_frame, orient="vertical", command=self.tree.yview)
        self.tree.configure(yscrollcommand=scrollbar.set)
        scrollbar.pack(side='right', fill='y')
        self.tree.pack(fill='both', expand=True, padx=5, pady=5)


        # 4. 自动任务指令
        auto_task_frame = ttk.LabelFrame(self.tab_debug, text="自动任务指令")
        auto_task_frame.pack(fill='x', padx=5, pady=5)
        
        auto_task_input_frame = ttk.Frame(auto_task_frame)
        auto_task_input_frame.pack(fill='x', padx=5, pady=5)

        ttk.Label(auto_task_input_frame, text="起飞高度 (米):").pack(side='left')
        self.entry_takeoff_alt = ttk.Entry(auto_task_input_frame, width=10)
        self.entry_takeoff_alt.pack(side='left', padx=5)
        self.entry_takeoff_alt.insert(0, "20")

        ttk.Button(auto_task_input_frame, text="自动起飞", command=self.start_auto_takeoff).pack(side='left', padx=5)
        

        # 5. 指令发送功能
        cmd_frame = ttk.LabelFrame(self.tab_debug, text="指令发送功能 (Mavlink)")
        cmd_frame.pack(fill='x', padx=5, pady=5)
        
        cmd_input_frame = ttk.Frame(cmd_frame)
        cmd_input_frame.pack(fill='x', padx=5, pady=5)
        
        ttk.Label(cmd_input_frame, text="目标无人机:").pack(side='left', padx=5)
        self.drone_select = ttk.Combobox(cmd_input_frame, values=["全部"], state="readonly", width=8)
        self.drone_select.set("全部")
        self.drone_select.pack(side='left', padx=5)
        
        ttk.Label(cmd_input_frame, text="输入指令:").pack(side='left')
        self.entry_command = ttk.Entry(cmd_input_frame, width=40)
        self.entry_command.pack(side='left', padx=5, fill='x', expand=True)
        self.entry_command.bind('<Return>', lambda e: self.send_swarm_command())
        self.entry_command.insert(0, "mode guided")
        
        ttk.Button(cmd_input_frame, text="发送指令", command=self.send_swarm_command).pack(side='left', padx=5)
        ttk.Label(cmd_input_frame, text="(支持: mode, takeoff, land, arm, disarm)").pack(side='left', padx=5)

        # 6. 日志输出
        log_frame = ttk.LabelFrame(self.tab_debug, text="输出面板")
        log_frame.pack(fill='both', expand=True, padx=5, pady=5)
        
        self.log_text = tk.Text(log_frame, height=8)
        self.log_text.pack(fill='both', expand=True, padx=5, pady=5)

    def log(self, message):
        timestamp = time.strftime("%H:%M:%S", time.localtime())
        self.log_text.insert('end', f"[{timestamp}] {message}\n")
        self.log_text.see('end')

    def open_network_settings(self):
        NetworkConfigDialog(self.root, self.drone_configs, self.update_drone_configs)

    def update_drone_configs(self, new_configs):
        self.drone_configs = new_configs
        self.update_drone_count_label()
        self.update_drone_selector()
        self.log(f"配置已更新，共 {len(self.drone_configs)} 台无人机。")

    def load_default_config(self):
        new_configs, found_path = load_default_config_from_file()
        if not found_path:
            self.log("错误: 未找到 waypack.txt 文件。")
            return

        try:
            self.drone_configs = new_configs
            self.update_drone_count_label()
            self.update_drone_selector()
            self.log(f"已从 {found_path} 加载默认配置。")

        except Exception as e:
            self.log(f"读取文件错误: {e}")

    def update_drone_count_label(self):
        self.lbl_drone_count.config(text=f"配置数量: {len(self.drone_configs)}")

    def update_drone_selector(self):
        ids = sorted(list(self.drone_configs.keys()))
        values = ["全部"] + [str(i) for i in ids]
        self.drone_select['values'] = values
        self.drone_select.set("全部")

    def connect_all(self):
        if not self.drone_configs:
            self.log("没有配置无人机。请先加载或设置。")
            return

        def log_async(message):
            self.root.after(0, lambda m=message: self.log(m))

        def register_drone(sysid, drone):
            self.drones[sysid] = drone


        def _connect_and_post_process(sysid, conn_str):
            if sysid in self.drones and self.drones[sysid].connected:
                return sysid, self.drones[sysid], True
            
            log_async(f"正在连接无人机 {sysid} ({conn_str})...")
            drone = DroneInstance(sysid, conn_str)
            if not drone.connect():
                log_async(f"无人机 {sysid} 连接失败或未收到心跳。")
                return sysid, None, False

            log_async(f"无人机 {sysid} 连接成功。")
            return sysid, drone, True

        def worker(max_workers=10):
            success_count = 0
            with concurrent.futures.ThreadPoolExecutor(max_workers=max_workers) as executor:
                futures = {executor.submit(_connect_and_post_process, sysid, conn_str) for sysid, conn_str in self.drone_configs.items()}

                for future in concurrent.futures.as_completed(futures):
                    sysid, drone, connected = future.result()
                    if connected and drone:
                        success_count += 1
                        self.root.after(0, register_drone, sysid, drone)
            
            log_async(f"{success_count}台无人机已成功连接")

        threading.Thread(target=worker, daemon=True).start()




    def start_auto_takeoff(self):
        try:
            alt = float(self.entry_takeoff_alt.get())
        except ValueError:
            self.log("错误: 起飞高度必须是一个有效的数字。")
            messagebox.showerror("输入错误", "起飞高度必须是一个有效的数字。")
            return

        if not self.drones:
            self.log("错误: 未连接任何无人机，无法执行自动起飞。")
            messagebox.showwarning("无连接", "未连接任何无人机。")
            return

        def log_async(message):
            self.root.after(0, lambda m=message: self.log(m))

        # 在后台线程中运行，以防UI阻塞
        threading.Thread(
            target=auto_takeoff_and_monitor,
            args=(self.drones, alt, log_async),
            daemon=True
        ).start()

    def send_swarm_command(self):
        cmd_str = self.entry_command.get()
        target_str = self.drone_select.get()
        
        targets = {}
        if target_str == "全部":
            targets = self.drones
        else:
            try:
                sysid = int(target_str)
                if sysid in self.drones:
                    targets = {sysid: self.drones[sysid]}
            except (ValueError, KeyError):
                pass
        
        if not targets:
            self.log("未选择有效的已连接无人机。")
            return
            
        # 使用统一的指令发送函数
        send_mav_command(targets, cmd_str, self.log)


    def start_mavlink_thread(self):
        self.thread = threading.Thread(target=self.mavlink_loop)
        self.thread.daemon = True
        self.thread.start()
        self.root.after(100, self.update_ui)

    def mavlink_loop(self):
        while self.running:
            for sysid, drone in self.drones.items():
                if drone.master:
                    if not drone.lock.acquire(blocking=False):
                        continue
                    while True:
                        try:
                            msg = drone.master.recv_match(blocking=False)
                            if not msg:
                                break
                            if msg.get_srcSystem() != sysid:
                                continue
                            
                            msg_type = msg.get_type()
                            
                            if msg_type == 'HEARTBEAT':
                                try:
                                    drone.data['mode'] = mavutil.mode_string_v10(msg)
                                except:
                                    drone.data['mode'] = f"Mode {msg.custom_mode}"
                                drone.data['custom_mode'] = msg.custom_mode
                                drone.data['armed'] = bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
                                drone.data['last_heartbeat'] = time.time()
                                drone.connected = True
                            
                            elif msg_type == 'GLOBAL_POSITION_INT':
                                lat = msg.lat / 1e7
                                lon = msg.lon / 1e7
                                alt = msg.relative_alt / 1000.0
                                drone.data['global_pos'] = f"{lat:.6f}, {lon:.6f}"
                                drone.data['alt'] = f"{alt:.1f}"
                            
                            elif msg_type == 'LOCAL_POSITION_NED':
                                x = msg.x
                                y = msg.y
                                z = msg.z
                                drone.data['local_pos'] = f"{x:.1f}, {y:.1f}, {z:.1f}"
                            
                            elif msg_type == 'SYS_STATUS':
                                bat = msg.battery_remaining
                                drone.data['bat'] = f"{bat}"
                                
                            elif msg_type == 'STATUSTEXT':
                                text = msg.text
                                self.root.after(0, lambda s=sysid, t=text: self.log(f"[无人机 {s} MSG]: {t}"))

                        except Exception:
                            break
                    drone.lock.release()
            time.sleep(0.02)

    def update_ui(self):
        if self.drone_configs:
            current_ids = sorted(list(self.drone_configs.keys()))

            for sysid in current_ids:
                drone = self.drones.get(sysid)
                status = "未连接"
                pos = "-"
                alt = "-"
                bat = "-"
                mode = "-"
                group_name = "初始"
                group_no = "初始"
                
                if drone:
                    is_alive = (time.time() - drone.data['last_heartbeat'] < 3)
                    status = "已连接" if is_alive else "连接断开"
                    pos = drone.data['global_pos'] if self.pos_display_var.get() == "Global" else drone.data['local_pos']
                    alt = drone.data['alt']
                    bat = drone.data['bat']
                    mode = drone.data['mode']
                    group_name = drone.data.get("group_name", group_name)
                    group_no = drone.data.get("group_no", group_no)

                values = (sysid, group_name, group_no, status, pos, alt, bat, mode)
                
                if self.tree.exists(sysid):
                    self.tree.item(sysid, values=values)
                else:
                    self.tree.insert('', 'end', iid=sysid, values=values)
        
        self.root.after(500, self.update_ui)

    def on_close(self):
        self.running = False
        self.root.destroy()
    

def send_mav_command(drones, cmd_str, log):
    if not cmd_str:
        return
        
    parts = cmd_str.strip().lower().split()
    cmd_type = parts[0]
    
    for sysid, drone in drones.items():
        if not drone or not drone.connected:
            log(f"[CMD] 无人机 {sysid} 未连接，跳过指令发送。")
            continue
            
        try:
            if cmd_type == "mode":
                if len(parts) < 2:
                    log(f"指令错误: mode 需要参数 (如 mode guided 或 mode 29)")
                    continue
                
                mode_input = parts[1].upper()
                mode_id = -1
                
                # 检查是否为数字模式ID
                if mode_input.isdigit():
                    mode_id = int(mode_input)
                else: # 否则按名称查找
                    mode_map = drone.master.mode_mapping()
                    if mode_map and mode_input in mode_map:
                        mode_id = mode_map[mode_input]
                    else:
                        log(f"[CMD] 无人机 {sysid} 模式 '{parts[1]}' 不在映射表中，请检查输入。")
                        continue
                
                if mode_id != -1:
                    drone.master.set_mode(mode_id)
                    log(f"[CMD] 无人机 {sysid} 切换模式至 {parts[1]}")

            elif cmd_type == "takeoff":
                alt = 10.0
                if len(parts) > 1:
                    try:
                        alt = float(parts[1])
                    except ValueError:
                        log(f"高度参数错误，使用默认值 10.0m")
                drone.master.mav.command_long_send(
                    sysid, 1,
                    mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
                    0, 0, 0, 0, 0, 0, 0, alt
                )
                log(f"[CMD] 无人机 {sysid} 发送起飞指令 (高度: {alt}m)")
                
            elif cmd_type == "land":
                drone.master.mav.command_long_send(
                    sysid, 1,
                    mavutil.mavlink.MAV_CMD_NAV_LAND,
                    0, 0, 0, 0, 0, 0, 0, 0
                )
                log(f"[CMD] 无人机 {sysid} 发送降落指令")
                
            elif cmd_type == "arm":
                drone.master.arducopter_arm()
                log(f"[CMD] 无人机 {sysid} 发送解锁(ARM)指令")
                
            elif cmd_type == "disarm":
                drone.master.arducopter_disarm()
                log(f"[CMD] 无人机 {sysid} 发送上锁(DISARM)指令")
            
            else:
                log(f"未知指令类型: {cmd_type}")
                break 
                
        except Exception as e:
            log(f"[CMD] 向无人机 {sysid} 发送指令失败: {e}")

def run_debug_mode(args):
    def log(msg):
        print(format_log(msg), flush=True)
    configs, found_path = load_default_config_from_file()
    if not found_path:
        log("错误: 未找到 waypack.txt 文件。")
        return 1
    if args.autolink or args.mavcmd:
        log(f"已从 {found_path} 加载默认配置。")
    drones = {}
    if args.autolink or args.mavcmd:
        drones = connect_drones(configs, log)
    if args.mavcmd:
        # 等待几秒确保所有无人机状态稳定
        log("等待2秒以确保连接稳定...")
        time.sleep(2)
        send_mav_command(drones, args.mavcmd, log)

    if args.autotakeoff:
        if not drones:
            log("错误: 未连接任何无人机，无法执行自动起飞。")
            return 1
        # 如果没有手动指定模式，先确保是GUIDED
        if not args.mavcmd or 'mode' not in args.mavcmd:
            send_mav_command(drones, "mode guided", log)
            time.sleep(1)
        auto_takeoff_and_monitor(drones, args.autotakeoff, log)
    return 0

def auto_takeoff_and_monitor(drones, takeoff_alt, log):
    log(f"启动自动起飞程序，目标高度: {takeoff_alt}米。")

    # 1. 切换到GUIDED模式并解锁
    for sysid, drone in drones.items():
        if not drone.connected:
            continue
        try:
            log(f"无人机 {sysid}: 切换到 GUIDED 模式...")
            mode_id = drone.master.mode_mapping().get('GUIDED', None)
            if mode_id is None:
                log(f"错误: 无法为无人机 {sysid} 找到 GUIDED 模式。")
                continue
            drone.master.set_mode(mode_id)
            time.sleep(0.5)

            log(f"无人机 {sysid}: 解锁...")
            drone.master.arducopter_arm()
            # 等待解锁确认
            drone.master.motors_armed_wait()
            log(f"无人机 {sysid}: 已解锁。")
            time.sleep(1)

        except Exception as e:
            log(f"无人机 {sysid} 在准备阶段失败: {e}")

    # 2. 发送起飞指令
    for sysid, drone in drones.items():
        if not drone.connected:
            continue
        try:
            log(f"无人机 {sysid}: 发送起飞指令...")
            drone.master.mav.command_long_send(
                sysid, 1,
                mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
                0, 0, 0, 0, 0, 0, 0, takeoff_alt
            )
        except Exception as e:
            log(f"无人机 {sysid} 发送起飞指令失败: {e}")

    # 3. 监控高度
    monitoring_drones = {sysid: drone for sysid, drone in drones.items() if drone.connected}
    start_time = time.time()
    timeout = 120  # 2分钟超时

    while monitoring_drones and time.time() - start_time < timeout:
        drones_to_remove = []
        for sysid, drone in monitoring_drones.items():
            msg = drone.master.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=1)
            if msg:
                current_alt = msg.relative_alt / 1000.0  # 转换为米
                log(f"无人机 {sysid}: 当前高度 {current_alt:.2f}m")
                if current_alt >= takeoff_alt * 0.95:
                    log(f"无人机 {sysid}: 高度 {takeoff_alt}m 到达。")
                    drones_to_remove.append(sysid)
        
        for sysid in drones_to_remove:
            del monitoring_drones[sysid]
            
        time.sleep(1)

    if not monitoring_drones:
        log("所有无人机均已到达目标高度。")
    else:
        log(f"自动起飞超时。仍有 {len(monitoring_drones)} 架无人机未到达高度。")

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--debug", action="store_true")
    parser.add_argument("--autolink", action="store_true")
    parser.add_argument("--mavcmd", type=str, help="向所有无人机发送MAVLink指令 (例如 'mode guided', 'takeoff 20')")
    parser.add_argument("--autotakeoff", type=float, help="连接后自动起飞到指定高度 (米)")

    args = parser.parse_args()

    if args.debug:
        code = run_debug_mode(args)
        sys.exit(code)

    root = tk.Tk()
    app = SwarmGCSApp(root)
    root.protocol("WM_DELETE_WINDOW", app.on_close)

    if args.autolink or args.mavcmd:
        def auto_connect():
            app.load_default_config()
            app.connect_all()
            if args.mavcmd:
                app.entry_command.delete(0, tk.END)
                app.entry_command.insert(0, args.mavcmd)
                # 等待连接建立
                root.after(2000, app.send_swarm_command)
        root.after(200, auto_connect)

    root.mainloop()

if __name__ == "__main__":
    main()
