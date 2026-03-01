#!/usr/bin/env bash
# 启动 5 个 SITL 实例，端口分别为 14550, 14560, 14570, 14580, 14590
# 每个实例使用 Tools/autotest/sim_vehicle.py 启动，输出日志到 build/sitl/sitl_N.log

set -euo pipefail
ROOT_DIR="$(cd "$(dirname "$0")/../.." && pwd)"
cd "$ROOT_DIR"

BASE_PORT=14550
PORT_STEP=10
NUM=5
PYTHON=python3
SIM=Tools/autotest/sim_vehicle.py

mkdir -p build/sitl_logs

pids=()
for i in $(seq 0 $((NUM-1))); do
    port=$((BASE_PORT + i * PORT_STEP))
    instance=$i
    logfile="build/sitl_logs/sitl_${instance}.log"
    echo "Starting SITL instance ${instance} -> port ${port} (log: ${logfile})"
    # 启动 SITL，并将 MAVLink 输出到 udpout:127.0.0.1:PORT
    nohup $PYTHON $SIM -v ArduCopter -f --instance ${instance} --out=udpout:127.0.0.1:${port} > ${logfile} 2>&1 &
    pids+=("$!")
    # 给每个实例一点时间启动
    sleep 1
done

# 等待全部实例初始化
echo "等待 8 秒让 SITL 实例启动并开始发送心跳..."
sleep 8

# 显示运行的 PID 列表
echo "启动完成，SITL PID: ${pids[*]}"

echo "现在启动 swarm_control.py 来连接这些端口 (14550,14560,...)
"
# 运行 swarm 控制脚本（在当前终端，可改为后台）
# 注意：脚本会等待用户按回车进行解锁/起飞/切换模式
python3 ArduCopter/swarm_control.py

# 如果脚本退出，继续不杀 SITL，用户可手动清理或关闭终端
echo "swarm_control.py 退出。SITL 仍在运行，若需停止请 kill PIDs: ${pids[*]}"
