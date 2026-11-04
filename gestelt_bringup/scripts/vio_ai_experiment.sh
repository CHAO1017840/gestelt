#!/usr/bin/env bash
# Gestelt GPS-denied VIO + AI 速度回退仿真一键启动脚本
# 对应文档: gestelt/AI速度回退说明.txt
#
# 用法:
#   vio_ai_experiment.sh [选项]     # 新建 tmux 会话并自动拉起整套仿真
#   vio_ai_experiment.sh stop       # 关闭整个仿真会话
#
# 选项:
#   -s SCENARIO   场景名, 默认 empty_map
#   -m            自动运行任务脚本 (launch run_mission:=true)
#   -r            随 launch 打开 RViz2
#   -a "CMD"      AI 速度节点的启动命令; 缺省使用零速度测试发布器
#   -h            显示帮助

set -u

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_DIR="$(cd "$SCRIPT_DIR/../.." && pwd)"
PX4_DIR="$WS_DIR/../PX4-Autopilot"
SESSION="gestelt-ai"

PX4_PANE="$SESSION:0.0"
GESTELT_PANE="$SESSION:0.1"
AI_PANE="$SESSION:0.2"
OPS_PANE="$SESSION:0.3"

SCENARIO="empty_map"
AUTO_MISSION="false"
USE_RVIZ="false"
AI_CMD=""

usage() {
    sed -n '5,14p' "${BASH_SOURCE[0]}" | sed 's/^# \{0,1\}//'
}

wait_for_text() {
    local pane="$1" pattern="$2" timeout_s="$3" desc="$4"
    local waited=0
    echo "[*] 等待 $desc (最多 ${timeout_s}s)..."
    while [ "$waited" -lt "$timeout_s" ]; do
        if tmux capture-pane -p -t "$pane" 2>/dev/null | grep -q "$pattern"; then
            echo "[*] $desc OK"
            return 0
        fi
        sleep 2
        waited=$((waited + 2))
    done
    echo "[!] 超时: 未检测到 $desc" >&2
    return 1
}

wait_process_gone() {
    local pattern="$1" timeout_s="$2"
    local waited=0
    while [ "$waited" -lt "$timeout_s" ]; do
        if ! pgrep -f "$pattern" >/dev/null 2>&1; then
            return 0
        fi
        sleep 1
        waited=$((waited + 1))
    done
    echo "[!] 超时: 进程仍存在: $pattern" >&2
    return 1
}

px4_make_cmd() {
    echo "cd $PX4_DIR && export GAZEBO_IP=127.0.0.1 && export GAZEBO_MASTER_URI=http://127.0.0.1:11345 && make px4_sitl gazebo-classic_iris_vision"
}

stop_session() {
    if ! tmux has-session -t "$SESSION" >/dev/null 2>&1; then
        echo "会话 $SESSION 不存在"
        return 0
    fi
    echo "[*] 向各窗格发送 Ctrl+C..."
    for pane in 0.0 0.1 0.2 0.3; do
        tmux send-keys -t "$SESSION:$pane" C-c 2>/dev/null || true
    done
    sleep 4
    tmux kill-session -t "$SESSION" 2>/dev/null || true
    echo "[*] 会话 $SESSION 已关闭"
    echo "[*] 如 gzserver 仍未退出, 请检查: pgrep -f 'gzserver|px4_sitl_default'"
}

while getopts "s:mra:h" flag; do
    case "${flag}" in
        s) SCENARIO=${OPTARG} ;;
        m) AUTO_MISSION="true" ;;
        r) USE_RVIZ="true" ;;
        a) AI_CMD=${OPTARG} ;;
        h) usage; exit 0 ;;
        *) usage; exit 1 ;;
    esac
done

if [ "${1:-}" = "stop" ]; then
    stop_session
    exit 0
fi

if ! command -v tmux >/dev/null 2>&1; then
    echo "[!] 未安装 tmux, 请先: sudo apt install tmux" >&2
    exit 1
fi

if [ ! -d "$PX4_DIR" ]; then
    echo "[!] 找不到 PX4 源码目录: $PX4_DIR" >&2
    exit 1
fi

if [ ! -f "$WS_DIR/install/setup.bash" ]; then
    echo "[!] 未找到 $WS_DIR/install/setup.bash, 请先 colcon build 并 source" >&2
    exit 1
fi

MISSION_PATH="$WS_DIR/install/gestelt_bringup/share/gestelt_bringup/scenarios/$SCENARIO.py"
if [ ! -f "$MISSION_PATH" ]; then
    echo "[!] 未找到场景脚本: $MISSION_PATH" >&2
    exit 1
fi

if tmux has-session -t "$SESSION" >/dev/null 2>&1; then
    echo "[*] 会话 $SESSION 已存在, 直接连接 (如需重建请先运行: $0 stop)"
    tmux attach-session -t "$SESSION"
    exit 0
fi

if [ -n "$AI_CMD" ]; then
    AI_FULL_CMD="cd $WS_DIR && source /opt/ros/humble/setup.bash && source install/setup.bash && sleep 8 && $AI_CMD"
else
    AI_FULL_CMD="cd $WS_DIR && source /opt/ros/humble/setup.bash && source install/setup.bash && sleep 8 && ros2 topic pub -r 50 /drone0/ai_imu/velocity geometry_msgs/msg/Vector3Stamped '{header: {frame_id: drone0_base_link}, vector: {x: 0.0, y: 0.0, z: 0.0}}'"
    echo "[!] 未提供 -a AI 速度命令, 使用零速度测试发布器 (仅供链路验证, 不代表真实 AI 速度)"
fi

GESTELT_CMD="cd $WS_DIR && source /opt/ros/humble/setup.bash && source install/setup.bash && ros2 launch gestelt_bringup scenario_ros2.launch.py scenario:=$SCENARIO run_mission:=$AUTO_MISSION use_rviz:=$USE_RVIZ use_gazebo:=false use_fake_drone:=false use_mavros:=true use_ai_velocity:=true ai_velocity_topic:=ai_imu/velocity ai_velocity_variance:=0.04 enforce_gps_off:=true mavros_fcu_url:=udp://:14540@127.0.0.1:14580"

OPS_CMD="bash $SCRIPT_DIR/vio_ai_ops.sh $SCENARIO $AUTO_MISSION $SESSION ${AI_PANE#*:} \"$AI_FULL_CMD\""

echo "[*] 创建 tmux 会话 $SESSION (2x2 布局)"
tmux new-session -d -s "$SESSION"
tmux split-window -t "$SESSION:0.0" -v
tmux split-window -t "$SESSION:0.1" -h
tmux split-window -t "$SESSION:0.0" -h

echo "[*] 窗格 0 (PX4 + Gazebo iris_vision) 启动"
tmux send-keys -t "$PX4_PANE" "$(px4_make_cmd)" C-m
tmux send-keys -t "$GESTELT_PANE" "$GESTELT_CMD" C-m

if ! wait_for_text "$PX4_PANE" "Ready for takeoff" 900 "PX4 首次启动"; then
    echo "[!] PX4 首次启动失败, 请打开会话检查日志: tmux attach -t $SESSION" >&2
    exit 1
fi

echo "[*] 设置 EKF2 参数 (EV_CTRL=7, GPS_CTRL=0, HGT_REF=0) 并保存"
tmux send-keys -t "$PX4_PANE" "param set EKF2_EV_CTRL 7" C-m
tmux send-keys -t "$PX4_PANE" "param set EKF2_GPS_CTRL 0" C-m
tmux send-keys -t "$PX4_PANE" "param set EKF2_HGT_REF 0" C-m
tmux send-keys -t "$PX4_PANE" "param save" C-m
sleep 2

echo "[*] 重启 PX4 使 EKF2_HGT_REF 生效"
tmux send-keys -t "$PX4_PANE" C-c
wait_process_gone "px4_sitl_default/bin/px4" 30 || true
wait_process_gone "gzserver" 30 || true
sleep 2

tmux send-keys -t "$PX4_PANE" "$(px4_make_cmd)" C-m
if ! wait_for_text "$PX4_PANE" "Ready for takeoff" 300 "PX4 参数重启"; then
    echo "[!] PX4 重启失败, 请打开会话检查日志: tmux attach -t $SESSION" >&2
    exit 1
fi

echo "[*] 开启故障注入开关 (SYS_FAILURE_EN=1)"
tmux send-keys -t "$PX4_PANE" "param set SYS_FAILURE_EN 1" C-m

echo "[*] 窗格 2 (AI 速度) 启动"
tmux send-keys -t "$AI_PANE" "$AI_FULL_CMD" C-m
echo "[*] 窗格 3 (任务 + 实验操作菜单) 启动"
tmux send-keys -t "$OPS_PANE" "$OPS_CMD" C-m

echo
echo "[*] 启动完成, 即将进入会话"
echo "[*] 窗格布局: 左上 PX4 | 右上 任务/操作菜单 | 左下 Gestelt+MAVROS | 右下 AI 速度"
echo "[*] 预飞检查务必确认 GPS guard 打印 'GPS disabled: EKF2_GPS_CTRL=0 and HIL_GPS blocked'"
echo
tmux attach-session -t "$SESSION"
