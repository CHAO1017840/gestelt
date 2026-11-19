#!/usr/bin/env bash
# Gestelt GPS-denied VIO + AI_IMU 推理诊断仿真启动脚本
# AI_IMU 速度积分路径默认禁用。
#
# 用法:
#   vio_ai_experiment.sh [选项]     # 新建 tmux 会话并自动拉起整套仿真
#   vio_ai_experiment.sh stop       # 关闭整个仿真会话
#
# 选项:
#   -s SCENARIO   场景名, 默认 rmuc2025
#   -H            仅自动起飞并保持悬停（默认）
#   -m            自动起飞并运行场景导航任务
#   -r            随 launch 打开 RViz2
#   -a "CMD"      自定义 AI_IMU 诊断命令
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

SCENARIO="rmuc2025"
AUTO_MISSION="false"
HOVER_ONLY="true"
USE_RVIZ="false"
AI_CMD=""

usage() {
    sed -n '5,15p' "${BASH_SOURCE[0]}" | sed 's/^# \{0,1\}//'
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

px4_make_cmd() {
    local world_setup=""
    if [ "$SCENARIO" = "rmuc2025" ]; then
        world_setup="export PX4_SITL_WORLD=$WS_DIR/gestelt_bringup/simulation/worlds/rmuc2025.world && export GAZEBO_MODEL_PATH=$WS_DIR/gestelt_bringup/simulation/models && export PX4_GUI_LOCK_MODELS=rmuc2025_field,rmuc2025_ground && "
    fi
    echo "cd $PX4_DIR && export GAZEBO_IP=127.0.0.1 && export GAZEBO_MASTER_URI=http://127.0.0.1:11345 && ${world_setup}make px4_sitl gazebo-classic_raynor"
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

while getopts "s:mHra:h" flag; do
    case "${flag}" in
        s) SCENARIO=${OPTARG} ;;
        m) AUTO_MISSION="true"; HOVER_ONLY="false" ;;
        H) HOVER_ONLY="true"; AUTO_MISSION="false" ;;
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
    AI_FULL_CMD=""
fi

GESTELT_CMD="cd $WS_DIR && source /opt/ros/humble/setup.bash && source install/setup.bash && ros2 launch gestelt_bringup scenario_ros2.launch.py scenario:=$SCENARIO back_end_planner:=ego use_entire_pcd_map:=true use_fake_sensor:=false run_mission:=false use_rviz:=$USE_RVIZ use_gazebo:=false use_fake_drone:=false use_mavros:=true use_ai_imu:=true use_ai_imu_comparison:=false use_ai_imu_velocity:=false use_gazebo_vio_bridge:=false ai_imu_axis_mapping:=x,y,z ai_imu_accel_correction_blend:=1.0 ai_imu_gyro_correction_blend:=0.0 enforce_gps_off:=true mavros_fcu_url:=udp://:14540@127.0.0.1:14580"

OPS_CMD="bash $SCRIPT_DIR/vio_ai_ops.sh $SCENARIO $AUTO_MISSION $SESSION ${AI_PANE#*:} \"$AI_FULL_CMD\" $HOVER_ONLY"

echo "[*] 创建 tmux 会话 $SESSION (2x2 布局)"
tmux new-session -d -s "$SESSION"
tmux split-window -t "$SESSION:0.0" -v
tmux split-window -t "$SESSION:0.1" -h
tmux split-window -t "$SESSION:0.0" -h

echo "[*] 窗格 0 (PX4 + Gazebo Raynor) 启动"
tmux send-keys -t "$PX4_PANE" "$(px4_make_cmd)" C-m
tmux send-keys -t "$GESTELT_PANE" "$GESTELT_CMD" C-m

if ! wait_for_text "$PX4_PANE" "Ready for takeoff" 900 "PX4 首次启动"; then
    echo "[!] PX4 首次启动失败, 请打开会话检查日志: tmux attach -t $SESSION" >&2
    exit 1
fi

echo "[*] 确认 EKF2 参数 (EV_CTRL=7, GPS_CTRL=0, HGT_REF=0)"
# install_raynor_sitl.sh has already written these values into parameters.bson,
# so the first PX4 start has loaded them. Avoid restarting PX4 from inside a
# tmux pane: if pxh is still alive, the restart command is interpreted by PX4.
tmux send-keys -t "$PX4_PANE" "param set EKF2_EV_CTRL 7" C-m
tmux send-keys -t "$PX4_PANE" "param set EKF2_GPS_CTRL 0" C-m
tmux send-keys -t "$PX4_PANE" "param set EKF2_HGT_REF 0" C-m
tmux send-keys -t "$PX4_PANE" "param save" C-m
sleep 2

echo "[*] 开启故障注入开关 (SYS_FAILURE_EN=1)"
tmux send-keys -t "$PX4_PANE" "param set SYS_FAILURE_EN 1" C-m

echo "[*] 窗格 2 仅供 AI_IMU 诊断；速度接管已禁用"
if [ "$HOVER_ONLY" = "true" ]; then
    echo "[*] 窗格 3 (悬停测试 + 实验操作菜单) 启动"
else
    echo "[*] 窗格 3 (任务 + 实验操作菜单) 启动"
fi
tmux send-keys -t "$OPS_PANE" "$OPS_CMD" C-m

echo
echo "[*] 启动完成, 即将进入会话"
echo "[*] 窗格布局: 左上 PX4 | 右上 测试/操作菜单 | 左下 Gestelt+MAVROS | 右下 AI 诊断"
if [ "$HOVER_ONLY" = "true" ]; then
    echo "[*] 自动化模式: 仅起飞和 HOVER，不发送 EGO 导航目标"
else
    echo "[*] 自动化模式: 起飞、MISSION 导航 + RMUC 2025 目标"
fi
echo "[*] 预飞检查务必确认 GPS guard 打印 'GPS disabled: EKF2_GPS_CTRL=0 and HIL_GPS blocked'"
echo
tmux attach-session -t "$SESSION"
