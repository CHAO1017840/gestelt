#!/usr/bin/env bash
# 由 vio_ai_experiment.sh 在 ops 窗格中运行的交互菜单
# 负责: 预飞提示、任务起飞、VIO 失效注入/恢复、AI 速度停止/重启、降落

set -u

SCENARIO="${1:-empty_map}"
AUTO_MISSION="${2:-false}"
SESSION="${3:-gestelt-ai}"
AI_PANE="${4:-0.2}"
AI_FULL_CMD="${5:-}"

WS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
MISSION_DIR="$WS_DIR/install/gestelt_bringup/share/gestelt_bringup/scenarios"

if [ -f "$WS_DIR/install/setup.bash" ]; then
    # shellcheck disable=SC1091
    source "$WS_DIR/install/setup.bash"
else
    # shellcheck disable=SC1091
    source /opt/ros/humble/setup.bash
fi

vio_fail() {
    # $1 = 1 注入 VIO 失效, 0 恢复 VIO
    ros2 service call /drone0/mavros/cmd/command mavros_msgs/srv/CommandLong \
        "{broadcast: false, command: 420, confirmation: 0, param1: 6.0, param2: ${1}.0, param3: 0.0, param4: 0.0, param5: 0.0, param6: 0.0, param7: 0.0}"
}

cat <<EOF
================ Gestelt VIO + AI 速度回退实验 ================
预飞检查 (在 px4 窗格执行):
  listener estimator_status_flags
      期望: cs_gps=False  cs_ev_pos=True  cs_ev_vel=True
  listener vehicle_local_position
      期望: xy_valid=True  v_xy_valid=True  dead_reckoning=False
在任意终端:
  ros2 topic hz /drone0/ai_imu/velocity        (应为 50-100 Hz)
  ros2 topic echo --once /drone0/mavros/state  (armed: false)
=============================================================
EOF

if [ "$AUTO_MISSION" != "true" ]; then
    read -r -p "[ops] 确认 GPS guard 已打印 GPS disabled 且预飞检查通过, 按 Enter 起飞任务..." _
    python3 "$MISSION_DIR/$SCENARIO.py"
else
    echo "[ops] run_mission:=true, 任务脚本由 launch 在节点就绪后自动执行"
fi

while true; do
    echo
    echo "---------- ops 菜单 ----------"
    echo " v: 注入 VIO 失效 (failure vio off)"
    echo " r: 恢复 VIO (failure vio ok)"
    echo " a: 停止 AI 速度发布"
    echo " A: 重启 AI 速度发布"
    echo " l: AUTO.LAND 降落"
    echo " q: 退出菜单 (仿真继续运行)"
    echo "------------------------------"
    read -r -n 1 -p "[ops] 选择: " KEY
    echo
    case "$KEY" in
        v) echo "[ops] 注入 VIO 失效"; vio_fail 1 ;;
        r) echo "[ops] 恢复 VIO"; vio_fail 0 ;;
        a) echo "[ops] 停止 AI 速度"; tmux send-keys -t "$SESSION:$AI_PANE" C-c ;;
        A)
            if [ -n "$AI_FULL_CMD" ]; then
                echo "[ops] 重启 AI 速度"
                tmux send-keys -t "$SESSION:$AI_PANE" "$AI_FULL_CMD" C-m
            else
                echo "[ops] 未提供 AI 速度命令, 无法重启"
            fi
            ;;
        l)
            ros2 service call /drone0/mavros/set_mode mavros_msgs/srv/SetMode \
                "{base_mode: 0, custom_mode: 'AUTO.LAND'}"
            ;;
        q) echo "[ops] 退出菜单"; break ;;
        *) echo "[ops] 无效按键" ;;
    esac
done
