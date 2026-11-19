#!/usr/bin/env bash
# 由 vio_ai_experiment.sh 在 ops 窗格中运行的交互菜单
# 负责: 预飞提示、任务起飞、VIO 失效注入/恢复和降落

set -u

SCENARIO="${1:-rmuc2025}"
AUTO_MISSION="${2:-false}"
SESSION="${3:-gestelt-ai}"
AI_PANE="${4:-0.2}"
AI_FULL_CMD="${5:-}"
HOVER_ONLY="${6:-false}"

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_DIR="$(cd "$SCRIPT_DIR/../.." && pwd)"
MISSION_DIR="$WS_DIR/install/gestelt_bringup/share/gestelt_bringup/scenarios"

if [ -f "$WS_DIR/install/setup.bash" ]; then
    set +u
    # shellcheck disable=SC1091
    source "$WS_DIR/install/setup.bash"
    set -u
else
    set +u
    # shellcheck disable=SC1091
    source /opt/ros/humble/setup.bash
    set -u
fi

run_mission_with_retry() {
    local attempt
    for attempt in 1 2 3; do
        echo "[ops] 启动任务脚本 $SCENARIO (尝试 $attempt/3)"
        if python3 "$MISSION_DIR/$SCENARIO.py"; then
            return 0
        fi
        sleep 5
    done
    echo "[ops] 任务脚本连续启动失败，请检查 /drone0/traj_server/state" >&2
    return 1
}

run_hover_only() {
    echo "[ops] Hover-only 模式: 自动起飞并保持 HOVER, 不发送导航目标"
    python3 "$SCRIPT_DIR/hover_only.py"
    local px4_listener="$WS_DIR/../PX4-Autopilot/build/px4_sitl_default/bin/px4-listener"
    if [ ! -x "$px4_listener" ]; then
        echo "[ops] 找不到 $px4_listener，状态监视器将显示 PX4 状态为 UNKNOWN" >&2
    fi
    local status_cmd="cd $WS_DIR && source /opt/ros/humble/setup.bash && source install/setup.bash && export PX4_LISTENER_BIN=$px4_listener && python3 $SCRIPT_DIR/status_monitor.py"
    if [ -n "$AI_FULL_CMD" ]; then
        tmux new-window -d -t "$SESSION:" -n status "$status_cmd"
        echo "[ops] 状态监视器在 tmux 窗口 status 中运行"
    else
        tmux send-keys -t "$SESSION:$AI_PANE" "$status_cmd" C-m
        echo "[ops] 状态监视器在右下 AI 窗格中运行"
    fi
}

vio_fail() {
    # $1 = 1 注入 VIO 失效, 0 恢复 VIO
    ros2 service call /drone0/mavros/cmd/command mavros_msgs/srv/CommandLong \
        "{broadcast: false, command: 420, confirmation: 0, param1: 6.0, param2: ${1}.0, param3: 0.0, param4: 0.0, param5: 0.0, param6: 0.0, param7: 0.0}"
}

cat <<EOF
================ Gestelt VIO + AI_IMU 推理诊断 ================
预飞检查 (在 px4 窗格执行):
  listener estimator_status_flags
      期望: cs_gps=False  cs_ev_vel=True
            起飞前 EKF2_EV_CTRL=7, VIO 正常融合位置和速度
  listener vehicle_local_position
      期望: xy_valid=True  v_xy_valid=True  dead_reckoning=False
在任意终端:
  ros2 topic hz /drone0/mavros/ai/imu_corrected  (约 30-40 Hz)
  ros2 topic echo --once /drone0/mavros/state  (armed: false)
  AI_IMU 速度积分和 ODOMETRY 接管默认禁用
  测试模式使用 COM_POS_FS_HOLD=1:
      首次有效解锁后，EKF2 与 Commander 保持水平 valid
      估计可能漂移；姿态和动力 failsafe 仍独立生效
=============================================================
EOF

if [ "$HOVER_ONLY" = "true" ]; then
    run_hover_only
elif [ "$AUTO_MISSION" != "true" ]; then
    read -r -p "[ops] 确认 GPS guard 已打印 GPS disabled 且预飞检查通过, 按 Enter 起飞任务..." _
    run_mission_with_retry
else
    echo "[ops] 等待导航节点发布状态后自动启动任务"
    sleep 8
    run_mission_with_retry
fi

while true; do
    echo
    echo "---------- ops 菜单 ----------"
    echo " f: AI_IMU 速度接管已禁用"
    echo " v: 手动注入 VIO 失效 (仅诊断; 不会启用 AI)"
    echo " r: 恢复 VIO (failure vio ok)"
    echo " a: 停止接管进程"
    echo " A: 重新加载接管命令"
    echo " l: AUTO.LAND 降落"
    echo " q: 退出菜单 (仿真继续运行)"
    echo "------------------------------"
    read -r -n 1 -p "[ops] 选择: " KEY
    echo
    case "$KEY" in
        f)
            echo "[ops] 不使用 AI_IMU 积分速度，避免错误速度污染导航"
            ;;
        v)
            echo "[ops] 注入 VIO 失效"
            echo "[ops] SITL 将保持水平 valid，请监控估计漂移并准备降落"
            vio_fail 1
            ;;
        r) echo "[ops] 恢复 VIO"; vio_fail 0 ;;
        a) echo "[ops] 停止接管进程"; tmux send-keys -t "$SESSION:$AI_PANE" C-c ;;
        A)
            echo "[ops] AI_IMU 速度接管已禁用"
            ;;
        l)
            ros2 service call /drone0/mavros/set_mode mavros_msgs/srv/SetMode \
                "{base_mode: 0, custom_mode: 'AUTO.LAND'}"
            ;;
        q) echo "[ops] 退出菜单"; break ;;
        *) echo "[ops] 无效按键" ;;
    esac
done
