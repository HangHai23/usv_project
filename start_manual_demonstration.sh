#!/usr/bin/env bash
# 仅启动手动控制与数据采集，不启动自动导航/追球控制器。
set -Eeo pipefail
PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="${PROJECT_DIR}/ros2_ws"
GOAL_SIDE="${1:-left}"
WEB_PORT="${WEB_PORT:-8080}"
PIDS=()
[[ "$GOAL_SIDE" == left || "$GOAL_SIDE" == right ]] || { echo "用法: $0 [left|right]"; exit 2; }
cleanup() {
    trap - INT TERM EXIT
    if ((${#PIDS[@]})); then
        kill -INT "${PIDS[@]}" 2>/dev/null || true
        wait "${PIDS[@]}" 2>/dev/null || true
    fi
}
trap 'exit 130' INT
trap 'exit 143' TERM
trap cleanup EXIT
source /opt/ros/humble/setup.bash
source "${WORKSPACE_DIR}/install/setup.bash"
set -u
cd "$WORKSPACE_DIR"
echo "请先停止其他联合启动脚本。保持CH5急停，CH8低位手动；检查后再解除急停。"
echo "本脚本会启用真实PWM输出；S/E/X仅控制记录，不控制电机。"
ros2 launch usv_imu_driver imu.launch.py </dev/null & PIDS+=("$!")
ros2 launch usv_udp_position udp_position.launch.py </dev/null & PIDS+=("$!")
ros2 launch usv_rc_teleop channel3_esc1_test.launch.py \
    start_rc_nodes:=true force_automatic_without_rc:=false \
    emergency_stop_enabled:=true dry_run:=false </dev/null & PIDS+=("$!")
ros2 launch usv_web_monitor web_monitor.launch.py port:="$WEB_PORT" </dev/null & PIDS+=("$!")
ros2 run usv_goal_navigation manual_demonstration --ros-args \
    --params-file "${WORKSPACE_DIR}/src/usv_goal_navigation/config/manual_demonstration.yaml" \
    -p goal_side:="$GOAL_SIDE"
