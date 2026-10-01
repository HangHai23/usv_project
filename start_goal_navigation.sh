#!/usr/bin/env bash
# One-command field-goal navigation with RC manual/automatic arbitration.
set -Eeo pipefail

PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="${PROJECT_DIR}/ros2_ws"
GOAL_SIDE="${1:-${GOAL_SIDE:-left}}"
WEB_PORT="${WEB_PORT:-8080}"
PIDS=()

if [[ "${GOAL_SIDE}" != "left" && "${GOAL_SIDE}" != "right" ]]; then
    echo "用法: $0 [left|right]" >&2
    exit 2
fi

cleanup() {
    trap - INT TERM EXIT
    if ((${#PIDS[@]})); then
        kill -INT "${PIDS[@]}" 2>/dev/null || true
        wait "${PIDS[@]}" 2>/dev/null || true
    fi
}
trap cleanup INT TERM EXIT

# ROS setup files may inspect unset variables, so enable nounset afterwards.
source /opt/ros/humble/setup.bash
if [[ ! -f "${WORKSPACE_DIR}/install/setup.bash" ]]; then
    echo "错误：工作空间尚未构建：${WORKSPACE_DIR}" >&2
    exit 1
fi
source "${WORKSPACE_DIR}/install/setup.bash"
set -u
cd "${WORKSPACE_DIR}"

echo "启动球门导航：目标=${GOAL_SIDE}，CH8低位=手动，CH8高位=自动，CH5=急停"
ros2 launch usv_imu_driver imu.launch.py & PIDS+=("$!")
ros2 launch usv_udp_position udp_position.launch.py & PIDS+=("$!")
ros2 launch usv_goal_navigation goal_navigation.launch.py \
    goal_side:="${GOAL_SIDE}" \
    armed:=true \
    require_mode:=true \
    command_topic:=/propulsion/automatic_command & PIDS+=("$!")
ros2 launch usv_rc_teleop channel3_esc1_test.launch.py \
    start_rc_nodes:=true \
    force_automatic_without_rc:=false \
    emergency_stop_enabled:=true \
    dry_run:=false & PIDS+=("$!")
ros2 launch usv_web_monitor web_monitor.launch.py port:="${WEB_PORT}" & PIDS+=("$!")

echo "系统已启动。先保持CH8低位验证遥控，再将CH8拨到高位进入自动导航。"
echo "切回CH8低位会重新由遥控器接管。CH5急停在两种模式下均有效。"
echo "网页监控：http://<Jetson-IP>:${WEB_PORT}"
echo "按 Ctrl+C 停止本脚本启动的全部节点。"
# Stop the remaining stack if any required process exits.
wait -n
