#!/usr/bin/env bash
# One-command formal tracking: RC manual/automatic arbitration and real PWM output.
set -Eeo pipefail

PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="${PROJECT_DIR}/ros2_ws"
WEB_PORT="${WEB_PORT:-8080}"
PIDS=()

cleanup() {
    trap - INT TERM EXIT
    if ((${#PIDS[@]})); then
        kill -INT "${PIDS[@]}" 2>/dev/null || true
        wait "${PIDS[@]}" 2>/dev/null || true
    fi
}
trap cleanup INT TERM EXIT

source /opt/ros/humble/setup.bash
if [[ ! -f "${WORKSPACE_DIR}/install/setup.bash" ]]; then
    echo "ERROR: workspace is not built: ${WORKSPACE_DIR}" >&2
    exit 1
fi
source "${WORKSPACE_DIR}/install/setup.bash"
set -u
cd "${WORKSPACE_DIR}"

echo "Starting FORMAL tracking (RC required, CH5 emergency stop, CH8 mode)..."
ros2 launch usv_imu_driver imu.launch.py & PIDS+=("$!")
ros2 launch usv_camera_driver camera.launch.py & PIDS+=("$!")
ros2 launch usv_yolo_detector yolo_detector.launch.py & PIDS+=("$!")
ros2 launch usv_ball_follow ball_follow.launch.py & PIDS+=("$!")
ros2 launch usv_rc_teleop channel3_esc1_test.launch.py \
    start_rc_nodes:=true \
    force_automatic_without_rc:=false \
    emergency_stop_enabled:=true \
    dry_run:=false & PIDS+=("$!")
ros2 launch usv_web_monitor web_monitor.launch.py port:="${WEB_PORT}" & PIDS+=("$!")
ros2 run usv_ball_follow telemetry_recorder & PIDS+=("$!")

echo "Tracking started. Web monitor: http://<Jetson-IP>:${WEB_PORT}"
echo "CSV directory: ${HOME}/Downloads/usv_logs"
echo "Press Ctrl+C to stop all nodes started by this script."
wait
