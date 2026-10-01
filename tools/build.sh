#!/usr/bin/env bash
set -eo pipefail

WORKSPACE="/home/hai/usv_project/ros2_ws"

source /opt/ros/humble/setup.bash
cd "${WORKSPACE}"
colcon build --symlink-install "$@"

echo
echo "Build complete. Run:"
echo "  source ${WORKSPACE}/install/setup.bash"
