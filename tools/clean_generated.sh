#!/usr/bin/env bash
set -euo pipefail

WORKSPACE="/home/hai/usv_project/ros2_ws"

if [[ ! -d "${WORKSPACE}/src" ]]; then
    echo "Refusing to clean: expected workspace source directory is missing." >&2
    exit 1
fi

for generated in build install log; do
    target="${WORKSPACE}/${generated}"
    if [[ -d "${target}" ]]; then
        rm -r -- "${target}"
        echo "Removed ${target}"
    fi
done

