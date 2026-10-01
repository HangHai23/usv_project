#!/usr/bin/env bash
set -euo pipefail

PROJECT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"

if [[ ! -x "${PROJECT_DIR}/.venv/bin/python" ]]; then
    echo "错误：虚拟环境不存在，请先运行 install.sh" >&2
    exit 2
fi

exec "${PROJECT_DIR}/.venv/bin/python" "${PROJECT_DIR}/chat.py"
