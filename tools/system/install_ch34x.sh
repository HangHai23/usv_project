#!/usr/bin/env bash
set -euo pipefail

DRIVER_DIR="/home/hai/usv_project/vendor/kernel/CH341SER"
TARGET_USER="hai"
KERNEL_RELEASE="$(uname -r)"

if [[ ${EUID} -ne 0 ]]; then
    echo "请使用 sudo 运行此脚本。" >&2
    exit 1
fi

if [[ ! -f "${DRIVER_DIR}/ch34x.ko" ]]; then
    echo "未找到 ${DRIVER_DIR}/ch34x.ko，请先在驱动目录执行 make。" >&2
    exit 1
fi

echo "[1/5] 移除可能抢占 CH340 设备的 brltty"
apt-get remove -y brltty

echo "[2/5] 安装为当前内核 ${KERNEL_RELEASE} 的模块"
make -C "${DRIVER_DIR}" install

echo "[3/5] 加载 USB 串口及 CH34x 模块"
modprobe usbserial
modprobe ch34x
udevadm settle

echo "[4/5] 将 ${TARGET_USER} 加入 dialout 串口访问组"
usermod -aG dialout "${TARGET_USER}"

echo "[5/5] 检查驱动与设备节点"
lsmod | grep -E '^(ch34x|usbserial)' || true
ls -l /dev/ttyUSB* 2>/dev/null || {
    echo "暂未发现 /dev/ttyUSB*，请拔插一次 IMU USB 线后再检查。"
    exit 2
}

echo
echo "安装完成。请注销并重新登录，使 dialout 用户组生效。"
