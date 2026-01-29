#!/bin/bash
# 灵巧手串口检测脚本（LZ Hand Port Detection Script）
# 自动检测USB串口对应的hand_id
# Automatically detect hand_id for USB serial ports

# 自动设置串口权限（Auto-set serial port permissions）
sudo chmod 666 /dev/ttyUSB* 2>/dev/null || true

# 获取脚本目录（Get script directory）
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PACKAGE_DIR="$(dirname "$SCRIPT_DIR")"
PYTHON_SCRIPT="$PACKAGE_DIR/scripts/detect_hand_ports.py"

if [ ! -f "$PYTHON_SCRIPT" ]; then
    echo "[ERROR] 找不到检测脚本: $PYTHON_SCRIPT"
    exit 1
fi

python3 "$PYTHON_SCRIPT" "$@"
