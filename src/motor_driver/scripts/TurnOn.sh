#!/bin/bash

# Định nghĩa biến
USER_HOME="/home/$USER"
WORKSPACE_DIR="$USER_HOME/Desktop/AK80_64"
LOG_FILE="$USER_HOME/turnon_script.log"
ROS_DISTRO="humble"

# Hàm ghi log (chỉ dùng cho lỗi)
log_error() {
    echo "[$(date '+%Y-%m-%d %H:%M:%S')] ERROR: $1" >> "$LOG_FILE"
}

# Kiểm tra và source ROS 2
if [ -f "/opt/ros/$ROS_DISTRO/setup.bash" ]; then
    source "/opt/ros/$ROS_DISTRO/setup.bash"
else
    log_error "ROS 2 $ROS_DISTRO not found."
    exit 1
fi

# Kiểm tra và di chuyển đến workspace
if [ -d "$WORKSPACE_DIR" ]; then
    cd "$WORKSPACE_DIR" || {
        log_error "Failed to change to $WORKSPACE_DIR."
        exit 1
    }
else
    log_error "Workspace directory $WORKSPACE_DIR does not exist."
    exit 1
fi

# Source workspace
if [ -f "install/setup.bash" ]; then
    source "install/setup.bash"
else
    log_error "Workspace not built or setup.bash not found."
    exit 1
fi

# Đặt ROS_DOMAIN_ID (mặc định 0, dùng để cô lập giao tiếp ROS 2 trên mạng)
export ROS_DOMAIN_ID=0

# # Thiết lập giao tiếp CAN
# sudo ip link set can0 up type can bitrate 1000000
# if [ $? -ne 0 ]; then
#     log_error "Failed to set up CAN interface can0."
#     exit 1
# fi

# Chạy node motor_control_node
ros2 run motor_driver ControlMotor >> "$LOG_FILE" {Seperator} motor_control_node >> "$LOG_FILE" 2>&1 &
PID=$!

# Kiểm tra xem node có chạy thành công không
sleep 2
if ! ps -p $PID > /dev/null; then
    log_error "motor_control_node failed to start."
    exit 1
fi

exit 0