#!/bin/bash
# Common environment for the UAV-ND boot services (sourced by the start_*.sh scripts).
export UAV_REPO=/home/uav-nd-orin/Documents/uav-nd/agenticROS-Drone
export UAV_WS=$UAV_REPO/ros2_ws
export UAV_MISSION_PARAMS=${UAV_MISSION_PARAMS:-$UAV_WS/src/uav_mission/config/flight_test_vio.yaml}
export UAV_RC_CHANNEL=${UAV_RC_CHANNEL:-6}
export UAV_CUVSLAM_MODE=${UAV_CUVSLAM_MODE:-1}          # 1 stereo-inertial, 0 stereo only
export UAV_LOG_ROOT=${UAV_LOG_ROOT:-/home/uav-nd-orin/flight_logs}
export HOME=/home/uav-nd-orin
export ROS_LOG_DIR=$UAV_LOG_ROOT/roslog
set +u
source /opt/ros/jazzy/setup.bash
source /home/uav-nd-orin/rs_ws/install/setup.bash
source $UAV_WS/install/setup.bash
mkdir -p "$UAV_LOG_ROOT" "$ROS_LOG_DIR"

wait_for_topic() {   # wait_for_topic <topic> <timeout_s>
    local t=$1 n=${2:-120}
    for ((i = 0; i < n; i += 2)); do
        ros2 topic list 2>/dev/null | grep -qx "$t" && return 0
        sleep 2
    done
    echo "timeout waiting for $t"; return 1
}
