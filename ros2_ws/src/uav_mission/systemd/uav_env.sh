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

# ---- Per-boot service log files ------------------------------------------------------------
# log_service_to_file <name>: from here on, everything the calling script (and the programs it
# starts) prints is also written, with a timestamp per line, to
#   ~/flight_logs/boot0001_<date>_<bootid>/<name>.log
# All services of one Orin boot share the folder. Folders are numbered because the clock reads
# 1969 after a boot without internet. Each line is written as soon as it is printed; `sudo poweroff`
# makes sure the last lines are on disk (pulling the battery can lose the last ~30 s).
boot_log_dir() {
    local id d n
    id=$(cut -c1-8 /proc/sys/kernel/random/boot_id)
    exec 8>"$UAV_LOG_ROOT/.boot_lock"; flock 8
    d=$(ls -1d "$UAV_LOG_ROOT"/boot[0-9][0-9][0-9][0-9]_*_"$id" 2>/dev/null | head -1)
    if [ -z "$d" ]; then
        n=$(( $(cat "$UAV_LOG_ROOT/.boot_counter" 2>/dev/null || echo 0) + 1 ))
        echo $n > "$UAV_LOG_ROOT/.boot_counter"
        d=$UAV_LOG_ROOT/$(printf boot%04d $n)_$(date +%Y%m%d_%H%M%S)_$id
        mkdir -p "$d"
        # keep the newest 60 boot folders
        ls -1d "$UAV_LOG_ROOT"/boot[0-9][0-9][0-9][0-9]_*_* 2>/dev/null | sort | head -n -60 | xargs -r rm -rf
    fi
    flock -u 8; exec 8>&-
    echo "$d"
}
log_service_to_file() {
    local f
    UAV_BOOT_LOG_DIR=$(boot_log_dir); export UAV_BOOT_LOG_DIR
    f=$UAV_BOOT_LOG_DIR/$1.log
    echo "===== $1 started $(date '+%F %T') (boot $(cat /proc/sys/kernel/random/boot_id)) =====" >> "$f"
    exec 3>&1
    # The logger ignores Ctrl-C/stop signals so it keeps writing until the service's last line.
    exec > >(trap '' INT TERM; while IFS= read -r l; do
                 printf '%(%F %T)T %s\n' -1 "$l" >> "$f"; printf '%s\n' "$l" >&3; done) 2>&1
}
