#!/bin/bash
# uav-mission.service: MAVROS (Cube over USB, WITH command plugins) + VIO bridge + fc_status + mission
# node + RC trigger + a flight-log recorder. Mission parameters: $UAV_MISSION_PARAMS (uav_env.sh).
#
# Nothing flies until the pilot flips ch6 LOW -> HIGH AND the mission's preflight checks pass
# (FC link, EKF position, battery, VIO healthy and EKF on source set 2). ch6 back to LOW in flight
# = abort (lands). Any flight-mode change on the transmitter takes over from the mission.
#
# Runs INSTEAD of uav-fclink (both need the Cube's one USB port; the units conflict).
source "$(dirname "$(readlink -f "$0")")/uav_env.sh"
log_service_to_file mission    # also log to ~/flight_logs/boot####_.../mission.log
# Job control: without it, bash starts background programs with Ctrl-C (SIGINT) ignored, so they
# could not be stopped cleanly and the recording would be cut off.
set -m
CUBE=/dev/serial/by-id/usb-Hex_ProfiCNC_CubeOrange_440045001451303437363830-if00

if pgrep -f '^/opt/ros/jazzy/lib/mavros/mavros_node' >/dev/null; then
    echo "ERROR: MAVROS is already running (another terminal, or uav-fclink.service). Stop it first."
    exit 1
fi
echo "mission parameters: $UAV_MISSION_PARAMS"
grep -E '^\s*(profile|square_side|takeoff_alt|path_speed):' "$UAV_MISSION_PARAMS" | sed 's/^ */  /'

echo "waiting for the Cube on USB..."
for ((i = 0; i < 120; i += 2)); do [ -e $CUBE ] && break; sleep 2; done
[ -e $CUBE ] || { echo "ERROR: Cube not found on USB ($CUBE)"; exit 1; }
dev=$(readlink -f $CUBE)
if command -v fuser >/dev/null && fuser "$dev" >/dev/null 2>&1; then
    echo "ERROR: $dev is in use by another program (MAVProxy? check_cube_params.py?): $(fuser "$dev" 2>&1)"
    exit 1
fi
ino=$(stat -L -c %i $CUBE)
echo "Cube at $dev"
wait_for_topic /visual_slam/tracking/odometry 240 \
    || echo "cuVSLAM not up yet; starting anyway (preflight refuses to start until VIO is healthy)"

# Numbered recordings (the Orin's clock reads 1969 without internet), like uav-fclink's.
n=$(( $(cat $UAV_LOG_ROOT/.run_counter 2>/dev/null || echo 0) + 1 ))
echo $n > $UAV_LOG_ROOT/.run_counter
LOG=$UAV_LOG_ROOT/$(printf run%04d $n)_$(date +%Y%m%d_%H%M%S)_mission
mkdir -p "$LOG"
cp "$UAV_MISSION_PARAMS" "$LOG/mission_params.yaml"
# Keep the newest 40 numbered mission recordings. Renamed folders are never deleted.
ls -1d $UAV_LOG_ROOT/run[0-9][0-9][0-9][0-9]_*_mission 2>/dev/null | sort | head -n -40 | xargs -r rm -rf

ros2 launch uav_mission mission.launch.py bridge:=true rc_trigger:=true rc_channel:=$UAV_RC_CHANNEL \
    params:=$UAV_MISSION_PARAMS &
LAUNCH=$!
sleep 10
ros2 bag record -s mcap -o "$LOG/bag" \
    /uav/mission/state /uav/fc_status /uav/rc_trigger/status /vio_bridge/status /uav/ekf_source_cmd \
    /visual_slam/tracking/odometry /mavros/odometry/out /mavros/local_position/pose \
    /mavros/local_position/velocity_local /mavros/setpoint_raw/local /mavros/state /mavros/rc/in \
    /mavros/battery /mavros/statustext/recv /mavros/imu/data > "$LOG/bag_record.log" 2>&1 &
BAG=$!
echo "recording to $LOG"; echo "$LOG" >> "$UAV_BOOT_LOG_DIR/recordings.txt"
echo "READY: flip ch6 LOW -> HIGH to start (status: ros2 topic echo /uav/mission/state)"

stop() {   # recorder first (closes the bag), then MAVROS + bridge + mission
    kill -INT $BAG 2>/dev/null; wait $BAG 2>/dev/null
    kill -INT $LAUNCH 2>/dev/null; wait $LAUNCH 2>/dev/null
}
trap 'stop; exit 0' TERM INT

# Watchdog: MAVROS does not notice when the Cube is unplugged/power-cycled and keeps a dead port.
# If the Cube disappears or comes back as a new device, exit so systemd restarts the stack.
while kill -0 $LAUNCH 2>/dev/null; do
    sleep 2
    if [ ! -e $CUBE ] || [ "$(stat -L -c %i $CUBE 2>/dev/null)" != "$ino" ]; then
        echo "Cube disconnected or re-plugged: restarting the mission stack"
        stop; exit 3
    fi
done
echo "mission launch exited"
stop; exit 1
