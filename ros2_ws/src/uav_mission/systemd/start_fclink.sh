#!/bin/bash
# uav-fclink.service: MAVROS link to the Cube (READ-ONLY plugin set) + VIO bridge + fc_status
# + a small data recorder. The Jetson cannot arm, change mode or write parameters with this.
# Can also be run by hand in a terminal when the service is not installed (Ctrl-C stops it).
#
# The Cube owns one USB port: stop this service before using MAVProxy, Mission Planner over USB
# or check_cube_params.py on the Orin:   sudo systemctl stop uav-fclink
source "$(dirname "$(readlink -f "$0")")/uav_env.sh"
log_service_to_file fclink     # also log to ~/flight_logs/boot####_.../fclink.log
# Job control: without it, bash starts background programs with Ctrl-C (SIGINT) ignored, so they
# could not be stopped cleanly and the recording would be cut off.
set -m
CUBE=/dev/serial/by-id/usb-Hex_ProfiCNC_CubeOrange_440045001451303437363830-if00

if pgrep -f '^/opt/ros/jazzy/lib/mavros/mavros_node' >/dev/null; then
    echo "ERROR: MAVROS is already running (another terminal, or uav-mission.service). Stop it first."
    exit 1
fi

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

# The bridge needs cuVSLAM; it waits by itself, but say so if VIO isn't up yet.
ros2 topic list 2>/dev/null | grep -qx /visual_slam/tracking/odometry \
    || echo "note: cuVSLAM not publishing yet; the bridge sends nothing to the Cube until it is"

# Recordings are numbered (run0001, run0002, ...) because the Orin's clock reads 1969 after a boot
# without internet (no RTC battery), so dates can't be trusted for naming or for cleanup.
n=$(( $(cat $UAV_LOG_ROOT/.run_counter 2>/dev/null || echo 0) + 1 ))
echo $n > $UAV_LOG_ROOT/.run_counter
LOG=$UAV_LOG_ROOT/$(printf run%04d $n)_$(date +%Y%m%d_%H%M%S)_fclink
mkdir -p "$LOG"
# Keep the newest 40 numbered recordings (by run number, not date). Folders renamed by hand
# (e.g. 2026-11-18_outdoor_vio_flight) don't match run*_fclink and are never deleted.
ls -1d $UAV_LOG_ROOT/run[0-9][0-9][0-9][0-9]_*_fclink 2>/dev/null | sort | head -n -40 | xargs -r rm -rf

ros2 launch uav_vio fc_link.launch.py bridge:=true &
LAUNCH=$!
sleep 8
ros2 bag record -s mcap -o "$LOG/bag" \
    /visual_slam/tracking/odometry /mavros/odometry/out /mavros/local_position/pose \
    /mavros/local_position/velocity_local /mavros/imu/data /mavros/rc/in /mavros/state \
    /mavros/statustext/recv /uav/fc_status /vio_bridge/status > "$LOG/bag_record.log" 2>&1 &
BAG=$!
echo "recording to $LOG"; echo "$LOG" >> "$UAV_BOOT_LOG_DIR/recordings.txt"

stop() {   # recorder first (closes the bag), then MAVROS + bridge
    kill -INT $BAG 2>/dev/null; wait $BAG 2>/dev/null
    kill -INT $LAUNCH 2>/dev/null; wait $LAUNCH 2>/dev/null
}
trap 'stop; exit 0' TERM INT

# Watchdog: MAVROS does not notice when the Cube is unplugged/power-cycled and keeps a dead port.
# If the Cube disappears or comes back as a new device, exit so systemd restarts the link.
while kill -0 $LAUNCH 2>/dev/null; do
    sleep 2
    if [ ! -e $CUBE ] || [ "$(stat -L -c %i $CUBE 2>/dev/null)" != "$ino" ]; then
        echo "Cube disconnected or re-plugged: restarting the link"
        stop; exit 3
    fi
done
echo "fc_link launch exited"
stop; exit 1
