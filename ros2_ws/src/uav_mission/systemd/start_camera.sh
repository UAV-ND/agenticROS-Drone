#!/bin/bash
# Start the D435i camera driver with the VIO config (IR pair + IMU, emitter off).
# Used by uav-camera.service; can also be run by hand in a terminal (Ctrl-C stops it).
source "$(dirname "$(readlink -f "$0")")/uav_env.sh"
CONFIG=$HOME/vio_config/d435i_vio.yaml
[ -f "$CONFIG" ] || { echo "ERROR: camera config $CONFIG not found"; exit 1; }

# Only one process can own the camera.
if pgrep -f realsense2_camera_node >/dev/null; then
    echo "ERROR: a realsense camera node is already running (another terminal or uav-camera.service). Stop it first."
    exit 1
fi

# Wait for the D435i (USB id 8086:0b3a) to enumerate; at boot it can take a few seconds.
dev=""
for ((i = 0; i < 60; i += 2)); do
    for d in /sys/bus/usb/devices/*; do
        if [ "$(cat $d/idVendor 2>/dev/null)" = 8086 ] && [ "$(cat $d/idProduct 2>/dev/null)" = 0b3a ]; then dev=$d; fi
    done
    [ -n "$dev" ] && break
    [ $i = 0 ] && echo "waiting for the D435i on USB..."
    sleep 2
done
[ -n "$dev" ] || { echo "ERROR: D435i not found on USB after 60 s (cable / port?)"; exit 1; }

speed=$(cat $dev/speed)
if [ "$speed" -ge 5000 ]; then
    echo "D435i found at $(basename $dev), USB 3 (${speed} Mb/s)"
else
    # The driver falls back to 15 fps on USB 2, which is too slow for VIO. Start anyway (useful for
    # bench checks) but say so loudly; check_camera.sh reports it as a FAIL.
    echo "WARNING: D435i is on USB 2 (${speed} Mb/s): VIO needs USB 3. Re-seat the cable in a blue USB 3 port."
fi

echo "starting realsense2_camera with $CONFIG"
exec ros2 launch realsense2_camera rs_launch.py config_file:=$CONFIG
