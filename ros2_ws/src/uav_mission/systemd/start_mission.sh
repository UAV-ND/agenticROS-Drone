#!/bin/bash
# uav-mission.service: MAVROS (Cube over USB, command plugins) + VIO bridge + fc_status + mission node
# + RC trigger (pilot starts the mission with ch6 LOW->HIGH) + a flight-log recorder.
# Nothing flies until the pilot flips the switch AND the mission's preflight checks pass.
source "$(dirname "$(readlink -f "$0")")/uav_env.sh"
wait_for_topic /visual_slam/tracking/odometry 240 || echo "cuVSLAM not up yet; starting anyway (preflight will refuse until VIO is healthy)"
for i in $(seq 1 60); do ls /dev/serial/by-id/*CubeOrange*if00 >/dev/null 2>&1 && break; sleep 2; done
LOG=$UAV_LOG_ROOT/$(date +%Y%m%d_%H%M%S)
mkdir -p "$LOG"
cp "$UAV_MISSION_PARAMS" "$LOG/mission_params.yaml"
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
trap 'kill -INT $BAG $LAUNCH 2>/dev/null; wait $BAG $LAUNCH' TERM INT
wait $LAUNCH
kill -INT $BAG 2>/dev/null; wait $BAG
