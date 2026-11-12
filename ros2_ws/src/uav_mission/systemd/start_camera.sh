#!/bin/bash
# uav-camera.service: D435i driver (IR pair + IMU, emitter off) with the VIO config.
source "$(dirname "$(readlink -f "$0")")/uav_env.sh"
exec ros2 launch realsense2_camera rs_launch.py config_file:=$HOME/vio_config/d435i_vio.yaml
