#!/bin/bash
# uav-cuvslam.service: cuVSLAM (Isaac ROS 4.6, Docker) on the live camera. Runs in the foreground
# with no time limit (unlike ~/vio_config/run_cuvslam_live.sh, which is for timed bench tests).
source "$(dirname "$(readlink -f "$0")")/uav_env.sh"
wait_for_topic /camera/camera/infra1/image_rect_raw 180 || exit 1
wait_for_topic /camera/camera/imu 60 || exit 1
docker rm -f uav_cuvslam >/dev/null 2>&1
exec docker run --rm --name uav_cuvslam --init --privileged --gpus all --network host --ipc host \
  --runtime nvidia \
  -v /usr/lib/aarch64-linux-gnu/tegra:/usr/lib/aarch64-linux-gnu/tegra:ro \
  -v /usr/share/vpi3:/usr/share/vpi3:ro \
  -v /usr/src/jetson_multimedia_api:/usr/src/jetson_multimedia_api:ro \
  --user "$(id -u):$(id -g)" -e HOME=/tmp -e ROS_LOG_DIR=/tmp/roslog \
  ${ROS_DOMAIN_ID:+-e ROS_DOMAIN_ID=$ROS_DOMAIN_ID} \
  -v $HOME/vio_config/cuvslam:/cfg:ro \
  --entrypoint bash uavnd/cuvslam:4.6 -c "source /opt/ros/jazzy/setup.bash && \
    export LD_LIBRARY_PATH=/usr/lib/aarch64-linux-gnu/tegra:\$LD_LIBRARY_PATH && \
    exec ros2 launch /cfg/cuvslam_live.launch.py tracking_mode:=$UAV_CUVSLAM_MODE"
