#!/bin/bash
# uav-cuvslam.service: cuVSLAM (Isaac ROS 4.6, Docker) on the live camera. Runs in the foreground
# with no time limit (unlike ~/vio_config/run_cuvslam_live.sh, which is for timed bench tests).
# Can also be run by hand in a terminal when the service is not installed (Ctrl-C stops it).
source "$(dirname "$(readlink -f "$0")")/uav_env.sh"
log_service_to_file cuvslam     # also log to ~/flight_logs/boot####_.../cuvslam.log
IMAGE=uavnd/cuvslam:4.6
docker image inspect $IMAGE >/dev/null 2>&1 || { echo "ERROR: Docker image $IMAGE not found"; exit 1; }

# Only one cuVSLAM may publish /visual_slam/*. A container from ~/vio_config/run_cuvslam_live.sh
# (bench tests) blocks the start; a leftover uav_cuvslam container (crash, old run) is replaced.
others=$(docker ps --filter ancestor=$IMAGE --format '{{.Names}}' | grep -vx uav_cuvslam)
if [ -n "$others" ]; then
    echo "ERROR: another cuVSLAM container is running ($others). Stop it first: docker stop $others"
    exit 1
fi
if docker ps -a --format '{{.Names}}' | grep -qx uav_cuvslam; then
    echo "removing old uav_cuvslam container"
    docker rm -f uav_cuvslam >/dev/null 2>&1
fi

echo "waiting for the camera topics (uav-camera.service)..."
wait_for_topic /camera/camera/infra1/image_rect_raw 180 || exit 1
wait_for_topic /camera/camera/infra2/image_rect_raw 60 || exit 1
wait_for_topic /camera/camera/imu 60 || exit 1
# VIO uses the pose at start as "level": the drone should be sitting level and still now.
echo "camera up; starting cuVSLAM (tracking_mode $UAV_CUVSLAM_MODE). Keep the drone level and still for a few seconds."
exec docker run --rm --name uav_cuvslam --init --privileged --gpus all --network host --ipc host \
  --runtime nvidia \
  -v /usr/lib/aarch64-linux-gnu/tegra:/usr/lib/aarch64-linux-gnu/tegra:ro \
  -v /usr/share/vpi3:/usr/share/vpi3:ro \
  -v /usr/src/jetson_multimedia_api:/usr/src/jetson_multimedia_api:ro \
  --user "$(id -u):$(id -g)" -e HOME=/tmp -e ROS_LOG_DIR=/tmp/roslog \
  ${ROS_DOMAIN_ID:+-e ROS_DOMAIN_ID=$ROS_DOMAIN_ID} \
  -v $HOME/vio_config/cuvslam:/cfg:ro \
  --entrypoint bash $IMAGE -c "source /opt/ros/jazzy/setup.bash && \
    export LD_LIBRARY_PATH=/usr/lib/aarch64-linux-gnu/tegra:\$LD_LIBRARY_PATH && \
    exec ros2 launch /cfg/cuvslam_live.launch.py tracking_mode:=$UAV_CUVSLAM_MODE"
