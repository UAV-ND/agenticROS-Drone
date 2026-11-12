#!/usr/bin/env bash
# Install MAVROS (ROS 2 Jazzy) on the UAV-ND Orin. Run by a human (needs sudo):
#   bash install_mavros.sh
# mavros_extras is required: the odometry plugin (/mavros/odometry/out -> MAVLink ODOMETRY) lives there.
# GeographicLib datasets are required or the mavros node refuses to start.
set -e
sudo apt-get update
sudo apt-get install -y ros-jazzy-mavros ros-jazzy-mavros-extras
DS=/opt/ros/jazzy/lib/mavros/install_geographiclib_datasets.sh
if [ -x "$DS" ]; then
  sudo "$DS"
else
  echo "Dataset script not found at $DS; trying the packaged tools"
  sudo geographiclib-get-geoids egm96-5
  sudo geographiclib-get-gravity egm96
  sudo geographiclib-get-magnetic emm2015
fi
echo "MAVROS installed. Check: ls /opt/ros/jazzy/share/mavros_extras && ls /usr/share/GeographicLib/geoids"
