# UAV-ND MVP mission: MAVROS (with command plugins) + fc_status + mission node (+ optional VIO bridge).
#
#   ros2 launch uav_mission mission.launch.py fcu_url:=udp://:14550@     # ArduPilot SITL
#   ros2 launch uav_mission mission.launch.py bridge:=true               # real Cube over USB + VIO
#
# Then:  ros2 service call /uav/mission/start std_srvs/srv/Trigger
#        ros2 service call /uav/mission/abort std_srvs/srv/Trigger
#        ros2 topic echo /uav/mission/state
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    vio = get_package_share_directory('uav_vio')
    mission_cfg = os.path.join(get_package_share_directory('uav_mission'), 'config', 'mission.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('fcu_url', default_value=
                              '/dev/serial/by-id/usb-Hex_ProfiCNC_CubeOrange_440045001451303437363830-if00:115200'),
        DeclareLaunchArgument('bridge', default_value='false'),
        DeclareLaunchArgument('params', default_value=mission_cfg),
        IncludeLaunchDescription(
            os.path.join(vio, 'launch', 'fc_link.launch.py'),
            launch_arguments={'fcu_url': LaunchConfiguration('fcu_url'),
                              'bridge': LaunchConfiguration('bridge'),
                              'allow_commands': 'true'}.items()),
        Node(package='uav_mission', executable='mission_node', name='uav_mission', output='screen',
             parameters=[LaunchConfiguration('params')]),
    ])
