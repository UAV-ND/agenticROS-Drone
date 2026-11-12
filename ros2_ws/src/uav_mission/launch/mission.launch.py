# UAV-ND MVP mission: MAVROS (with command plugins) + fc_status + mission node (+ optional VIO bridge).
#
#   ros2 launch uav_mission mission.launch.py fcu_url:=udp://:14550@     # ArduPilot SITL
#   ros2 launch uav_mission mission.launch.py bridge:=true               # real Cube over USB + VIO
#   ... fake_die:=true die_x:=4.0 die_y:=4.0                             # SITL: simulated die detector
#   ... rc_trigger:=true [rc_channel:=6]                                  # pilot starts it with an RC switch
#
# Then:  ros2 service call /uav/mission/start std_srvs/srv/Trigger
#        ros2 service call /uav/mission/abort std_srvs/srv/Trigger
#        ros2 topic echo /uav/mission/state
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
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
        DeclareLaunchArgument('fake_die', default_value='false'),
        DeclareLaunchArgument('rc_trigger', default_value='false'),
        DeclareLaunchArgument('rc_channel', default_value='6'),
        DeclareLaunchArgument('die_x', default_value='4.0'),
        DeclareLaunchArgument('die_y', default_value='4.0'),
        DeclareLaunchArgument('hide_after_s', default_value='-1.0'),
        DeclareLaunchArgument('hide_for_s', default_value='0.0'),
        IncludeLaunchDescription(
            os.path.join(vio, 'launch', 'fc_link.launch.py'),
            launch_arguments={'fcu_url': LaunchConfiguration('fcu_url'),
                              'bridge': LaunchConfiguration('bridge'),
                              'allow_commands': 'true'}.items()),
        Node(package='uav_mission', executable='mission_node', name='uav_mission', output='screen',
             parameters=[LaunchConfiguration('params')]),
        Node(package='uav_mission', executable='rc_trigger', name='rc_trigger', output='screen',
             parameters=[{'channel': LaunchConfiguration('rc_channel')}],
             condition=IfCondition(LaunchConfiguration('rc_trigger'))),
        Node(package='uav_mission', executable='fake_die', name='fake_die', output='screen',
             parameters=[{'die_x': LaunchConfiguration('die_x'), 'die_y': LaunchConfiguration('die_y'),
                          'hide_after_s': LaunchConfiguration('hide_after_s'),
                          'hide_for_s': LaunchConfiguration('hide_for_s')}],
             condition=IfCondition(LaunchConfiguration('fake_die'))),
    ])
