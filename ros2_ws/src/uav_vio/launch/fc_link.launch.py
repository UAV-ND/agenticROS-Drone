# UAV-ND: MAVROS link to the Cube Orange (+ optional VIO bridge and status monitor).
#
#   ros2 launch uav_vio fc_link.launch.py                           # MAVROS + fc_status
#   ros2 launch uav_vio fc_link.launch.py bridge:=true              # + vio_mavros_bridge (cuVSLAM must run)
#   ros2 launch uav_vio fc_link.launch.py fcu_url:=udp://:14550@    # e.g. ArduPilot SITL
#
# Default link: Cube USB (/dev/ttyACM0 via by-id). The 40-pin UART (/dev/ttyTHS1 -> TELEM2) receives
# from the Cube but the Cube never hears the Orin (Nov 5, unresolved hardware fault); use
# fcu_url:=/dev/ttyTHS1:921600 once that's fixed.
#
# READ-ONLY plugin set: no setpoint/command/param/mission plugins, so nothing here can arm, change
# mode or write parameters. (rc_io is only read: /mavros/rc/in shows the source switch. Nothing
# publishes rc/override, and ArduPilot ignores RC_CHANNELS_OVERRIDE from MAVROS's sysid 1 unless
# SYSID_MYGCS is changed.) What MAVROS does send: its own HEARTBEAT, TIMESYNC/SYSTEM_TIME (needed to
# map camera stamps to FCU time) and, when the bridge runs, ODOMETRY. ArduPilot only fuses the
# ODOMETRY once a human sets VISO_TYPE 1 and the EK3_SRC2_* params.
# The mission layer (uav_mission, later) will add the command plugins in its own launch file.
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

CUBE_USB = '/dev/serial/by-id/usb-Hex_ProfiCNC_CubeOrange_440045001451303437363830-if00:115200'
READ_ONLY_PLUGINS = ['sys_status', 'sys_time', 'imu', 'local_position', 'odometry', 'rc_io']


def generate_launch_description():
    cfg = os.path.join(get_package_share_directory('uav_vio'), 'config')
    return LaunchDescription([
        DeclareLaunchArgument('fcu_url', default_value=CUBE_USB),
        DeclareLaunchArgument('bridge', default_value='false'),
        DeclareLaunchArgument('status', default_value='true'),
        Node(
            package='mavros', executable='mavros_node', output='screen',  # node names itself 'mavros' -> /mavros/...
            parameters=[{
                'fcu_url': LaunchConfiguration('fcu_url'),
                'gcs_url': '',
                'tgt_system': 1,
                'tgt_component': 1,
                'fcu_protocol': 'v2.0',
                'plugin_allowlist': READ_ONLY_PLUGINS,
            }, os.path.join(cfg, 'mavros_params.yaml')],
        ),
        Node(
            package='uav_vio', executable='vio_mavros_bridge', name='vio_mavros_bridge', output='screen',
            parameters=[os.path.join(cfg, 'vio_bridge.yaml')],
            condition=IfCondition(LaunchConfiguration('bridge'))),
        Node(
            package='uav_vio', executable='fc_status', name='fc_status', output='screen',
            condition=IfCondition(LaunchConfiguration('status'))),
    ])
