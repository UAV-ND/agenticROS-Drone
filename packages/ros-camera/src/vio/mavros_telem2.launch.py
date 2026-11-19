# UAV-ND: MAVROS on the Cube Orange + the VIO bridge.
# Default link: Cube USB (/dev/ttyACM0 via by-id). The 40-pin UART (/dev/ttyTHS1 -> TELEM2) receives
# from the Cube but the Cube never hears the Orin (Nov 5, unresolved hardware fault); use
# fcu_url:=/dev/ttyTHS1:921600 once that's fixed.
#   source /opt/ros/jazzy/setup.bash
#   ros2 launch mavros_telem2.launch.py                  # MAVROS only
#   ros2 launch mavros_telem2.launch.py bridge:=true     # + vio_mavros_bridge.py (needs cuVSLAM running)
#
# Only the plugins below are loaded. No setpoint/command/param/mission plugins, so nothing here can
# (rc_io is only read here: /mavros/rc/in shows the source switch. Nothing publishes rc/override, and
# ArduPilot ignores RC_CHANNELS_OVERRIDE from MAVROS's sysid 1 unless SYSID_MYGCS is changed.)
# arm, change mode or write parameters. What MAVROS does send: its own HEARTBEAT, TIMESYNC/SYSTEM_TIME
# (needed to map camera stamps to FCU time) and, when the bridge runs, ODOMETRY.
# ArduPilot only fuses the ODOMETRY once a human sets VISO_TYPE 1 and the EK3_SRC2_* params.
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

HERE = os.path.dirname(os.path.abspath(__file__))


def generate_launch_description():
    fcu_url = LaunchConfiguration('fcu_url')
    return LaunchDescription([
        DeclareLaunchArgument(
            'fcu_url',
            default_value='/dev/serial/by-id/usb-Hex_ProfiCNC_CubeOrange_440045001451303437363830-if00:115200'),
        DeclareLaunchArgument('bridge', default_value='false'),
        Node(
            package='mavros', executable='mavros_node', output='screen',  # node names itself 'mavros' -> /mavros/...
            parameters=[{
                'fcu_url': fcu_url,
                'gcs_url': '',
                'tgt_system': 1,
                'tgt_component': 1,
                'fcu_protocol': 'v2.0',
                'plugin_allowlist': ['sys_status', 'sys_time', 'imu', 'local_position', 'odometry', 'rc_io'],
            }, os.path.join(HERE, 'mavros_params.yaml')],
        ),
        # vio_mavros_bridge.py is a plain script (not a colcon package), so start it as a process.
        ExecuteProcess(
            cmd=['python3', os.path.join(HERE, 'vio_mavros_bridge.py'), '--ros-args',
                 '--params-file', os.path.join(HERE, 'vio_bridge.yaml')],
            output='screen', condition=IfCondition(LaunchConfiguration('bridge'))),
    ])

