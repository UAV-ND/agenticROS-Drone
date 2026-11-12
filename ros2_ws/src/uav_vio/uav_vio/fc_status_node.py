#!/usr/bin/env python3
"""UAV-ND flight-controller / VIO status monitor (read-only; never commands the FC).

Subscribes  /mavros/state               (mavros_msgs/State: connected, armed, mode)
            /mavros/rc/in               (mavros_msgs/RCIn: ch10 = EKF source switch, RC10_OPTION 90)
            /mavros/statustext/recv     (mavros_msgs/StatusText: "Using EKF Source Set N", ...)
            /vio_bridge/status          (std_msgs/String JSON from vio_mavros_bridge)
            /uav/ekf_source_cmd         (std_msgs/UInt8: source set the mission switched to by
                                         command; ArduPilot does not announce those switches)
Publishes   /uav/fc_status              (std_msgs/String JSON, 1 Hz)

Logs a warning whenever the warning set changes, e.g. EKF on source set 2 while the bridge is in
HOLD. Note: /mavros/rc/in only updates if the Cube streams RC_CHANNELS (SR0_RC_CHAN > 0).
"""
import json

import rclpy
from mavros_msgs.msg import RCIn, State, StatusText
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import String, UInt8

from uav_vio.fc_status_core import FcStatus, source_set_from_rc


class FcStatusNode(Node):
    def __init__(self):
        super().__init__('fc_status')
        self.rc_channel = int(self.declare_parameter('source_switch_channel', 10).value)
        self.s = FcStatus()
        self.prev_warnings = []
        self.prev_source = None
        # MAVROS publishes several topics BEST_EFFORT (statustext/recv among them); a RELIABLE
        # subscriber silently gets nothing. A BEST_EFFORT subscriber matches either kind.
        be = qos_profile_sensor_data
        self.create_subscription(State, '/mavros/state', self.on_state, be)
        self.create_subscription(RCIn, '/mavros/rc/in', self.on_rc, be)
        self.create_subscription(StatusText, '/mavros/statustext/recv', self.on_text, be)
        self.create_subscription(String, '/vio_bridge/status', self.on_bridge, 10)
        self.create_subscription(UInt8, '/uav/ekf_source_cmd', self.on_source_cmd, 10)
        self.pub = self.create_publisher(String, '/uav/fc_status', 10)
        self.create_timer(1.0, self.tick)

    def now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def on_state(self, m: State):
        self.s.connected, self.s.armed, self.s.mode = m.connected, m.armed, m.mode
        self.s.t_state = self.now()

    def on_rc(self, m: RCIn):
        i = self.rc_channel - 1
        self.s.rc_source_set = source_set_from_rc(m.channels[i]) if len(m.channels) > i else None
        self.s.t_rc = self.now()

    def on_text(self, m: StatusText):
        self.s.on_statustext(m.text)
        if 'EKF' in m.text or 'VisOdom' in m.text:
            self.get_logger().info(f'FC: {m.text}')

    def on_source_cmd(self, m: UInt8):
        self.s.on_source_command(m.data)
        self.get_logger().info(f'EKF source set {m.data} commanded by mission (FC accepted)')

    def on_bridge(self, m: String):
        self.s.on_bridge_status(m.data, self.now())

    def tick(self):
        now = self.now()
        d = self.s.summary(now)
        self.pub.publish(String(data=json.dumps(d)))
        if d['source_set'] != self.prev_source:
            self.get_logger().info(f"EKF source set -> {d['source_set']} "
                                   f"(rc {d['rc_source_set']}, fc {d['fc_source_set']})")
            self.prev_source = d['source_set']
        if d['warnings'] != self.prev_warnings:
            for w in d['warnings']:
                self.get_logger().warn(w)
            if not d['warnings']:
                self.get_logger().info('all clear')
            self.prev_warnings = d['warnings']


def main():
    rclpy.init()
    node = FcStatusNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
