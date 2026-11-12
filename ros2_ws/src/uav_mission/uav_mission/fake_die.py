#!/usr/bin/env python3
"""SITL stand-in for the die detector: "sees" a die lying at (die_x, die_y) in the local frame.

Publishes /uav/die/target (geometry_msgs/PointStamped, frame base_link): the die's offset from the
drone in body FLU metres (x forward, y left), z = confidence, at `rate_hz`, but only while the die
is inside the downward camera's ground footprint (half-width = altitude * tan(half_fov)) and the
drone is above `min_alt`. Same contract the real detector must meet.

Test knobs: noise_std (m), and hide_after_s / hide_for_s to make the die vanish once (e.g. to test
"target lost 2 s -> resume sweep").
"""
import math
import random

import rclpy
from geometry_msgs.msg import PointStamped, PoseStamped
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data


def yaw_from_quat(q) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def die_offset_body(die_xy, pos_xy, yaw):
    """World ENU offset of the die rotated into body FLU (forward, left)."""
    dx, dy = die_xy[0] - pos_xy[0], die_xy[1] - pos_xy[1]
    c, s = math.cos(yaw), math.sin(yaw)
    return c * dx + s * dy, -s * dx + c * dy


class FakeDie(Node):
    def __init__(self):
        super().__init__('fake_die')
        P = self.declare_parameter
        self.die = (float(P('die_x', 4.0).value), float(P('die_y', 4.0).value))
        self.half_fov = math.radians(float(P('half_fov_deg', 30.0).value))
        self.min_alt = float(P('min_alt', 1.0).value)
        self.noise = float(P('noise_std', 0.03).value)
        self.hide_after = float(P('hide_after_s', -1.0).value)
        self.hide_for = float(P('hide_for_s', 0.0).value)
        self.ground_z = None
        self.pose = None
        self.t_first = None
        self.create_subscription(PoseStamped, '/mavros/local_position/pose', self.on_pose,
                                 qos_profile_sensor_data)
        self.pub = self.create_publisher(PointStamped, '/uav/die/target', 10)
        self.create_timer(1.0 / float(P('rate_hz', 10.0).value), self.tick)
        self.get_logger().info(f'fake die at {self.die}, half FOV {math.degrees(self.half_fov):.0f} deg, '
                               f'hide_after {self.hide_after} s for {self.hide_for} s')

    def on_pose(self, m):
        p, q = m.pose.position, m.pose.orientation
        if self.ground_z is None:
            self.ground_z = p.z          # first pose = on the ground before takeoff
        self.pose = (p.x, p.y, p.z, yaw_from_quat(q))

    def tick(self):
        if self.pose is None:
            return
        x, y, z, yaw = self.pose
        alt = z - self.ground_z
        if alt < self.min_alt:
            return
        fwd, left = die_offset_body(self.die, (x, y), yaw)
        half = alt * math.tan(self.half_fov)
        if abs(fwd) > half or abs(left) > half:
            return
        now = self.get_clock().now().nanoseconds * 1e-9
        self.t_first = self.t_first if self.t_first is not None else now
        if self.hide_after >= 0 and self.hide_after <= now - self.t_first < self.hide_after + self.hide_for:
            return
        m = PointStamped()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = 'base_link'
        m.point.x = fwd + random.gauss(0.0, self.noise)
        m.point.y = left + random.gauss(0.0, self.noise)
        m.point.z = 0.9
        self.pub.publish(m)


def main():
    rclpy.init()
    node = FakeDie()
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
