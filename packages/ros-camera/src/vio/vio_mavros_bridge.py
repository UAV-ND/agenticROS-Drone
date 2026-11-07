#!/usr/bin/env python3
"""UAV-ND VIO -> MAVROS bridge node (rclpy). Math and gating live in vio_bridge_core.py.

  source /opt/ros/jazzy/setup.bash
  python3 vio_mavros_bridge.py --ros-args --params-file vio_bridge.yaml

Subscribes  /visual_slam/tracking/odometry  (cuVSLAM, camera_link in cuVSLAM odom)
            /camera/camera/imu              (only when level_mode = camera_imu, until leveled)
Publishes   /mavros/odometry/out            (nav_msgs/Odometry, frame_id odom / child base_link:
                                             MAVROS's odometry plugin converts to MAVLink ODOMETRY
                                             LOCAL_FRD / BODY_FRD with its odom_ned / base_link_frd TFs)
            /vio_bridge/status              (std_msgs/String, JSON, 1 Hz)

Nothing is published until cuVSLAM is initialized and passes the gates; output stops on jumps,
gaps or high covariance and resumes after `recover_frames` good frames. While it is off, EKF3
falls back to source set 1 (ARK Flow). This node never talks to the flight controller itself.
"""
import json
import os
import sys

import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, qos_profile_sensor_data
from scipy.spatial.transform import Rotation as Rot
from sensor_msgs.msg import Imu
from std_msgs.msg import String

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from vio_bridge_core import Bridge, GateParams, Sample, mount_transform, se3  # noqa: E402


def stamp_s(st) -> float:
    return st.sec + st.nanosec * 1e-9


class VioMavrosBridge(Node):
    def __init__(self):
        super().__init__('vio_mavros_bridge')
        P = self.declare_parameter
        self.in_topic = P('input_topic', '/visual_slam/tracking/odometry').value
        self.out_topic = P('output_topic', '/mavros/odometry/out').value
        self.frame_id = P('frame_id', 'odom').value
        self.child_frame_id = P('child_frame_id', 'base_link').value
        mount = [P('cam_x', 0.0).value, P('cam_y', 0.0).value, P('cam_z', 0.0).value,
                 P('cam_roll_deg', 0.0).value, P('cam_pitch_down_deg', 30.0).value,
                 P('cam_yaw_deg', 0.0).value]
        defaults = GateParams()
        gate = GateParams(**{k: type(v)(P(k, v).value) for k, v in vars(defaults).items()})
        level_mode = P('level_mode', 'none').value
        self.max_rate = float(P('max_rate_hz', 30.0).value)
        self.bridge = Bridge(mount_transform(*mount), gate, level_mode,
                             level_samples=int(P('level_samples', 200).value))
        self.last_out = None
        self.last_R_oc = None
        self.prev_state = None

        self.pub = self.create_publisher(Odometry, self.out_topic,
                                         QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE,
                                                    history=HistoryPolicy.KEEP_LAST))
        self.status_pub = self.create_publisher(String, '/vio_bridge/status', 10)
        # BEST_EFFORT matches both reliable and best-effort publishers
        self.create_subscription(Odometry, self.in_topic, self.on_odom, qos_profile_sensor_data)
        self.imu_sub = None
        if level_mode == 'camera_imu':
            self.imu_sub = self.create_subscription(Imu, P('imu_topic', '/camera/camera/imu').value,
                                                    self.on_imu, qos_profile_sensor_data)
        self.create_timer(1.0, self.report)
        self.get_logger().info(
            f'{self.in_topic} -> {self.out_topic} ({self.frame_id}/{self.child_frame_id}); mount xyz '
            f'{mount[:3]} rpy_deg [{mount[3]}, {mount[4]} down, {mount[5]}]; level_mode {level_mode}; '
            f'gates: {vars(gate)}')

    def on_imu(self, m: Imu):
        a, w = m.linear_acceleration, m.angular_velocity
        self.bridge.add_imu([a.x, a.y, a.z], [w.x, w.y, w.z], self.last_R_oc)
        if self.bridge.state != Bridge.WAIT_LEVEL and self.imu_sub is not None:
            self.get_logger().info(f'Leveled from camera IMU. Camera tilt below horizontal: '
                                   f'{self.bridge.measured_tilt_deg:.1f} deg (cam_pitch_down_deg param: '
                                   f'{self.get_parameter("cam_pitch_down_deg").value})')
            self.destroy_subscription(self.imu_sub)
            self.imu_sub = None

    def on_odom(self, m: Odometry):
        p, q = m.pose.pose.position, m.pose.pose.orientation
        R_oc = Rot.from_quat([q.x, q.y, q.z, q.w]).as_matrix()
        self.last_R_oc = R_oc
        tw = m.twist.twist
        s = Sample(stamp_s(m.header.stamp), se3(R_oc, [p.x, p.y, p.z]),
                   np.array([tw.linear.x, tw.linear.y, tw.linear.z]),
                   np.array([tw.angular.x, tw.angular.y, tw.angular.z]),
                   np.array(m.pose.covariance).reshape(6, 6),
                   np.array(m.twist.covariance).reshape(6, 6))
        now = self.get_clock().now().nanoseconds * 1e-9
        out = self.bridge.process(s, now=now)
        self.log_transitions()
        if out is None:
            return
        if self.max_rate > 0 and self.last_out is not None and out.stamp - self.last_out < 1.0 / self.max_rate - 2e-3:
            return
        self.last_out = out.stamp
        o = Odometry()
        o.header.stamp = m.header.stamp          # camera time; MAVROS maps it to FCU time (timesync)
        o.header.frame_id = self.frame_id
        o.child_frame_id = self.child_frame_id
        o.pose.pose.position.x, o.pose.pose.position.y, o.pose.pose.position.z = map(float, out.p)
        (o.pose.pose.orientation.x, o.pose.pose.orientation.y,
         o.pose.pose.orientation.z, o.pose.pose.orientation.w) = map(float, out.q_xyzw)
        o.twist.twist.linear.x, o.twist.twist.linear.y, o.twist.twist.linear.z = map(float, out.v_b)
        o.twist.twist.angular.x, o.twist.twist.angular.y, o.twist.twist.angular.z = map(float, out.w_b)
        o.pose.covariance = [float(x) for x in out.pose_cov.flatten()]
        o.twist.covariance = [float(x) for x in out.twist_cov.flatten()]
        self.pub.publish(o)

    def log_transitions(self):
        st = self.bridge.state
        if st != self.prev_state:
            # rclpy fixes the severity per call site, so use two separate calls
            if st == Bridge.HOLD:
                self.get_logger().warn(f'state {self.prev_state} -> HOLD ({self.bridge.stats.last_reject}); output off')
            else:
                self.get_logger().info(f'state {self.prev_state} -> {st}')
            self.prev_state = st

    def report(self):
        s = self.bridge.stats
        d = {k: v for k, v in vars(s).items() if k != 'events'}
        d['state'] = self.bridge.state
        d['measured_tilt_deg'] = self.bridge.measured_tilt_deg
        d['jump_absorbed_m'] = round(d['jump_absorbed_m'], 3)
        self.status_pub.publish(String(data=json.dumps(d)))


def main():
    rclpy.init()
    node = VioMavrosBridge()
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
