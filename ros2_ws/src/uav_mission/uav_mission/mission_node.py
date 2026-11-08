#!/usr/bin/env python3
"""UAV-ND MVP mission node: runs mission_core.Mission at 10 Hz and commands the FC via uav_gnc.

Operator / AI-agent interface (the agent only starts, aborts and reads status):
  /uav/mission/start   std_srvs/Trigger   refused with the reason if preflight checks fail
  /uav/mission/abort   std_srvs/Trigger   airborne -> LAND, before takeoff -> ABORTED
  /uav/mission/state   std_msgs/String    JSON, 2 Hz

Inputs: /mavros/state, /mavros/local_position/{pose,velocity_local}, /mavros/battery,
/mavros/statustext/recv + /mavros/rc/in (EKF source set), /mavros/odometry/out (VIO bridge output;
fresh = VIO healthy). Needs MAVROS launched with command plugins (launch/mission.launch.py).
The RC pilot can always take over by changing mode; the mission then stops commanding (PILOT).
"""
import json
import math
from collections import deque

import rclpy
from geometry_msgs.msg import PoseStamped, TwistStamped
from mavros_msgs.msg import RCIn, State, StatusText
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import BatteryState
from std_msgs.msg import String, UInt8
from std_srvs.srv import Trigger

from uav_gnc.fc_commander import FcCommander
from uav_gnc.limits import Limits
from uav_mission.mission_core import CommandGate, Inputs, Mission, Params
from uav_vio.fc_status_core import FcStatus, source_set_from_rc


def yaw_from_quat(q) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class MissionNode(Node):
    def __init__(self):
        super().__init__('uav_mission')
        P = self.declare_parameter
        params = Params(**{k: type(v)(P(k, v).value) for k, v in vars(Params()).items()})
        limits = Limits(**{k: type(v)(P(k, v).value) for k, v in vars(Limits()).items()})
        self.link_timeout = float(P('link_timeout_s', 3.0).value)   # MAVROS state comes at ~1 Hz
        self.pose_timeout = float(P('pose_timeout_s', 0.5).value)
        self.vio_timeout = float(P('vio_timeout_s', 0.5).value)
        self.rc_channel = int(P('source_switch_channel', 10).value)

        self.m = Mission(p=params)
        self.gate = CommandGate(float(P('retry_s', 1.5).value))
        self.fc = FcCommander(self, limits)
        self.fc.on_ekf_source_accepted = self.on_source_accepted
        # tells fc_status (and anyone else) about commanded switches, which the FC does not announce
        self.pub_src = self.create_publisher(UInt8, '/uav/ekf_source_cmd', 10)
        self.fcs = FcStatus()
        self.state = None
        self.t_state = self.t_pose = self.t_vio = None
        self.pose = None
        self.vz = 0.0
        self.battery = None
        self.n_events = 0
        self.fc_errors = deque(maxlen=3)   # recent FC warnings/errors, e.g. 'Arm: Throttle (RC3) ...'

        be = qos_profile_sensor_data
        self.create_subscription(State, '/mavros/state', self.on_state, be)
        self.create_subscription(PoseStamped, '/mavros/local_position/pose', self.on_pose, be)
        self.create_subscription(TwistStamped, '/mavros/local_position/velocity_local', self.on_vel, be)
        self.create_subscription(BatteryState, '/mavros/battery', self.on_batt, be)
        self.create_subscription(StatusText, '/mavros/statustext/recv', self.on_text, be)
        self.create_subscription(RCIn, '/mavros/rc/in', self.on_rc, be)
        self.create_subscription(Odometry, '/mavros/odometry/out', self.on_vio, be)
        self.create_service(Trigger, '/uav/mission/start', self.srv_start)
        self.create_service(Trigger, '/uav/mission/abort', self.srv_abort)
        self.pub = self.create_publisher(String, '/uav/mission/state', 10)
        self.create_timer(0.1, self.tick)
        self.create_timer(0.5, self.publish_state)
        self.get_logger().info(f'mission params {vars(params)}; limits {vars(limits)}')

    # ---- inputs -------------------------------------------------------------------------------
    def now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def on_state(self, m):
        self.state, self.t_state = m, self.now()

    def on_pose(self, m):
        p, q = m.pose.position, m.pose.orientation
        self.pose, self.t_pose = (p.x, p.y, p.z, yaw_from_quat(q)), self.now()

    def on_vel(self, m):
        self.vz = m.twist.linear.z

    def on_batt(self, m):
        pct = m.percentage
        self.battery = pct if math.isfinite(pct) and 0.0 <= pct <= 1.0 else None

    def on_rc(self, m):
        i = self.rc_channel - 1
        self.fcs.rc_source_set = source_set_from_rc(m.channels[i]) if len(m.channels) > i else None

    def on_text(self, m):
        self.fcs.on_statustext(m.text)
        if m.severity <= StatusText.WARNING and (not self.fc_errors or self.fc_errors[-1] != m.text):
            self.fc_errors.append(m.text)       # (MAVProxy forwards each text twice: skip repeats)

    def on_source_accepted(self, source_set):
        self.fcs.on_source_command(source_set)
        self.pub_src.publish(UInt8(data=int(source_set)))
        self.get_logger().warn(f'EKF switched to source set {source_set} by mission command')

    def on_vio(self, _m):
        self.t_vio = self.now()

    def inputs(self) -> Inputs:
        now = self.now()
        st = self.state
        link_ok = (st is not None and st.connected and self.t_state is not None
                   and now - self.t_state <= self.link_timeout)
        pose_ok = self.t_pose is not None and now - self.t_pose <= self.pose_timeout
        x, y, z, yaw = self.pose if self.pose else (0.0, 0.0, 0.0, 0.0)
        return Inputs(now=now, link_ok=link_ok, armed=bool(st and st.armed), mode=st.mode if st else '',
                      pose_ok=pose_ok, x=x, y=y, z=z, yaw=yaw, vz=self.vz, battery=self.battery,
                      vio_ok=self.t_vio is not None and now - self.t_vio <= self.vio_timeout,
                      source_set=self.fcs.effective_source_set())

    # ---- operator services --------------------------------------------------------------------
    def srv_start(self, _req, res):
        if not self.fc.services_ready():
            res.success, res.message = False, 'MAVROS command services not available (launch with commands)'
        else:
            res.success, res.message = self.m.start(self.inputs())
            if res.success:                 # fresh run: new origin for the limits, fresh gate
                self.fc.origin = None
                self.gate.last.clear()
                self.fc_errors.clear()
        if res.success:
            self.get_logger().info(f'start: {res.message}')
        else:
            self.get_logger().warn(f'start refused: {res.message}')
        return res

    def srv_abort(self, _req, res):
        res.success, res.message = self.m.abort()
        self.get_logger().warn(f'abort: {res.message}')
        return res

    # ---- main loop ----------------------------------------------------------------------------
    def tick(self):
        i = self.inputs()
        wanted = self.m.step(i)
        if self.m.ground_z is not None and self.fc.origin is None:
            self.fc.set_origin(i.x, i.y, self.m.ground_z)
        for _, a, b, why in self.m.events[self.n_events:]:
            # rclpy fixes the severity per call site, so use two separate calls
            if b in ('FAILSAFE', 'RTL', 'PILOT', 'ABORTED'):
                self.get_logger().warn(f'{a} -> {b}: {why}')
            else:
                self.get_logger().info(f'{a} -> {b}: {why}')
        self.n_events = len(self.m.events)
        for c in self.gate.filter(wanted, i.now):
            self.fc.execute(c)

    def publish_state(self):
        now = self.now()
        i = self.inputs()
        d = self.m.summary(now)
        d.update({'link_ok': i.link_ok, 'armed': i.armed, 'mode': i.mode, 'pose_ok': i.pose_ok,
                  'alt': round(i.z - self.m.ground_z, 2) if self.m.ground_z is not None else None,
                  'battery': i.battery, 'vio_ok': i.vio_ok, 'source_set': i.source_set,
                  'fc_errors': list(self.fc_errors),
                  'preflight_problems': self.m.preflight_problems(i) if self.m.state == 'IDLE' else []})
        self.pub.publish(String(data=json.dumps(d)))


def main():
    rclpy.init()
    node = MissionNode()
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
