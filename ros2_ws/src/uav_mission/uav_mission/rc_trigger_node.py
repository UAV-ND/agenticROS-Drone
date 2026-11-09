#!/usr/bin/env python3
"""Start / abort the mission from an RC switch (default ch6). Logic in rc_trigger_core.py.

Subscribes  /mavros/rc/in          (needs SR0_RC_CHAN > 0 on the Cube)
            /uav/mission/state     (JSON, to know whether the mission is flying)
Calls       /uav/mission/start, /uav/mission/abort   (std_srvs/Trigger), like an operator would
Publishes   /uav/rc_trigger/status (std_msgs/String, 1 Hz)

LOW -> HIGH starts (edge-triggered, debounced), HIGH -> LOW while flying aborts (mission lands).
The mission's preflight checks still decide whether a start is accepted.
"""
import json

import rclpy
from mavros_msgs.msg import RCIn
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import String
from std_srvs.srv import Trigger

from uav_mission.rc_trigger_core import RcTrigger, TriggerParams


class RcTriggerNode(Node):
    def __init__(self):
        super().__init__('rc_trigger')
        P = self.declare_parameter
        self.channel = int(P('channel', 6).value)
        p = TriggerParams(**{k: type(v)(P(k, v).value) for k, v in vars(TriggerParams()).items()})
        self.trig = RcTrigger(p)
        self.mission_state = ''
        self.pwm = None
        self.prev_status = None
        self.create_subscription(RCIn, '/mavros/rc/in', self.on_rc, qos_profile_sensor_data)
        self.create_subscription(String, '/uav/mission/state', self.on_state, 10)
        self.cli = {k: self.create_client(Trigger, f'/uav/mission/{k}') for k in ('start', 'abort')}
        self.pub = self.create_publisher(String, '/uav/rc_trigger/status', 10)
        self.create_timer(0.1, self.tick)
        self.create_timer(1.0, self.report)
        self.get_logger().info(f'RC trigger on ch{self.channel}: LOW->HIGH starts, HIGH->LOW in flight '
                               f'{"aborts" if p.abort_on_low else "does nothing"}; params {vars(p)}')

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def on_rc(self, m):
        i = self.channel - 1
        self.pwm = m.channels[i] if len(m.channels) > i else None
        self.trig.on_rc(self.pwm, self.now())

    def on_state(self, m):
        try:
            self.mission_state = json.loads(m.data).get('state', '')
        except ValueError:
            pass

    def tick(self):
        action = self.trig.step(self.now(), self.mission_state)
        if self.trig.status != self.prev_status:
            self.get_logger().info(f'ch{self.channel}: {self.trig.status}')
            self.prev_status = self.trig.status
        if action:
            self.call(action)

    def call(self, action):
        cli = self.cli[action]
        if not cli.service_is_ready():
            self.get_logger().error(f'{action}: mission service not available (is mission_node running?)')
            return
        fut = cli.call_async(Trigger.Request())

        def done(f):
            try:
                r = f.result()
            except Exception as e:  # noqa: BLE001
                self.get_logger().error(f'{action}: call failed ({e})')
                return
            if r.success:
                self.get_logger().warn(f'{action}: ACCEPTED ({r.message})')
            else:
                self.get_logger().error(f'{action}: REFUSED ({r.message}). Flip ch{self.channel} LOW then HIGH to retry')
        fut.add_done_callback(done)
        self.get_logger().warn(f'ch{self.channel} -> {action} requested')

    def report(self):
        self.pub.publish(String(data=json.dumps({
            'channel': self.channel, 'pwm': self.pwm, 'switch': self.trig.stable,
            'ready_to_start': self.trig.ready, 'mission_state': self.mission_state,
            'status': self.trig.status})))


def main():
    rclpy.init()
    node = RcTriggerNode()
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
