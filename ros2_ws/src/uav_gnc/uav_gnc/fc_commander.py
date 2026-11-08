"""UAV-ND flight-controller commander: the ONLY code that sends commands to the Cube (via MAVROS).

Used as a library inside the mission node, so exactly one process commands the FC. What it can do
is deliberately narrow (agreed 2026-11-07): set mode, arm, GUIDED takeoff, position targets,
velocity targets, switch the EKF source set. It never writes FC parameters and never disarms in the air.

Safety limits are applied here, not in the mission logic, so every caller gets them:
  * hold targets are clamped to `max_alt` (above the ground height given at arming) and to a
    horizontal radius `max_radius` around the takeoff point
  * velocity targets are capped to `max_speed` / `max_vz`, and the outward component is removed at
    the radius / altitude limits
  * takeoff altitude is clamped to `max_alt`
  * modes are restricted to an allowlist

MAVROS services/topics used (plugins: sys_status, command, setpoint_raw):
  /mavros/set_mode, /mavros/cmd/arming, /mavros/cmd/command, /mavros/setpoint_raw/local
"""
import math

from mavros_msgs.msg import PositionTarget
from mavros_msgs.srv import CommandBool, CommandLong, SetMode

from uav_gnc.limits import Limits, clamp_hold, clamp_velocity

MAV_CMD_NAV_TAKEOFF = 22
MAV_CMD_SET_EKF_SOURCE_SET = 42007
ALLOWED_MODES = {'GUIDED', 'LAND', 'RTL', 'LOITER', 'BRAKE'}

# position + yaw; ignore velocity, acceleration and yaw rate
POS_YAW_MASK = (PositionTarget.IGNORE_VX | PositionTarget.IGNORE_VY | PositionTarget.IGNORE_VZ |
                PositionTarget.IGNORE_AFX | PositionTarget.IGNORE_AFY | PositionTarget.IGNORE_AFZ |
                PositionTarget.IGNORE_YAW_RATE)
# velocity + yaw; ignore position, acceleration and yaw rate
VEL_YAW_MASK = (PositionTarget.IGNORE_PX | PositionTarget.IGNORE_PY | PositionTarget.IGNORE_PZ |
                PositionTarget.IGNORE_AFX | PositionTarget.IGNORE_AFY | PositionTarget.IGNORE_AFZ |
                PositionTarget.IGNORE_YAW_RATE)


class FcCommander:
    def __init__(self, node, limits: Limits):
        self.node, self.lim, self.log = node, limits, node.get_logger()
        self.origin = None            # (x, y, z) at arming: centre of max_radius, base of max_alt
        # Called with the set number when the FC ACCEPTS a source-set command. ArduPilot switches
        # silently on that command, so this is the only way the rest of the system learns of it.
        self.on_ekf_source_accepted = None
        self.pos = None               # latest local ENU position, set by the node (velocity limits)
        self.cli_mode = node.create_client(SetMode, '/mavros/set_mode')
        self.cli_arm = node.create_client(CommandBool, '/mavros/cmd/arming')
        self.cli_cmd = node.create_client(CommandLong, '/mavros/cmd/command')
        self.pub_sp = node.create_publisher(PositionTarget, '/mavros/setpoint_raw/local', 10)

    def set_origin(self, x, y, z):
        self.origin = (x, y, z)

    def services_ready(self) -> bool:
        return all(c.service_is_ready() for c in (self.cli_mode, self.cli_arm, self.cli_cmd))

    # ---- dispatch for mission_core.Cmd ----------------------------------------------------------
    def execute(self, cmd):
        fn = getattr(self, 'do_' + cmd.kind, None)
        if fn is None:
            self.log.error(f'unknown command {cmd}')
            return
        fn(*cmd.args)

    def do_mode(self, mode: str):
        if mode not in ALLOWED_MODES:
            self.log.error(f'mode {mode} not allowed')
            return
        self._call(self.cli_mode, SetMode.Request(custom_mode=mode), f'set_mode {mode}',
                   ok=lambda r: r.mode_sent)

    def do_arm(self):
        self._call(self.cli_arm, CommandBool.Request(value=True), 'arm', ok=lambda r: r.success)

    def do_takeoff(self, alt: float):
        alt = min(alt, self.lim.max_alt)
        req = CommandLong.Request(command=MAV_CMD_NAV_TAKEOFF, param7=float(alt))
        self._call(self.cli_cmd, req, f'takeoff {alt:.1f} m', ok=lambda r: r.success)

    def do_ekf_source(self, source_set: int):
        req = CommandLong.Request(command=MAV_CMD_SET_EKF_SOURCE_SET, param1=float(source_set))
        self._call(self.cli_cmd, req, f'EKF source set {source_set}', ok=lambda r: r.success,
                   on_ok=(lambda: self.on_ekf_source_accepted(source_set))
                   if self.on_ekf_source_accepted else None)

    def do_hold(self, x, y, z, yaw):
        if self.origin is None:
            self.log.error('hold before origin set; ignored')
            return
        (x, y, z), clamped = clamp_hold((x, y, z), self.origin, self.lim)
        if clamped:
            self.log.warn(f'hold target clamped to ({x:.2f}, {y:.2f}, {z:.2f})')
        sp = PositionTarget()
        sp.header.stamp = self.node.get_clock().now().to_msg()
        sp.coordinate_frame = PositionTarget.FRAME_LOCAL_NED   # MAVROS converts ENU -> NED
        sp.type_mask = POS_YAW_MASK
        sp.position.x, sp.position.y, sp.position.z = float(x), float(y), float(z)
        sp.yaw = float(yaw) if math.isfinite(yaw) else 0.0
        self.pub_sp.publish(sp)

    def do_velocity(self, vx, vy, vz, yaw):
        """World ENU velocity (m/s) + yaw. ArduPilot stops if these stop for 3 s: stream them."""
        if self.origin is None or self.pos is None:
            self.log.error('velocity before origin/position known; ignored')
            return
        (vx, vy, vz), clamped = clamp_velocity((vx, vy, vz), self.pos, self.origin, self.lim)
        if clamped:
            self.log.debug(f'velocity clamped to ({vx:.2f}, {vy:.2f}, {vz:.2f})')
        sp = PositionTarget()
        sp.header.stamp = self.node.get_clock().now().to_msg()
        sp.coordinate_frame = PositionTarget.FRAME_LOCAL_NED   # MAVROS converts ENU -> NED
        sp.type_mask = VEL_YAW_MASK
        sp.velocity.x, sp.velocity.y, sp.velocity.z = float(vx), float(vy), float(vz)
        sp.yaw = float(yaw) if math.isfinite(yaw) else 0.0
        self.pub_sp.publish(sp)

    # ---- helpers --------------------------------------------------------------------------------
    def _call(self, client, req, what, ok, on_ok=None):
        if not client.service_is_ready():
            self.log.warn(f'{what}: MAVROS service not ready')
            return
        fut = client.call_async(req)

        def done(f):
            try:
                r = f.result()
            except Exception as e:  # noqa: BLE001
                self.log.warn(f'{what}: call failed ({e})')
                return
            if ok(r):
                self.log.info(f'{what}: accepted')
                if on_ok:
                    on_ok()
            else:
                self.log.warn(f'{what}: rejected ({r})')
        fut.add_done_callback(done)
        self.log.info(f'{what}: sent')
