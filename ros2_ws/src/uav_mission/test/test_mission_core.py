"""Mission state machine tests against a crude ArduCopter + downward-camera stand-in (no ROS)."""
import math

from uav_mission.mission_core import (ABORTED, CENTER, DONE, FAILSAFE, HOVER, IDLE, LAND, PATH, PILOT,
                                      PREFLIGHT, RTL, SWEEP, TAKEOFF, TRANSIT, Cmd, CommandGate, Inputs,
                                      Mission, Params)

DT = 0.1
FULL = [PREFLIGHT, TAKEOFF, TRANSIT, SWEEP]


class FakeCopter:
    """Modes, arming, GUIDED takeoff, position / velocity targets, LAND/RTL descent, die camera."""

    def __init__(self, die=None):
        self.t, self.mode, self.armed = 0.0, 'STABILIZE', False
        self.x = self.y = self.z = self.vz = 0.0
        self.yaw = 0.3                              # not aligned with the axes on purpose
        self.link_ok, self.battery, self.vio_ok, self.source_set = True, 0.9, True, 1
        self.takeoff_target = self.pos_target = self.vel = None
        self.t_vel = -1e9
        self.die, self.camera_on = die, True        # die (x, y) or None
        self.det_seq, self.det_frames, self.t_det, self.det = 0, 0, None, (0.0, 0.0)
        self.home_ok = True
        self.sent = []

    def apply(self, cmds):
        for c in cmds:
            self.sent.append(c)
            if not self.link_ok:
                continue
            if c.kind == 'mode':
                self.mode = c.args[0]
                self.pos_target = self.vel = None
            elif c.kind == 'arm' and self.mode == 'GUIDED':
                self.armed = True
            elif c.kind == 'takeoff' and self.armed and self.mode == 'GUIDED':
                self.takeoff_target = c.args[0]
            elif c.kind == 'hold' and self.mode == 'GUIDED':
                self.pos_target, self.vel = c.args[:3], None
            elif c.kind == 'velocity' and self.mode == 'GUIDED':
                self.vel, self.t_vel, self.pos_target = c.args[:3], self.t, None
            elif c.kind == 'ekf_source':
                self.source_set = c.args[0]

    def physics(self):
        self.t += DT
        vx = vy = 0.0
        vz = None
        if self.armed and self.mode == 'GUIDED':
            if self.vel is not None and self.t - self.t_vel < 3.0:
                vx, vy, vz = self.vel
            elif self.pos_target is not None:
                tx, ty, tz = self.pos_target
                vx, vy = [max(-2.0, min(2.0, 1.5 * e)) for e in (tx - self.x, ty - self.y)]
                vz = max(-1.0, min(1.0, 1.5 * (tz - self.z)))
            elif self.takeoff_target is not None:
                vz = max(-1.0, min(1.0, 1.5 * (self.takeoff_target - self.z)))
        elif self.mode in ('LAND', 'RTL') and self.armed:
            vz = -0.5 if self.z > 0.05 else 0.0
            if self.mode == 'RTL':
                vx, vy = [max(-2.0, min(2.0, -1.5 * e)) for e in (self.x, self.y)]
        self.vz = vz or 0.0
        self.x, self.y, self.z = self.x + vx * DT, self.y + vy * DT, self.z + self.vz * DT
        if self.mode in ('LAND', 'RTL') and self.z <= 0.05 and self.armed:
            self.armed, self.z, self.vz, self.takeoff_target = False, 0.0, 0.0, None
        self.camera()

    def camera(self):
        """Downward camera, half FOV 30 deg, detections in body FLU like fake_die."""
        seen = False
        if self.die is not None and self.camera_on and self.z > 1.0:
            dx, dy = self.die[0] - self.x, self.die[1] - self.y
            c, s = math.cos(self.yaw), math.sin(self.yaw)
            fwd, left = c * dx + s * dy, -s * dx + c * dy
            half = self.z * math.tan(math.radians(30))
            if abs(fwd) <= half and abs(left) <= half:
                seen = True
                self.det_frames = self.det_frames + 1 if (self.t_det is not None and self.t - self.t_det <= 0.3) else 1
                self.det_seq += 1
                self.t_det, self.det = self.t, (fwd, left)

    def inputs(self):
        return Inputs(now=self.t, link_ok=self.link_ok, armed=self.armed, mode=self.mode, pose_ok=True,
                      x=self.x, y=self.y, z=self.z, yaw=self.yaw, vz=self.vz, battery=self.battery,
                      vio_ok=self.vio_ok, source_set=self.source_set, home_ok=self.home_ok,
                      det_seq=self.det_seq,
                      det_frames=self.det_frames, t_det=self.t_det, det_fwd=self.det[0],
                      det_left=self.det[1])


def run(m, fc, seconds, hook=None, gate=None):
    gate = gate or CommandGate(0.5)
    for _ in range(int(seconds / DT)):
        if hook:
            hook(m, fc)
        fc.apply(gate.filter(m.step(fc.inputs()), fc.t))
        fc.physics()
    return [e[2] for e in m.events]


def started(params=None, die=(4.0, 4.0)):
    m, fc = Mission(p=params or Params()), FakeCopter(die)
    ok, msg = m.start(fc.inputs())
    assert ok, msg
    return m, fc


def when(state):
    """Hook helper: run f(m, fc) on every tick while the mission is in `state`."""
    def deco(f):
        return lambda m, fc: f(m, fc) if m.state == state else None
    return deco


# ---- nominal paths ---------------------------------------------------------------------------
def test_full_mission_finds_and_centers_on_die():
    m, fc = started()
    states = run(m, fc, 200)
    assert states == FULL + [CENTER, LAND, DONE]
    land_t = next(e[0] for e in m.events if e[2] == LAND)
    assert not fc.armed
    # landed over the die (LAND descends straight down in the fake)
    assert math.hypot(fc.x - 4.0, fc.y - 4.0) < 0.25, (fc.x, fc.y, land_t)
    kinds = [c.kind for c in fc.sent]
    assert kinds.index('mode') < kinds.index('arm') < kinds.index('takeoff') < kinds.index('hold')
    assert 'velocity' in kinds


def to_takeoff_frame(fc, x, y):
    c, s = math.cos(fc.yaw), math.sin(fc.yaw)          # fake yaw is constant = yaw at arming
    return c * x + s * y, -s * x + c * y


def test_no_target_sweeps_whole_area_then_rtl():
    m, fc = started(die=None)
    lefts, fwds = [], []

    def rec(m, fc):
        if m.state == SWEEP:
            f, l = to_takeoff_frame(fc, fc.x, fc.y)
            fwds.append(f)
            lefts.append(l)
    states = run(m, fc, 200, hook=rec)
    assert states == FULL + [RTL, DONE]
    p = Params()
    assert min(lefts) < p.area_y_min + 0.2 and max(lefts) > p.area_y_max - 0.2   # every lane flown
    assert min(fwds) < p.area_x_min + 0.2 and max(fwds) > p.area_x_max - 0.2    # full lane length


def test_sweep_without_detector_returns_to_start_and_lands():
    p = Params(sweep_end='return', takeoff_alt=1.5, area_x_min=0.5, area_x_max=2.0, area_y_min=0.0,
               area_y_max=1.0, lane_spacing=0.5, hover_s=3.0)
    m, fc = started(p, die=None)
    states = run(m, fc, 150)
    assert states == [PREFLIGHT, TAKEOFF, TRANSIT, SWEEP, PATH, HOVER, LAND, DONE]
    assert 'returning to start' in m.events[4][3]
    assert math.hypot(fc.x, fc.y) < 0.3 and RTL not in states


def test_transit_and_sweep_respect_speed():
    m, fc = started(die=None)
    prev = {}

    def check(m, fc):
        if m.state in (TRANSIT, SWEEP) and m.follower is not None:
            c = m.follower.carrot
            if 'c' in prev and prev['state'] == m.state:
                assert math.dist(c, prev['c']) <= Params().sweep_speed * DT + 1e-6
            prev.update(c=c, state=m.state)
    run(m, fc, 200, hook=check)


def test_sweep_altitude_is_takeoff_altitude():
    m, fc = started(die=None)
    zs = []
    run(m, fc, 200, hook=lambda m, fc: zs.append(fc.z) if m.state == SWEEP else None)
    assert min(zs) > 2.7 and max(zs) < 3.3


def test_flicker_below_5_frames_does_not_trigger_center():
    m, fc = started()
    shown = {'n': 0}

    def flicker(m, fc):                     # camera only works for 3 frames in a row, then blind
        fc.camera_on = shown['n'] < 3
        if fc.t_det is not None and fc.t - fc.t_det < 1e-9:
            shown['n'] += 1
    states = run(m, fc, 200, hook=flicker)
    assert CENTER not in states and states[-2:] == [RTL, DONE]


def test_target_lost_resumes_sweep_where_it_left_off():
    m, fc = started()
    info = {}

    def hide(m, fc):
        if m.state == CENTER and 'resume' not in info:
            info['resume'] = m.sweep_resume[0]
            info['t'] = fc.t
        if 't' in info:
            fc.camera_on = fc.t - info['t'] > 4.0      # blind for 4 s after centering starts
        if m.state == SWEEP and info.get('resumed') is None and 'resume' in info:
            info['resumed'] = m.follower.pts[1]        # first point after the drone's position
    states = run(m, fc, 250, hook=hide)
    assert states[:6] == FULL + [CENTER, SWEEP]
    assert m.events[5][3].startswith('target lost')
    assert info['resumed'] == info['resume']           # heads back to the exact sweep point
    assert states[-3:] == [CENTER, LAND, DONE]         # finds it again on the resumed sweep


def test_center_streams_velocity_every_tick():
    m, fc = started()
    gate, n = CommandGate(1.5), {'ticks': 0, 'vel': 0}

    def count(m, fc):
        if m.state == CENTER:
            n['ticks'] += 1
    run(m, fc, 200, hook=count, gate=gate)
    n['vel'] = sum(1 for c in fc.sent if c.kind == 'velocity')
    assert n['vel'] >= n['ticks'] - 1 > 0


# ---- global rules in the new states ---------------------------------------------------------
def test_pilot_override_in_sweep_stops_commands():
    m, fc = started(die=None)
    run(m, fc, 120, hook=when(SWEEP)(lambda m, fc: setattr(fc, 'mode', 'LOITER')))
    assert m.state == PILOT and fc.armed
    assert m.step(fc.inputs()) == []


def test_pilot_override_in_center():
    m, fc = started()
    run(m, fc, 200, hook=when(CENTER)(lambda m, fc: setattr(fc, 'mode', 'LOITER')))
    assert m.state == PILOT


def test_low_battery_in_sweep_rtl():
    m, fc = started(die=None)
    states = run(m, fc, 200, hook=when(SWEEP)(lambda m, fc: setattr(fc, 'battery', 0.25)))
    assert states[-2:] == [RTL, DONE] and 'low battery' in m.events[-2][3]


def test_time_limit_rtl():
    m, fc = started(Params(time_limit_s=25.0), die=None)
    states = run(m, fc, 200)
    assert RTL in states and m.events[states.index(RTL)][3].startswith('time limit')


def test_vio_loss_on_set2_failsafe_switches_to_flow_and_lands():
    m, fc = started(die=None)
    fc.source_set = 2
    states = run(m, fc, 200, hook=when(SWEEP)(lambda m, fc: setattr(fc, 'vio_ok', False)))
    assert states[-3:] == [FAILSAFE, LAND, DONE]
    assert fc.source_set == 1
    first_ekf = next(i for i, c in enumerate(fc.sent) if c.kind == 'ekf_source')
    first_land = next(i for i, c in enumerate(fc.sent) if c == Cmd('mode', ('LAND',)))
    assert first_ekf < first_land


def test_vio_loss_in_center_failsafe():
    m, fc = started()
    fc.source_set = 2
    states = run(m, fc, 200, hook=when(CENTER)(lambda m, fc: setattr(fc, 'vio_ok', False)))
    assert states[-3:] == [FAILSAFE, LAND, DONE]


def test_vio_loss_on_set1_is_ignored():
    m, fc = started()
    fc.vio_ok = False
    states = run(m, fc, 200)
    assert FAILSAFE not in states and states[-1] == DONE


def test_link_loss_failsafe_then_land_when_link_returns():
    m, fc = started(die=None)
    lost = {}

    def link(m, fc):
        if m.state == SWEEP and 't' not in lost:
            lost['t'], fc.link_ok = fc.t, False
        if 't' in lost and fc.t - lost['t'] > 5.0:
            fc.link_ok = True
    states = run(m, fc, 200, hook=link)
    assert states[-3:] == [FAILSAFE, LAND, DONE]


def test_vio_loss_during_rtl_lands():
    m, fc = started(die=None)
    fc.source_set = 2

    def h(m, fc):
        if m.state == SWEEP:
            fc.battery = 0.2
        if m.state == RTL:
            fc.vio_ok = False
    states = run(m, fc, 200, hook=h)
    assert states[-3:] == [RTL, LAND, DONE]


def test_abort_airborne_lands_and_before_takeoff_aborts():
    m, fc = started(die=None)
    run(m, fc, 200, hook=when(SWEEP)(lambda m, fc: m.abort() if not m.abort_requested else None))
    assert [e[2] for e in m.events][-2:] == [LAND, DONE]
    m2, fc2 = started()
    m2.abort()
    run(m2, fc2, 2)
    assert m2.state == ABORTED and not fc2.sent


def test_disarm_in_flight_aborts():
    m, fc = started(die=None)
    run(m, fc, 100, hook=when(TRANSIT)(lambda m, fc: setattr(fc, 'armed', False)))
    assert m.state == ABORTED


# ---- preflight / takeoff / plumbing (unchanged behaviour) ----------------------------------
def test_start_refused_with_reasons():
    m, fc = Mission(), FakeCopter()
    fc.link_ok, fc.battery = False, 0.2
    ok, msg = m.start(fc.inputs())
    assert not ok and 'no FC link' in msg and 'battery' in msg and m.state == IDLE
    m2 = Mission(p=Params(require_vio=True))
    ok, msg = m2.start(FakeCopter().inputs())
    assert not ok and 'VIO required' in msg


def test_preflight_timeout_when_arming_refused():
    m, fc = started(Params(preflight_timeout_s=5.0))
    fc.apply = lambda cmds: fc.sent.extend(cmds)
    run(m, fc, 10)
    assert m.state == ABORTED and 'preflight timeout' in m.reason


def test_takeoff_sent_once_when_climb_starts():
    m, fc = started(die=None)
    run(m, fc, 30, gate=CommandGate(1.5))
    assert len([c for c in fc.sent if c.kind == 'takeoff']) == 1


def test_takeoff_resent_if_still_on_ground():
    m, fc = started(Params(takeoff_retry_s=5.0), die=None)
    real_apply = fc.apply
    fc.apply = lambda cmds: real_apply([c for c in cmds if c.kind != 'takeoff' or fc.t > 10.0])
    run(m, fc, 40, gate=CommandGate(1.5))
    assert TRANSIT in [e[2] for e in m.events]


def test_mode_not_resent_once_fc_is_in_it():
    m, fc = started()
    run(m, fc, 200, gate=CommandGate(1.5))
    assert [c for c in fc.sent if c == Cmd('mode', ('LAND',))] == [Cmd('mode', ('LAND',))]


def test_restart_after_finished_mission_only_on_ground():
    m, fc = started()
    run(m, fc, 200)
    assert m.state == DONE
    ok, _ = m.start(fc.inputs())
    assert ok and m.state == PREFLIGHT and m.ground_z is None and m.follower is None
    m2, fc2 = started(die=None)
    run(m2, fc2, 120, hook=when(SWEEP)(lambda m, fc: setattr(fc, 'mode', 'LOITER')))
    assert m2.state == PILOT and fc2.armed
    ok, _ = m2.start(fc2.inputs())
    assert not ok and m2.state == PILOT


def test_command_gate_dedupes_oneshots_and_streams_setpoints():
    g = CommandGate(1.5)
    c = Cmd('mode', ('GUIDED',))
    assert g.filter([c], 0.0) == [c]
    assert g.filter([c], 1.0) == []
    assert g.filter([c], 1.6) == [c]
    h = Cmd('hold', (1.0, 2.0, 3.0, 0.0))
    assert g.filter([h], 2.0) == [h] and g.filter([h], 2.1) == [h]



# ---- simple profiles for the first real cuVSLAM flights -----------------------------------
def test_hover_profile_holds_takeoff_point_then_lands():
    m, fc = started(Params(profile='hover', takeoff_alt=1.5, hover_s=8.0), die=None)
    xy = []
    states = run(m, fc, 60, hook=lambda m, fc: xy.append((fc.x, fc.y, fc.z)) if m.state == HOVER else None)
    assert states == [PREFLIGHT, TAKEOFF, HOVER, LAND, DONE]
    assert all(math.hypot(x, y) < 0.05 and abs(z - 1.5) < 0.2 for x, y, z in xy)
    assert 'velocity' not in [c.kind for c in fc.sent]


def test_square_profile_flies_square_in_takeoff_frame_and_returns():
    m, fc = started(Params(profile='square', takeoff_alt=1.5, square_side=1.0, hover_s=3.0), die=None)
    pts = []
    states = run(m, fc, 90, hook=lambda m, fc: pts.append((fc.x, fc.y)) if m.state == PATH else None)
    assert states == [PREFLIGHT, TAKEOFF, PATH, HOVER, LAND, DONE]
    # the far corner in the takeoff frame (yaw 0.3 rad): forward 1 m, left 1 m
    c, s = math.cos(0.3), math.sin(0.3)
    corner = (c * 1 - s * 1, s * 1 + c * 1)
    assert min(math.dist(p, corner) for p in pts) < 0.15
    assert math.hypot(fc.x, fc.y) < 0.3                  # landed back near the start


def test_unknown_profile_refused():
    m = Mission(p=Params(profile='circle'))
    ok, msg = m.start(FakeCopter().inputs())
    assert not ok and 'unknown profile' in msg


def test_vio_loss_in_hover_profile_failsafe():
    m, fc = started(Params(profile='hover', takeoff_alt=1.5, hover_s=30.0), die=None)
    fc.source_set = 2
    states = run(m, fc, 60, hook=when(HOVER)(lambda m, fc: setattr(fc, 'vio_ok', False)))
    assert states[-3:] == [FAILSAFE, LAND, DONE]



def test_gps_denied_sends_origin_before_arming():
    m, fc = started(Params(profile='hover', takeoff_alt=1.5, hover_s=3.0), die=None)
    fc.home_ok = False
    real_apply = fc.apply

    def apply(cmds):
        for c in cmds:
            if c.kind == 'origin':
                fc.home_ok = True               # FC sets home from the origin
            if c.kind == 'arm' and not fc.home_ok:
                fc.sent.append(c)               # "AHRS: waiting for home": refused
                continue
            real_apply([c])
    fc.apply = apply
    states = run(m, fc, 40)
    kinds = [c.kind for c in fc.sent]
    assert kinds[0] == 'origin' and kinds.index('origin') < kinds.index('arm')
    assert states == [PREFLIGHT, TAKEOFF, HOVER, LAND, DONE]
