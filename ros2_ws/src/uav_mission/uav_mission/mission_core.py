"""UAV-ND MVP mission state machine: pure Python, no ROS (unit-tested in test/test_mission_core.py).

Structure follows the old uavnd-aimm-2026 main_controller.py (one handler per state, each decides
the next state), reshaped for ROS: every tick the node passes in an `Inputs` snapshot and gets back
the commands the mission *wants* right now. `CommandGate` turns that wish list into actual FC calls
(one-shot commands sent on change and re-sent every `retry_s` while still wanted; setpoints
streamed every tick), so a lost reply just means a retry.

Profiles (param `profile`), chosen after TAKEOFF:
  hover   TAKEOFF -> HOVER (hold the takeoff point for hover_s) -> LAND -> DONE
  square  TAKEOFF -> PATH (square of square_side m, relative to the takeoff pose; back to start)
          -> HOVER (hover_s) -> LAND -> DONE
  sweep   the full MVP mission below. With no detector running nothing is ever seen, so it just
          flies every lane; sweep_end 'return' then flies back to the start and lands there
          (PATH -> HOVER -> LAND) instead of the diagram's RTL.
States (mvp/uav_simplified_mission_state_machine_v3.svg):
  IDLE -> PREFLIGHT -> TAKEOFF --3 m, stable 2 s--> TRANSIT --at search area--> SWEEP
  SWEEP --target seen 5 frames--> CENTER --centered 5 s--> LAND -> DONE
  CENTER --target lost 2 s--> SWEEP (resumes where it left off)
  SWEEP --sweep done, no target--> RTL -> DONE
Global rules for airborne states:
  link lost, or VIO lost while EKF uses it (source set 2)  -> FAILSAFE (EKF to set 1, hold, land)
  low battery or mission time limit                         -> RTL
  pilot changes mode on the RC                              -> PILOT (mission stops commanding)

Geometry: the square and the search area are given in the TAKEOFF FRAME (metres forward / left of
where the drone armed, along its heading then), converted to local ENU here. Under VIO the EKF's
"east/north" are really the camera's start pose, so a takeoff-relative frame is the only intuitive
one. The search area is a placeholder rectangle until the competition field is known.
"""
import math
from dataclasses import dataclass, field
from typing import Optional

from uav_gnc.guidance import CenterController, PathFollower, PidGains, sweep_path

(IDLE, PREFLIGHT, TAKEOFF, HOVER, PATH, TRANSIT, SWEEP, CENTER, LAND, RTL, FAILSAFE, DONE, PILOT,
 ABORTED) = ('IDLE', 'PREFLIGHT', 'TAKEOFF', 'HOVER', 'PATH', 'TRANSIT', 'SWEEP', 'CENTER', 'LAND',
             'RTL', 'FAILSAFE', 'DONE', 'PILOT', 'ABORTED')
TERMINAL = {DONE, PILOT, ABORTED}
GUIDED_FLIGHT = {TAKEOFF, HOVER, PATH, TRANSIT, SWEEP, CENTER}   # mission flies these in GUIDED
PROFILES = ('hover', 'square', 'sweep')
AIRBORNE = GUIDED_FLIGHT | {LAND, RTL, FAILSAFE}

# Flight modes the mission itself puts the FC in, per state. Anything else (after a grace period)
# means the pilot took over.
ALLOWED_MODES = {s: {'GUIDED'} for s in GUIDED_FLIGHT}
ALLOWED_MODES.update({FAILSAFE: {'GUIDED', 'LAND'}, LAND: {'LAND'}, RTL: {'RTL', 'LAND'}})

VIO_SOURCE_SET = 2
FLOW_SOURCE_SET = 1


@dataclass
class Params:
    profile: str = 'sweep'            # 'hover' | 'square' | 'sweep' (see module docstring)
    hover_s: float = 10.0             # HOVER duration (hover / square profiles)
    square_side: float = 1.0          # m; + = forward then left of the takeoff pose, - = mirrored
    path_speed: float = 0.3           # m/s along the square
    takeoff_alt: float = 3.0          # m above the ground height at arming (also sweep altitude)
    alt_tol: float = 0.3              # m
    vz_tol: float = 0.2               # m/s
    stable_s: float = 2.0             # "At 3 m, stable 2 s"
    preflight_timeout_s: float = 20.0
    takeoff_timeout_s: float = 30.0
    takeoff_retry_s: float = 5.0      # re-send takeoff only if still on the ground after this
    min_start_battery: float = 0.5    # fraction
    rtl_battery: float = 0.3          # fraction
    time_limit_s: float = 600.0       # from takeoff
    failsafe_hold_s: float = 3.0      # "Hold on flow, then land"
    mode_grace_s: float = 3.0         # time allowed for a mode change we asked for to show up
    require_vio: bool = False         # refuse to start unless VIO is healthy and EKF is on set 2
    # search area (placeholder until the field is known): TAKEOFF FRAME, x forward / y left, m
    sweep_end: str = 'rtl'            # 'rtl' (diagram) | 'return' (fly back to start, land there)
    area_x_min: float = 2.0
    area_x_max: float = 6.0
    area_y_min: float = 2.0
    area_y_max: float = 6.0
    lane_spacing: float = 1.0         # m; must be < camera ground footprint at takeoff_alt
    transit_speed: float = 0.5        # m/s
    sweep_speed: float = 0.5          # m/s
    max_lead: float = 1.0             # m the position target may run ahead of the drone
    wp_tol: float = 0.3               # m "at search area" / end of sweep
    # target ("die") handling
    seen_frames: int = 5              # "Target seen 5 frames"
    lost_s: float = 2.0               # "Target lost 2 s"
    det_timeout_s: float = 0.3        # a detection older than this is not "seen now"
    center_tol: float = 0.15          # m (moving-average offset) counted as centered
    centered_s: float = 5.0           # "Centered for 5 s"
    center_kp: float = 0.6
    center_ki: float = 0.05
    center_kd: float = 0.1
    center_max_speed: float = 0.5     # m/s per axis
    center_ma_len: int = 5            # moving-average length (main_controller MAX_MA_*_LEN)
    alt_kp: float = 0.8               # (m/s)/m altitude hold while centering with velocity setpoints


@dataclass
class Inputs:
    now: float
    link_ok: bool                     # MAVROS connected and state fresh
    armed: bool = False
    mode: str = ''
    pose_ok: bool = False             # local position fresh (EKF has a position)
    x: float = 0.0                    # local ENU, m
    y: float = 0.0
    z: float = 0.0
    yaw: float = 0.0                  # ENU, rad
    vz: float = 0.0                   # m/s, up positive
    battery: Optional[float] = None   # fraction 0..1, None = unknown
    vio_ok: bool = False              # bridge output fresh
    source_set: Optional[int] = None  # EKF source set in use
    # latest target detection: body FLU offset of the target from the drone (m)
    det_seq: int = 0                  # increments per detection message
    det_frames: int = 0               # consecutive detections without a gap
    t_det: Optional[float] = None     # time of the latest detection
    det_fwd: float = 0.0
    det_left: float = 0.0


@dataclass(frozen=True)
class Cmd:
    kind: str     # 'mode' | 'arm' | 'takeoff' | 'hold' (x,y,z,yaw) | 'velocity' (vx,vy,vz,yaw) | 'ekf_source'
    args: tuple = ()


STREAMED = {'hold', 'velocity'}       # setpoints: sent every tick, never deduplicated


@dataclass
class Mission:
    p: Params = field(default_factory=Params)
    state: str = IDLE
    reason: str = ''
    t_state: float = 0.0
    t_prev: Optional[float] = None
    t_takeoff: Optional[float] = None
    ground_z: Optional[float] = None
    yaw_hold: float = 0.0             # heading kept for the whole flight (yaw at arming)
    hold: Optional[tuple] = None      # (x, y, z, yaw) for FAILSAFE / HOVER
    home: Optional[tuple] = None      # (x, y) at arming
    t_stable: Optional[float] = None
    abort_requested: bool = False
    follower: Optional[PathFollower] = None
    sweep_resume: Optional[tuple] = None   # (carrot, remaining points) when CENTER interrupted SWEEP
    center: Optional[CenterController] = None
    last_det_seq: int = 0
    t_centered: Optional[float] = None
    events: list = field(default_factory=list)

    # ---- operator interface -------------------------------------------------------------------
    def preflight_problems(self, i: Inputs) -> list:
        """Why the mission would refuse to start now (empty = OK)."""
        p, probs = self.p, []
        if not i.link_ok:
            probs.append('no FC link')
        if i.armed:
            probs.append('already armed')
        if not i.pose_ok:
            probs.append('no EKF local position')
        if p.profile not in PROFILES:
            probs.append(f'unknown profile {p.profile!r} (use one of {PROFILES})')
        if p.sweep_end not in ('rtl', 'return'):
            probs.append(f"unknown sweep_end {p.sweep_end!r} (use 'rtl' or 'return')")
        if i.battery is not None and i.battery < p.min_start_battery:
            probs.append(f'battery {i.battery:.0%} < {p.min_start_battery:.0%}')
        if p.require_vio and not (i.vio_ok and i.source_set == VIO_SOURCE_SET):
            probs.append(f'VIO required (vio_ok {i.vio_ok}, source set {i.source_set})')
        return probs

    def start(self, i: Inputs) -> tuple:
        if self.state in TERMINAL and not i.armed:
            self._reset()                   # finished (DONE / PILOT / ABORTED) and on the ground
        if self.state != IDLE:
            return False, f'mission already {self.state}'
        probs = self.preflight_problems(i)
        if probs:
            return False, '; '.join(probs)
        self._go(PREFLIGHT, i.now, 'start requested')
        return True, 'started'

    def abort(self) -> tuple:
        if self.state in TERMINAL or self.state == IDLE:
            return False, f'nothing to abort (state {self.state})'
        self.abort_requested = True
        return True, 'abort requested'

    # ---- main tick ----------------------------------------------------------------------------
    def step(self, i: Inputs) -> list:
        """Advance the state machine one tick; return the commands wanted right now."""
        dt = 0.0 if self.t_prev is None else max(0.0, i.now - self.t_prev)
        self.t_prev = i.now
        if self.state in TERMINAL or self.state == IDLE:
            return []
        self._global_rules(i)
        if self.state in TERMINAL:          # a global rule ended the mission: send nothing
            return []
        return getattr(self, '_' + self.state.lower())(i, dt)

    # ---- transitions that apply to many states ------------------------------------------------
    def _global_rules(self, i: Inputs):
        s, p = self.state, self.p
        if s == PREFLIGHT:
            if self.abort_requested:
                self._go(ABORTED, i.now, 'aborted by operator before takeoff')
            return
        # airborne states from here on
        if i.link_ok and not i.armed and s in GUIDED_FLIGHT | {FAILSAFE}:
            self._go(ABORTED, i.now, 'disarmed in flight state (pilot disarm or crash detect)')
            return
        if i.link_ok and i.now - self.t_state > p.mode_grace_s and i.mode not in ALLOWED_MODES[s]:
            self._go(PILOT, i.now, f'pilot override: mode {i.mode} in {s}')
            return
        vio_lost = i.source_set == VIO_SOURCE_SET and not i.vio_ok
        if s in GUIDED_FLIGHT and (not i.link_ok or vio_lost):
            self._go(FAILSAFE, i.now, 'FC link lost' if not i.link_ok else 'VIO lost on EKF source set 2',
                     hold_from=i)
            return
        if s == RTL and vio_lost:
            # FAILSAFE holds with GUIDED targets, which RTL mode ignores: switch to flow and land.
            # (Link loss during RTL needs nothing from us: the FC flies the RTL on its own.)
            self._go(LAND, i.now, 'VIO lost during RTL')
            return
        if s in GUIDED_FLIGHT:
            if self.abort_requested:
                self._go(LAND, i.now, 'aborted by operator')
            elif i.battery is not None and i.battery < p.rtl_battery:
                self._go(RTL, i.now, f'low battery {i.battery:.0%}')
            elif self.t_takeoff is not None and i.now - self.t_takeoff > p.time_limit_s:
                self._go(RTL, i.now, f'time limit {p.time_limit_s:.0f} s')

    # ---- per-state handlers -------------------------------------------------------------------
    def _preflight(self, i: Inputs, dt) -> list:
        if i.now - self.t_state > self.p.preflight_timeout_s:
            self._go(ABORTED, i.now, 'preflight timeout (could not enter GUIDED / arm)')
            return []
        if i.mode != 'GUIDED':
            return [Cmd('mode', ('GUIDED',))]
        if not i.armed:
            return [Cmd('arm')]
        self.ground_z, self.t_takeoff, self.yaw_hold, self.home = i.z, i.now, i.yaw, (i.x, i.y)
        self._go(TAKEOFF, i.now, 'armed in GUIDED')
        return self._takeoff(i, dt)

    def _takeoff(self, i: Inputs, dt) -> list:
        p = self.p
        target = self.ground_z + p.takeoff_alt
        if i.now - self.t_state > p.takeoff_timeout_s:
            self._go(LAND, i.now, f'takeoff timeout (z {i.z - self.ground_z:.2f} m)')
            return self._land(i, dt)
        if abs(i.z - target) < p.alt_tol and abs(i.vz) < p.vz_tol:
            self.t_stable = self.t_stable if self.t_stable is not None else i.now
            if i.now - self.t_stable >= p.stable_s:
                why = f'at {p.takeoff_alt:.1f} m, stable {p.stable_s:.0f} s'
                if p.profile == 'hover':
                    self._go(HOVER, i.now, why)
                    self.hold = (self.home[0], self.home[1], target, self.yaw_hold)
                    return self._hover(i, dt)
                if p.profile == 'square':
                    self.follower = PathFollower([(i.x, i.y, target)] + self._square_points(target),
                                                 p.path_speed, p.max_lead, p.wp_tol)
                    self._go(PATH, i.now, why)
                    return self._path(i, dt)
                start = self._sweep_points()[0]
                self.follower = PathFollower([(i.x, i.y, target), start], p.transit_speed,
                                             p.max_lead, p.wp_tol)
                self._go(TRANSIT, i.now, f'at {p.takeoff_alt:.1f} m, stable {p.stable_s:.0f} s')
                return self._transit(i, dt)
        else:
            self.t_stable = None
        # Send takeoff on entry; re-send only if still on the ground after takeoff_retry_s (motor
        # spool-up takes a moment, and ArduPilot rejects a takeoff once it is flying).
        t_in = i.now - self.t_state
        if i.z - self.ground_z < 0.3 and (t_in < 0.5 or t_in > p.takeoff_retry_s):
            return [Cmd('takeoff', (p.takeoff_alt,))]
        return []

    def _hover(self, i: Inputs, dt) -> list:
        if i.now - self.t_state >= self.p.hover_s:
            self._go(LAND, i.now, f'hover {self.p.hover_s:.0f} s done')
            return self._land(i, dt)
        return [Cmd('hold', self.hold)]

    def _path(self, i: Inputs, dt) -> list:
        pos = (i.x, i.y, i.z)
        if self.follower.done(pos):
            self.hold = (*self.follower.pts[-1], self.yaw_hold)
            self._go(HOVER, i.now, 'path done, back at start')
            return self._hover(i, dt)
        return [self._carrot_cmd(pos, dt)]

    def _transit(self, i: Inputs, dt) -> list:
        pos = (i.x, i.y, i.z)
        if self.follower.done(pos):
            self.follower = PathFollower(self._sweep_points(), self.p.sweep_speed, self.p.max_lead,
                                         self.p.wp_tol)
            self._go(SWEEP, i.now, 'at search area')
            return self._sweep(i, dt)
        return [self._carrot_cmd(pos, dt)]

    def _sweep(self, i: Inputs, dt) -> list:
        p, pos = self.p, (i.x, i.y, i.z)
        if self._seen_now(i) and i.det_frames >= p.seen_frames:
            self.sweep_resume = self.follower.progress()
            self.center = CenterController(
                PidGains(p.center_kp, p.center_ki, p.center_kd, out_max=p.center_max_speed),
                ma_len=p.center_ma_len)
            self.t_centered = None
            self._go(CENTER, i.now, f'target seen {i.det_frames} frames')
            return self._center(i, dt)
        if self.follower.done(pos):
            if p.sweep_end == 'return':
                z = self.ground_z + p.takeoff_alt
                self.follower = PathFollower([pos, (self.home[0], self.home[1], z)], p.transit_speed,
                                             p.max_lead, p.wp_tol)
                self._go(PATH, i.now, 'sweep done, no target: returning to start')
                return self._path(i, dt)
            self._go(RTL, i.now, 'sweep done, no target')
            return self._rtl(i, dt)
        return [self._carrot_cmd(pos, dt)]

    def _center(self, i: Inputs, dt) -> list:
        p = self.p
        if i.det_seq != self.last_det_seq and self._seen_now(i):
            self.center.add(i.det_fwd, i.det_left)
            self.last_det_seq = i.det_seq
        if i.t_det is None or i.now - i.t_det > p.lost_s:
            carrot, rest = self.sweep_resume
            self.follower = PathFollower([(i.x, i.y, i.z), carrot] + list(rest), p.sweep_speed,
                                         p.max_lead, p.wp_tol)
            self._go(SWEEP, i.now, f'target lost {p.lost_s:.0f} s: resuming sweep')
            return self._sweep(i, dt)
        off = self.center.offset()
        if off is not None and math.hypot(*off) < p.center_tol:
            self.t_centered = self.t_centered if self.t_centered is not None else i.now
            if i.now - self.t_centered >= p.centered_s:
                self._go(LAND, i.now, f'centered {p.centered_s:.0f} s (offset {math.hypot(*off):.2f} m)')
                return self._land(i, dt)
        else:
            self.t_centered = None
        vx, vy = self.center.velocity_world(i.yaw, dt)
        vz = self.p.alt_kp * (self.ground_z + p.takeoff_alt - i.z)
        return [Cmd('velocity', (vx, vy, vz, self.yaw_hold))]

    def _failsafe(self, i: Inputs, dt) -> list:
        cmds = []
        if i.source_set == VIO_SOURCE_SET and not i.vio_ok:
            cmds.append(Cmd('ekf_source', (FLOW_SOURCE_SET,)))
        if i.now - self.t_state >= self.p.failsafe_hold_s:
            self._go(LAND, i.now, 'failsafe hold done')
            return cmds + self._land(i, dt)
        return cmds + [Cmd('hold', self.hold)]

    def _land(self, i: Inputs, dt=0.0) -> list:
        if i.link_ok and not i.armed:
            self._go(DONE, i.now, 'landed and disarmed')
            return []
        cmds = [] if i.mode == 'LAND' else [Cmd('mode', ('LAND',))]
        if i.source_set == VIO_SOURCE_SET and not i.vio_ok:
            cmds.insert(0, Cmd('ekf_source', (FLOW_SOURCE_SET,)))
        return cmds

    def _rtl(self, i: Inputs, dt=0.0) -> list:
        if i.link_ok and not i.armed:
            self._go(DONE, i.now, 'RTL complete, disarmed')
            return []
        return [] if i.mode in ('RTL', 'LAND') else [Cmd('mode', ('RTL',))]

    # ---- helpers ------------------------------------------------------------------------------
    def _sweep_points(self):
        p = self.p
        body = sweep_path(p.area_x_min, p.area_x_max, p.area_y_min, p.area_y_max, p.lane_spacing,
                          self.ground_z + p.takeoff_alt)
        return [self._takeoff_to_local(f, l, z) for f, l, z in body]

    def _takeoff_to_local(self, fwd, left, z):
        c, s = math.cos(self.yaw_hold), math.sin(self.yaw_hold)
        return (self.home[0] + c * fwd - s * left, self.home[1] + s * fwd + c * left, z)

    def _square_points(self, z):
        """Square in the takeoff pose's frame: forward s, left s, back, right; ends at the start."""
        s = self.p.square_side
        return [self._takeoff_to_local(f, l, z) for f, l in [(s, 0.0), (s, s), (0.0, s), (0.0, 0.0)]]

    def _carrot_cmd(self, pos, dt) -> Cmd:
        x, y, z = self.follower.step(pos, dt)
        return Cmd('hold', (x, y, z, self.yaw_hold))

    def _seen_now(self, i: Inputs) -> bool:
        return i.t_det is not None and i.now - i.t_det <= self.p.det_timeout_s

    def _reset(self):
        p, events = self.p, self.events
        self.__init__(p=p)
        self.events = events                # keep history across runs (the node logs from it)

    def _go(self, state: str, now: float, reason: str, hold_from: Optional[Inputs] = None):
        self.events.append((now, self.state, state, reason))
        self.state, self.reason, self.t_state, self.t_stable = state, reason, now, None
        if hold_from is not None:
            h = hold_from
            self.hold = (h.x, h.y, h.z, self.yaw_hold if self.t_takeoff is not None else h.yaw)

    def summary(self, now: float) -> dict:
        d = {
            'state': self.state, 'reason': self.reason,
            'in_state_s': round(now - self.t_state, 1) if self.state != IDLE else None,
            'mission_s': round(now - self.t_takeoff, 1) if self.t_takeoff is not None else None,
        }
        if self.follower is not None and self.state in (PATH, TRANSIT, SWEEP):
            d['carrot'] = [round(v, 2) for v in self.follower.carrot]
            d['path_point'] = f'{self.follower.seg + 1}/{len(self.follower.pts)}'
        if self.state == CENTER and self.center is not None and self.center.offset() is not None:
            d['target_offset'] = [round(v, 2) for v in self.center.offset()]
        return d


class CommandGate:
    """One-shot commands: send when changed, re-send every `retry_s` while still wanted.
    Setpoints (STREAMED kinds): pass through every tick (ArduPilot drops velocity targets after 3 s)."""

    def __init__(self, retry_s: float = 1.5):
        self.retry_s = retry_s
        self.last = {}                  # kind -> (Cmd, t_sent)

    def filter(self, wanted: list, now: float) -> list:
        out = []
        for c in wanted:
            if c.kind in STREAMED:
                out.append(c)
                continue
            prev = self.last.get(c.kind)
            if prev is None or prev[0] != c or now - prev[1] >= self.retry_s:
                self.last[c.kind] = (c, now)
                out.append(c)
        return out
