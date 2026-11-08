"""UAV-ND MVP mission state machine: pure Python, no ROS (unit-tested in test/test_mission_core.py).

Structure follows the old uavnd-aimm-2026 main_controller.py (one handler per state, each decides
the next state), reshaped for ROS: every tick the node passes in an `Inputs` snapshot and gets back
the commands the mission *wants* right now. `CommandGate` turns that wish list into actual FC calls
(sent on change, re-sent every `retry_s` while still wanted), so a lost reply just means a retry.

Implemented so far (mvp/uav_simplified_mission_state_machine_v3.svg):
  IDLE -> PREFLIGHT -> TAKEOFF -> HOVER -> LAND -> DONE
  HOVER stands in for Transit / Grid sweep / Center on die until those are built.
Global rules for airborne states:
  link lost, or VIO lost while EKF uses it (source set 2)  -> FAILSAFE (EKF to set 1, hold, land)
  low battery or mission time limit                         -> RTL
  pilot changes mode on the RC                              -> PILOT (mission stops commanding)
"""
from dataclasses import dataclass, field
from typing import Optional

IDLE, PREFLIGHT, TAKEOFF, HOVER, LAND, RTL, FAILSAFE, DONE, PILOT, ABORTED = (
    'IDLE', 'PREFLIGHT', 'TAKEOFF', 'HOVER', 'LAND', 'RTL', 'FAILSAFE', 'DONE', 'PILOT', 'ABORTED')
TERMINAL = {DONE, PILOT, ABORTED}
AIRBORNE = {TAKEOFF, HOVER, LAND, RTL, FAILSAFE}

# Flight modes the mission itself puts the FC in, per state. Anything else (after a grace period)
# means the pilot took over.
ALLOWED_MODES = {
    TAKEOFF: {'GUIDED'},
    HOVER: {'GUIDED'},
    FAILSAFE: {'GUIDED', 'LAND'},
    LAND: {'LAND'},
    RTL: {'RTL', 'LAND'},
}

VIO_SOURCE_SET = 2
FLOW_SOURCE_SET = 1


@dataclass
class Params:
    takeoff_alt: float = 3.0          # m above the ground height at arming
    alt_tol: float = 0.3              # m
    vz_tol: float = 0.2               # m/s
    stable_s: float = 2.0             # "At 3 m, stable 2 s"
    hover_s: float = 10.0             # placeholder for transit/sweep
    preflight_timeout_s: float = 20.0
    takeoff_timeout_s: float = 30.0
    takeoff_retry_s: float = 5.0      # re-send takeoff only if still on the ground after this
    min_start_battery: float = 0.5    # fraction
    rtl_battery: float = 0.3          # fraction
    time_limit_s: float = 600.0       # from takeoff
    failsafe_hold_s: float = 3.0      # "Hold on flow, then land"
    mode_grace_s: float = 3.0         # time allowed for a mode change we asked for to show up
    require_vio: bool = False         # refuse to start unless VIO is healthy and EKF is on set 2


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


@dataclass(frozen=True)
class Cmd:
    kind: str                         # 'mode' | 'arm' | 'takeoff' | 'hold' | 'ekf_source'
    args: tuple = ()


@dataclass
class Mission:
    p: Params = field(default_factory=Params)
    state: str = IDLE
    reason: str = ''
    t_state: float = 0.0
    t_takeoff: Optional[float] = None
    ground_z: Optional[float] = None
    hold: Optional[tuple] = None      # (x, y, z, yaw) target for HOVER / FAILSAFE
    t_stable: Optional[float] = None
    abort_requested: bool = False
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
        if self.state in TERMINAL or self.state == IDLE:
            return []
        self._global_rules(i)
        if self.state in TERMINAL:          # a global rule ended the mission: send nothing
            return []
        handler = getattr(self, '_' + self.state.lower())
        return handler(i)

    # ---- transitions that apply to many states ------------------------------------------------
    def _global_rules(self, i: Inputs):
        s, p = self.state, self.p
        if s == PREFLIGHT:
            if self.abort_requested:
                self._go(ABORTED, i.now, 'aborted by operator before takeoff')
            return
        # airborne states from here on
        if i.link_ok and not i.armed and s in (TAKEOFF, HOVER, FAILSAFE):
            self._go(ABORTED, i.now, 'disarmed in flight state (pilot disarm or crash detect)')
            return
        if i.link_ok and i.now - self.t_state > p.mode_grace_s and i.mode not in ALLOWED_MODES[s]:
            self._go(PILOT, i.now, f'pilot override: mode {i.mode} in {s}')
            return
        vio_lost = i.source_set == VIO_SOURCE_SET and not i.vio_ok
        if s in (TAKEOFF, HOVER) and (not i.link_ok or vio_lost):
            self._go(FAILSAFE, i.now, 'FC link lost' if not i.link_ok else 'VIO lost on EKF source set 2',
                     hold_from=i)
            return
        if s == RTL and vio_lost:
            # FAILSAFE holds with GUIDED targets, which RTL mode ignores: switch to flow and land.
            # (Link loss during RTL needs nothing from us: the FC flies the RTL on its own.)
            self._go(LAND, i.now, 'VIO lost during RTL')
            return
        if s in (TAKEOFF, HOVER):
            if self.abort_requested:
                self._go(LAND, i.now, 'aborted by operator')
            elif i.battery is not None and i.battery < p.rtl_battery:
                self._go(RTL, i.now, f'low battery {i.battery:.0%}')
            elif self.t_takeoff is not None and i.now - self.t_takeoff > p.time_limit_s:
                self._go(RTL, i.now, f'time limit {p.time_limit_s:.0f} s')

    # ---- per-state handlers -------------------------------------------------------------------
    def _preflight(self, i: Inputs) -> list:
        if i.now - self.t_state > self.p.preflight_timeout_s:
            self._go(ABORTED, i.now, 'preflight timeout (could not enter GUIDED / arm)')
            return []
        if i.mode != 'GUIDED':
            return [Cmd('mode', ('GUIDED',))]
        if not i.armed:
            return [Cmd('arm')]
        self.ground_z = i.z
        self.t_takeoff = i.now
        self._go(TAKEOFF, i.now, 'armed in GUIDED')
        return self._takeoff(i)

    def _takeoff(self, i: Inputs) -> list:
        p = self.p
        target = self.ground_z + p.takeoff_alt
        if i.now - self.t_state > p.takeoff_timeout_s:
            self._go(LAND, i.now, f'takeoff timeout (z {i.z - self.ground_z:.2f} m)')
            return self._land(i)
        if abs(i.z - target) < p.alt_tol and abs(i.vz) < p.vz_tol:
            self.t_stable = self.t_stable if self.t_stable is not None else i.now
            if i.now - self.t_stable >= p.stable_s:
                self._go(HOVER, i.now, f'at {p.takeoff_alt:.1f} m, stable {p.stable_s:.0f} s',
                         hold_from=i, hold_z=target)
                return self._hover(i)
        else:
            self.t_stable = None
        # Send takeoff on entry; re-send only if still on the ground after takeoff_retry_s (motor
        # spool-up takes a moment, and ArduPilot rejects a takeoff once it is flying).
        t_in = i.now - self.t_state
        if i.z - self.ground_z < 0.3 and (t_in < 0.5 or t_in > p.takeoff_retry_s):
            return [Cmd('takeoff', (p.takeoff_alt,))]
        return []

    def _hover(self, i: Inputs) -> list:
        if i.now - self.t_state >= self.p.hover_s:
            self._go(LAND, i.now, 'hover done')
            return self._land(i)
        return [Cmd('hold', self.hold)]

    def _failsafe(self, i: Inputs) -> list:
        cmds = []
        if i.source_set == VIO_SOURCE_SET and not i.vio_ok:
            cmds.append(Cmd('ekf_source', (FLOW_SOURCE_SET,)))
        if i.now - self.t_state >= self.p.failsafe_hold_s:
            self._go(LAND, i.now, 'failsafe hold done')
            return cmds + self._land(i)
        return cmds + [Cmd('hold', self.hold)]

    def _land(self, i: Inputs) -> list:
        if i.link_ok and not i.armed:
            self._go(DONE, i.now, 'landed and disarmed')
            return []
        cmds = [] if i.mode == 'LAND' else [Cmd('mode', ('LAND',))]
        if i.source_set == VIO_SOURCE_SET and not i.vio_ok:
            cmds.insert(0, Cmd('ekf_source', (FLOW_SOURCE_SET,)))
        return cmds

    def _rtl(self, i: Inputs) -> list:
        if i.link_ok and not i.armed:
            self._go(DONE, i.now, 'RTL complete, disarmed')
            return []
        return [] if i.mode in ('RTL', 'LAND') else [Cmd('mode', ('RTL',))]

    # ---- helpers ------------------------------------------------------------------------------
    def _reset(self):
        p, events = self.p, self.events
        self.__init__(p=p)
        self.events = events                # keep history across runs (the node logs from it)

    def _go(self, state: str, now: float, reason: str, hold_from: Optional[Inputs] = None,
            hold_z: Optional[float] = None):
        self.events.append((now, self.state, state, reason))
        self.state, self.reason, self.t_state, self.t_stable = state, reason, now, None
        if hold_from is not None:
            h = hold_from
            self.hold = (h.x, h.y, hold_z if hold_z is not None else h.z, h.yaw)

    def summary(self, now: float) -> dict:
        return {
            'state': self.state, 'reason': self.reason,
            'in_state_s': round(now - self.t_state, 1) if self.state != IDLE else None,
            'mission_s': round(now - self.t_takeoff, 1) if self.t_takeoff is not None else None,
            'hold': [round(v, 2) for v in self.hold] if self.hold else None,
        }


class CommandGate:
    """Send a wanted command when it changes, and re-send it every `retry_s` while still wanted."""

    def __init__(self, retry_s: float = 1.5):
        self.retry_s = retry_s
        self.last = {}                  # kind -> (Cmd, t_sent)

    def filter(self, wanted: list, now: float) -> list:
        out = []
        for c in wanted:
            prev = self.last.get(c.kind)
            if prev is None or prev[0] != c or now - prev[1] >= self.retry_s:
                self.last[c.kind] = (c, now)
                out.append(c)
        return out
