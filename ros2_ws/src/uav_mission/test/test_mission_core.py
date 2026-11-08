"""Mission state machine tests against a crude ArduCopter stand-in (no ROS)."""
from uav_mission.mission_core import (ABORTED, DONE, FAILSAFE, HOVER, IDLE, LAND, PILOT, PREFLIGHT, RTL,
                                      TAKEOFF, Cmd, CommandGate, Inputs, Mission, Params)

DT = 0.1


class FakeCopter:
    """Just enough ArduCopter behaviour: modes, arming, GUIDED takeoff climb, LAND/RTL descent."""

    def __init__(self):
        self.t, self.mode, self.armed, self.z, self.vz = 0.0, 'STABILIZE', False, 0.0, 0.0
        self.link_ok, self.battery, self.vio_ok, self.source_set = True, 0.9, True, 1
        self.takeoff_target = None
        self.sent = []

    def apply(self, cmds):
        for c in cmds:
            self.sent.append(c)
            if c.kind == 'mode' and self.link_ok:
                self.mode = c.args[0]
            elif c.kind == 'arm' and self.link_ok and self.mode == 'GUIDED':
                self.armed = True
            elif c.kind == 'takeoff' and self.armed and self.mode == 'GUIDED':
                self.takeoff_target = c.args[0]
            elif c.kind == 'ekf_source' and self.link_ok:
                self.source_set = c.args[0]

    def physics(self):
        self.t += DT
        target = None
        if self.mode == 'GUIDED' and self.takeoff_target is not None:
            target = self.takeoff_target
        elif self.mode in ('LAND', 'RTL') and self.armed:
            target = 0.0
        if target is not None:
            err = target - self.z
            self.vz = max(-1.0, min(1.0, err * 1.5))
            if abs(err) < 0.02:
                self.vz = 0.0
            self.z += self.vz * DT
        if self.mode in ('LAND', 'RTL') and self.z <= 0.05 and self.armed:
            self.armed, self.z, self.vz, self.takeoff_target = False, 0.0, 0.0, None

    def inputs(self):
        return Inputs(now=self.t, link_ok=self.link_ok, armed=self.armed, mode=self.mode, pose_ok=True,
                      z=self.z, vz=self.vz, battery=self.battery, vio_ok=self.vio_ok,
                      source_set=self.source_set)


def run(m, fc, seconds, hook=None, gate=None):
    gate = gate or CommandGate(0.5)
    for _ in range(int(seconds / DT)):
        if hook:
            hook(m, fc)
        fc.apply(gate.filter(m.step(fc.inputs()), fc.t))
        fc.physics()
    return [e[2] for e in m.events]


def started(params=None):
    m, fc = Mission(p=params or Params(hover_s=5.0)), FakeCopter()
    ok, msg = m.start(fc.inputs())
    assert ok, msg
    return m, fc


def test_nominal_flight():
    m, fc = started()
    states = run(m, fc, 60)
    assert states == [PREFLIGHT, TAKEOFF, HOVER, LAND, DONE]
    assert not fc.armed and fc.z == 0.0
    kinds = [c.kind for c in fc.sent]
    assert kinds.index('mode') < kinds.index('arm') < kinds.index('takeoff') < kinds.index('hold')
    assert m.hold[2] == m.ground_z + 3.0


def test_hover_only_after_stable_2s():
    m, fc = started()
    t_hover = []
    run(m, fc, 30, hook=lambda m, fc: t_hover.append(fc.t) if m.state == HOVER and not t_hover else None)
    t_takeoff = [e[0] for e in m.events if e[2] == TAKEOFF][0]
    assert t_hover[0] - t_takeoff > 2.0 + 3.0 / 1.0 * 0.5   # climb takes time, then 2 s stable


def test_start_refused_with_reasons():
    m, fc = Mission(), FakeCopter()
    fc.link_ok, fc.battery = False, 0.2
    ok, msg = m.start(fc.inputs())
    assert not ok and 'no FC link' in msg and 'battery' in msg and m.state == IDLE
    m2 = Mission(p=Params(require_vio=True))
    ok, msg = m2.start(FakeCopter().inputs())       # source set 1 -> VIO not in use
    assert not ok and 'VIO required' in msg


def test_pilot_override_stops_commands():
    m, fc = started()

    def pilot(m, fc):
        if m.state == HOVER and fc.mode == 'GUIDED':
            fc.mode = 'LOITER'
    run(m, fc, 40, hook=pilot)
    assert m.state == PILOT
    assert m.step(fc.inputs()) == []                # mission silent after override
    assert fc.armed                                 # mission did not land or disarm


def test_mode_change_we_asked_for_is_not_override():
    m, fc = started(Params(hover_s=3.0))
    states = run(m, fc, 60)
    assert PILOT not in states


def test_low_battery_rtl():
    m, fc = started(Params(hover_s=30.0))

    def drain(m, fc):
        if m.state == HOVER:
            fc.battery = 0.25
    states = run(m, fc, 60, hook=drain)
    assert states[-2:] == [RTL, DONE]
    assert any(c == Cmd('mode', ('RTL',)) for c in fc.sent)


def test_time_limit_rtl():
    m, fc = started(Params(hover_s=100.0, time_limit_s=15.0))
    states = run(m, fc, 60)
    assert RTL in states and m.events[states.index(RTL)][3].startswith('time limit')


def test_vio_loss_on_set2_failsafe_switches_to_flow_and_lands():
    m, fc = started(Params(hover_s=30.0))
    fc.source_set = 2

    def lose(m, fc):
        if m.state == HOVER:
            fc.vio_ok = False
    states = run(m, fc, 60, hook=lose)
    assert states[-3:] == [FAILSAFE, LAND, DONE]
    assert Cmd('ekf_source', (1,)) in fc.sent
    assert fc.source_set == 1
    first_ekf = next(i for i, c in enumerate(fc.sent) if c.kind == 'ekf_source')
    first_land = next(i for i, c in enumerate(fc.sent) if c == Cmd('mode', ('LAND',)))
    assert first_ekf < first_land                   # switch to flow before descending


def test_vio_loss_on_set1_is_ignored():
    m, fc = started()
    fc.vio_ok = False                               # flow in use: VIO health irrelevant
    states = run(m, fc, 60)
    assert FAILSAFE not in states and states[-1] == DONE


def test_link_loss_failsafe_then_land_when_link_returns():
    m, fc = started(Params(hover_s=30.0))
    lost = {'t': None}

    def link(m, fc):
        if m.state == HOVER and lost['t'] is None:
            lost['t'] = fc.t
            fc.link_ok = False
        if lost['t'] is not None and fc.t - lost['t'] > 5.0:
            fc.link_ok = True
    states = run(m, fc, 80, hook=link)
    assert states[-3:] == [FAILSAFE, LAND, DONE]


def test_vio_loss_during_rtl_lands():
    m, fc = started(Params(hover_s=30.0))
    fc.source_set = 2

    def h(m, fc):
        if m.state == HOVER:
            fc.battery = 0.2
        if m.state == RTL:
            fc.vio_ok = False
    states = run(m, fc, 60, hook=h)
    assert states[-3:] == [RTL, LAND, DONE]


def test_abort_airborne_lands_and_before_takeoff_aborts():
    m, fc = started(Params(hover_s=30.0))
    run(m, fc, 40, hook=lambda m, fc: m.abort() if m.state == HOVER and not m.abort_requested else None)
    assert [e[2] for e in m.events][-2:] == [LAND, DONE]
    m2, fc2 = started()
    m2.abort()
    run(m2, fc2, 2)
    assert m2.state == ABORTED and not fc2.sent


def test_preflight_timeout_when_arming_refused():
    m, fc = started(Params(preflight_timeout_s=5.0))
    fc.apply = lambda cmds: fc.sent.extend(cmds)    # FC ignores everything
    run(m, fc, 10)
    assert m.state == ABORTED and 'preflight timeout' in m.reason


def test_disarm_in_flight_aborts():
    m, fc = started(Params(hover_s=30.0))
    run(m, fc, 40, hook=lambda m, fc: setattr(fc, 'armed', False) if m.state == HOVER else None)
    assert m.state == ABORTED


def test_command_gate_retries_and_dedupes():
    g = CommandGate(1.5)
    c = Cmd('mode', ('GUIDED',))
    assert g.filter([c], 0.0) == [c]
    assert g.filter([c], 1.0) == []
    assert g.filter([c], 1.6) == [c]
    assert g.filter([Cmd('mode', ('LAND',))], 1.7) == [Cmd('mode', ('LAND',))]


def test_takeoff_sent_once_when_climb_starts():
    m, fc = started()
    run(m, fc, 30, gate=CommandGate(1.5))
    assert len([c for c in fc.sent if c.kind == 'takeoff']) == 1


def test_takeoff_resent_if_still_on_ground():
    m, fc = started(Params(takeoff_retry_s=5.0, takeoff_timeout_s=30.0))
    real_apply = fc.apply
    fc.apply = lambda cmds: real_apply([c for c in cmds if c.kind != 'takeoff' or fc.t > 10.0])
    run(m, fc, 40, gate=CommandGate(1.5))
    assert len([c for c in fc.sent if c.kind == 'takeoff']) >= 1 and HOVER in [e[2] for e in m.events]


def test_mode_not_resent_once_fc_is_in_it():
    m, fc = started()
    run(m, fc, 60, gate=CommandGate(1.5))
    assert [c for c in fc.sent if c == Cmd('mode', ('LAND',))] == [Cmd('mode', ('LAND',))]


def test_restart_after_finished_mission_only_on_ground():
    m, fc = started()
    run(m, fc, 60)
    assert m.state == DONE
    ok, _ = m.start(fc.inputs())
    assert ok and m.state == PREFLIGHT and m.ground_z is None and m.t_takeoff is None
    m2, fc2 = started(Params(hover_s=30.0))
    run(m2, fc2, 40, hook=lambda m, fc: setattr(fc, 'mode', 'LOITER') if m.state == HOVER else None)
    assert m2.state == PILOT and fc2.armed
    ok, msg = m2.start(fc2.inputs())
    assert not ok and m2.state == PILOT             # still airborne under pilot control: refuse
