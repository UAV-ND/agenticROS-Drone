from uav_mission.rc_trigger_core import RcTrigger, TriggerParams

LO, HI, MIDV = 982, 2006, 1500


def drive(trig, seq, state='IDLE', t0=0.0, dt=0.1):
    """seq: list of (pwm, seconds). Returns actions with their times."""
    out, t = [], t0
    for pwm, secs in seq:
        for _ in range(int(round(secs / dt))):
            trig.on_rc(pwm, t)
            a = trig.step(t, state(t) if callable(state) else state)
            if a:
                out.append((round(t, 1), a))
            t += dt
    return out


def test_low_then_high_starts_once():
    acts = drive(RcTrigger(), [(LO, 1.0), (HI, 2.0)])
    assert [a for _, a in acts] == ['start']


def test_high_at_boot_does_nothing_until_cycled():
    tr = RcTrigger()
    assert drive(tr, [(HI, 3.0)]) == []
    assert [a for _, a in drive(tr, [(LO, 1.0), (HI, 1.0)], t0=3.0)] == ['start']


def test_debounce_ignores_short_blip():
    acts = drive(RcTrigger(), [(LO, 1.0), (HI, 0.3), (LO, 1.0)])
    assert acts == []


def test_each_start_needs_a_new_low():
    tr = RcTrigger()
    acts = drive(tr, [(LO, 1.0), (HI, 1.0), (HI, 3.0)])          # stays high: one start
    assert [a for _, a in acts] == ['start']
    acts = drive(tr, [(LO, 1.0), (HI, 1.0)], t0=5.0)             # refused start -> cycle -> again
    assert [a for _, a in acts] == ['start']


def test_low_in_flight_aborts_but_not_when_idle():
    tr = RcTrigger()
    acts = drive(tr, [(LO, 1.0), (HI, 1.0), (LO, 1.0)], state=lambda t: 'HOVER' if t > 1.6 else 'IDLE')
    assert [a for _, a in acts] == ['start', 'abort']
    tr2 = RcTrigger()
    acts = drive(tr2, [(LO, 1.0), (HI, 1.0), (LO, 1.0)], state='DONE')   # finished: low is just "ready"
    assert [a for _, a in acts] == ['start']


def test_abort_on_low_can_be_disabled():
    tr = RcTrigger(TriggerParams(abort_on_low=False))
    acts = drive(tr, [(LO, 1.0), (HI, 1.0), (LO, 1.0)], state=lambda t: 'SWEEP' if t > 1.6 else 'IDLE')
    assert [a for _, a in acts] == ['start']


def test_middle_position_and_no_start_while_flying():
    tr = RcTrigger()
    assert drive(tr, [(LO, 1.0), (MIDV, 2.0)]) == []
    tr2 = RcTrigger()
    acts = drive(tr2, [(LO, 1.0), (HI, 1.0)], state='SWEEP')      # mission already flying
    assert acts == [] and 'already' in tr2.status


def test_rc_loss_blocks_actions():
    tr = RcTrigger()
    drive(tr, [(LO, 1.0)])
    for i in range(30):                                          # 3 s with no RC messages
        assert tr.step(1.0 + i * 0.1, 'IDLE') is None
    assert tr.status == 'no RC data'
    acts = drive(tr, [(HI, 1.0)], t0=4.0)                        # RC back, HIGH: start (was LOW before)
    assert [a for _, a in acts] == ['start']


def test_invalid_pwm_ignored():
    tr = RcTrigger()
    acts = drive(tr, [(0, 2.0)])
    assert acts == [] and tr.status == 'no RC data'
