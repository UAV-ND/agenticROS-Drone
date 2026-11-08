import json

from uav_vio.fc_status_core import (FcStatus, aux_switch_position, source_set_from_rc,
                                    source_set_from_statustext)


def test_aux_switch_thresholds():
    assert aux_switch_position(0) is None
    assert aux_switch_position(982) == 0
    assert aux_switch_position(1199) == 0
    assert aux_switch_position(1200) == 1
    assert aux_switch_position(1500) == 1
    assert aux_switch_position(1800) == 1
    assert aux_switch_position(1801) == 2
    assert source_set_from_rc(1494) == 2
    assert source_set_from_rc(2006) == 3


def test_statustext_parsing():
    assert source_set_from_statustext('Using EKF Source Set 2') == 2
    assert source_set_from_statustext('EKF3 IMU0 is using external nav data') is None
    assert source_set_from_statustext('') is None


def healthy(now):
    s = FcStatus(connected=True, t_state=now)
    s.on_bridge_status(json.dumps({'state': 'TRACKING', 'published': 100}), now)
    return s


def test_all_clear_on_set1():
    s = healthy(10.0)
    s.rc_source_set = 1
    assert s.warnings(10.5) == []


def test_set2_with_bridge_hold_warns():
    s = healthy(10.0)
    s.on_statustext('Using EKF Source Set 2')
    assert s.warnings(10.5) == []
    s.on_bridge_status(json.dumps({'state': 'HOLD'}), 11.0)
    w = s.warnings(11.5)
    assert len(w) == 1 and 'source set 2' in w[0]


def test_set2_with_stale_bridge_warns():
    s = healthy(10.0)
    s.rc_source_set = 2
    s.t_state = 20.0
    assert any('bridge not tracking' in w for w in s.warnings(20.0))


def test_link_loss_warns():
    s = healthy(10.0)
    assert any('no MAVROS state' in w for w in s.warnings(20.0))
    s.t_state = 20.0
    s.connected = False
    assert any('not connected' in w for w in s.warnings(20.0))


def test_fc_text_overrides_rc_and_mismatch_warns():
    s = healthy(10.0)
    s.rc_source_set = 2
    s.on_statustext('Using EKF Source Set 1')
    assert s.effective_source_set() == 1
    assert any('RC switch says set 2' in w for w in s.warnings(10.0))


def test_ext_nav_flag_resets_on_switch():
    s = FcStatus()
    s.on_statustext('Using EKF Source Set 2')
    s.on_statustext('EKF3 IMU0 is using external nav data')
    assert s.ext_nav_in_use
    s.on_statustext('Using EKF Source Set 1')
    assert not s.ext_nav_in_use


def test_bad_bridge_json_ignored():
    s = FcStatus()
    s.on_bridge_status('not json', 1.0)
    assert s.bridge_state is None and s.t_bridge is None


def test_commanded_switch_updates_source_and_clears_warning():
    s = healthy(10.0)
    s.on_statustext('Using EKF Source Set 2')
    s.on_bridge_status(json.dumps({'state': 'HOLD'}), 10.0)
    assert any('source set 2' in w for w in s.warnings(10.5))
    s.on_source_command(1)                  # FC accepted MAV_CMD_SET_EKF_SOURCE_SET 1 (silently)
    assert s.effective_source_set() == 1
    assert not any('source set 2' in w for w in s.warnings(10.5))
