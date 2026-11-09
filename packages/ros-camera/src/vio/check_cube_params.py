#!/usr/bin/env python3
"""Compare the Cube's live parameters with the known-good file (READ-ONLY: never writes parameters).

  ~/mav_venv/bin/python check_cube_params.py                       # live Cube over USB vs known-good
  ~/mav_venv/bin/python check_cube_params.py --live-file dump.parm # an existing dump instead

Nothing else may hold the Cube's USB port (stop MAVROS / MAVProxy first). Sends only
PARAM_REQUEST_LIST, so (unlike MAVProxy) it does not change stream rates.
Exit code 0 = all CRITICAL parameters match, 1 = mismatch (fix before flying), 2 = no data.
"""
import argparse
import os
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
KNOWN = os.path.join(HERE, 'cube_known_good_2026-11-09.parm')
PORT = '/dev/serial/by-id/usb-Hex_ProfiCNC_CubeOrange_440045001451303437363830-if00'
# Parameters that break GPS-denied flight if wrong (the five that silently reverted on 2026-11-09 first).
CRITICAL = ['VISO_TYPE', 'EK3_SRC1_VELXY', 'EK3_SRC2_POSXY', 'EK3_SRC2_VELXY', 'EK3_SRC2_YAW',
            'VISO_ORIENT', 'VISO_POS_X', 'VISO_POS_Y', 'VISO_POS_Z', 'VISO_DELAY_MS',
            'EK3_SRC1_POSXY', 'EK3_SRC1_POSZ', 'EK3_SRC1_VELZ', 'EK3_SRC1_YAW',
            'EK3_SRC2_POSZ', 'EK3_SRC2_VELZ', 'EK3_SRC_OPTIONS', 'EK3_RNG_USE_HGT', 'RNGFND1_TYPE',
            'RNGFND1_MIN_CM', 'FLOW_TYPE', 'GPS1_TYPE', 'AHRS_EKF_TYPE', 'RC10_OPTION', 'RTL_ALT']
STREAMS = ['SR0_POSITION', 'SR0_EXTRA1', 'SR0_EXT_STAT', 'SR0_RC_CHAN', 'SR0_EXTRA3', 'SR0_EXTRA2']


def load(path):
    out = {}
    for line in open(path):
        line = line.split('#')[0].strip()
        if line:
            k, v = line.replace('\t', ',').replace(' ', ',').split(',')[:2]
            out[k] = float(v)
    return out


def read_live(port):
    from pymavlink import mavutil
    m = mavutil.mavlink_connection(port, baud=115200, source_system=254, autoreconnect=False)
    if not m.wait_heartbeat(timeout=15):
        return {}
    m.mav.param_request_list_send(m.target_system, m.target_component)
    params, total, t_last = {}, None, time.time()
    while time.time() - t_last < 5:
        msg = m.recv_match(type='PARAM_VALUE', blocking=True, timeout=1)
        if msg is None:
            continue
        t_last, total = time.time(), msg.param_count
        params[msg.param_id] = msg.param_value
        if total and len(params) >= total:
            break
    return params


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--port', default=PORT)
    ap.add_argument('--known', default=KNOWN)
    ap.add_argument('--live-file', help='compare this dump instead of reading the Cube')
    a = ap.parse_args()
    known = load(a.known)
    live = load(a.live_file) if a.live_file else read_live(a.port)
    if not live:
        print('No parameters received (Cube connected? port free?)')
        return 2
    print(f'{len(live)} live parameters vs {len(known)} known-good ({os.path.basename(a.known)})')
    bad = [k for k in CRITICAL if k not in live or abs(live[k] - known[k]) > 1e-6]
    for k in CRITICAL:
        flag = 'OK ' if k not in bad else 'BAD'
        print(f'  {flag} {k:16s} live {live.get(k, "missing")!s:>8}  known-good {known[k]:g}')
    for k in STREAMS:
        if k in live and abs(live[k] - known[k]) > 1e-6:
            print(f'  note {k} live {live[k]:g} vs saved {known[k]:g} (MAVProxy sets 4 Hz in RAM; power-cycle to restore)')
    other = sorted(k for k in known if k in live and k not in CRITICAL + STREAMS and not k.startswith('STAT_')
                   and abs(live[k] - known[k]) > 1e-4)   # STAT_*: runtime/boot counters, always change
    if other:
        print(f'  {len(other)} other parameters differ: ' + ', '.join(f'{k} {known[k]:g}->{live[k]:g}' for k in other[:20]))
    print('RESULT: ' + ('ALL CRITICAL PARAMETERS MATCH' if not bad else f'{len(bad)} CRITICAL MISMATCH(ES): {", ".join(bad)} -> fix before flying'))
    return 1 if bad else 0


if __name__ == '__main__':
    sys.exit(main())
