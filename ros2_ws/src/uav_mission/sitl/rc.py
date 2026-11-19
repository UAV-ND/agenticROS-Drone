"""Fake transmitter for SITL: send 16 PWM channels to UDP 5501. Usage: rc.py ch=pwm ... (others kept)."""
import json, os, socket, struct, sys
STATE = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'rc_state.json')   # gitignored
ch = {i: 1500 for i in range(1, 17)}
ch.update({3: 1000, 5: 1000, 6: 1000, 9: 1425, 10: 1000})   # throttle low, arm LOW, ch6 LOW, mode LOITER, ARK/set1
try:
    ch.update({int(k): v for k, v in json.load(open(STATE)).items()})
except Exception:
    pass
for a in sys.argv[1:]:
    k, v = a.split('='); ch[int(k)] = int(v)
json.dump(ch, open(STATE, 'w'))
pkt = struct.pack('<16H', *[ch[i] for i in range(1, 17)])
s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
for _ in range(3):
    s.sendto(pkt, ('127.0.0.1', 5501))
print('RC', {k: ch[k] for k in (3, 5, 6, 9, 10)})
