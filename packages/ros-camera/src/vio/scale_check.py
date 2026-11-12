#!/usr/bin/env python3
"""
scale_check.py <traj.txt> <tape_m> [--still-speed 0.03] [--min-stop 1.0]

Scale test for a cuVSLAM trajectory recorded as: still on X -> walk to B -> stop -> back to X -> ...
<traj.txt> is the "t x y z" file summarize_odom.py writes next to the odometry bag
(~/vio_runs/<run>/odom_traj.txt). <tape_m> is the tape-measured X->B distance.

Finds the stops (windowed speed below --still-speed for at least --min-stop seconds), takes the
first stop as X, labels each stop X or B by distance, and compares the measured B distance with
the tape. Pass bars: every B stop within 5 % of the tape, and the final stop within 5 cm of X.
"""
import argparse
import sys

import numpy as np

SCALE_TOL = 0.05   # B distance within 5 % of the tape
RETURN_TOL = 0.05  # final stop within 5 cm of X


def stops(t, P, still_speed, min_stop, win=0.5):
    # Speed over a +-win/2 window, robust to per-frame jitter.
    lo = np.searchsorted(t, t - win / 2)
    hi = np.clip(np.searchsorted(t, t + win / 2), 0, len(t) - 1)
    dt = np.maximum(t[hi] - t[lo], 1e-6)
    v = np.linalg.norm(P[hi] - P[lo], axis=1) / dt
    still = v < still_speed
    out, i = [], 0
    while i < len(t):
        if still[i]:
            j = i
            while j + 1 < len(t) and still[j + 1]:
                j += 1
            if t[j] - t[i] >= min_stop:
                out.append((t[i], t[j], P[i:j + 1].mean(axis=0)))
            i = j + 1
        else:
            i += 1
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("traj")
    ap.add_argument("tape_m", type=float)
    ap.add_argument("--still-speed", type=float, default=0.03, help="m/s (default 0.03)")
    ap.add_argument("--min-stop", type=float, default=1.0, help="s (default 1.0)")
    a = ap.parse_args()

    D = np.loadtxt(a.traj)
    t, P = D[:, 0] - D[0, 0], D[:, 1:4]
    S = stops(t, P, a.still_speed, a.min_stop)
    if len(S) < 2:
        sys.exit(f"Found {len(S)} stop(s); need X and at least one B. Try --still-speed / --min-stop.")

    X = S[0][2]
    print(f"Tape X->B: {a.tape_m:.3f} m   path length {np.linalg.norm(np.diff(P, axis=0), axis=1).sum():.2f} m")
    print(f"{'stop':>4} {'from':>7} {'to':>7} {'label':>5} {'dist 3D':>8} {'dist xy':>8} {'err':>7}")
    b_d, ok = [], True
    for k, (t0, t1, p) in enumerate(S):
        d3 = np.linalg.norm(p - X)
        dxy = np.linalg.norm((p - X)[:2])
        label = "B" if d3 > a.tape_m / 2 else "X"
        err = ""
        if label == "B":
            e = d3 / a.tape_m - 1
            b_d.append(d3)
            err = f"{e * 100:+.1f}%"
            ok &= abs(e) <= SCALE_TOL
        print(f"{k:>4} {t0:>6.1f}s {t1:>6.1f}s {label:>5} {d3:>7.3f}m {dxy:>7.3f}m {err:>7}")

    if not b_d:
        print(f"\nNo stop farther than {a.tape_m / 2:.2f} m from X: cuVSLAM never saw the camera reach B.")
        print("FAIL (scale is badly off, or B was never held still long enough)")
        return 1

    ret = np.linalg.norm(S[-1][2] - X)
    ok_ret = ret <= RETURN_TOL
    b = float(np.median(b_d))
    print(f"\nB stops: {len(b_d)}   median {b:.3f} m vs tape {a.tape_m:.3f} m  ->  scale {b / a.tape_m:.3f} "
          f"({(b / a.tape_m - 1) * 100:+.1f}%)  [{'PASS' if ok else 'FAIL'}: each within {SCALE_TOL * 100:.0f}%]")
    print(f"Final stop to X: {ret * 100:.1f} cm  [{'PASS' if ok_ret else 'FAIL'}: within {RETURN_TOL * 100:.0f} cm]")
    print("OVERALL:", "PASS" if ok and ok_ret else "FAIL")
    return 0 if ok and ok_ret else 1


if __name__ == "__main__":
    sys.exit(main())
