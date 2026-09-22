"""Measure the servo's OWN settling error (no outer-loop help) for a few gain settings.  gain_probe.py <joint> [step=60]

For each config: single goal write to P-step, wait until the motor reports stopped, settle, read the shortfall
(goal - pos); same back to P; same for P+step and back. Reports shortfall, peak load and time-to-stop per move.
Gains are EEPROM writes made with torque off and restored afterwards. Small moves only; torque capped at 35%.
"""
import sys, time
from bus import Bus, JOINTS

joint = sys.argv[1]
step = int(sys.argv[2]) if len(sys.argv) > 2 else 60
CONFIGS = [
    ("P16 I0  acc8", {"P": 16, "I": 0, "D": 32}, 8),
    ("P32 I0  acc8", {"P": 32, "I": 0, "D": 32}, 8),
    ("P32 I4  acc8", {"P": 32, "I": 4, "D": 32}, 8),
    ("P32 I16 acc8", {"P": 32, "I": 16, "D": 32}, 8),
    ("P32 I16 acc32", {"P": 32, "I": 16, "D": 32}, 32),
]

b = Bus()
P = b.read(joint, "pos")
lo, hi = b.read(joint, "min"), b.read(joint, "max")
assert lo + 80 < P - step and P + step < hi - 80, f"{joint} at {P} too close to limits [{lo},{hi}]"
print(f"{joint} P={P}  step={step}  original gains P={b.read(joint,'P')} I={b.read(joint,'I')} D={b.read(joint,'D')}")


def raw_move(goal, timeout=6.0):
    b.write(joint, "goal", goal)
    t0, peak, stopped = time.time(), 0, 0
    while time.time() - t0 < timeout:
        time.sleep(0.05)
        peak = max(peak, abs(b.read(joint, "load")))
        if time.time() - t0 > 0.3 and not b.read(joint, "moving"):
            stopped += 1
            if stopped >= 2:
                break
        else:
            stopped = 0
    t_stop = time.time() - t0
    time.sleep(1.0)
    pos = b.pos_avg(joint)
    return goal - pos, peak, t_stop, b.read(joint, "load")


for name, gains, acc in CONFIGS:
    saved = b.hold_all(torque_limit=350, accel=acc, gains={joint: gains})
    try:
        time.sleep(0.5)
        rows = []
        for goal in (P - step, P, P + step, P):
            short, peak, t_stop, hold = raw_move(goal)
            rows.append(f"->{goal - P:+4d}: short {short:+6.1f} peak {peak:3d} hold {hold:+4d} t {t_stop:3.1f}s")
        print(f"{name:14s} | " + " | ".join(rows) + f" | temp {b.read(joint, 'temp')}", flush=True)
        assert b.read(joint, "temp") < 55, "hot; stopping"
    finally:
        b.release_all(saved)
        time.sleep(0.5)
print(f"restored gains P={b.read(joint,'P')} I={b.read(joint,'I')} D={b.read(joint,'D')}  torque={b.read(joint,'torque_en')}")
b.close()
