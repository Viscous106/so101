"""SO-101 bidirectional repeatability benchmark - data collection for ONE joint.

  collect.py <joint> [delta_steps=45] [n_pairs=15]

The wrist camera is the independent sensor: if the arm really returns to the same pose the image is identical, and any
pose error shows up as image shift. Phases (every capture is logged with the encoder reading next to the image):
  static  5 captures without moving           -> noise floor of the camera measurement itself
  stair   monotonic staircase through P       -> pixels per encoder step (and proof the camera tracks real motion)
  repeat  P approached from P+delta / P-delta -> unidirectional scatter and bidirectional (reversal) error
All joints hold position under a 35% torque cap; the arm never leaves its rest pose by more than delta on one joint.
"""
import json, sys, time, pathlib
import cv2, numpy as np
from bus import Bus, JOINTS
from cam import Cam

joint = sys.argv[1]
delta = int(sys.argv[2]) if len(sys.argv) > 2 else 45
n_pairs = int(sys.argv[3]) if len(sys.argv) > 3 else 15
COMP = len(sys.argv) > 4 and sys.argv[4] == "comp"   # outer-loop correction: re-command the goal by the remaining encoder error
STAIR = (-40, -20, -10, -5, 0, 5, 10, 20, 40)
run = pathlib.Path("runs") / f"{joint}{'_comp' if COMP else ''}_{time.strftime('%Y%m%d_%H%M%S')}"
run.mkdir(parents=True)
log = open(run / "captures.jsonl", "w")

b, c = Bus(), Cam()
P = b.read(joint, "pos")
lo, hi = b.read(joint, "min"), b.read(joint, "max")
assert lo + 80 < P - 60 and P + 60 < hi - 80, f"{joint} at {P} is too close to its limits [{lo},{hi}]"
meta = {"compensated": COMP, "joint": joint, "P": P, "delta": delta, "n_pairs": n_pairs, "stair": STAIR, "started": time.strftime("%F %T"),
        "camera": "YUYV 1280x720 exposure 0.5s gain 0, 4-frame mean", "torque_limit": 350, "accel": 8, "speed": 150,
        "servo": {j: {k: b.read(j, k) for k in ("P", "I", "D", "cw_dead", "ccw_dead", "pos", "temp", "volt")} for j in JOINTS}}
(run / "meta.json").write_text(json.dumps(meta, indent=1))
idx = 0


def capture(phase, approach, goal, peak=0):
    global idx
    img = c.stack(4)
    gray = cv2.cvtColor(img.clip(0, 255), cv2.COLOR_BGR2GRAY)
    cv2.imwrite(str(run / f"{idx:03d}.png"), (gray * 256).clip(0, 65535).astype(np.uint16))
    if idx == 0:
        cv2.imwrite(str(run / "reference.jpg"), img.clip(0, 255).astype(np.uint8), [cv2.IMWRITE_JPEG_QUALITY, 90])
    rec = {"idx": idx, "phase": phase, "approach": approach, "goal": goal, "pos": round(b.pos_avg(joint), 2),
           "others": {j: b.read(j, "pos") for j in JOINTS if j != joint}, "load": b.read(joint, "load"), "peak_load": peak,
           "temp": b.read(joint, "temp"), "t": round(time.time(), 2)}
    log.write(json.dumps(rec) + "\n"); log.flush()
    print(f"{idx:03d} {phase:6s} approach {approach:+d} goal {goal} pos {rec['pos']:.1f} load {rec['load']:+d} peak {peak}", flush=True)
    idx += 1




def arrive(goal):
    """Move to goal; in COMP mode use bus.py's closed-loop trim to cancel steady-state encoder offset."""
    return b.move(joint, goal, trim=COMP)


saved = b.hold_all()
try:
    time.sleep(1.0)
    for _ in range(5):
        capture("static", 0, P)
    b.move(joint, P - 60, settle=0.3)
    for s in STAIR:                                   # always arriving from below
        peak = arrive(P + s); capture("stair", +1, P + s, peak)
    for k in range(n_pairs):
        b.move(joint, P + delta, settle=0.3); peak = arrive(P); capture("repeat", -1, P, peak)   # arrive from above
        b.move(joint, P - delta, settle=0.3); peak = arrive(P); capture("repeat", +1, P, peak)   # arrive from below
        assert b.read(joint, "temp") < 55, "servo getting hot; stopping"
    for _ in range(3):
        capture("static_end", 0, P)
    print("DONE", flush=True)
except Exception as e:
    print("ABORTED:", repr(e), flush=True)
    try:
        b.write(joint, "goal", P); time.sleep(1.0)
    except Exception:
        pass
finally:
    b.release_all(saved); b.close(); c.close(); log.close()
    print("released: torque off, settings restored", flush=True)
