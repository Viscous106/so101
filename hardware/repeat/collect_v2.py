"""SO-101 bidirectional repeatability benchmark - data collection for ONE joint.

  collect.py <joint> [delta_steps=45] [n_pairs=15]

The wrist camera is the independent sensor: if the arm really returns to the same pose the image is identical, and any
pose error shows up as image shift. Phases (every capture is logged with the encoder reading next to the image):
  static  5 captures without moving           -> noise floor of the camera measurement itself
  stair   monotonic staircase through P       -> pixels per encoder step (and proof the camera tracks real motion)
  repeat  P approached from P+delta / P-delta -> unidirectional scatter and bidirectional (reversal) error
All joints hold position under a 35% torque cap; the arm never leaves its rest pose by more than delta on one joint.
"""
import json, os, sys, time, pathlib
import cv2, numpy as np
from bus import Bus, JOINTS
from cam import Cam

joint = sys.argv[1]
delta = int(sys.argv[2]) if len(sys.argv) > 2 else 45
n_pairs = int(sys.argv[3]) if len(sys.argv) > 3 else 15
COMP = len(sys.argv) > 4 and sys.argv[4] == "comp"   # outer-loop correction: re-command the goal by the remaining encoder error
STAIR = (-40, -20, -10, -5, 0, 5, 10, 20, 40)
# Torque caps were never the limiter for the gripper (load plateaued at ~11% duty whatever the cap); the shortfall
# is P-loop steady-state error against the return spring. Kept at 700 for gripper so the cap can't become the
# limiter once P is raised (see P_GAIN); 350 elsewhere as in the original protocol.
TORQUE_LIMIT = 700 if joint == "gripper" else 350
# Position-loop gains for the joint under test (EEPROM, restored after the run). LeRobot leaves P=16 I=0; measured
# with gain_probe.py on shoulder_lift, 2026-09-22 (raw shortfall on a 60-count move against gravity):
#   P16 I0 -> -29 counts, P32 I0 -> -22, P32 I4 -> -10, P32 I16 -> -1 (all four moves within +-1.4 counts).
GAINS = {"P": 32, "I": 16, "D": 32}
run = pathlib.Path("runs") / f"{joint}{'_comp' if COMP else ''}_{time.strftime('%Y%m%d_%H%M%S')}"
run.mkdir(parents=True)
log = open(run / "captures.jsonl", "w")

# 0.5 s (5000) suited the dim lamp.py setup on 2026-09-21; in daylight it saturates 93% of the frame. 10 ms measured
# mean 113 / p99 193 / 0% saturated on 2026-09-22. Override with EXPOSURE=<0.1 ms units> in the environment.
EXPOSURE = int(os.environ.get("EXPOSURE", 100))
# second, fixed third-person camera (Logitech C270 on 2026-09-22): it watches the marker/jaws from a static viewpoint,
# so analyze3.py can track the tool instead of registering a scene that moves with the wrist. THIRD=0 disables it.
THIRD_DEV = os.environ.get("THIRD", "/dev/video2")
THIRD_EXPOSURE = int(os.environ.get("THIRD_EXPOSURE", 10))   # C270 against a window: 10 + auto gain = mean ~70, no clipping
b, c = Bus(), Cam("/dev/video0", exposure=EXPOSURE)
c3 = Cam(THIRD_DEV, exposure=THIRD_EXPOSURE) if THIRD_DEV != "0" else None
if c3 is not None:
    print("third-person camera gain/mean:", c3.auto_gain(), flush=True)
P = b.read(joint, "pos")
lo, hi = b.read(joint, "min"), b.read(joint, "max")
assert lo + 80 < P - 60 and P + 60 < hi - 80, f"{joint} at {P} is too close to its limits [{lo},{hi}]"
# image region that moves with the camera and must be excluded from registration by analyze2.py:
# the jaws plus the marker taped into them (MARKER=0 in the environment for the empty-gripper layout of 2026-09-21).
MASK_POLY = ([[350, 720], [470, 560], [500, 0], [1000, 0], [960, 560], [1020, 720]] if os.environ.get("MARKER", "1") == "1"
             else [[300, 720], [480, 440], [830, 440], [1010, 720]])
meta = {"compensated": COMP, "joint": joint, "P": P, "delta": delta, "n_pairs": n_pairs, "stair": STAIR, "started": time.strftime("%F %T"),
        "mask_poly": MASK_POLY,
        "camera": f"YUYV 1280x720 exposure {EXPOSURE / 10000:.4f}s gain 0, 4-frame mean",
        "third_camera": None if c3 is None else f"{THIRD_DEV} YUYV {c3.size[0]}x{c3.size[1]} exposure {THIRD_EXPOSURE / 10000:.4f}s gain {c3.gain} (auto, mean {c3.mean:.0f}), 4-frame mean, files t###.png",
        "torque_limit": TORQUE_LIMIT, "gains": GAINS, "accel": 8, "speed": 150,
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
    if c3 is not None:
        img3 = c3.stack(4)
        gray3 = cv2.cvtColor(img3.clip(0, 255), cv2.COLOR_BGR2GRAY)
        cv2.imwrite(str(run / f"t{idx:03d}.png"), (gray3 * 256).clip(0, 65535).astype(np.uint16))
        if idx == 0:
            cv2.imwrite(str(run / "reference_third.jpg"), img3.clip(0, 255).astype(np.uint8), [cv2.IMWRITE_JPEG_QUALITY, 90])
    rec = {"idx": idx, "phase": phase, "approach": approach, "goal": goal, "pos": round(b.pos_avg(joint), 2),
           "others": {j: b.read(j, "pos") for j in JOINTS if j != joint}, "load": b.read(joint, "load"), "peak_load": peak,
           "current": b.read(joint, "current"),
           "temp": b.read(joint, "temp"), "t": round(time.time(), 2)}
    log.write(json.dumps(rec) + "\n"); log.flush()
    print(f"{idx:03d} {phase:6s} approach {approach:+d} goal {goal} pos {rec['pos']:.1f} load {rec['load']:+d} peak {peak}", flush=True)
    idx += 1




def arrive(goal):
    """Move to goal; in COMP mode use bus.py's closed-loop trim to cancel steady-state encoder offset."""
    return b.move(joint, goal, trim=COMP)


saved = b.hold_all(torque_limit=TORQUE_LIMIT, gains={joint: GAINS})
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
    keep = os.environ.get("HOLD", "0") == "1"      # HOLD=1: keep torque on afterwards (batch runs, no sag between joints)
    b.release_all(saved, keep_hold=keep); b.close(); c.close(); log.close()
    if c3 is not None:
        c3.close()
    print("released: settings restored, torque " + ("kept ON (HOLD=1)" if keep else "off"), flush=True)
