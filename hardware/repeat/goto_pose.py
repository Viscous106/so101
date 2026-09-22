"""Autonomous arm positioning for the benchmark (no hand-posing).

  goto_pose.py joint=target ...        move those joints (encoder counts), e.g. shoulder_lift=2200 elbow_flex=2830
  goto_pose.py --preset measure        the 2026-09-22 side-camera measurement pose
  goto_pose.py --fix-margins           move any benchmark joint that is too close to a limit back inside the margin
  goto_pose.py --check                 report limit margins and the marker tip position in the third-person camera
  options: --release (torque off at the end; default keeps every joint holding), --no-cam (skip the camera watchdog)

Safety: torque is held on all six joints throughout; moves are made in <= STEP counts with the I-term gains
(P32/I16) so joints land within a few counts; shoulder_lift is moved first when it raises the tip and last when it
lowers it; after every step the red marker tip is located in the third-person frame and the move aborts if the tip
drifts into the bottom of the frame (toward the tabletop) or leaves the frame.
"""
import os, sys, time
from bus import Bus, JOINTS

PRESETS = {"measure": {"shoulder_pan": 2082, "shoulder_lift": 2202, "elbow_flex": 2834, "wrist_flex": 1180, "wrist_roll": 1082}}
BENCH = ("shoulder_lift", "elbow_flex", "wrist_flex")
BENCH_MARGIN = 141          # collect_v2.py asserts lo+80 < P-60 and P+60 < hi-80
OTHER_MARGIN = 40
FIX_EXTRA = 40              # when fixing a margin violation, go this much further inside
STEP = 80                   # counts per move (~7 deg)
GAINS = {"P": 32, "I": 16, "D": 32}
TIP_BOTTOM_FRAC = 0.85      # abort if the tip is below this fraction of the frame height


def tip_in_camera():
    """(x, y_bottom, W, H) of the red marker tip in the third-person frame, None if not found / no camera."""
    try:
        import cv2
        from cam import Cam
        c = Cam(os.environ.get("THIRD", "/dev/video2"), exposure=10); c.auto_gain()
        bgr = c.stack(3).clip(0, 255).astype("uint8"); c.close()
        hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
        red = cv2.inRange(hsv, (0, 120, 100), (5, 255, 255)) | cv2.inRange(hsv, (170, 120, 100), (180, 255, 255))
        n, _, stats, _ = cv2.connectedComponentsWithStats(red)
        reds = [(stats[i, 4], i) for i in range(1, n) if stats[i, 4] > 150]
        if not reds:
            return None
        i = max(reds)[1]
        return int(stats[i, 0] + stats[i, 2] / 2), int(stats[i, 1] + stats[i, 3]), bgr.shape[1], bgr.shape[0]
    except Exception as e:
        print("camera check unavailable:", e)
        return None


args = sys.argv[1:]
use_cam = "--no-cam" not in args
release = "--release" in args
targets = {}
if "--preset" in args:
    targets.update(PRESETS[args[args.index("--preset") + 1]])
for a in args:
    if "=" in a and not a.startswith("--"):
        j, v = a.split("="); targets[j] = int(v)

b = Bus()
pos = {j: b.read(j, "pos") for j in JOINTS}
lim = {j: (b.read(j, "min"), b.read(j, "max")) for j in JOINTS}


def margins(p):
    return {j: (p[j] - (lim[j][0] + BENCH_MARGIN), (lim[j][1] - BENCH_MARGIN) - p[j]) for j in BENCH}


def report(p):
    for j in JOINTS:
        lo, hi = lim[j]
        m = margins(p).get(j)
        flag = "" if m is None else ("  OK" if min(m) >= 0 else f"  TOO CLOSE (low {m[0]:+d}, high {m[1]:+d})")
        print(f"  {j:14s} {p[j]:4d}  limits [{lo},{hi}]  torque={b.read(j, 'torque_en')}{flag}")


if "--fix-margins" in args:
    for j in BENCH:
        lo, hi = lim[j]
        if pos[j] < lo + BENCH_MARGIN:
            targets[j] = lo + BENCH_MARGIN + FIX_EXTRA
        elif pos[j] > hi - BENCH_MARGIN:
            targets[j] = hi - BENCH_MARGIN - FIX_EXTRA

print("current pose:"); report(pos)
if "--check" in args or not targets:
    if use_cam:
        t = tip_in_camera()
        print("marker tip in third-person frame:", "not found" if t is None else f"x={t[0]} y={t[1]} of {t[2]}x{t[3]} ({100 * t[1] / t[3]:.0f}% down)")
    b.close(); sys.exit(0)

for j, v in targets.items():
    lo, hi = lim[j]
    m = BENCH_MARGIN if j in BENCH else OTHER_MARGIN
    targets[j] = int(min(max(v, lo + m), hi - m))
    if targets[j] != v:
        print(f"  {j}: target {v} clamped to {targets[j]} (limits [{lo},{hi}] + margin)")

# shoulder_lift: decreasing raises the tip (2026-09-22: 2486 -> 2213 lifted it); do it first when raising, last when lowering
order = [j for j in JOINTS if j in targets and j != "shoulder_lift"]
if "shoulder_lift" in targets:
    order = (["shoulder_lift"] + order) if targets["shoulder_lift"] < pos["shoulder_lift"] else (order + ["shoulder_lift"])
print("moving", {j: f"{pos[j]}->{targets[j]}" for j in order})

saved = b.hold_all(torque_limit=500, gains={j: GAINS for j in order})
try:
    time.sleep(0.5)
    for j in order:
        while abs(b.read(j, "pos") - targets[j]) > 3:
            cur = b.read(j, "pos")
            nxt = cur + max(-STEP, min(STEP, targets[j] - cur))
            peak = b.move(j, nxt, tol=8, settle=0.3, trim=True)
            now = b.pos_avg(j, n=3)
            print(f"  {j:14s} -> {nxt:4d}  at {now:6.1f}  peak load {peak:3d}  temp {b.read(j, 'temp')}", flush=True)
            assert b.read(j, "temp") < 55, f"{j} hot; stopping"
            if use_cam and j in BENCH:
                t = tip_in_camera()
                if t is None:
                    raise RuntimeError("marker tip left the third-person frame; stopping")
                if t[1] > TIP_BOTTOM_FRAC * t[3]:
                    raise RuntimeError(f"marker tip at {100 * t[1] / t[3]:.0f}% of frame height - too close to the table; stopping")
            if abs(now - targets[j]) <= 3:
                break
    print("done.")
except Exception as e:
    print("ABORTED:", repr(e), flush=True)
finally:
    b.release_all(saved, keep_hold=not release)
    final = {j: b.read(j, "pos") for j in JOINTS}
    print("final pose (torque " + ("off" if release else "held") + "):"); report(final)
    b.close()
