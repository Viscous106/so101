"""Identify which physical joint each servo ID (1-6) drives, without assuming any prior label.

  label_motors.py            nudge every ID once, report
  label_motors.py <id>       nudge just one ID (a closer look)

For each ID: torque on for that ID ONLY (everything else stays passive), nudge by a small clamped step
(<=5 deg, slow speed - same guard as arm.py), grab a before/after frame from both cameras, compute how much
each camera's image changed, then command it straight back to its original position and torque off again
before moving to the next ID. Nothing is left displaced or holding torque when the script exits.

Prints a table: ID, the expected joint (from today's extensive testing + the FK check against the official
SO-101 URDF, so a real cross-check rather than a guess), raw position before/after, and how much each camera's
view changed - large wrist-camera movement means a shoulder/elbow joint (it carries the wrist camera with it),
large third-person movement concentrated near the jaws means gripper, near-frame-wide movement in both means
wrist_flex/wrist_roll. Use this to confirm or correct the mapping before trusting it for anything further.
"""
import sys, time
import cv2, numpy as np
import scservo_sdk as scs

PORT, BAUD = "/dev/ttyACM0", 1_000_000
IDS = [1, 2, 3, 4, 5, 6]
# established today via bus.py/arm.py (used in every collect_v2.py run), cross-checked against the official
# SO-101 URDF via hardware/fk_check.py (nudging "shoulder_lift" moved the FK-predicted height the expected way) -
# printed here for comparison, not assumed.
EXPECTED = {1: "shoulder_pan", 2: "shoulder_lift", 3: "elbow_flex", 4: "wrist_flex", 5: "wrist_roll", 6: "gripper"}
STEP_DEG, SPEED, ACCEL, LIMIT_MARGIN, TIMEOUT = 5.0, 100, 8, 40, 4.0
A_MIN, A_MAX, A_TORQUE, A_ACCEL, A_GOAL, A_SPEED, A_POS, A_MOVING = 9, 11, 40, 41, 42, 46, 56, 66

port, ph = scs.PortHandler(PORT), scs.PacketHandler(0)
assert port.openPort() and port.setBaudRate(BAUD), f"cannot open {PORT}"


def r(i, addr, n=2):
    v, res, _ = (ph.read1ByteTxRx if n == 1 else ph.read2ByteTxRx)(port, i, addr)
    assert res == scs.COMM_SUCCESS, f"read id {i} addr {addr} failed"
    return v


def w(i, addr, v, n=2):
    res, err = (ph.write1ByteTxRx if n == 1 else ph.write2ByteTxRx)(port, i, addr, int(v))
    assert res == scs.COMM_SUCCESS and not err, f"write id {i} addr {addr} failed"


def goto(i, goal, tries=3):
    """Move id i to goal, re-commanding the residual error like bus.py's trim - a plain move (as arm.py does)
    settles a few counts short under friction/backlash, which is why the first pass of this script did not revert
    exactly (drift up to 20 counts)."""
    lo, hi = r(i, A_MIN) + LIMIT_MARGIN, r(i, A_MAX) - LIMIT_MARGIN
    goal = max(lo, min(hi, goal))
    cmd = goal
    w(i, A_GOAL, r(i, A_POS)); w(i, A_ACCEL, ACCEL, 1); w(i, A_SPEED, SPEED); w(i, A_TORQUE, 1, 1)
    for _ in range(tries):
        w(i, A_GOAL, cmd)
        t0 = time.time()
        while time.time() - t0 < TIMEOUT:
            time.sleep(0.1)
            if not r(i, A_MOVING, 1) and abs(r(i, A_POS) - cmd) < 12:
                break
        err = goal - r(i, A_POS)
        if abs(err) < 3:
            break
        cmd = max(lo, min(hi, cmd + err))
    return r(i, A_POS)


def grab(dev):
    cap = cv2.VideoCapture(dev, cv2.CAP_V4L2)
    if not cap.isOpened():
        return None
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG")); cap.set(3, 1280); cap.set(4, 720)
    for _ in range(8):
        cap.read()
    frames = [cap.read() for _ in range(4)]
    cap.release()
    frames = [f.astype(np.float32) for ok, f in frames if ok]
    return np.mean(frames, 0) if frames else None


def diff(a, b):
    if a is None or b is None:
        return None
    return float(np.abs(a - b).mean())


targets = [int(sys.argv[1])] if len(sys.argv) > 1 else IDS
rows = []
for i in targets:
    before_pos = r(i, A_POS)
    lo, hi = r(i, A_MIN), r(i, A_MAX)
    step = STEP_DEG if before_pos + round(STEP_DEG * 4096 / 360) < hi - LIMIT_MARGIN else -STEP_DEG
    w0, t0img = grab("/dev/video0"), grab("/dev/video2")
    after_pos = goto(i, before_pos + round(step * 4096 / 360))
    time.sleep(0.3)
    w1, t1img = grab("/dev/video0"), grab("/dev/video2")
    goto(i, before_pos)                      # revert before moving on
    w(i, A_TORQUE, 0, 1)
    rows.append((i, EXPECTED.get(i, "?"), before_pos, after_pos, step, diff(w0, w1), diff(t0img, t1img)))
    print(f"id {i}  expected={EXPECTED.get(i, '?'):14s}  pos {before_pos}->{after_pos} ({step:+.0f} deg)  "
          f"wrist_cam_diff={rows[-1][5]}  third_cam_diff={rows[-1][6]}", flush=True)

port.closePort()
print("\nsummary (compare wrist/third diffs against expectation before trusting):")
print(f"{'id':3s} {'expected':14s} {'moved':>7s} {'wrist_diff':>11s} {'third_diff':>11s}")
for i, name, b, a, step, wd, td in rows:
    print(f"{i:<3d} {name:14s} {a - b:+7d} {('%.2f' % wd) if wd is not None else 'n/a':>11s} {('%.2f' % td) if td is not None else 'n/a':>11s}")
