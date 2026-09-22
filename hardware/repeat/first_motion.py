"""First motion: hold everything where it is (35% torque cap), open the gripper 3 degrees, come back, release."""
import time, cv2, numpy as np
from bus import Bus, JOINTS
from cam import Cam

b, c = Bus(), Cam()
before = {j: b.read(j, "pos") for j in JOINTS}
saved = b.hold_all()
try:
    time.sleep(1.0)
    print("--- holding (torque on, limit 35%) ---")
    for j in JOINTS:
        p, load, cur = b.read(j, "pos"), b.read(j, "load"), b.read(j, "current")
        print(f"{j:14s} pos {before[j]} -> {p} (jump {p - before[j]:+d})  load {load:+d}  current {cur}")
        assert abs(p - before[j]) < 15, "joint jumped on torque-on; aborting"
    a = c.stack(3)
    g0 = b.read("gripper", "pos")
    peak = b.move("gripper", g0 + 34); g1 = b.pos_avg("gripper"); bimg = c.stack(3)
    print(f"gripper {g0} -> {g1:.1f} (asked {g0 + 34}), peak load {peak}")
    peak = b.move("gripper", g0); g2 = b.pos_avg("gripper")
    print(f"gripper back -> {g2:.1f} (asked {g0}), peak load {peak}")
    d = np.abs(bimg - a).mean(2); ys, xs = np.where(d > 25)
    if len(xs):
        print(f"image change: {(d > 25).mean() * 100:.2f}% of pixels; bbox x {xs.min()}-{xs.max()} y {ys.min()}-{ys.max()}")
    else:
        print("no image change")
    cv2.imwrite("nudge_side_by_side.jpg", np.hstack([a, bimg]).clip(0, 255).astype(np.uint8), [cv2.IMWRITE_JPEG_QUALITY, 85])
finally:
    b.release_all(saved); b.close(); c.close()
    print("released: torque off, original settings restored")
