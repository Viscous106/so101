"""Draw strokes using only the operations proven reliable today, avoiding the one that has not been:
coordinated multi-joint IK "rise" (free orientation) reliably spikes elbow_flex's load even for a pure lift,
confirmed three independent ways (full IK, small-step IK, single-joint command) - see the conversation, not
re-derived here. So:
  - lateral moves (IK, Z held fixed): proven safe throughout dry runs and reachability testing
  - descent (Z-only, small steps, IK): proven safe - this is exactly how find_table_z.py found contact, twice
  - lift-off: a plain single-joint elbow_flex decrease (arm.py-style guarded write), NOT IK - proven to move
    freely and visibly clear the pen (confirmed via camera), where the IK-based equivalent kept failing

  draw_simple.py --dry-run
  draw_simple.py
"""
import json, pathlib, sys, time
import numpy as np
from lerobot.motors import Motor, MotorCalibration, MotorNormMode
from lerobot.motors.feetech import FeetechMotorsBus
from ik import make_kinematics, solve_ik, JOINT_ORDER
from draw_test_shape import strokes as TEST_STROKES

ARM_JOINTS = JOINT_ORDER[:-1]
CAL_PATH = pathlib.Path.home() / ".cache/huggingface/lerobot/calibration/robots/so_follower/lab_follower.json"
TABLE_Z_PATH = pathlib.Path(__file__).parent / "table_z.json"

SPEED, ACCEL = 60, 6
SETTLE_S, MAX_WAIT_S, POS_TOL = 0.04, 3.0, 0.7
LOAD_STOP = 500
DESCEND_STEP_M, DESCEND_LOAD_STOP = 0.0005, 200   # gentle - stop at first real contact, do not press hard
LATERAL_STEP_M = 0.003
LIFT_DEG, LIFT_TRIES = 1.5, 4   # per-nudge elbow_flex decrease, matches what visually cleared the pen earlier


def load_bus():
    cal = json.loads(CAL_PATH.read_text())
    motors = {n: Motor(cal[n]["id"], "sts3215", MotorNormMode.DEGREES) for n in ARM_JOINTS}
    calibration = {n: MotorCalibration(id=cal[n]["id"], drive_mode=cal[n]["drive_mode"], homing_offset=cal[n]["homing_offset"],
                                        range_min=cal[n]["range_min"], range_max=cal[n]["range_max"]) for n in ARM_JOINTS}
    return FeetechMotorsBus(port="/dev/ttyACM0", motors=motors, calibration=calibration)


def loads(bus):
    return {j: bus.read("Present_Load", j, normalize=False) & 0x3FF for j in ARM_JOINTS}


def goto(bus, deg, load_stop=LOAD_STOP):
    for j, d in zip(ARM_JOINTS, deg):
        bus.write("Goal_Position", j, float(d))
    t0 = time.time()
    while time.time() - t0 < MAX_WAIT_S:
        time.sleep(SETTLE_S)
        l = loads(bus)
        if max(l.values()) >= load_stop:
            return "stop", l
        cur = np.array([bus.read("Present_Position", j) for j in ARM_JOINTS])
        if np.max(np.abs(cur - deg)) < POS_TOL:
            return "reached", l
    return "timeout", l


def lateral_move(bus, kin, seed, x, y, step_m=LATERAL_STEP_M):
    """Z held fixed at whatever seed's FK currently gives - never asks IK to change height."""
    start = kin.forward_kinematics(seed)[:3, 3]
    z = start[2]
    dist = float(np.hypot(x - start[0], y - start[1]))
    n = max(1, int(np.ceil(dist / step_m)))
    for i in range(1, n + 1):
        p = start + (np.array([x, y, z]) - start) * (i / n)
        t = kin.forward_kinematics(seed).copy(); t[:3, 3] = p
        solved, ok, err = solve_ik(kin, seed, t, orientation_weight=0.0)
        if not ok:
            return "ik_fail", seed
        seed = solved
        status, l = goto(bus, solved[:5])
        if status == "stop":
            return "stop", seed
    return "reached", seed


def descend_to_contact(bus, kin, seed, step_m=DESCEND_STEP_M, max_drop_m=0.006):
    start_z = kin.forward_kinematics(seed)[:3, 3][2]
    z = start_z
    for _ in range(int(max_drop_m / step_m)):
        z -= step_m
        t = kin.forward_kinematics(seed).copy(); t[2, 3] = z
        solved, ok, err = solve_ik(kin, seed, t, orientation_weight=0.0)
        if not ok:
            return "ik_fail", seed
        status, l = goto(bus, solved[:5], load_stop=DESCEND_LOAD_STOP)
        if status == "stop":
            return "contact", solved
        seed = solved
    return "no_contact", seed


def lift_off(bus, seed_deg, lift_deg=4.0):
    """NOT general IK. Every attempt to lift via IK (which keeps choosing to increase elbow_flex) has instantly
    hit near-max load with zero movement, reproduced from multiple configurations and torque_limit settings -
    but elbow_flex moves freely by hand (no obstruction) and shoulder_lift alone lifts fine (see conversation).
    So: shoulder_lift decrease only, all other joints held fixed, with push-convergence (trim) for the same
    steady-state shortfall documented since the P16/I0 findings much earlier - not the elbow_flex mystery."""
    goal = bus.read("Present_Position", "shoulder_lift") - lift_deg
    cmd = goal
    for _ in range(LIFT_TRIES):
        bus.write("Goal_Position", "shoulder_lift", cmd)
        t0 = time.time()
        while time.time() - t0 < 2.0:
            time.sleep(0.05)
            l = bus.read("Present_Load", "shoulder_lift", normalize=False) & 0x3FF
            if l > 400:
                break
            p = bus.read("Present_Position", "shoulder_lift")
            if abs(p - cmd) < 0.5:
                break
        err = goal - bus.read("Present_Position", "shoulder_lift")
        if abs(err) < 0.3:
            break
        cmd = cmd + err


def run(strokes, dry_run):
    table_z = json.loads(TABLE_Z_PATH.read_text())["contact_z_m"]
    kin = make_kinematics()
    if dry_run:
        seed = np.array([17.0, 38.9, 75.8, -92.2, -85.2, 0.0])
        for si, pts in enumerate(strokes):
            for x, y in pts:
                t = kin.forward_kinematics(seed).copy(); t[0, 3], t[1, 3] = x, y
                solved, ok, err = solve_ik(kin, seed, t, orientation_weight=0.0)
                print(f"stroke {si} point ({x:.4f},{y:.4f}): ok={ok} err={err*1000:.2f}mm")
                if ok:
                    seed = solved
        return

    bus = load_bus(); bus.connect()
    try:
        for j in ARM_JOINTS:
            bus.write("Acceleration", j, ACCEL, normalize=False)
            bus.write("Goal_Velocity", j, SPEED, normalize=False)
            # torque_limit was left at 350 (35%) from earlier single-joint backlash testing (bus.py's hold_all
            # default) - not enough for elbow_flex/shoulder_lift to lift the forearm+wrist+gripper assembly
            # against gravity in an extended reach. Every load spike blamed on "kinematics" or "obstruction" in
            # this session was actually the servo maxing out an insufficient torque cap while trying (and
            # failing) to lift real weight. 600 (60%) still leaves real headroom below the 1000 max.
            bus.write("Torque_Limit", j, 600, normalize=False)
        cur = np.array([bus.read("Present_Position", j) for j in ARM_JOINTS])
        for j, d in zip(ARM_JOINTS, cur):
            bus.write("Goal_Position", j, float(d))
        for j in ARM_JOINTS:
            bus.write("Torque_Enable", j, 1, normalize=False)
        seed = np.array(list(cur) + [0.0])

        for si, pts in enumerate(strokes):
            x0, y0 = pts[0]
            status, seed = lateral_move(bus, kin, seed, x0, y0)
            print(f"stroke {si}: travel to start -> {status}")
            if status != "reached":
                continue
            status, seed = descend_to_contact(bus, kin, seed)
            print(f"stroke {si}: descend -> {status}  z={kin.forward_kinematics(seed)[2,3]:.4f}")
            if status != "contact":
                lift_off(bus, seed)
                continue
            for x, y in pts[1:]:
                status, seed = lateral_move(bus, kin, seed, x, y)
                print(f"  draw to ({x:.4f},{y:.4f}) -> {status}")
                if status == "stop":
                    break
            lift_off(bus, seed)
            print(f"stroke {si}: lifted off")
    finally:
        for j in ARM_JOINTS:
            bus.write("Torque_Enable", j, 0, normalize=False)
        bus.disconnect(disable_torque=False)
        print("torque off, disconnected")


if __name__ == "__main__":
    run(TEST_STROKES, dry_run="--dry-run" in sys.argv)
