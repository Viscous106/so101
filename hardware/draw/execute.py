"""Execute a list of real-world strokes on the real arm via IK, position-only frame (see ik.py, table_z.json).

Not run directly - draw_test_shape.py and draw_image.py both import run_strokes() from here.

Safety model: slow (SPEED/ACCEL well below the day's single-joint defaults - this drives 4 joints continuously
for potentially thousands of waypoints), load-monitored throughout (not just at first contact - LOAD_STOP here
is set for *sustained drawing contact*, which find_table_z.py's calibration showed jumps from ~0 to ~1000 within
0.5mm of the table, so some load during pen-down is normal and expected, not a fault condition). Aborts and
retracts if any joint's load exceeds LOAD_STOP; goal is always written as current position before torque-on, so
there is never a snap. Gripper is never touched.
"""
import json, pathlib, time
import numpy as np
from lerobot.motors import Motor, MotorCalibration, MotorNormMode
from lerobot.motors.feetech import FeetechMotorsBus
from ik import make_kinematics, solve_ik, JOINT_ORDER

ARM_JOINTS = JOINT_ORDER[:-1]
CAL_PATH = pathlib.Path.home() / ".cache/huggingface/lerobot/calibration/robots/so_follower/lab_follower.json"
TABLE_Z_PATH = pathlib.Path(__file__).parent / "table_z.json"

SPEED, ACCEL = 60, 6
SETTLE_S, MAX_WAIT_S, POS_TOL = 0.04, 3.0, 0.7
LOAD_STOP = 500          # sustained drawing contact is expected to be nonzero; this catches a genuine jam/skid,
                          # not intended pen-down force (calibration saw ~1000 just 0.5mm past first contact)
PEN_LIFT_M = 0.006


def load_bus():
    cal = json.loads(CAL_PATH.read_text())
    motors = {n: Motor(cal[n]["id"], "sts3215", MotorNormMode.DEGREES) for n in ARM_JOINTS}
    calibration = {n: MotorCalibration(id=cal[n]["id"], drive_mode=cal[n]["drive_mode"], homing_offset=cal[n]["homing_offset"],
                                        range_min=cal[n]["range_min"], range_max=cal[n]["range_max"]) for n in ARM_JOINTS}
    return FeetechMotorsBus(port="/dev/ttyACM0", motors=motors, calibration=calibration)


def loads(bus):
    return {j: bus.read("Present_Load", j, normalize=False) & 0x3FF for j in ARM_JOINTS}


def goto(bus, deg):
    for j, d in zip(ARM_JOINTS, deg):
        bus.write("Goal_Position", j, float(d))
    t0 = time.time()
    while time.time() - t0 < MAX_WAIT_S:
        time.sleep(SETTLE_S)
        l = loads(bus)
        if max(l.values()) >= LOAD_STOP:
            return "stop", l
        cur = np.array([bus.read("Present_Position", j) for j in ARM_JOINTS])
        if np.max(np.abs(cur - deg)) < POS_TOL:
            return "reached", l
    return "timeout", l


def move_to(bus, kin, seed, target_xyz, step_m):
    """Move the tool to target_xyz via small interpolated Cartesian steps, each solved and commanded
    individually - NOT a single direct joint-space jump.

    goto() commands several joints to independent final positions at once; each joint reaches its own target on
    its own schedule, so the *actual* Cartesian path between two configurations is uncontrolled even when both
    endpoints are safe - it can dip through the table transiently in between. This bit us for real: retracting
    after find_table_z.py's fine pass left the arm ~1.1mm above contact instead of the intended 6mm (the retract
    step itself was a single direct jump), and the very next single-jump move in draw_test_shape.py immediately
    maxed elbow_flex's load - almost certainly a transient dip into the table between the two (otherwise safe)
    endpoints. Every move in this pipeline must go through here.

    Returns (status, seed, loads) where status is "reached", "stop" (load spike - caller should retract) or
    "ik_fail".
    """
    start = kin.forward_kinematics(seed)[:3, 3]
    dist = float(np.linalg.norm(np.array(target_xyz) - start))
    n = max(1, int(np.ceil(dist / step_m)))
    l = {}
    for i in range(1, n + 1):
        p = start + (np.array(target_xyz) - start) * (i / n)
        t = kin.forward_kinematics(seed).copy(); t[:3, 3] = p
        solved, ok, err = solve_ik(kin, seed, t, orientation_weight=0.0)
        if not ok:
            return "ik_fail", seed, {}
        seed = solved
        if bus is None:
            continue
        status, l = goto(bus, solved[:5])
        if status == "stop":
            return "stop", seed, l
    return "reached", seed, l


def run_strokes(strokes_xy, step_m=0.003, dry_run=False):
    """strokes_xy: list of point-lists, each [[x,y],...] in metres, robot base frame, already mapped to the
    drawing plane. dry_run=True plans the whole trajectory (IK only, no motion) and reports reachability."""
    table_z = json.loads(TABLE_Z_PATH.read_text())["contact_z_m"]
    pen_down, pen_up = table_z, table_z + PEN_LIFT_M
    travel_z = pen_up + PEN_LIFT_M   # extra margin above pen_up for lateral travel between strokes
    kin = make_kinematics()

    bus = None if dry_run else load_bus()
    try:
        if not dry_run:
            bus.connect()
            for j in ARM_JOINTS:
                bus.write("Acceleration", j, ACCEL, normalize=False)
                bus.write("Goal_Velocity", j, SPEED, normalize=False)
            cur = np.array([bus.read("Present_Position", j) for j in ARM_JOINTS])
            for j, d in zip(ARM_JOINTS, cur):
                bus.write("Goal_Position", j, float(d))
            for j in ARM_JOINTS:
                bus.write("Torque_Enable", j, 1, normalize=False)
            seed = np.array(list(cur) + [0.0])
        else:
            seed = np.array([0.0, 45.0, 60.0, -75.0, -85.0, 0.0])  # a mid-range guess; only used to report reachability

        cur_xyz = kin.forward_kinematics(seed)[:3, 3].copy()

        def move(target_xyz, label):
            nonlocal seed, cur_xyz
            status, seed, l = move_to(bus, kin, seed, target_xyz, step_m)
            if status == "reached":
                cur_xyz = np.array(target_xyz)
            return status, l

        # clear the table before any lateral travel, regardless of where we actually start
        status, l = move([cur_xyz[0], cur_xyz[1], travel_z], "initial lift")
        if status != "reached":
            print(f"could not even clear the table at the start ({status}, loads={l}) - stopping, not attempting to draw")
            return {"aborted": True, "stage": "initial-lift", "loads": l}

        n_ik_fail = 0
        for si, pts in enumerate(strokes_xy):
            x0, y0 = pts[0]
            status, l = move([x0, y0, travel_z], "travel")
            if status == "ik_fail":
                n_ik_fail += 1; continue
            status, l = move([x0, y0, pen_down], "descend")
            if status == "stop":
                print(f"LOAD STOP descending for stroke {si}: loads={l} - retracting and aborting")
                move([x0, y0, travel_z], "abort-retract")
                return {"aborted": True, "stroke": si, "point": 0, "loads": l}
            if status == "ik_fail":
                n_ik_fail += 1; continue
            for pi, (x, y) in enumerate(pts[1:], start=1):
                status, l = move([x, y, pen_down], "draw")
                if status == "ik_fail":
                    n_ik_fail += 1; continue
                if status == "stop":
                    print(f"LOAD STOP at stroke {si} point {pi}: loads={l} - retracting and aborting")
                    move([x, y, travel_z], "abort-retract")
                    return {"aborted": True, "stroke": si, "point": pi, "loads": l}
            move([pts[-1][0], pts[-1][1], travel_z], "lift")
        print(f"done: {len(strokes_xy)} strokes, {n_ik_fail} IK failures")
        return {"aborted": False, "ik_failures": n_ik_fail}
    finally:
        if bus is not None:
            for j in ARM_JOINTS:
                bus.write("Torque_Enable", j, 0, normalize=False)
            bus.disconnect(disable_torque=False)
            print("torque off, disconnected")
