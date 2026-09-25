"""Find the gripper_frame_link Z (position-only IK frame) at which the pen tip touches the table.

  find_table_z.py

Slow (2mm/step), load-monitored descent, planned with position-only IK (orientation free - see
hardware/draw/ik_check.py and the dry runs in this session: holding orientation rigidly fixed over-constrained
the solver near this arm's actual pose and made it saturate wrist_flex at its URDF limit after ~1.5cm; letting
orientation float converges cleanly through 20+cm and orientation drifts only ~1.5deg on wrist_roll per 12cm of
travel on its own - so a consistent-enough frame for the pen-offset math without forcing it).

This Z (in the frame FIXED by the orientation the solver naturally settles into) already includes the
unmeasured pen-tip length and the table height - both cancel into one contact number, valid for any pipeline
that reuses this same IK setup (make_kinematics(), orientation_weight=0.0) for its own planning.

Gripper is never touched (left alone, whatever state it's in).
"""
import json, pathlib, sys, time
import numpy as np
from lerobot.motors import Motor, MotorCalibration, MotorNormMode
from lerobot.motors.feetech import FeetechMotorsBus
sys.path.insert(0, str(pathlib.Path(__file__).parent))
from ik import make_kinematics, solve_ik, JOINT_ORDER

import os
ARM_JOINTS = JOINT_ORDER[:-1]  # exclude gripper entirely - not read, not written, not touched
STEP_M = float(os.environ.get("STEP_M", 0.002))
# sign-magnitude decoded (0-1023); today's normal dynamic loads topped ~185. First run (STEP_M=0.002, LOAD_STOP=320)
# hit 996 on the very first step - the pen was already almost touching the paper - so contact is sharp (paper on a
# hard table has far less give than the taped marker we dealt with earlier); a finer step and lower stop catch it
# more gently and precisely.
LOAD_WARN, LOAD_STOP = 100, int(os.environ.get("LOAD_STOP", 150))
SPEED, ACCEL = 60, 6              # slower than the day's single-joint default (100/8) - this moves 4 joints at once
SETTLE_S, MAX_WAIT_S = 0.05, 4.0
RETRACT_M = float(os.environ.get("RETRACT_M", 0.010))

CAL_PATH = pathlib.Path.home() / ".cache/huggingface/lerobot/calibration/robots/so_follower/lab_follower.json"
cal = json.loads(CAL_PATH.read_text())
motors = {n: Motor(cal[n]["id"], "sts3215", MotorNormMode.DEGREES) for n in ARM_JOINTS}
calibration = {n: MotorCalibration(id=cal[n]["id"], drive_mode=cal[n]["drive_mode"], homing_offset=cal[n]["homing_offset"],
                                    range_min=cal[n]["range_min"], range_max=cal[n]["range_max"]) for n in ARM_JOINTS}

bus = FeetechMotorsBus(port="/dev/ttyACM0", motors=motors, calibration=calibration)
bus.connect()
kin = make_kinematics()
contact_z = None


def loads():
    return {j: bus.read("Present_Load", j, normalize=False) & 0x3FF for j in ARM_JOINTS}  # magnitude only, sign bit 10 dropped


def goto(deg, tol=0.6):
    for j, d in zip(ARM_JOINTS, deg):
        bus.write("Goal_Position", j, float(d))
    t0 = time.time()
    while time.time() - t0 < MAX_WAIT_S:
        time.sleep(SETTLE_S)
        l = loads()
        m = max(l.values())
        if m >= LOAD_STOP:
            return "stop", l
        cur = np.array([bus.read("Present_Position", j) for j in ARM_JOINTS])
        if np.max(np.abs(cur - deg)) < tol:
            return "reached", l
    return "timeout", l


try:
    for j in ARM_JOINTS:
        bus.write("Acceleration", j, ACCEL, normalize=False)
        bus.write("Goal_Velocity", j, SPEED, normalize=False)
    cur = np.array([bus.read("Present_Position", j) for j in ARM_JOINTS])
    for j, d in zip(ARM_JOINTS, cur):
        bus.write("Goal_Position", j, float(d))       # goal = current before torque-on, no snap
    for j in ARM_JOINTS:
        bus.write("Torque_Enable", j, 1, normalize=False)

    deg6 = np.array(list(cur) + [0.0])                 # gripper slot unused by kin (position-only, 5 joints matter)
    T = kin.forward_kinematics(deg6)
    print(f"start xyz: {T[:3, 3].round(4)}  deg: {cur.round(1)}")
    seed, z = deg6.copy(), T[2, 3]
    contact_z = None
    while True:
        z -= STEP_M
        target = T.copy(); target[2, 3] = z
        solved, ok, err = solve_ik(kin, seed, target, orientation_weight=0.0)
        if not ok:
            print(f"IK stopped converging at z={z:.4f} (err {err * 1000:.1f}mm) - stopping, no table found in range")
            break
        status, l = goto(solved[:5])
        print(f"z={z:+.4f}  status={status:8s}  loads={l}", flush=True)
        if status == "stop":
            contact_z = z + STEP_M   # last position that was NOT in contact
            print(f"\nCONTACT just above z={contact_z:.4f}")
            # retract in the SAME small steps as the descent, not one big jump: a single solve_ik+goto covering
            # RETRACT_M in one shot (20x STEP_M) does not guarantee a monotonic Cartesian path - each joint reaches
            # its own target on its own schedule, so the tool can dip back through contact transiently even though
            # both endpoints are clear. Confirmed for real: this exact single-jump retract left the arm only
            # ~0.7mm clear instead of the intended 10mm, and a previous run left it close enough to still be
            # touching. See execute.py's move_to() docstring for the same finding on the drawing side.
            rz = z
            for _ in range(int(round(RETRACT_M / STEP_M))):
                rz += STEP_M
                back = target.copy(); back[2, 3] = rz
                solved_back, ok_back, _ = solve_ik(kin, solved, back, orientation_weight=0.0)
                if not ok_back:
                    print(f"retract IK failed at z={rz:.4f} - stopping retract early")
                    break
                goto(solved_back[:5])
                solved = solved_back
            print(f"retracted to z={rz:.4f}")
            break
        seed = solved
finally:
    for j in ARM_JOINTS:
        bus.write("Torque_Enable", j, 0, normalize=False)
    bus.disconnect(disable_torque=False)
    print("torque off, disconnected")

if contact_z is not None:
    out = pathlib.Path(__file__).parent / "table_z.json"
    out.write_text(json.dumps({"contact_z_m": contact_z, "note": "gripper_frame_link Z, position-only IK "
                                "(orientation_weight=0.0), same make_kinematics() setup as ik.py - includes "
                                "unmeasured pen-tip length and table height combined into this one number"}, indent=1))
    print(f"saved to {out}")
