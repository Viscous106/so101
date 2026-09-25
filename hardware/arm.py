"""Guarded direct control of the SO-101 follower over the Feetech bus.

  arm.py read                      positions of all joints (no writes)
  arm.py calib                     write a LeRobot calibration JSON from the servos' EEPROM (no motion)
  arm.py nudge <joint> <deg>       move ONE joint by at most MAX_STEP_DEG, slowly, inside its EEPROM limits
  arm.py relax                     torque OFF on every joint (the arm will sag - support it)

Safety: one joint per call, step clamp, slow speed/accel, goal pre-set to the present position before torque-on.
"""
import json, sys, time, pathlib
import scservo_sdk as scs

PORT, BAUD = "/dev/ttyACM0", 1_000_000
JOINTS = {"shoulder_pan": 1, "shoulder_lift": 2, "elbow_flex": 3, "wrist_flex": 4, "wrist_roll": 5, "gripper": 6}
MAX_STEP_DEG, LIMIT_MARGIN, SPEED, ACCEL = 5.0, 40, 150, 8   # steps/s ~13 deg/s
A_MIN, A_MAX, A_OFFSET, A_TORQUE, A_ACCEL, A_GOAL, A_SPEED, A_POS, A_LOAD, A_MOVING = 9, 11, 31, 40, 41, 42, 46, 56, 60, 66

port, ph = scs.PortHandler(PORT), scs.PacketHandler(0)

def r(i, addr, n=2):
    v, res, _ = (ph.read1ByteTxRx if n == 1 else ph.read2ByteTxRx)(port, i, addr)
    if res != scs.COMM_SUCCESS: raise IOError(f"read id {i} addr {addr}: {ph.getTxRxResult(res)}")
    return v

def w(i, addr, v, n=2):
    res, err = (ph.write1ByteTxRx if n == 1 else ph.write2ByteTxRx)(port, i, addr, v)
    if res != scs.COMM_SUCCESS or err: raise IOError(f"write id {i} addr {addr}: {ph.getTxRxResult(res)} err={err}")

def signmag(v, bit=11):  # Feetech stores the homing offset as sign-magnitude
    return -(v & ((1 << bit) - 1)) if v & (1 << bit) else v

def main():
    cmd = sys.argv[1] if len(sys.argv) > 1 else "read"
    assert port.openPort() and port.setBaudRate(BAUD), f"cannot open {PORT}"
    try:
        if cmd == "read":
            for name, i in JOINTS.items():
                print(f"{name:14s} pos={r(i, A_POS):4d} limits=[{r(i, A_MIN)},{r(i, A_MAX)}] torque={r(i, A_TORQUE, 1)} load={r(i, A_LOAD)}")
        elif cmd == "calib":
            cal = {n: {"id": i, "drive_mode": 0, "homing_offset": signmag(r(i, A_OFFSET)), "range_min": r(i, A_MIN), "range_max": r(i, A_MAX)} for n, i in JOINTS.items()}
            out = pathlib.Path.home() / ".cache/huggingface/lerobot/calibration/robots/so101_follower/lab_follower.json"
            out.parent.mkdir(parents=True, exist_ok=True); out.write_text(json.dumps(cal, indent=4))
            print(json.dumps(cal, indent=2)); print("wrote", out)
        elif cmd == "nudge":
            name, deg = sys.argv[2], float(sys.argv[3]); i = JOINTS[name]
            deg = max(-MAX_STEP_DEG, min(MAX_STEP_DEG, deg))
            pos, lo, hi = r(i, A_POS), r(i, A_MIN) + LIMIT_MARGIN, r(i, A_MAX) - LIMIT_MARGIN
            goal = max(lo, min(hi, pos + round(deg * 4096 / 360)))
            print(f"{name}: {pos} -> {goal} (clamped to [{lo},{hi}], {deg:+.1f} deg requested)")
            w(i, A_GOAL, pos); w(i, A_ACCEL, ACCEL, 1); w(i, A_SPEED, SPEED); w(i, A_TORQUE, 1, 1); w(i, A_GOAL, goal)
            t0 = time.time()
            while time.time() - t0 < 4:
                time.sleep(0.15)
                if not r(i, A_MOVING, 1) and abs(r(i, A_POS) - goal) < 12: break
            print(f"{name}: now {r(i, A_POS)} load={r(i, A_LOAD)} (torque left ON for this joint so it holds)")
        elif cmd == "relax":
            for name, i in JOINTS.items(): w(i, A_TORQUE, 0, 1)
            print("torque off on all joints")
        else:
            print(__doc__)
    finally:
        port.closePort()

main()
