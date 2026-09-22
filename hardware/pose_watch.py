"""Live read-only joint readout for positioning the arm by hand (torque off). Ctrl-C to stop.

  pose_watch.py            print current positions every 0.5 s
  pose_watch.py target     also show the delta to the 2026-09-21 measurement pose

Writes nothing to the servos.
"""
import sys, time
import scservo_sdk as scs

PORT, BAUD = "/dev/ttyACM0", 1_000_000
JOINTS = {"shoulder_pan": 1, "shoulder_lift": 2, "elbow_flex": 3, "wrist_flex": 4, "wrist_roll": 5, "gripper": 6}
A_POS, A_TORQUE = 56, 40
# pose recorded in runs/shoulder_pan_20260921_005201/meta.json (camera looking down at the tabletop)
TARGET = {"shoulder_pan": 1915, "shoulder_lift": 929, "elbow_flex": 2334, "wrist_flex": 3199, "wrist_roll": 1099, "gripper": 1721}

port, ph = scs.PortHandler(PORT), scs.PacketHandler(0)
assert port.openPort() and port.setBaudRate(BAUD), f"cannot open {PORT}"
show_target = len(sys.argv) > 1 and sys.argv[1] == "target"
try:
    while True:
        cols = []
        for name, i in JOINTS.items():
            pos, res, _ = ph.read2ByteTxRx(port, i, A_POS)
            tq, res2, _ = ph.read1ByteTxRx(port, i, A_TORQUE)
            if res != scs.COMM_SUCCESS or res2 != scs.COMM_SUCCESS:
                cols.append(f"{name}=??")
                continue
            s = f"{name}={pos:4d}{'*' if tq else ' '}"
            if show_target:
                s += f"({pos - TARGET[name]:+5d})"
            cols.append(s)
        print("  ".join(cols), flush=True)
        time.sleep(0.5)
except KeyboardInterrupt:
    pass
finally:
    port.closePort()
