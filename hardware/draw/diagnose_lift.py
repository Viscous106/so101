"""One bounded lift command with position/current feedback and guaranteed cleanup."""
import json
import pathlib
import signal
import time

from draw_reference import Arm, DEFAULT_OUT


def main():
    def interrupt(signum, frame):
        raise KeyboardInterrupt(f"Signal {signum}")
    signal.signal(signal.SIGTERM, interrupt)
    arm = Arm(DEFAULT_OUT)
    try:
        pose = arm.setup()
        joint = "shoulder_lift"
        arm.write("Goal_Velocity", joint, 100, raw=True)
        arm.write("Torque_Limit", joint, 900, raw=True)
        start = pose[joint]
        arm.target(joint, start - 3)
        for i in range(32):
            time.sleep(0.25)
            arm.health()
            values = {r: arm.read(r, joint, raw=True) for r in (
                "Present_Position", "Goal_Position", "Present_Load", "Present_Current",
                "Moving", "Present_Temperature", "Torque_Enable", "Status")}
            values["degrees"] = arm.position(joint)
            arm.log("lift_diagnostic", **values)
            print(json.dumps(values), flush=True)
            if values["Present_Temperature"] >= 50:
                raise RuntimeError("Diagnostic temperature stop")
            if i >= 3 and arm.load(joint) > 500 and abs(values["degrees"] - start) < 0.2:
                raise RuntimeError("Sustained loaded stall")
    finally:
        arm.close()


if __name__ == "__main__":
    main()
