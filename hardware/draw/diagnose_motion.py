"""Check the two reported faulty directions using tiny moves and signed loads."""
import json
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
        arm.lift(pose["shoulder_lift"])
        for joint in ("elbow_flex", "wrist_roll"):
            start = arm.position(joint)
            goal = start + 1.5
            cal = arm.bus.calibration[joint]
            raw = goal * 4095 / 360 + (cal.range_min + cal.range_max) / 2
            if not cal.range_min + 100 < raw < cal.range_max - 100:
                raise RuntimeError("Insufficient range for diagnostic")
            # This isolated +1.5 degree test is the only exception to the
            # historical direction guard; ordinary drawing keeps that guard.
            arm.write("Goal_Position", joint, goal)
            for i in range(12):
                time.sleep(0.4)
                arm.health()
                actual = arm.position(joint)
                signed = arm.signed_load(joint)
                current = arm.read("Present_Current", joint, raw=True)
                print(f"TEST {joint} start={start:.3f} goal={goal:.3f} actual={actual:.3f} "
                      f"delta={actual-start:.3f} signed_load={signed} current={current}", flush=True)
                arm.log("direction_test", joint=joint, start=start, goal=goal,
                        actual=actual, signed_load=signed, current=current)
                if abs(signed) > 300:
                    raise RuntimeError("Diagnostic load stop")
        arm.photograph("direction_test")
    finally:
        arm.close()


if __name__ == "__main__":
    main()
