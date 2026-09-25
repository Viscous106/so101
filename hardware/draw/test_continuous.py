"""Bounded two-degree pan stroke to test low-force continuous ink delivery."""
import fcntl
import json
import signal
import subprocess
import time

from draw_reference import Arm, JOINTS, ROOT, save_json


OUT = ROOT / "hardware/draw/runs/continuous_test"


def touch(arm, anchor, threshold=-28, clearance=3, step=0.15):
    if not -80 <= threshold <= -28:
        raise ValueError("Contact threshold outside tested range")
    arm.clear = False
    if not 1.5 <= clearance <= 3:
        raise ValueError("Contact search clearance outside tested range")
    if not 0.1 <= step <= 0.25:
        raise ValueError("Contact step exceeds bounded approach range")
    arm.move("shoulder_lift", anchor - clearance, tolerance=0.3)
    time.sleep(0.5)
    command = arm.position("shoulder_lift")
    while command < anchor + 3:
        command += step
        arm.target("shoulder_lift", command)
        time.sleep(0.25)
        arm.health()
        load = arm.signed_load("shoulder_lift")
        arm.log("light_contact", command=command, actual=arm.position("shoulder_lift"), load=load)
        if abs(load) >= 250:
            raise RuntimeError("Light contact load stop")
        if load <= threshold:
            time.sleep(0.12)
            if arm.signed_load("shoulder_lift") <= threshold:
                return arm.position("shoulder_lift")
    raise RuntimeError("No light contact within search envelope")


def main():
    signal.signal(signal.SIGTERM, lambda signum, frame: (_ for _ in ()).throw(KeyboardInterrupt()))
    if subprocess.run(["fuser", "/dev/ttyACM0"], capture_output=True).returncode == 0:
        raise RuntimeError("Serial port already owned")
    OUT.mkdir(parents=True, exist_ok=True)
    with open("/tmp/so101-drawing.lock", "w") as lock:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        arm = Arm(OUT)
        hit = None
        try:
            pose = arm.setup()
            arm.lift(pose["shoulder_lift"], clearance=12)
            arm.target("wrist_flex", -98.98901098901099)
            arm.target("wrist_roll", -71.25274725274726)
            time.sleep(1)
            arm.move("elbow_flex", 53, max_step=6)
            arm.move("elbow_flex", 55, max_step=6)
            arm.move("shoulder_pan", 26, max_step=8)
            time.sleep(1)
            arm.photograph("before")
            hit = touch(arm, 58)
            arm.photograph("contact")
            samples = []
            start = arm.position("shoulder_pan")
            for i in range(1, 9):
                # Explicitly scoped contact-motion experiment, not a bypass for
                # ordinary lateral travel. Total sweep is limited to 2 degrees.
                goal = start + i * 0.25
                cal = arm.bus.calibration["shoulder_pan"]
                raw = goal * 4095 / 360 + (cal.range_min + cal.range_max) / 2
                if not cal.range_min + 40 <= raw <= cal.range_max - 40:
                    raise RuntimeError("Contact stroke outside encoder limits")
                arm.write("Goal_Position", "shoulder_pan", round(raw), raw=True)
                time.sleep(0.3)
                arm.health(force=True)
                loads = {j: arm.signed_load(j) for j in JOINTS}
                if max(map(abs, loads.values())) >= 250:
                    raise RuntimeError(f"Contact stroke load stop: {loads}")
                actual = arm.position("shoulder_pan")
                if abs(actual - goal) > 1:
                    raise RuntimeError("Contact stroke tracking stop")
                samples.append({"goal": goal, "actual": actual, "loads": loads})
                print("STROKE", json.dumps(samples[-1]), flush=True)
            arm.photograph("end_contact")
            arm.lift(hit, clearance=12)
            time.sleep(2)
            arm.photograph("after")
            save_json(OUT / "result.json", {"contact": hit, "samples": samples})
            print("TEST COMPLETE; parking above paper", flush=True)
            arm.move("shoulder_lift", 0, timeout=45, max_step=6)
            arm.move("elbow_flex", 85, timeout=45, max_step=6)
        finally:
            arm.close()


if __name__ == "__main__":
    main()
