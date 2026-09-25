"""Single-owner, watchdog-protected recovery and camera calibration session.

Only bounded recovery and verified lift/stamp primitives are exposed. JSON
commands arrive on stdin; silence, EOF, errors or signals release all torque.
"""
import fcntl
import json
import pathlib
import select
import shutil
import signal
import subprocess
import sys
import time

from draw_reference import Arm, JOINTS, ROOT, save_json


OUT = ROOT / "hardware/draw/runs/camera_calibration"


def recovery_health(arm, initial=False):
    for joint in JOINTS:
        raw = arm.read("Present_Position", joint, raw=True)
        cal = arm.bus.calibration[joint]
        margin = 0 if initial and joint == "elbow_flex" else 12
        if not cal.range_min + margin <= raw <= cal.range_max - margin:
            raise RuntimeError(f"Recovery position stop: {joint} {raw}")
        if arm.read("Present_Temperature", joint, raw=True) >= 50:
            raise RuntimeError(f"Recovery temperature stop: {joint}")
        if arm.load(joint) >= 300:
            raise RuntimeError(f"Recovery load stop: {joint}")


def recover(arm):
    arm.bus.connect()
    pose = {j: arm.position(j) for j in JOINTS}
    print("RECOVERY_START", json.dumps(pose), flush=True)
    if (40 < pose["shoulder_lift"] < 75 and 35 < pose["elbow_flex"] < 65
            and -101 < pose["wrist_flex"] < -80):
        arm.bus.disconnect(disable_torque=False)
        pose = arm.setup()
        arm.lift(pose["shoulder_lift"])
        arm.photograph("recovered")
        return pose
    # This inward-only recovery envelope is separate from drawing setup.
    if not (-5 < pose["shoulder_lift"] < 65 and 40 < pose["elbow_flex"] < 99
            and -101 < pose["wrist_flex"] < -80 and 5 < pose["shoulder_pan"] < 35):
        raise RuntimeError("Pose outside the camera-reviewed recovery envelope")
    recovery_health(arm, initial=True)
    for joint in JOINTS:
        arm.write("Goal_Position", joint, arm.read("Present_Position", joint, raw=True), raw=True)
        arm.write("Acceleration", joint, 4, raw=True)
        arm.write("Goal_Velocity", joint, 70, raw=True)
        arm.write("Torque_Limit", joint, 900 if joint == "shoulder_lift" else 600, raw=True)
        arm.write("Torque_Enable", joint, 1, raw=True)
    if pose["elbow_flex"] > 90:
        # Only retract from the upper elbow limit; never drive farther into it.
        target = pose["elbow_flex"] - 3
        arm.write("Goal_Position", "elbow_flex", target)
        deadline = time.monotonic() + 8
        while arm.position("elbow_flex") > pose["elbow_flex"] - 1.5:
            recovery_health(arm, initial=True)
            if time.monotonic() > deadline:
                raise RuntimeError("Initial inward elbow recovery did not move")
            time.sleep(0.2)
        time.sleep(0.5)
    recovery_health(arm)
    # The first fold retracts the tip away from the paper in the reviewed pose.
    # No pan/roll travel occurs until the fold and raised posture are established.
    for joint, target in (("elbow_flex", 50), ("wrist_flex", -98), ("shoulder_lift", 48)):
        deadline = time.monotonic() + 100
        correction = 0
        while abs(arm.position(joint) - target) > 0.6:
            recovery_health(arm)
            if time.monotonic() > deadline:
                raise RuntimeError(f"Recovery timeout: {joint}")
            current = arm.position(joint)
            command = current + max(-2, min(2, target - current)) + correction
            cal = arm.bus.calibration[joint]
            raw = command * 4095 / 360 + (cal.range_min + cal.range_max) / 2
            if not cal.range_min + 24 <= raw <= cal.range_max - 24:
                raise RuntimeError("Recovery target exceeds guarded range")
            arm.write("Goal_Position", joint, round(raw), raw=True)
            time.sleep(0.8)
            actual = arm.position(joint)
            if not arm.read("Moving", joint, raw=True):
                correction = max(-3, min(3, command - actual))
            print(f"RECOVER {joint} actual={actual:.2f} load={arm.signed_load(joint)}", flush=True)
    arm.clear = True
    arm.move("wrist_roll", -71.3, tolerance=0.3)
    arm.start = {j: arm.position(j) for j in JOINTS}
    arm.photograph("recovered")
    return arm.start


def snapshot(arm, name):
    arm.photograph(name)
    pose = {j: arm.position(j) for j in JOINTS}
    record = {"time": time.time(), "pose": pose, "clear": arm.clear,
              "loads": {j: arm.signed_load(j) for j in JOINTS}}
    save_json(OUT / name / "pose.json", record)
    print("SAMPLE", name, json.dumps(record), flush=True)
    return record


def main():
    def interrupt(signum, frame):
        raise KeyboardInterrupt(f"Signal {signum}")
    signal.signal(signal.SIGTERM, interrupt)
    if subprocess.run(["fuser", "/dev/ttyACM0"], capture_output=True).returncode == 0:
        raise RuntimeError("Serial bus already owned")
    OUT.mkdir(parents=True, exist_ok=True)
    with open("/tmp/so101-drawing.lock", "w") as lock:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        arm = Arm(OUT)
        anchor = None
        try:
            recover(arm)
            print("READY", flush=True)
            deadline = time.monotonic() + 180
            while True:
                arm.health()
                if time.monotonic() > deadline:
                    raise RuntimeError("Calibration command watchdog expired")
                ready, _, _ = select.select([sys.stdin], [], [], 0.25)
                if not ready:
                    continue
                line = sys.stdin.readline()
                if not line:
                    break
                cmd = json.loads(line)
                deadline = time.monotonic() + 180
                if cmd["action"] == "finish":
                    break
                if cmd["action"] == "status":
                    snapshot(arm, cmd.get("name", "status"))
                elif cmd["action"] == "move":
                    joint, goal = cmd["joint"], float(cmd["goal"])
                    bounds = {"shoulder_pan": (8, 33), "shoulder_lift": (38, 65),
                              "elbow_flex": (42, 67), "wrist_roll": (-90, -65)}
                    if joint not in bounds or not bounds[joint][0] <= goal <= bounds[joint][1]:
                        raise RuntimeError("Calibration move outside envelope")
                    if abs(goal - arm.position(joint)) > 8:
                        raise RuntimeError("Calibration move exceeds 8 degrees")
                    arm.move(joint, goal, tolerance=0.22)
                    snapshot(arm, cmd.get("name", "moved"))
                elif cmd["action"] == "stamp":
                    if "anchor" in cmd:
                        anchor = cmd["anchor"]
                    hit = arm.stamp(anchor, clearance=2.5)
                    anchor = hit
                    name = cmd["name"]
                    (OUT / name).mkdir(exist_ok=True)
                    shutil.copy2(OUT / "latest_contact/desk.jpg", OUT / name / "contact.jpg")
                    record = snapshot(arm, name)
                    record["contact_lift"] = hit
                    save_json(OUT / name / "pose.json", record)
                else:
                    raise ValueError("Unknown calibration command")
                print("READY", flush=True)
        finally:
            arm.close()


if __name__ == "__main__":
    main()
