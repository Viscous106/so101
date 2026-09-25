"""Bounded, single-owner pen-contact experiments; no full-image execution."""
import argparse
import fcntl
import json
import math
import select
import signal
import subprocess
import sys
import time

from calibration_console import recover
from draw_reference import Arm, JOINTS, ROOT, save_json

OUT = ROOT / "hardware/draw/runs/reference_OjPuJYdomotd/contact_lab"
BOUNDS = {"shoulder_pan": (8, 34), "shoulder_lift": (30, 65),
          "elbow_flex": (45, 80), "wrist_flex": (-99.8, -90),
          "wrist_roll": (-73, -69)}


def validate_pose(pose):
    for joint, value in pose.items():
        if joint not in BOUNDS or not math.isfinite(value):
            raise ValueError("Unknown joint or non-finite command")
        if not BOUNDS[joint][0] <= value <= BOUNDS[joint][1]:
            raise ValueError(f"Outside local envelope: {joint} {value}")


class Lab:
    def __init__(self, arm):
        self.arm = arm
        self.contact = None
        self.count = 0

    def sample(self, name, photo=True):
        arm = self.arm
        arm.health(force=True)
        record = {"time": time.time(), "pose": {j: arm.position(j) for j in JOINTS},
                  "goals": {j: float(arm.read("Goal_Position", j)) for j in JOINTS},
                  "loads": {j: arm.signed_load(j) for j in JOINTS}, "clear": arm.clear}
        if photo:
            arm.photograph(name)
        save_json(OUT / f"{name}.json", record)
        print("SAMPLE", name, json.dumps(record), flush=True)
        return record

    def clearance(self):
        if self.contact is not None:
            self.coupled_retreat()
            self.arm.lift(self.arm.position("shoulder_lift"), clearance=12)
            self.contact = None
        if not self.arm.clear:
            raise RuntimeError("No verified pen clearance")

    def go(self, pan, elbow, height):
        target = {"shoulder_pan": float(pan), "elbow_flex": float(elbow),
                  "shoulder_lift": float(height)}
        validate_pose(target)
        self.clearance()
        if height > 46:
            raise ValueError("Travel height must be at most 46 degrees")
        self.arm.move("shoulder_lift", min(height, self.arm.position("shoulder_lift")), tolerance=.9, max_step=6, timeout=35)
        self.arm.move("elbow_flex", elbow, max_step=6, timeout=35)
        self.arm.move("shoulder_pan", pan, max_step=6, timeout=35)
        self.arm.move("shoulder_lift", height, tolerance=.9, max_step=3, timeout=35)

    def touch(self, anchor, threshold, slow=False):
        if not 45 <= anchor <= 61 or not -48 <= threshold <= -20:
            raise ValueError("Contact search outside its bounds")
        arm = self.arm
        if not arm.clear:
            raise RuntimeError("Contact search requires a fresh lift")
        arm.clear = False
        if not slow:
            arm.move("shoulder_lift", anchor - 4, tolerance=.3, timeout=25)
        command = arm.position("shoulder_lift")
        deadline = time.monotonic() + 30
        while command < anchor + 1.5 and time.monotonic() < deadline:
            command += .1
            validate_pose({"shoulder_lift": command})
            arm.target("shoulder_lift", command)
            time.sleep(.15)
            arm.health()
            signed = arm.signed_load("shoulder_lift")
            arm.log("lab_descent", command=command, actual=arm.position("shoulder_lift"), load=signed)
            if abs(signed) >= 180:
                raise RuntimeError("Contact load stop")
            if signed <= threshold:
                time.sleep(.15)
                confirm = arm.signed_load("shoulder_lift")
                if abs(confirm) >= 180:
                    raise RuntimeError("Confirmed contact load stop")
                if confirm <= threshold:
                    self.contact = arm.position("shoulder_lift")
                    return self.contact
        raise RuntimeError("No gentle contact inside search bounds")

    def coupled_retreat(self):
        arm = self.arm
        joints = JOINTS[:4]
        initial = {j: arm.position(j) for j in joints}
        target = dict(initial)
        target["shoulder_lift"] -= 3
        target["elbow_flex"] += .8
        validate_pose(target)
        command = {j: float(arm.read("Goal_Position", j)) for j in joints}
        bias = {j: command[j] - initial[j] for j in joints}
        started = time.monotonic()
        settled = 0
        while time.monotonic() - started < 15:
            elapsed = time.monotonic() - started
            u = min(1., elapsed / 4.)
            blend = u*u*(3-2*u)
            actual = {j: arm.position(j) for j in joints}
            for j in joints:
                desired = initial[j] + blend*(target[j]-initial[j])
                error = desired - actual[j]
                bias[j] = max(-4, min(4, bias[j] + max(-.12, min(.12, .35*error))))
                wanted = desired + bias[j] + .5*error
                command[j] += max(-.25, min(.25, wanted-command[j]))
            validate_pose(command)
            for j, value in command.items():
                cal = arm.bus.calibration[j]
                raw = value*4095/360 + (cal.range_min+cal.range_max)/2
                if not cal.range_min+24 <= raw <= cal.range_max-24:
                    raise RuntimeError("Coupled retreat encoder limit")
            arm.bus.sync_write("Goal_Position", command)
            time.sleep(.1)
            arm.health()
            loads = {j: arm.signed_load(j) for j in JOINTS}
            if max(map(abs, loads.values())) >= 250 or loads["shoulder_lift"] < -100:
                raise RuntimeError(f"Coupled retreat load stop: {loads}")
            if any(abs(actual[j]-initial[j]) > 5 for j in joints):
                raise RuntimeError("Coupled retreat exceeded local displacement")
            arm.log("coupled_retreat", actual=actual, target=target, command=dict(command), loads=loads)
            settled = settled+1 if u == 1 and all(abs(actual[j]-target[j]) < .4 for j in joints) else 0
            if settled >= 3:
                return
        raise RuntimeError("Coupled retreat failed measured-position tracking")

    def line(self, delta_pan, delta_elbow, slope=-.53, duration=4):
        if self.contact is None or not .5 <= duration <= 12:
            raise ValueError("A bounded stroke requires contact")
        if not all(math.isfinite(v) for v in (delta_pan, delta_elbow, slope, duration)):
            raise ValueError("Non-finite stroke")
        if abs(delta_pan) > 3 or abs(delta_elbow) > 3 or not -.8 <= slope <= -.3:
            raise ValueError("Stroke exceeds the three-degree experiment envelope")
        arm = self.arm
        # Preserve initial loaded holding goals instead of assuming encoders
        # equal commands; all three drawing axes are updated in one packet.
        joints = JOINTS[:3]
        initial = {j: float(arm.read("Goal_Position", j)) for j in joints}
        start = {j: arm.position(j) for j in joints}
        delta = {"shoulder_pan": delta_pan, "shoulder_lift": slope * delta_elbow,
                 "elbow_flex": delta_elbow}
        endpoint = {j: initial[j] + delta[j] for j in joints}
        validate_pose(endpoint)
        steps = max(1, math.ceil(duration / .1))
        records = []
        for i in range(1, steps + 1):
            # Smooth endpoints avoid abrupt velocity changes at contact.
            u = i / steps
            blend = u * u * (3 - 2 * u)
            goal = {j: initial[j] + blend * delta[j] for j in joints}
            validate_pose(goal)
            for j, value in goal.items():
                cal = arm.bus.calibration[j]
                raw = value * 4095 / 360 + (cal.range_min + cal.range_max) / 2
                if not cal.range_min + 24 <= raw <= cal.range_max - 24:
                    raise RuntimeError("Stroke outside encoder guard")
            arm.bus.sync_write("Goal_Position", goal)
            time.sleep(.1)
            arm.health()
            loads = {j: arm.signed_load(j) for j in JOINTS}
            if max(map(abs, loads.values())) >= 180 or loads["shoulder_lift"] < -90:
                raise RuntimeError(f"Gentle-stroke load stop: {loads}")
            actual = {j: arm.position(j) for j in joints}
            if any(abs(actual[j] - start[j]) > abs(delta[j]) + 3 for j in joints):
                raise RuntimeError("Stroke motion escaped its local envelope")
            record = {"goal": goal, "actual": actual, "loads": loads}
            arm.log("lab_stroke", **record)
            records.append(record)
        time.sleep(.3)
        self.count += 1
        save_json(OUT / f"stroke_{self.count:02d}.json", records)
        return records[-1]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--watchdog", type=float, default=180)
    args = parser.parse_args()
    if not 10 <= args.watchdog <= 180:
        parser.error("Watchdog must be within 10-180 seconds")
    signal.signal(signal.SIGTERM, lambda *_: (_ for _ in ()).throw(KeyboardInterrupt()))
    if subprocess.run(["fuser", "/dev/ttyACM0"], capture_output=True).returncode == 0:
        raise RuntimeError("Serial bus already owned")
    OUT.mkdir(parents=True, exist_ok=True)
    with open("/tmp/so101-drawing.lock", "w") as lock:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        arm = Arm(OUT)
        try:
            recover(arm)
            arm.move("shoulder_lift", 38, tolerance=.9, max_step=6, timeout=30)
            arm.move("wrist_flex", -94, tolerance=.3, max_step=2, timeout=20)
            arm.target("wrist_roll", -71.25274725274726)
            lab = Lab(arm)
            settings = {j: {r: arm.read(r, j, raw=True) for r in
                        ("P_Coefficient", "D_Coefficient", "I_Coefficient", "CW_Dead_Zone", "CCW_Dead_Zone")}
                        for j in JOINTS}
            save_json(OUT / "servo_settings.json", settings)
            print("READ_ONLY_SETTINGS", json.dumps(settings), flush=True)
            lab.sample("recovered")
            deadline = time.monotonic() + args.watchdog
            print("READY", flush=True)
            while time.monotonic() < deadline:
                arm.health()
                if not select.select([sys.stdin], [], [], .2)[0]:
                    continue
                raw = sys.stdin.readline()
                if not raw:
                    break
                command = json.loads(raw)
                action = command.pop("action")
                name = command.pop("name", action)
                if not name.replace("_", "").isalnum():
                    raise ValueError("Invalid sample name")
                if action == "finish":
                    lab.clearance()
                    arm.move("shoulder_lift", 0, max_step=6, timeout=45)
                    arm.move("elbow_flex", 85, max_step=6, timeout=45)
                    arm.move("shoulder_pan", 30, max_step=6, timeout=30)
                    lab.sample(name)
                    break
                if action == "go":
                    lab.go(**command)
                elif action == "touch":
                    lab.touch(**command)
                elif action == "line":
                    lab.line(**command)
                elif action == "lift":
                    lab.clearance()
                elif action != "status":
                    raise ValueError("Unknown lab action")
                lab.sample(name)
                deadline = time.monotonic() + args.watchdog
                print("READY", flush=True)
            else:
                raise RuntimeError("Contact lab watchdog expired")
        except BaseException as exc:
            arm.log("abort", error=str(exc))
            raise
        finally:
            arm.close()


if __name__ == "__main__":
    main()
