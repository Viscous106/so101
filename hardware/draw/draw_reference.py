"""Guarded, resumable stippling of the supplied Taj Mahal reference.

prepare and capture never connect to the arm. run owns the bus exclusively,
checks each lift before lateral movement, and releases torque on every exit.
"""
import argparse
import fcntl
import json
import math
import os
import pathlib
import select
import shutil
import signal
import subprocess
import sys
import time

import cv2
import numpy as np

from camera_devices import DESK_CAMERA, WRIST_CAMERA

ROOT = pathlib.Path(__file__).resolve().parents[2]
DEFAULT_OUT = ROOT / "hardware/draw/runs/pasteboard_taj"
PAN_STEP = 0.42
ROLL_STEP = 1.4
JOINTS = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll"]


def save_json(path, value):
    tmp = path.with_suffix(".tmp")
    with tmp.open("w") as stream:
        stream.write(json.dumps(value, indent=2) + "\n")
        stream.flush()
        os.fsync(stream.fileno())
    tmp.replace(path)
    # Persist the rename too, not just the file contents, before the next mark.
    directory = os.open(path.parent, os.O_RDONLY | os.O_DIRECTORY)
    try:
        os.fsync(directory)
    finally:
        os.close(directory)


def prepare(out, reference):
    out.mkdir(parents=True, exist_ok=True)
    if (out / "state.json").exists():
        raise RuntimeError("Drawing already has a checkpoint; use a new output directory")
    source = cv2.imread(str(reference))
    if source is None:
        raise RuntimeError(f"Cannot decode {reference}")
    cv2.imwrite(str(out / "reference.png"), source)
    grid = np.zeros((11, 31), np.uint8)
    # Trace the distinctive stepped dome, minarets and three doors from the reference.
    paths = [
        [(0, 10), (30, 10)],
        [(2, 10), (2, 3), (1, 2), (2, 1), (4, 1), (5, 2), (4, 3), (4, 10)],
        [(26, 10), (26, 3), (25, 2), (26, 1), (28, 1), (29, 2), (28, 3), (28, 10)],
        [(7, 10), (7, 5), (10, 5), (10, 3), (11, 3), (11, 2), (13, 2),
         (15, 0), (17, 2), (19, 2), (19, 3), (20, 3), (20, 5), (23, 5), (23, 10)],
        [(9, 10), (9, 8), (11, 8), (11, 10)],
        [(19, 10), (19, 8), (21, 8), (21, 10)],
        [(13, 10), (13, 7), (14, 6), (16, 6), (17, 7), (17, 10)],
    ]
    for points in paths:
        cv2.polylines(grid, [np.array(points, np.int32)], False, 1, 1)
    points = []
    for row, y in enumerate(range(grid.shape[0] - 1, -1, -1)):
        columns = np.flatnonzero(grid[y]).tolist()
        if row % 2:
            columns.reverse()
        points.extend({"row": row, "x": x, "y": y} for x in columns)
    plan = {"source": "https://www.pasteboard.co/tiM_n4RpBsQR.png", "grid": grid.tolist(),
            "pan_step": PAN_STEP, "roll_step": ROLL_STEP, "points": points}
    save_json(out / "plan.json", plan)
    preview = np.full((360, 1000, 3), 250, np.uint8)
    for y, x in np.argwhere(grid):
        cv2.circle(preview, (35 + int(x) * 31, 25 + int(y) * 30), 5, (25, 25, 25), -1)
    cv2.imwrite(str(out / "plan.png"), preview)
    for row in grid:
        print("".join("#" if v else "." for v in row))
    print(f"{len(points)} stamps; pan span {30 * PAN_STEP:.1f} deg; roll travel {10 * ROLL_STEP:.1f} deg")


def capture(out):
    sys.path.insert(0, str(ROOT / "hardware/repeat"))
    from cam import Cam

    out.mkdir(parents=True, exist_ok=True)
    for name, dev, exposure, gain in [("desk", DESK_CAMERA, 40, 64),
                                       ("wrist", WRIST_CAMERA, 39, 0)]:
        cam = None
        try:
            cam = Cam(dev, exposure=exposure, gain=gain)
            cam.auto_gain(target=110, tries=3)
            frame = cam.stack(3).clip(0, 255).astype(np.uint8)
            if not cv2.imwrite(str(out / f"{name}.jpg"), frame):
                raise RuntimeError("Camera image write failed")
            print(f"CAMERA {name}: {out / (name + '.jpg')}", flush=True)
        except Exception:
            if name != "wrist":
                raise
            print("CAMERA wrist unavailable; desk image saved", flush=True)
        finally:
            if cam is not None:
                cam.close()


class Arm:
    def __init__(self, out):
        from execute import load_bus
        self.bus = load_bus()
        self.out = out
        self.last_health = 0.0
        self.clear = False
        self.state = None
        self.last_hit = None
        self.start = None
        self.offsets = {}
        self.logfile = (out / "telemetry.jsonl").open("a", buffering=1)

    def log(self, kind, **values):
        self.logfile.write(json.dumps({"time": time.time(), "event": kind, **values}) + "\n")

    def read(self, reg, joint, raw=False):
        for attempt in range(4):
            try:
                return self.bus.read(reg, joint, normalize=not raw)
            except Exception:
                if attempt == 3:
                    raise
                time.sleep(0.12)

    def write(self, reg, joint, value, raw=False):
        for attempt in range(3):
            try:
                return self.bus.write(reg, joint, value, normalize=not raw)
            except Exception:
                if attempt == 2:
                    raise
                time.sleep(0.12)

    def position(self, joint):
        return float(self.read("Present_Position", joint))

    def load(self, joint):
        # LeRobot decodes sign-magnitude even with normalize=False.
        return abs(self.signed_load(joint))

    def signed_load(self, joint):
        return int(self.read("Present_Load", joint, raw=True))

    def target(self, joint, degrees):
        cal = self.bus.calibration[joint]
        raw = degrees * 4095 / 360 + (cal.range_min + cal.range_max) / 2
        if not math.isfinite(degrees) or not cal.range_min + 24 <= raw <= cal.range_max - 24:
            raise RuntimeError(f"Unsafe {joint} target {degrees:.2f}, raw={raw:.1f}")
        if joint != "shoulder_lift" and not self.clear:
            raise RuntimeError("Lateral movement requires a verified lift")
        self.write("Goal_Position", joint, round(raw), raw=True)

    def health(self, force=False):
        if not force and time.monotonic() - self.last_health < 2:
            return
        readings = {}
        for joint in JOINTS:
            temp = self.read("Present_Temperature", joint, raw=True)
            raw = self.read("Present_Position", joint, raw=True)
            load = self.load(joint)
            readings[joint] = {"temperature": temp, "raw": raw, "load": load,
                               "signed_load": self.signed_load(joint)}
            cal = self.bus.calibration[joint]
            if temp >= 55:
                raise RuntimeError(f"Temperature stop: {joint} {temp} C")
            if not cal.range_min + 12 <= raw <= cal.range_max - 12:
                raise RuntimeError(f"Joint limit stop: {joint} raw={raw}")
            if joint in ("elbow_flex", "wrist_flex") and load >= 700:
                raise RuntimeError(f"Unexpected load on held joint {joint}: {load}")
        self.last_health = time.monotonic()
        self.log("health", joints=readings)

    def setup(self):
        self.bus.connect()
        self.health(force=True)
        pose = {joint: self.position(joint) for joint in JOINTS}
        if not (40 < pose["shoulder_lift"] < 75 and 35 < pose["elbow_flex"] < 65
                and -101 < pose["wrist_flex"] < -80):
            raise RuntimeError(f"Arm needs posture restoration before drawing: {pose}")
        for joint in JOINTS:
            self.write("Goal_Position", joint, pose[joint])
            self.write("Acceleration", joint, 6, raw=True)
            self.write("Goal_Velocity", joint, 100, raw=True)
            self.write("Torque_Limit", joint, 900 if joint == "shoulder_lift" else 600, raw=True)
            self.write("Torque_Enable", joint, 1, raw=True)
        self.start = pose
        print("POSE", json.dumps(pose), flush=True)
        self.log("start", pose=pose)
        return pose

    def move(self, joint, goal, tolerance=0.18, timeout=18, max_step=3.0):
        if not 0 < max_step <= 8:
            raise ValueError("Move step must be within (0, 8] degrees")
        cal = self.bus.calibration[joint]
        midpoint = (cal.range_min + cal.range_max) / 2
        goal = (round(goal * 4095 / 360 + midpoint) - midpoint) * 360 / 4095
        origin = self.position(joint)
        if abs(goal - origin) <= tolerance:
            return origin
        key = (joint, 1 if goal > origin else -1)
        correction = self.offsets.get(key, 0.0)
        deadline = time.monotonic() + timeout
        last = origin
        still = 0
        while time.monotonic() < deadline:
            self.health()
            current = self.position(joint)
            if abs(current - goal) <= tolerance:
                return current
            step = max(-max_step, min(max_step, goal - current))
            command = current + step + correction
            self.target(joint, command)
            time.sleep(0.8)
            actual = self.position(joint)
            load = self.load(joint)
            # A stopped servo can hold several degrees away from its goal under
            # gravity. Carry the measured offset into the next bounded command.
            if not self.read("Moving", joint, raw=True):
                bound = 6.0 if joint == "shoulder_lift" else 3.0
                correction = max(-bound, min(bound, command - actual))
                self.offsets[key] = correction
            self.log("move", joint=joint, goal=goal, command=command,
                     actual=actual, load=load, correction=correction)
            if joint != "shoulder_lift" and load > 500:
                time.sleep(0.15)
                if self.load(joint) > 500:
                    raise RuntimeError(f"Sustained load during {joint} move: {load}")
            if joint == "shoulder_lift" and goal > origin and self.signed_load(joint) < -120:
                raise RuntimeError("Unexpected contact during height positioning")
            if abs(actual - last) < 0.09:
                still += 1
            else:
                still = 0
            if still >= 10:
                raise RuntimeError(f"Stalled {joint} at {actual:.2f}, wanted {goal:.2f}")
            last = actual
        raise RuntimeError(f"Move timeout {joint}: actual {self.position(joint):.2f}, target {goal:.2f}")

    def lift(self, contact, clearance=6.0):
        if not 6 <= clearance <= 12:
            raise ValueError("Lift clearance must be within [6, 12] degrees")
        self.clear = False
        self.move("shoulder_lift", contact - clearance, tolerance=0.45, timeout=18,
                  max_step=6.0 if clearance > 6 else 3.0)
        time.sleep(0.25)
        actual = self.position("shoulder_lift")
        load = self.load("shoulder_lift")
        if actual > contact - clearance + 1.0 or load >= 250:
            raise RuntimeError(f"Lift not clear: contact={contact:.2f}, actual={actual:.2f}, load={load}")
        self.clear = True
        self.log("clear", position=actual, load=load, contact=contact)

    def stamp(self, anchor, clearance=2.5):
        self.clear = False
        before = self.position("shoulder_lift")
        if anchor is not None:
            self.move("shoulder_lift", anchor - clearance, tolerance=0.45)
            time.sleep(0.4)
        start = self.position("shoulder_lift")
        end = min(self.start["shoulder_lift"] + 12, start + 12)
        if anchor is not None:
            end = min(end, anchor + 4)
        command = start
        while command < end:
            self.health()
            command += 0.15
            self.target("shoulder_lift", command)
            time.sleep(0.22)
            signed = self.signed_load("shoulder_lift")
            self.log("descent", command=command, actual=self.position("shoulder_lift"), signed_load=signed)
            if abs(signed) >= 300:
                raise RuntimeError("Excessive load in contact search")
            # Gravity requires positive holding torque here. A small sustained
            # negative torque indicates the pen is pressing against the paper.
            if signed > -60:
                continue
            time.sleep(0.05)
            if self.signed_load("shoulder_lift") > -60:
                continue
            hit = self.position("shoulder_lift")
            self.last_hit = hit
            if anchor is not None and abs(hit - anchor) > 4:
                raise RuntimeError(f"Unexpected contact height {hit:.2f} vs {anchor:.2f}")
            time.sleep(0.2)
            live = pathlib.Path("/tmp/so101-camera-live/desk.jpg")
            if live.exists():
                self.photograph("latest_contact")
            self.lift(hit)
            self.log("contact", hit=hit, anchor=anchor, before=before)
            return hit
        raise RuntimeError("No contact found within bounded search")

    def photograph(self, name):
        live = pathlib.Path("/tmp/so101-camera-live/desk.jpg")
        if live.exists() and time.time() - live.stat().st_mtime < 3:
            destination = self.out / name
            destination.mkdir(parents=True, exist_ok=True)
            shutil.copy2(live, destination / "desk.jpg")
            print(f"CAMERA desk: {destination / 'desk.jpg'}", flush=True)
            return
        process = subprocess.Popen([sys.executable, "-u", __file__, "capture", "--out", str(self.out / name)])
        deadline = time.monotonic() + 40
        try:
            while process.poll() is None:
                self.health()
                if time.monotonic() >= deadline:
                    raise RuntimeError("Camera capture timed out")
                time.sleep(0.25)
            if process.returncode:
                raise RuntimeError("Camera capture failed")
        finally:
            if process.poll() is None:
                process.send_signal(signal.SIGINT)
                try:
                    process.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    process.terminate()  # Camera subprocess owns no motors.
                    process.wait(timeout=5)

    def close(self):
        if self.bus.is_connected:
            for joint in JOINTS:
                try:
                    self.write("Torque_Enable", joint, 0, raw=True)
                except Exception as error:
                    print(f"CLEANUP ERROR {joint}: {error}", flush=True)
            try:
                self.bus.disconnect(disable_torque=False)
            finally:
                print("Torque off; bus disconnected", flush=True)
        self.logfile.close()


def run(out, limit, review_after=0, pan_center=None):
    plan = json.loads((out / "plan.json").read_text())
    state_path = out / "state.json"
    state = json.loads(state_path.read_text()) if state_path.exists() else None
    if state and state["next"] >= len(plan["points"]):
        print("All planned stamps already completed")
        return
    owner = subprocess.run(["fuser", "/dev/ttyACM0"], capture_output=True)
    if owner.returncode == 0:
        raise RuntimeError(f"Serial bus is already in use: {owner.stdout.decode().strip()}")
    with open("/tmp/so101-drawing.lock", "w") as lock:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        arm = Arm(out)
        try:
            pose = arm.setup()
            if state is None or state["next"] == 0:
                if pan_center is not None:
                    pose = dict(pose, shoulder_pan=pan_center)
                state = {"base": pose, "next": 0, "table_h": None, "first_contact": None}
                save_json(state_path, state)
            else:
                for joint in ("elbow_flex", "wrist_flex"):
                    if abs(pose[joint] - state["base"][joint]) > 3.0:
                        raise RuntimeError(f"Held joint {joint} changed too much for automatic resume")
            arm.lift(pose["shoulder_lift"])
            if state["next"]:
                # Restore the original held motor goals after the pen is clear;
                # commanding a new sagged baseline would offset existing marks.
                for joint in ("elbow_flex", "wrist_flex"):
                    arm.target(joint, state["base"][joint])
                    time.sleep(0.8)
                    arm.health(force=True)
            arm.photograph(f"before_{state['next']:03d}")
            first = state["next"]
            end = min(len(plan["points"]), first + limit) if limit else len(plan["points"])
            last_row = None
            for index in range(first, end):
                point = plan["points"][index]
                started = time.monotonic()
                base = state["base"]
                roll = base["wrist_roll"] - point["row"] * plan["roll_step"]
                pan = base["shoulder_pan"] + (point["x"] - 15) * plan["pan_step"]
                row_changed = point["row"] != last_row
                if row_changed:
                    arm.move("wrist_roll", roll, tolerance=0.24)
                arm.move("shoulder_pan", pan)
                hit = arm.stamp(state["table_h"], clearance=2.5 if row_changed else 1.5)
                last_row = point["row"]
                if state["first_contact"] is None:
                    state["first_contact"] = hit
                if abs(hit - state["first_contact"]) > 12:
                    raise RuntimeError("Contact height drift exceeded 12 degrees")
                state["table_h"] = hit if state["table_h"] is None else 0.7 * state["table_h"] + 0.3 * hit
                state["next"] = index + 1
                save_json(state_path, state)
                elapsed = time.monotonic() - started
                print(f"STAMP {index + 1}/{len(plan['points'])} row={point['row']} col={point['x']} "
                      f"contact={hit:.2f} anchor={state['table_h']:.2f} seconds={elapsed:.1f}", flush=True)
                arm.log("stamp", index=index, point=point, contact=hit, seconds=elapsed)
                if (index + 1) % 8 == 0 or index + 1 == end:
                    arm.photograph(f"after_{index + 1:03d}")
                if review_after and index + 1 == review_after:
                    print("REVIEW: camera checkpoint ready; enter continue within 90 seconds", flush=True)
                    deadline = time.monotonic() + 90
                    while True:
                        arm.health()
                        if time.monotonic() > deadline:
                            raise RuntimeError("Camera review timed out")
                        readable, _, _ = select.select([sys.stdin], [], [], 0.25)
                        if readable:
                            if sys.stdin.readline().strip() != "continue":
                                raise RuntimeError("Camera review stopped the run")
                            break
            print(f"COMPLETED {state['next']}/{len(plan['points'])} stamps", flush=True)
        except BaseException as error:
            arm.log("abort", error=str(error))
            print(f"ABORT: {error}", flush=True)
            raise
        finally:
            arm.close()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("action", choices=["prepare", "capture", "run"])
    parser.add_argument("--out", type=pathlib.Path, default=DEFAULT_OUT)
    parser.add_argument("--reference", type=pathlib.Path, default=pathlib.Path("/tmp/so101-reference.png"))
    parser.add_argument("--limit", type=int, default=0)
    parser.add_argument("--review-after", type=int, default=0)
    parser.add_argument("--pan-center", type=float)
    args = parser.parse_args()
    if args.limit < 0:
        parser.error("--limit must be nonnegative")
    def interrupt(signum, frame):
        raise KeyboardInterrupt(f"Signal {signum}")
    signal.signal(signal.SIGTERM, interrupt)
    if args.action == "prepare":
        prepare(args.out, args.reference)
    elif args.action == "capture":
        capture(args.out)
    else:
        run(args.out, args.limit, args.review_after, args.pan_center)


if __name__ == "__main__":
    main()
