"""Camera-reviewed, force-limited continuous outline using the measured row axis."""
import fcntl
import json
import math
import pathlib
import select
import signal
import subprocess
import sys
import time

import numpy as np

from calibration_console import recover
from draw_reference import Arm, JOINTS, ROOT, save_json
from test_continuous import touch


OUT = ROOT / "hardware/draw/runs/continuous_taj"
SLOPE = -0.53  # Measured contact-lift change per elbow degree, not a force scale.
PATHS = [
    [(0, 10), (30, 10)],
    [(2, 10), (2, 3), (1, 2), (2, 1), (4, 1), (5, 2), (4, 3), (4, 10)],
    [(26, 10), (26, 3), (25, 2), (26, 1), (28, 1), (29, 2), (28, 3), (28, 10)],
    [(7, 10), (7, 5), (10, 5), (10, 3), (11, 3), (11, 2), (13, 2),
     (15, 0), (17, 2), (19, 2), (19, 3), (20, 3), (20, 5), (23, 5), (23, 10)],
    [(9, 10), (9, 8), (11, 8), (11, 10)],
    [(19, 10), (19, 8), (21, 8), (21, 10)],
    [(13, 10), (13, 7), (14, 6), (16, 6), (17, 7), (17, 10)],
]


def grid_to_joints(point):
    x, y = point
    return np.array([16 + (x - 15) * 0.36, 51 + (10 - y) * 1.4])


def interpolate(points, step=0.3):
    previous = np.asarray(points[0], dtype=float)
    for point in points[1:]:
        point = np.asarray(point, dtype=float)
        count = max(1, math.ceil(float(np.max(np.abs(point - previous))) / step))
        for index in range(1, count + 1):
            yield previous + (point - previous) * index / count
        previous = point


def validate_contact_goal(joint, goal):
    limits = {"shoulder_pan": (8, 33), "elbow_flex": (45, 69), "shoulder_lift": (43, 66)}
    if not math.isfinite(goal) or not limits[joint][0] <= goal <= limits[joint][1]:
        raise RuntimeError(f"Contact command outside local envelope: {joint} {goal}")


class Outline:
    def __init__(self, arm):
        self.arm = arm
        self.height = 59.0
        self.last_hit = None

    def write_contact(self, joint, goal):
        validate_contact_goal(joint, goal)
        cal = self.arm.bus.calibration[joint]
        raw = goal * 4095 / 360 + (cal.range_min + cal.range_max) / 2
        if not cal.range_min + 24 <= raw <= cal.range_max - 24:
            raise RuntimeError("Contact command outside calibrated encoder range")
        self.arm.write("Goal_Position", joint, round(raw), raw=True)

    def stroke(self, points, name):
        arm = self.arm
        pan, elbow = points[0]
        expected = self.height + SLOPE * (elbow - 51)
        travel = expected - 12
        if self.last_hit is not None:
            travel = min(travel, self.last_hit - 12)
        arm.move("shoulder_lift", travel, max_step=6, timeout=30)
        arm.move("elbow_flex", float(elbow), max_step=6, timeout=30)
        arm.move("shoulder_pan", float(pan), max_step=8, timeout=30)
        hit = touch(arm, expected, threshold=-60)
        initial = np.array([arm.position(j) for j in ("shoulder_pan", "elbow_flex")])
        command_height = float(arm.read("Goal_Position", "shoulder_lift"))
        bias = np.array([float(arm.read("Goal_Position", j)) - arm.position(j)
                         for j in ("shoulder_pan", "elbow_flex")])
        force_correction = 0.0
        samples = []
        goals = list(interpolate([initial, *points]))
        for index, goal in enumerate(goals):
            for attempt in range(24):
                commands = goal + bias
                height_goal = command_height + SLOPE * (goal[1] - initial[1]) + force_correction
                self.write_contact("shoulder_lift", float(height_goal))
                for joint, command in zip(("elbow_flex", "shoulder_pan"), commands[::-1]):
                    self.write_contact(joint, float(command))
                time.sleep(0.25)
                arm.health()
                loads = {j: arm.signed_load(j) for j in JOINTS}
                if max(map(abs, loads.values())) >= 250 or loads["shoulder_lift"] < -120:
                    raise RuntimeError(f"Drawing force stop: {loads}")
                actual = np.array([arm.position(j) for j in ("shoulder_pan", "elbow_flex")])
                # Slowly identify the holding offset under this pen load instead
                # of treating the encoder goal as the physical tip position.
                observed_bias = commands - actual
                bias = np.clip(0.6 * bias + 0.4 * observed_bias, [-1.5, -2.5], [1.5, 2.5])
                if loads["shoulder_lift"] > -48:
                    force_correction += 0.18 if loads["shoulder_lift"] >= 0 else 0.12
                elif loads["shoulder_lift"] < -76:
                    force_correction -= 0.12
                if abs(force_correction) > 3:
                    raise RuntimeError("Contact-plane correction exceeded three degrees")
                record = {"name": name, "index": index, "goal": goal.tolist(),
                          "actual": actual.tolist(), "loads": loads,
                          "force_correction": force_correction}
                arm.log("stroke_step", **record)
                samples.append(record)
                if (np.max(np.abs(actual - goal)) <= 0.45
                        and -100 <= loads["shoulder_lift"] <= -48):
                    break
            else:
                raise RuntimeError("Contact trajectory did not track its waypoint")
            if index % 20 == 0:
                print(f"DRAW {name} {index + 1}/{len(goals)} load={loads['shoulder_lift']}", flush=True)
        final_hit = arm.position("shoulder_lift")
        arm.lift(final_hit, clearance=12)
        self.last_hit = final_hit
        observed_height = final_hit - SLOPE * (actual[1] - 51)
        if abs(observed_height - self.height) > 4:
            raise RuntimeError("Measured drawing plane changed too far")
        self.height = 0.7 * self.height + 0.3 * observed_height
        time.sleep(1)
        arm.photograph(name)
        save_json(OUT / name / "samples.json", samples)
        print(f"STROKE COMPLETE {name}, plane={self.height:.2f}", flush=True)


def review(arm):
    print("REVIEW: enter continue within 180 seconds", flush=True)
    deadline = time.monotonic() + 180
    while time.monotonic() < deadline:
        arm.health()
        if select.select([sys.stdin], [], [], 0.25)[0]:
            if sys.stdin.readline().strip() != "continue":
                raise RuntimeError("Camera review stopped the run")
            return
    raise RuntimeError("Camera review watchdog expired")


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
        try:
            recover(arm)
            arm.target("wrist_flex", -98.98901098901099)
            arm.target("wrist_roll", -71.25274725274726)
            time.sleep(1)
            drawing = Outline(arm)
            # A held-out closed rectangle tests the new row axis and reversals.
            drawing.stroke([np.array(p) for p in [(24, 54), (28, 54), (28, 59), (24, 59), (24, 54)]], "box_check")
            review(arm)
            state = {"next_stroke": 0, "height": drawing.height, "total": len(PATHS)}
            save_json(OUT / "state.json", state)
            for index, path in enumerate(PATHS):
                drawing.stroke([grid_to_joints(p) for p in path], f"outline_{index + 1:02d}")
                state.update(next_stroke=index + 1, height=drawing.height)
                save_json(OUT / "state.json", state)
                if index == 1:
                    review(arm)
            arm.move("shoulder_lift", 0, max_step=6, timeout=45)
            arm.move("elbow_flex", 85, max_step=6, timeout=45)
            time.sleep(2)
            arm.photograph("finished")
            print("COMPLETED 7/7 continuous outlines; parked above paper", flush=True)
        except BaseException as error:
            arm.log("abort", error=str(error))
            print(f"ABORT: {error}", flush=True)
            raise
        finally:
            arm.close()


if __name__ == "__main__":
    main()
