"""Complete the reference with camera-verified rows and large pen-up travel."""
import fcntl
import json
import signal
import subprocess
import time

from calibration_console import recover
from draw_reference import Arm, JOINTS, ROOT, save_json
from test_continuous import touch


OUT = ROOT / "hardware/draw/runs/paper_taj_final"
SOURCE = ROOT / "hardware/draw/runs/pasteboard_taj_verified/plan.json"


class TravelArm(Arm):
    def move(self, joint, goal, tolerance=0.18, timeout=25, max_step=None):
        if max_step is None:
            max_step = 8 if joint == "shoulder_pan" else 6
            if joint == "shoulder_lift" and goal > self.position(joint):
                max_step = 3
        return super().move(joint, goal, tolerance=tolerance, timeout=timeout, max_step=max_step)


def main():
    def interrupt(signum, frame):
        raise KeyboardInterrupt(f"Signal {signum}")
    signal.signal(signal.SIGTERM, interrupt)
    if subprocess.run(["fuser", "/dev/ttyACM0"], capture_output=True).returncode == 0:
        raise RuntimeError("Serial bus already owned")
    OUT.mkdir(parents=True, exist_ok=True)
    plan = json.loads(SOURCE.read_text())
    points = sorted(plan["points"], key=lambda p: (p["row"], p["x"]))
    state_path = OUT / "state.json"
    state = json.loads(state_path.read_text()) if state_path.exists() else {"next": 0, "anchor": 59.8, "elbow": 51}
    if state["next"] == len(points):
        print("Already completed")
        return
    with open("/tmp/so101-drawing.lock", "w") as lock:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        arm = TravelArm(OUT)
        try:
            recover(arm)
            if not arm.clear:
                raise RuntimeError("Recovery did not establish a raised pose")
            arm.target("wrist_flex", -98.98901098901099)
            arm.target("wrist_roll", -71.25274725274726)
            time.sleep(1)
            arm.photograph("before")
            previous_row = None
            for index in range(state["next"], len(points)):
                point = points[index]
                elbow = 51 + point["row"] * 1.4
                pan = 16 + (point["x"] - 15) * 0.36
                anchor = state["anchor"] - 0.53 * (elbow - state["elbow"])
                if not 49 <= anchor <= 65:
                    raise RuntimeError("Paper height outside calibrated work envelope")
                row_change = point["row"] != previous_row
                if row_change:
                    # Use a full raised transfer and one approach direction to
                    # avoid the observed direction-dependent elbow offset.
                    arm.move("shoulder_lift", min(arm.position("shoulder_lift"), anchor - 12))
                    if previous_row is None:
                        arm.move("elbow_flex", elbow - 2)
                    arm.move("elbow_flex", elbow, tolerance=0.25)
                    arm.move("shoulder_pan", pan - 0.7, tolerance=0.25)
                arm.move("shoulder_pan", pan, tolerance=0.18)
                started = time.monotonic()
                hit = touch(arm, anchor, threshold=-60,
                            clearance=2 if row_change else 1.5, step=0.25)
                if abs(hit - anchor) > 3:
                    raise RuntimeError("Contact plane deviated beyond local search bound")
                contact_pose = {j: arm.position(j) for j in JOINTS}
                time.sleep(0.12)
                arm.lift(hit, clearance=12)
                state.update(next=index + 1, anchor=0.5 * anchor + 0.5 * hit, elbow=elbow)
                save_json(state_path, state)
                arm.log("final_stamp", index=index, point=point, contact_pose=contact_pose,
                        contact=hit, anchor=state["anchor"])
                print(f"MARK {index + 1}/{len(points)} row={point['row']} x={point['x']} "
                      f"contact={hit:.2f} seconds={time.monotonic()-started:.1f}", flush=True)
                row_done = index + 1 == len(points) or points[index + 1]["row"] != point["row"]
                if row_done or (index + 1) % 8 == 0:
                    time.sleep(0.6)
                    arm.photograph(f"after_{index + 1:03d}")
                previous_row = point["row"]
            arm.move("shoulder_lift", 0, max_step=6, timeout=45)
            arm.move("elbow_flex", 85, max_step=6, timeout=45)
            arm.move("shoulder_pan", 30, max_step=8, timeout=30)
            time.sleep(2)
            arm.photograph("finished")
            print("COMPLETED 119/119 marks", flush=True)
        except BaseException as error:
            arm.log("abort", error=str(error))
            print("ABORT", str(error), flush=True)
            raise
        finally:
            arm.close()


if __name__ == "__main__":
    main()
