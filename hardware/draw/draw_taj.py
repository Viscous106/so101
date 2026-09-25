"""Draw the Taj Mahal outline (hardware/draw/assets/taj_grid.npy) via raster scan: wrist_roll steps through
rows monotonically DEcreasing only (its only proven-reliable direction - see conversation, and ik.py's
elbow-guard comment for the same finding on elbow_flex), shoulder_pan sweeps columns within each row
(proven reliable both directions, with retry). Each "on" cell is a stamp (touch/mark/lift), matching
draw_shoulder_only.py's proven contact method. Never asks elbow_flex or wrist_roll to increase.

  draw_taj.py [--dry-run]
"""
import json, pathlib, sys, time
import numpy as np
from execute import load_bus, ARM_JOINTS
from draw_shoulder_only import setup, move_joint, descend_shoulder_lift, lift

ROW_STEP_DEG = 2.2
COL_STEP_DEG = 0.42
MAX_ROW_RETRIES = 2
REF_PATH = pathlib.Path(__file__).parent / "taj_reference.json"


def run(dry_run=False, start_row=0):
    grid = np.load("assets/taj_grid.npy")
    rows, cols = grid.shape
    print(f"grid: {rows} rows x {cols} cols, {grid.sum()} stamps")

    if dry_run:
        # pure plan printout, no hardware connection at all - not a partial-motion run
        for r in range(rows):
            cols_on = [c for c in range(cols) if grid[r, c]]
            if cols_on:
                print(f"row {r:2d}: wrist_roll step {r}, columns {cols_on}")
        print(f"total stamps: {grid.sum()}, rows used: {sum(1 for r in range(rows) if grid[r].any())}")
        return

    bus = load_bus(); bus.connect()
    try:
        base = setup(bus)
        if start_row == 0 or not REF_PATH.exists():
            pan0, roll0 = base["shoulder_pan"], base["wrist_roll"]
            REF_PATH.write_text(json.dumps({"pan0": pan0, "roll0": roll0}))
        else:
            # resuming after a crash: reuse the ORIGINAL reference, not current position - shoulder_pan is
            # wherever the last completed stamp left it (could be far from center), and re-deriving pan0 from
            # that would misalign every remaining row against the rows already drawn.
            ref = json.loads(REF_PATH.read_text())
            pan0, roll0 = ref["pan0"], ref["roll0"]
        print(f"reference: shoulder_pan={pan0:.2f} wrist_roll={roll0:.2f} (start_row={start_row})")

        done = int(grid[:start_row].sum())
        for r in range(start_row, rows):
            cols_on = [c for c in range(cols) if grid[r, c]]
            if not cols_on:
                continue
            roll_target = roll0 - r * ROW_STEP_DEG
            ok = move_joint(bus, "wrist_roll", roll_target)
            print(f"row {r:2d}/{rows}: wrist_roll -> {roll_target:.2f} {'ok' if ok else 'FAILED - skipping row'}")
            if not ok:
                continue
            # sweep left to right within the row - shoulder_pan is bidirectional-safe so order doesn't matter
            # for safety, only for efficiency; left-to-right keeps consecutive moves small.
            for c in cols_on:
                pan_target = pan0 + (c - cols / 2) * COL_STEP_DEG
                ok = move_joint(bus, "shoulder_pan", pan_target)
                if not ok:
                    print(f"    col {c}: shoulder_pan FAILED, skipping this stamp")
                    continue
                descend_shoulder_lift(bus)
                lift(bus, deg=6.0)
                done += 1
            print(f"  row {r} done ({len(cols_on)} stamps, {done}/{grid.sum()} total)")
        print(f"finished: {done}/{grid.sum()} stamps placed")
    finally:
        for j in ARM_JOINTS:
            bus.write("Torque_Enable", j, 0, normalize=False)
        bus.disconnect(disable_torque=False)
        print("torque off, disconnected")


if __name__ == "__main__":
    start_row = next((int(a.split("=")[1]) for a in sys.argv if a.startswith("--start-row=")), 0)
    run(dry_run="--dry-run" in sys.argv, start_row=start_row)
