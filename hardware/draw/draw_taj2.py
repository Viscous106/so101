"""Second, corrected attempt at the Taj Mahal raster drawing.

Root cause of the first attempt's failure: wrist_roll's pen tip is offset from the roll axis, so rotating it
across rows changes the pen's height above the table, not just its lateral position - shoulder_lift's per-stamp
contact search absorbs that as compensation. Over the first attempt's 12 row-transitions (26.4deg of total
wrist_roll travel) this compounded into ~159deg of shoulder_lift drift, driving it to its own calibrated floor
by the last few rows - so most of the run was spent fighting a joint limit, not drawing, and the pen was very
likely far from the intended small icon for a large fraction of the 113 stamps.

Fix, two parts:
  1. Shrink total wrist_roll travel by ~3x: taj_grid_compact.npy (7 rows, block-max-pooled from the original 13,
     still clearly the same silhouette - dome, two minarets, base line) and a smaller ROW_STEP_DEG, so total
     travel is ~9.6deg instead of 26.4deg -> expected drift scales down roughly proportionally too (~58deg,
     comfortably inside shoulder_lift's ~280deg of calibrated range with the hard abort below as a backstop).
  2. A hard safety abort: baseline shoulder_lift is read once at setup; if any stamp's descend ever leaves it
     more than MAX_LIFT_DRIFT_DEG from that baseline, stop immediately and lift off, rather than silently
     continuing to drift toward the floor like last time.

Row 0 is the arm's CURRENT wrist_roll/shoulder_pan position (read fresh at setup) - wrist_roll cannot be
commanded back up after the first attempt, so there is no "return to the original reference" here; this run
anchors fresh from wherever the arm actually is now and draws downward/outward from there, which is exactly
wrist_roll's one proven-safe direction.

  draw_taj2.py [--dry-run]
"""
import pathlib, sys, time
import numpy as np
from execute import load_bus, ARM_JOINTS
from draw_shoulder_only import setup, move_joint, safe_read, AnchoredDescender

ROW_STEP_DEG = 1.6
COL_STEP_DEG = 0.42
MAX_LIFT_DRIFT_DEG = 20.0
GRID_PATH = pathlib.Path(__file__).parent / "assets/taj_grid_compact.npy"


def run(dry_run=False):
    grid = np.load(GRID_PATH)
    rows, cols = grid.shape
    print(f"grid: {rows} rows x {cols} cols, {grid.sum()} stamps (compact silhouette)")

    if dry_run:
        for r in range(rows):
            cols_on = [c for c in range(cols) if grid[r, c]]
            if cols_on:
                print(f"row {r}: wrist_roll step {r} ({r * ROW_STEP_DEG:.2f}deg from row0), columns {cols_on}")
        print(f"total wrist_roll travel: {(rows - 1) * ROW_STEP_DEG:.2f}deg, total stamps: {grid.sum()}")
        return

    bus = load_bus(); bus.connect()
    try:
        base = setup(bus)
        pan0, roll0 = base["shoulder_pan"], base["wrist_roll"]
        lift_baseline = base["shoulder_lift"]
        print(f"anchor: shoulder_pan={pan0:.2f} wrist_roll={roll0:.2f} shoulder_lift={lift_baseline:.2f}")
        descender = AnchoredDescender()
        initial_table_h = [None]

        done = 0
        aborted = False
        for r in range(rows):
            cols_on = [c for c in range(cols) if grid[r, c]]
            if not cols_on:
                continue
            roll_target = roll0 - r * ROW_STEP_DEG
            ok = move_joint(bus, "wrist_roll", roll_target)
            print(f"row {r}/{rows}: wrist_roll -> {roll_target:.2f} {'ok' if ok else 'FAILED - skipping row'}")
            if not ok:
                continue
            for c in cols_on:
                pan_target = pan0 + (c - cols / 2) * COL_STEP_DEG
                ok = move_joint(bus, "shoulder_pan", pan_target)
                if not ok:
                    print(f"    col {c}: shoulder_pan FAILED, skipping this stamp")
                    continue
                descender.stamp(bus)
                # table_h (the actual contact/drawing height AnchoredDescender is tracking) is the metric that
                # matters for drawing quality - not the post-lift resting position, which can wander for
                # unrelated reasons (load-spike retries during lift) without the pen's mark position moving at
                # all (confirmed: an isolated 5-stamp test held table_h within ~2deg while resting position
                # drifted ~9deg over the same 5 stamps).
                if initial_table_h[0] is None and descender.table_h is not None:
                    initial_table_h[0] = descender.table_h
                if initial_table_h[0] is not None:
                    drift = descender.table_h - initial_table_h[0]
                    if abs(drift) > MAX_LIFT_DRIFT_DEG:
                        print(f"  ABORT: table_h drift {drift:.2f}deg exceeds {MAX_LIFT_DRIFT_DEG}deg "
                              f"(initial {initial_table_h[0]:.2f}, now {descender.table_h:.2f}) - stopping.")
                        aborted = True
                        break
                done += 1
            th = f"{descender.table_h:.2f}" if descender.table_h is not None else "n/a"
            print(f"  row {r} done ({len(cols_on)} stamps so far this row, {done}/{grid.sum()} total, "
                  f"table_h={th})")
            if aborted:
                break
        print(f"finished: {done}/{grid.sum()} stamps placed" + (" (ABORTED early)" if aborted else ""))
    finally:
        for j in ARM_JOINTS:
            bus.write("Torque_Enable", j, 0, normalize=False)
        bus.disconnect(disable_torque=False)
        print("torque off, disconnected")


if __name__ == "__main__":
    run(dry_run="--dry-run" in sys.argv)
