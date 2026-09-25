"""Draw a small 'L' (two 2cm legs, +X and +Y from an anchor point) to validate the whole pipeline before
risking the full portrait: coordinate scale, pen-down/pen-up sequencing, contact quality, and (by looking at
which physical direction each leg is drawn in) which way +X and +Y actually point on the paper.

  draw_test_shape.py --dry-run     IK reachability check only, no motion
  draw_test_shape.py               real motion
"""
import sys
import numpy as np
from execute import run_strokes

ANCHOR = (0.2383, -0.0530)   # last confirmed real xyz from find_table_z.py's fine pass, same pose/frame
LEG_M = 0.02

x0, y0 = ANCHOR
strokes = [
    [[x0, y0], [x0 + LEG_M, y0]],   # +X leg
    [[x0, y0], [x0, y0 + LEG_M]],   # +Y leg
]

if __name__ == "__main__":
    result = run_strokes(strokes, dry_run="--dry-run" in sys.argv)
    print(result)
