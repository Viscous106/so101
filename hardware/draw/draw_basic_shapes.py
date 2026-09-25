"""Basic shape tests: a straight line, then a circle, using draw_simple's hybrid motion (IK for descent and
lateral moves - both proven reliable all day; shoulder_lift-only for lift-off - see draw_simple.py).

  draw_basic_shapes.py --dry-run
  draw_basic_shapes.py line
  draw_basic_shapes.py circle
  draw_basic_shapes.py            both, line then circle
"""
import sys
import numpy as np
from draw_simple import run

ANCHOR = (0.2401, -0.0535)   # current actual xyz at script-write time (read fresh via ik_check-style FK if stale)
LINE_LEN = 0.03
CIRCLE_R = 0.015
CIRCLE_N = 24

x0, y0 = ANCHOR
line = [[x0, y0], [x0 + LINE_LEN, y0]]
circle = [[x0 + CIRCLE_R + CIRCLE_R * np.cos(t), y0 + CIRCLE_R * np.sin(t)]
          for t in np.linspace(0, 2 * np.pi, CIRCLE_N + 1)]

SHAPES = {"line": [line], "circle": [circle]}

if __name__ == "__main__":
    args = [a for a in sys.argv[1:] if not a.startswith("--")]
    which = args[0] if args else None
    strokes = SHAPES[which] if which else [line, circle]
    run(strokes, dry_run="--dry-run" in sys.argv)
