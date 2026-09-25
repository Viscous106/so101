---
name: so101-draw
description: Make the physical SO-101 (Feetech STS3215) arm draw a requested image on paper via LeRobot - reference image to raster grid to stamped marks, with camera verification. Use for any "make the arm draw X" task on this rig, and read it BEFORE touching /dev/ttyACM0, because it encodes hardware faults, thermal limits and failure modes that are expensive to rediscover.
---

# Drawing with the SO-101 arm

This skill encodes what it actually takes to get marks on paper with **this specific arm**. Much of it is
counter-intuitive and was learned by breaking things. Read "Hard-won facts" before writing any motion code.

**Honest status:** the pipeline below produces individual marks reliably and holds contact height stable. A
complete, recognizable multi-row picture had **not** been finished end-to-end at the time this skill was
written. The blockers are documented in "Known-unfinished" — do not assume a clean run.

## The rig

| Thing | Value |
|---|---|
| Serial port | `/dev/ttyACM0` @ 1000000 baud, motor ids 1-6, model 777 (STS3215) |
| Python | `/home/machine/so101-yash/.venv/bin/python` (always `-u`, see below) |
| Calibration | `~/.cache/huggingface/lerobot/calibration/robots/so_follower/lab_follower.json` |
| URDF | `hardware/urdf/so-arm100/Simulation/SO101/so101_new_calib.urdf` |
| Wrist camera (first-person) | `/dev/video0` |
| Overhead-ish camera (third-person) | `/dev/video2` (Logitech C270, mounted very close — arm dominates frame) |
| Camera helper | `hardware/repeat/cam.py` → `Cam(dev, exposure=...)`, `.auto_gain()` |
| Read-only diagnostic | `hardware/probe.py` — pos/goal/limits/torque/load/V/**temp**/moving for all 6 |
| Guarded manual jog | `hardware/arm.py nudge <joint> <deg>` (clamped ≤5°/call), `relax`, `read` |

Joint ranges from calibration (raw counts):

```
shoulder_pan   873-2932      wrist_flex   969-3291
shoulder_lift  930-3298      wrist_roll   0-4095  (unrestricted)
elbow_flex     755-2983      gripper      1586-3138
```

## Non-negotiable safety rules

1. **One process owns the bus.** A second process reading `/dev/ttyACM0` while another holds it produces
   `ConnectionError` / corrupted packets and kills the run. Never run `probe.py` while a drawing script is live.
   Check with `ps aux | grep python` before connecting.
2. **Always `finally:` torque off + disconnect.** Every script. A crash must not leave a joint powered against a
   stall — that is how motors cook.
3. **Watch `Present_Temperature` on any joint under sustained load** and abort above ~55°C. Observed for real:
   `shoulder_lift` reached **58°C** fighting gravity in a sprawled pose. Torque off and let it cool (it returns
   to ~35°C in a couple of minutes). STS3215 tolerate ~70°C, so 58°C is a warning, not damage — but it is the
   point to stop.
4. **Check joints are clear of their limits before starting.** See the gravity-sag failure below; starting from a
   limit-pinned pose silently corrupts everything downstream.
5. **`python -u` for anything backgrounded.** Python block-buffers stdout to a file, so a `nohup ... > log` run
   shows an **empty log** while the script is alive. This produced a multi-minute "nothing is happening" stall
   where neither the log nor a Monitor could see that the script was grinding against a joint limit. Without
   `-u` you are flying blind.
6. **Stop a stuck run with `kill -INT`, not `kill`/`-9`.** SIGINT raises `KeyboardInterrupt`, which unwinds
   through `try/finally` and actually disables torque. SIGTERM/SIGKILL skip the `finally` and can leave torque on.

## Hard-won facts (read before coding)

### This arm has permanent one-direction motor faults

`elbow_flex` and `wrist_roll` **cannot be commanded to increase.** Signature: instantly reports near-max
`Present_Load` (~980-1004) with `Present_Current` ≈ 0-1 and `Moving=1`, and **zero actual position change.**

Ruled out by testing: software, mechanical jam (both move freely by hand), `Torque_Limit`, and position
dependence (reproduced at multiple positions and torque limits). This is a dead H-bridge leg on one side.
**Decreasing always works cleanly.** Design around it — never command the broken direction, and guard IK
output (`ik.py` rejects any solution that increases `elbow_flex`).

`wrist_flex` once failed in both directions at one position, but later moved 36° → -92.75° in 5° steps with
loads of only 28-60. Treat it as **usable but verify**, not as faulted.

`shoulder_pan` is bidirectional and reliable, but hits transient serial glitches — needs genuine retry.
`shoulder_lift` is bidirectional but fights real gravity load and shows classic P16/I0 steady-state shortfall;
it needs `Torque_Limit` 600-900 and incremental stepping.

### Three different "degree" conventions — never mix them

1. **Raw counts** 0-4095 (what `probe.py` prints as `pos=`)
2. **`arm.py` "deg from centre"** — raw offset by homing, what `probe.py` prints in parentheses
3. **LeRobot `MotorNormMode.DEGREES`** — calibration-derived, what the drawing scripts read/write

The same physical pose reads as `pos=2706 (~57.8 deg)` in `probe.py` and `55.65` in a drawing script. Always
state which convention a number is in. Most confusion in this project traced back to this.

### IK: `inverse_kinematics()` is one solver step, not a solve

`lerobot.model.kinematics.RobotKinematics.inverse_kinematics()` is built for a realtime loop where the target
moves slightly each tick. A single call from a seed 15° off barely moves. You **must** iterate, feeding output
back as the next seed, until the FK position converges — `ik.py::solve_ik()` does this (~5 iterations to <1mm).

Use **position-only IK** (`orientation_weight=0.0`); it converges far more robustly on this arm than
orientation-locked IK.

### Stamp, never drag

Lateral motion while the pen touches paper **jams on static friction** — confirmed repeatedly. The working
primitive is: touch down → mark → lift → move → touch down again. One stamp per point. This applies to
rotational drag (`wrist_roll` while in contact) too.

### Contact detection is a cliff, so back off after tripping it

Load goes from ~0 to ~1000 within a fraction of a degree. Because load is only sampled every ~50ms, detection
*always* overshoots into a hard press. A hard press then jams any subsequent move. So: step finely (0.15°),
and after the load trips, **back off ~0.35°** to settle into a light touch.

**Caveat — the backoff may not actually be moving the joint.** In the valid 5-stamp paper test, every
post-backoff reading came out *above* the hit position (`contact at 52.04 -> backed off to 52.22`, when the
commanded target was 51.69). A 0.35° command is small enough to vanish into the P16/I0 steady-state shortfall.
Treat the light-touch mechanism as unproven, and if marks come out too heavy, suspect this first.

### The per-stamp drift bug (the big one)

**Symptom:** over a long run, `shoulder_lift` walks away in one direction — 159° across 113 stamps in the first
attempt, ending pinned at its own calibrated floor, so most of the run was fighting a joint limit instead of
drawing.

**Wrong diagnosis (cost a whole rebuild):** blaming `wrist_roll` row-transition geometry. Reducing row count
from 13→7 did **not** fix it, because the drift was ~2.7-2.9° **per stamp**, including within a single row
where `wrist_roll` never moved.

**Real cause:** the stamp cycle was *relative*. `descend` searched upward from wherever the previous `lift`
happened to leave the joint, and `lift`'s fixed -6° is not a true inverse of whatever `descend` did. Small
per-stamp errors compound without bound.

**Fix — `AnchoredDescender` in `draw_shoulder_only.py`:** remember `table_h`, the **absolute** height where
contact was last found. Before every search, jump directly to `table_h - clearance` (absolute move), then do a
short local search. A single stamp's inaccuracy can no longer leak into the next one. Real height changes are
absorbed via a damped EMA update of `table_h`, with fallback to a full-range search if the local search misses.

**Evidence so far — weaker than it looks, re-verify before trusting it.** The only valid measurement is an
isolated 5-stamp test on real paper: `table_h` went 52.04 → 50.09, i.e. ~2° total (~0.4°/stamp) versus
~2.8°/stamp with the old relative cycle. That is a real improvement but a small sample.

Do **not** be reassured by the later 7-row run that appeared to hold `table_h` within ~0.85° over 11+ stamps —
that run was detecting a **joint limit**, not paper (see the gravity-sag failure below), and a hard mechanical
stop is trivially stable. `AnchoredDescender` has **not** yet been validated across a long multi-stamp run on
actual paper.

**Corollary — measure the right thing.** Track drift of `table_h` (the actual mark height). Do **not** use the
post-lift resting position as your abort metric: it wanders ~9° over 5 stamps for unrelated reasons (load-spike
retries during lift) while the pen's mark position does not move at all. Aborting on it gives false alarms.

### Gravity sag between runs will corrupt your baseline

With torque off the arm is limp and **sags under gravity**, potentially pinning joints against their limits.
This is normal, not a fault — but it caused a nasty failure:

1. Arm sagged while torque was off; `shoulder_lift` came to rest at raw 941 (floor is 930).
2. Next run's `setup()` read that pose as its baseline.
3. The first contact search immediately "found contact" — actually the **joint limit**, not paper.
4. `table_h` anchored to that bogus value; every later stamp tried to pre-position 2.5° *below* the hard limit.
5. Result: endless load spikes, no motion, empty buffered log, an apparently hung script.

Tell-tale in the log: `contact at -103.12 -> backed off to -103.12` — **backoff did not change position**,
which is impossible on real paper. Also `elbow_flex` at 97° and `wrist_flex` at 36° when the working pose is
~52° and ~-92°: that sprawl is what put a huge moment arm on `shoulder_lift` and overheated it.

**Always re-establish posture before drawing** (see pre-flight), and refuse to start if any joint is within a
few degrees of a calibrated limit.

### Restoring a collapsed arm: fold before you lift

If the arm has sprawled, do **not** just drive `shoulder_lift` up — it will fight the full moment arm and
overheat. Order to try (only step 2 is actually confirmed; see caveats below):

1. `elbow_flex` → working value (this is a **decrease**, its good direction)
2. `wrist_flex` → working value (also a decrease; moved cleanly at loads 28-60)
3. `shoulder_lift` → working value, now much lighter

Use ~3-5° steps, `Torque_Limit` 900, and log load and temperature every step.

**What was actually observed:** only step 2 completed — `wrist_flex` moved 36° → -92.75° cleanly at loads of
28-60. Step 1 `elbow_flex` **stalled after 2.5°** (97.49 → 94.95) at load=124, and step 3 `shoulder_lift`
recovered +107° but then **overheated at 58°C**, plausibly because elbow was still sprawled and the moment arm
was still huge.

**Untested hypothesis:** load=124 is far too low for a real jam, so the `elbow_flex` stall may just be a move
slower than the 4s settle window allowed, which would mean a settle window of 8s+ fixes it. **This was never
verified** — try it, but do not assume it.

### Other traps

- **`move_joint` returns `True` even when it never converged** (implicit `return True` after the outer loop).
  Do not trust its return value as proof of arrival — re-read position if it matters.
- **`Torque_Limit` (RAM, addr 48) cannot exceed `Max_Torque_Limit` (EEPROM, addr 16).**
- **Wrap every read in retry** (`safe_read`): a dropped packet raises "There is no status packet!" and a single
  glitch should not kill a multi-hour run. One did, 2 stamps into a 131-stamp run.
- Beware bulk regex edits: replacing `bus.read(` → `safe_read(bus, ` once rewrote `safe_read`'s own body into
  infinite recursion.
- Make long runs **resumable** (`--start-row=N` plus a persisted reference JSON), and on resume reuse the
  **original** anchor — re-deriving it from the current pose misaligns finished rows against remaining ones.

## Workflow

### 1. Reference image → coarse binary grid

The user supplies a rough reference ("a Taj Mahal should look roughly like this sketch"). Exact reproduction is
not the goal; a recognizable silhouette is.

A thin, noisy hand-sketch rasterizes badly at coarse resolution. What worked: draw a **clean programmatic
silhouette** (`cv2.rectangle/ellipse/circle`), run **Canny** to get an outline, then rasterize that to a small
binary grid. Keep it coarse — every "on" cell is a separate physical stamp.

Sizing: `COL_STEP_DEG = 0.42` ≈ **1.8 mm** per column at ~250 mm reach, so 30 columns ≈ **5.4 cm** wide.
A 7×30 grid ≈ 91 stamps; a 13×30 ≈ 131. Budget real time per stamp (descend + settle + lift).

Downsample rows with block-max-pooling (`g[a:b].any(axis=0)`) — it preserves a silhouette far better than
nearest-neighbour sampling. Always print the grid as ASCII and eyeball it before running:

```
..............##..............
..###......########......###..
..#.#.....##......###....#.#..
..#.#...###.######.###...#.#..
..#.#...#............#...#.#..
.##.#####............#####.###
.#############################
```

### 2. Choose axes

Rows and columns must map to joints that can actually move that way:

- **Columns:** `shoulder_pan` — bidirectional, reliable with retry. Good.
- **Rows:** this is the hard one. `wrist_roll` was used because it only ever needs to *decrease*, but it is a
  poor choice: the pen tip is offset from the roll axis, so rolling changes pen **height**, forcing
  `shoulder_lift` to compensate; and because its increase direction is dead, **once you leave a row you can
  never return to it** — no re-inspection, no touch-up, ever. Prefer a bidirectional axis for rows if you can
  find one, and keep total row travel small regardless.

### 3. Pre-flight (do not skip)

```
ps aux | grep python            # nobody else on the bus
probe.py                        # torque=0, temps <40C, and NO joint near its raw limit
```

If any joint sits near a limit or the pose is sprawled, restore posture first (fold before lift, above).
Then verify the pen actually reaches paper with one manual stamp. `contact at X -> backed off to X`
(**identical** values) means you are detecting a **joint limit**, not paper — stop and fix posture. Differing
values are consistent with real paper but do **not** prove the backoff worked (see the caveat above); also
sanity-check that the contact height sits well inside the joint's range, not within a few degrees of a limit.

### 4. Run it

- `python -u script.py > log 2>&1 &`, then Monitor the log with a filter that also matches failure signatures
  (`ABORT|FAILED|Traceback|giving up|no contact`), not just progress lines. A filter that only matches success
  is silent during a crash-loop, which looks identical to "still running."
- Include a **hard abort** on `table_h` drift (~20°) so a runaway cannot repeat the 159° episode.
- Make it resumable.

### 5. Verify with the cameras

This is part of the task, not optional — check progress from the cameras and debug if there is no progress.

`/dev/video2` needs real exposure control to show pencil marks; use `Cam("/dev/video2", exposure=75)` +
`auto_gain()` rather than a raw `cv2.VideoCapture` grab. Be aware the third-person camera is **fixed** — it
shows one region of the table regardless of where the pen is, so "I don't see the drawing" may mean the pen
was never in that region, not that nothing was drawn. Cross-check with the wrist camera `/dev/video0`.

## Known-unfinished / next steps

- **Row axis needs redesign.** `wrist_roll`'s dead increase direction makes rows one-way and couples row
  position to pen height. This is the main structural obstacle to a clean multi-row picture.
- **Posture restoration was mid-flight** when work last stopped: `wrist_flex` restored (-92.75°),
  `shoulder_lift` recovered to ~32° (clear of its floor), `elbow_flex` still sprawled at ~91-95° versus its
  working ~52°. The next thing to try is a longer settle window, but that is an untested hypothesis — if it
  stalls again at a low load, the cause is still unknown.
- A full recognizable drawing has not yet been confirmed on paper via camera.

## File map

| File | Role |
|---|---|
| `draw/ik.py` | `make_kinematics()`, `solve_ik()` (iterated; rejects `elbow_flex` increases) |
| `draw/execute.py` | `load_bus()`, `ARM_JOINTS`, interpolated Cartesian motion |
| `draw/draw_shoulder_only.py` | Core primitives: `safe_read`, `setup`, `move_joint`, `descend_shoulder_lift`, `lift`, **`AnchoredDescender`** |
| `draw/draw_taj2.py` | Raster driver: grid → rows/columns → stamps, with drift abort |
| `probe.py` | Read-only state/temperature check |
| `arm.py` | Guarded manual jog |
| `repeat/cam.py` | `Cam` with exposure + `auto_gain()` |
