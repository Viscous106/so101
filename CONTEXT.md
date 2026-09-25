# SO-101 Drawing Robot — Full Context

*Written for an assistant with zero prior knowledge of this project. Everything needed is in this file.
Last updated: 2026-09-23.*

## Latest status: power-cut recovery completed (2026-09-23)

`hardware/draw/runs/paper_taj_final/state.json` is now **119/119**. The
controller resumed from the surviving 90-mark checkpoint, finished the remaining
marks, parked away from the artwork, saved a photo, and disabled torque. This is
a completed rough, slanted/dashed Taj outline, NOT an accurate reproduction or a
validated sim-to-real controller. The earlier incomplete-run descriptions below
are historical.

Evidence in `hardware/draw/runs/paper_taj_final/finished/`:
- `desk.jpg`: unobscured original camera photo before torque-off.
- `upright_photo.jpg`: rotated, enlarged and brightness-adjusted crop of that
  real photo; no generated drawing, perspective correction or retouching.
- `torque_off_desk.jpg`: confirms the sagged pen rests outside the artwork.

A final independent read-only probe found all six motors at torque 0, temperatures
33-35 C, and raw positions [2248, 2640, 2652, 1007, 1237, 1670]. The camera watcher
was stopped and the serial bus released. Always re-check the live pose and scene
before any future motion; do not assume this parked pose persists.

The reboot swapped the numbered cameras. Drawing capture and monitoring now use
stable USB identities in `hardware/draw/camera_devices.py`: Logitech 046d:0825 is
the desk camera and Sonix USB2.0_CAM1 is the wrist camera. At this boot they are
`/dev/video0` and `/dev/video2`, respectively, the reverse of the historical table.
The machine account was added to the video group for subsequent logins; narrowly
scoped device ACLs restored access in the current session. Both fresh views were
checked before torque-on.

At mark 94, one shoulder temperature read of 57 C triggered a full torque-off
abort. Two immediate independent probes and 30 additional read-only sweeps all
reported shoulder 34 C and other motors 33-34 C, with all torque off. The run was
then resumed; the 55 C cutoff was never raised or filtered away. The cause of the
isolated high reading remains unconfirmed. Power loss also left NUL bytes at the
old telemetry tail; retain that evidence rather than treating the whole JSONL as
valid without checking it.

`save_json()` now fsyncs checkpoint contents and the containing directory around
atomic replacement. This update applies to subsequent processes; the running
controller had already imported its earlier implementation. The final checkpoint
was re-saved with the durable implementation after completion. All 18 offline
tests pass, including checkpoint failure-path tests.

The user asked about running multiple simulation instances. This machine exposes
20 logical CPUs and about 16 GB RAM; 4-8 headless workers are a starting point to
benchmark, not a measured speedup. No parallel simulation workers were launched.
`rehearse_reference.py` remains an OFFLINE KINEMATIC model, not calibrated motor or
pen-contact dynamics. Useful parallel fitting must score candidates against real
camera/joint measurements and reserve held-out tests. Hardware must retain one
serial owner, and simulation success alone must never authorize a real trajectory.

## Correction from the 2026-09-23 camera-verified retest

**Read this before the historical findings below.** The installed LeRobot
`FeetechMotorsBus.read()` decodes sign-magnitude even with `normalize=False`.
Masking its `Present_Load` result with `& 0x3FF` is incorrect: `-20` becomes
`1004`, and `-28` becomes `996`. Several older drawing scripts, and the original
interpretation in sections 4.1 and 4.6, used that incorrect mask. Use
`abs(bus.read("Present_Load", joint, normalize=False))` for magnitude and retain
the signed value when evaluating contact. Do not treat the old apparent
near-1000 readings as measured overloads.

Small guarded tests with the pen lifted showed `elbow_flex` increasing by
1.407 degrees and `wrist_roll` increasing by 1.670 degrees, with actual loads
0 and 20 respectively. The previously asserted permanent one-direction faults
are not supported by these tests. This does not certify every position or load.

The new `hardware/draw/draw_reference.py` uses signed loads, measured-position
compensation for gravity undershoot, verified clearance before lateral moves,
55 C temperature aborts, persistent checkpoints, and torque-off cleanup.
Three contact tests at shoulder-pan 12, 12.84, and 13.68 degrees found contact
at shoulder-lift 62.330 degrees and lifted to 56.00-56.18 degrees. New small
marks were visible beside the old scribble. Contact was detected by sustained
signed shoulder load <= -60 in this posture, with an absolute-load stop at 300;
these thresholds are posture-specific, not universal force measurements.

Evidence and the original failed attempts are in
`hardware/draw/runs/pasteboard_taj/`; the requested 119-point outline is in
`hardware/draw/runs/pasteboard_taj_verified/`. Check the latter's checkpoint
and photos for actual completion. The old text below is historical context,
not proof that a full drawing has finished.

### Latest pause and geometry finding (2026-09-23)

**Later autonomous update:** The user explicitly requested no further help or
confirmation questions and larger steps. Both camera views were checked and a
bounded inward elbow recovery succeeded. `calibration_console.py` owns the bus
exclusively and provides monitored recovery/calibration primitives. Its final
8-degree command guard aborted a requested 8.11-degree return, leaving torque
off; this was a software travel bound, not a motor fault.

Camera calibration produced distinct pan/elbow displacements. A two-degree
continuous pan stroke made ink with true loads below 60, but subsequent closed
two-axis strokes did NOT produce a reliable closed shape. A larger continuous
test stalled short of an elbow waypoint under contact (actual 57.14 degrees,
target 57.82, load -156), and stopped. Do not infer that continuous full-image
drawing is validated. A single temperature read of 102 C also caused an abort;
immediate independent probes and three read-only follow-ups were all 33-35 C,
with all torque off. The temperature cutoff was not removed or raised.

`draw_final_outline.py` is the current full-picture attempt. It uses pan for
columns and elbow for rows, fixed wrist goals, large bounded PEN-UP travel,
12-degree verified lifts, a local contact search at signed load <= -60, and
same-direction row approaches. Output/checkpoint:
`hardware/draw/runs/paper_taj_final/`. Check `state.json` and camera photos for
actual completion; do not assume the 119 marks have finished. The older
`continuous_taj` and `pasteboard_taj_verified` runs are incomplete tests.

The old request to wait for setup confirmation below has been superseded by
the user's explicit autonomous-operation instruction and the subsequent
camera-reviewed recovery. Fresh pose/scene checks are still required.

The verified run stopped at **43/119 stamps**, after the baseline and the first
upper row. Its 90-second camera-review timeout disabled all five arm motors and
disconnected. It is NOT a completed drawing. Do not resume the old wrist-roll
mapping just to finish the counter: camera review showed too little row spacing.

The user supplied a prior operator's camera-calibration/simulation example:
`https://www.pasteboard.co/ZY4qc9l1KNuq.png`. The screenshot discusses modelling
an insecure pen mount and rehearsing trajectories, not proof of this setup's
calibration. MuJoCo 3.13.0 and a SO101 XML model already exist locally.

`hardware/draw/rehearse_reference.py` now compares the old mapping with a
pan/elbow/height-compensated candidate, without opening hardware. With a zero
assumed pen offset, the nominal model predicts roughly 58.7 x 2.0 mm for the
old mapping versus 54 x 22 mm for the candidate. Two other assumed pen offsets
also predict a very flat old drawing. These are **kinematic sensitivity tests**,
NOT fitted contact dynamics, measured millimetres, or a calibrated digital twin.
Outputs explicitly prohibit hardware execution until camera calibration and
held-out real tests pass. See `hardware/draw/runs/mujoco_rehearsal/comparison.png`.

After the pause, both the live camera view and the arm pose changed greatly.
A read-only probe found raw positions [2136, 2100, 2981, 1080, 1028], all arm
torques off, temperatures 33-37 C. Elbow raw 2981 is only two counts from its
2983 upper limit. The assistant asked whether the arm/camera/paper had been
repositioned and whether the arm was resting safely. **Wait for that answer and
re-check the scene before any further motion.** The old drawing baseline and
camera pixel coordinates are no longer safe assumptions. `Arm.setup()` should
reject this pose; do not bypass its guard to start a drawing.

Next: establish a safe folded pose, collect bounded pen-tip/contact measurements
in the camera frame, validate return-to-target repeatability and a small 2D
shape, then fit/rehearse a fresh drawing. Two boxes alone do not identify a
general perspective mapping; use additional non-collinear samples and held-out
targets. No ruler is needed for pixel-space accuracy, but do not claim metric
accuracy without a measured or otherwise validated scale.

---

## 1. What we are trying to do

We have a **physical 6-DoF robot arm** (SO-101, with Feetech STS3215 servos) on a desk, holding a **pen**, with
a sheet of paper under it and **two USB cameras** watching. The goal:

> Give the robot a drawing task ("draw the Taj Mahal"). It finds/derives a simple reference image, draws that
> picture on the paper, and **uses the cameras to check its own progress** — if nothing is appearing, it must
> debug and fix it itself, then finish the drawing.

The user's exact framing: *"you will be given some task to draw and you have to take a reference image of what
I asked you to draw and you make the lerobot draw that thing by tracking the progress via the cameras till the
end."* An exact photographic reproduction is **not** wanted — a recognizable simple line drawing is the target
(for the Taj Mahal: dome, two minarets, base platform).

Longer-term ambition: extend to the "seven wonders" once one drawing works reliably.

The user has asked for **autonomous operation** — figure things out, debug, iterate, don't stop to ask
permission for each step.

**Historical bottom line at the time of this section:** individual marks worked,
but no complete drawing had been produced. See the latest status above for the
completed rough outline and the remaining accuracy limitations.

---

## 2. The physical setup

| Item | Detail |
|---|---|
| Arm | SO-101, 6 joints: `shoulder_pan`, `shoulder_lift`, `elbow_flex`, `wrist_flex`, `wrist_roll`, `gripper` |
| Servos | Feetech STS3215 (`model=777`), ids 1-6 in the order above |
| Serial bus | `/dev/ttyACM0` @ 1000000 baud |
| Control library | LeRobot (`SO101Follower`, `FeetechMotorsBus`, `MotorNormMode.DEGREES`) |
| Python | `/home/machine/so101-yash/.venv/bin/python` |
| Calibration file | `~/.cache/huggingface/lerobot/calibration/robots/so_follower/lab_follower.json` |
| URDF | `hardware/urdf/so-arm100/Simulation/SO101/so101_new_calib.urdf` |
| Wrist camera | `/dev/video0` (first-person, looks along the arm) |
| Third-person camera | `/dev/video2` (Logitech C270, mounted very close — the arm fills much of the frame) |
| Repo root | `/home/machine/so101-yash` |

The pen is held in the gripper. The paper is on the desk. A ruler and a notebook are also on the desk and show
up in camera shots.

**Joint limits (raw servo counts, from the calibration file):**

```
shoulder_pan   873-2932        wrist_flex   969-3291
shoulder_lift  930-3298        wrist_roll   0-4095   (effectively unrestricted)
elbow_flex     755-2983        gripper      1586-3138
```

---

## 3. CRITICAL — read before running anything

1. **Only one process may touch `/dev/ttyACM0` at a time.** A second process reading while another holds the
   bus causes `ConnectionError` / corrupted packets and kills the run. Always `ps aux | grep python` first.
   Never run the diagnostic `probe.py` while a drawing script is live.
2. **Every script must have `finally:` that disables torque and disconnects.** A crash leaving a joint powered
   into a stall is how motors get cooked.
3. **Watch `Present_Temperature` and abort above ~55°C.** This happened for real: `shoulder_lift` hit **58°C**
   fighting gravity. Torque off, and it returns to ~35°C in about two minutes. STS3215 tolerate ~70°C, so 58°C
   was a warning, not damage — but that is the point to stop.
4. **Always run background scripts with `python -u`.** Python block-buffers stdout to a file, so a
   `nohup script.py > log` run shows a **completely empty log** while running. This cost us several minutes of
   staring at a blank log while the script was silently grinding against a joint limit.
5. **Stop a stuck script with `kill -INT <pid>`, never `kill -9`.** SIGINT raises `KeyboardInterrupt`, which
   unwinds through `try/finally` and actually turns torque off. SIGKILL skips it and leaves torque on.
6. **The arm goes limp and sags when torque is off.** This is normal, not a fault — but it means the pose you
   left it in is *not* the pose you'll find. Always re-read position at the start of a run, and never assume a
   stored baseline is still valid.

---

## 4. Hard-won findings (the expensive knowledge)

### 4.1 Two joints have permanent one-direction motor faults

`elbow_flex` and `wrist_roll` **cannot be commanded to increase.**

Signature: instantly reports near-max `Present_Load` (~980-1004) with `Present_Current` ≈ 0-1 and `Moving=1`,
but **zero actual position change**.

Ruled out by testing: software bugs, mechanical jam (both move freely by hand), `Torque_Limit` settings, and
position dependence (reproduced at many positions and torque limits). Conclusion: a dead H-bridge leg in the
motor driver. **Decreasing always works cleanly.**

Consequences: design so these joints only ever decrease. `ik.py` actively rejects any IK solution that
increases `elbow_flex`. And note that because `wrist_roll` can't go back up, **any scan that uses it as an axis
can never revisit an earlier position** — no touch-ups, no re-inspection.

`wrist_flex` once appeared to fail in both directions, but later moved 36° → -92.75° cleanly at loads of 28-60.
Treat it as **usable but verify**, not faulted.

`shoulder_pan` is bidirectional and reliable, but gets transient serial glitches — needs retry logic.
`shoulder_lift` is bidirectional but carries the gravity load and has a steady-state shortfall (see 4.3).

### 4.2 Three incompatible "degree" conventions — the #1 source of confusion

1. **Raw counts** 0-4095 — what `probe.py` prints as `pos=`
2. **`arm.py` "deg from centre"** — what `probe.py` prints in parentheses
3. **LeRobot `MotorNormMode.DEGREES`** — calibration-derived, what the drawing scripts read and write

The *same physical pose* reads as `pos=2706 (~57.8 deg)` in `probe.py` and `55.65` in a drawing script. Always
state which convention a number is in. Most confusion in this project traced back to mixing these up.

### 4.3 Servos undershoot every command (steady-state error)

A commanded move consistently falls short — e.g. `elbow_flex` moves ~2° for every 3° commanded (~33% shortfall),
due to low P gain / no integral term.

**This looks exactly like a stall and fooled us.** A stall detector comparing "actual vs commanded" with a tight
threshold will false-positive on totally normal motion. We wrongly concluded `elbow_flex` was jammed because a
5° command produced 2.5° of movement.

**Correct approach:** re-read the actual position and re-issue the command from there, iteratively. Don't treat
shortfall as failure. Detect a real stall as *consecutive readings with near-zero movement*, not as
position-vs-command error.

A corollary: **very small commands (< ~1°) often produce no measurable movement at all**, because they vanish
into the shortfall/stiction. This is why a "back off 0.35° for a light touch" step never actually did anything.

### 4.4 IK: `inverse_kinematics()` is one solver step, not a solve

`lerobot.model.kinematics.RobotKinematics.inverse_kinematics()` is designed for a realtime loop where the
target moves slightly each tick. **A single call from a seed 15° away barely moves the joints.**

You must call it **repeatedly, feeding its output back as the next seed**, until forward kinematics converges.
`hardware/draw/ik.py::solve_ik()` does this — about 5 iterations to reach <1mm.

Also: use **position-only IK** (`orientation_weight=0.0`). It converges far more robustly on this arm than
orientation-locked IK.

### 4.5 Stamp, never drag

Moving the pen laterally while it touches paper **jams on static friction** — confirmed repeatedly. Same for
rotational drag (rolling the wrist while in contact).

The working primitive is **one stamp per point**: touch down → mark → lift → move → touch down again. Slower,
but it works. All drawing is therefore dotted/stippled rather than continuous strokes.

### 4.6 Contact detection — measured load profile (IMPORTANT, this is solid data)

We profiled a slow descent, sampling load at every 0.15° step. Result:

```
Free air:                    load = 0-36
Just after a commanded move: load spikes to ~104  (acceleration transient, NOT contact)
Real paper contact:          load = 1004, SUSTAINED
```

There is a huge empty gap between 104 and 1004, which makes reliable detection easy **if** you avoid the
transient.

**The bug this revealed:** the original detector used `load_stop=150` and sampled load *immediately* after
writing a move command — so it caught its own acceleration transient and reported "contact" without the joint
having moved at all. Phantom contacts.

**The fix (now implemented in `AnchoredDescender`):**
- `load_stop = 300` (sits in the empty gap)
- sleep **0.22s after each step** before sampling load, so the transient dies first
- require **2 consecutive** readings above threshold
- sleep **0.4s** after the bigger pre-positioning jump

**Verified result:** contact height held at **58.99 → 59.04 → 59.03** across 3 stamps (0.04° drift), and the
detected contact matches the independently measured real contact height of ~58.8°.

### 4.7 The per-stamp drift bug (the biggest single problem)

**Symptom:** over a long run, `shoulder_lift` walked away in one direction — **159° over 113 stamps** in the
first full attempt — ending pinned against its own calibrated floor. So most of that run was fighting a joint
limit instead of drawing, and the "drawing" it produced was scattered and unrecognizable.

**Wrong diagnosis that cost a full rebuild:** we blamed the geometry of using `wrist_roll` as the row axis
(the pen tip is offset from the roll axis, so rolling changes pen height). We rebuilt around a 7-row grid to
cut that travel by 3x. **It did not help**, because the drift was ~2.7-2.9° *per stamp* — including *within* a
single row where `wrist_roll` never moved at all.

**Real cause:** the stamp cycle was **relative**. Each descent searched upward from wherever the previous lift
happened to leave the joint, and a fixed -6° lift is not a true inverse of whatever the descent did. Small
per-stamp errors compounded without bound.

**Fix — the `AnchoredDescender` class:** remember `table_h`, the **absolute** height where contact was last
found. Before every search, jump directly to `table_h - clearance` (an absolute move), then do a short local
search. One stamp's error can no longer leak into the next. Real height changes are absorbed by a damped EMA
update of `table_h`, with a fallback to a full-range search if the local search misses.

**Progression of the drift, measured:**
- original relative cycle: ~2.8°/stamp
- anchored, old detector: ~0.4-0.75°/stamp (better, phantom contacts still leaking)
- anchored + fixed detector (4.6): **~0.01°/stamp** — effectively solved

**Also important — measure the right thing.** Track drift of `table_h` (the actual mark height). Do **not** use
the post-lift resting position as your health metric: it wanders ~9° over 5 stamps for unrelated reasons while
the pen's mark position doesn't move at all.

### 4.8 Gravity sag can silently corrupt everything

Real failure chain that produced an apparently hung script:

1. Arm sagged while torque was off; `shoulder_lift` came to rest at raw **941** (its floor is **930**).
2. The next run's setup read that pose as its baseline.
3. The first contact search instantly "found contact" — actually the **joint limit**, not paper.
4. `table_h` anchored to that bogus value; every later stamp tried to pre-position 2.5° *below* a hard limit.
5. Endless load spikes, no motion, empty buffered log, apparently hung script.

**Tell-tale signs:**
- `contact at -103.12 -> backed off to -103.12` — the position **didn't change**, impossible on real paper.
- The contact height sits within a few degrees of a calibrated limit.
- A sprawled pose (`elbow_flex` ~97°, `wrist_flex` ~36°) when working values are ~52° and ~-92°.

**Always verify before drawing** that no joint is near a limit and that contact is found comfortably mid-range.

### 4.9 Restoring a collapsed arm: fold before you lift

If the arm has sprawled, do **not** just drive `shoulder_lift` up — it fights the full moment arm and
overheats (this is exactly how we hit 58°C). Correct order:

1. `elbow_flex` → working value (~52). This is a **decrease**, its good direction.
2. `wrist_flex` → working value (~-92 to -99). Also a decrease. Moves cleanly at loads 28-60.
3. `shoulder_lift` → working value (~55). Now much lighter because the arm is folded.

Use 3° steps, `Torque_Limit` 900, `Goal_Velocity` 100, a settle window of **8s** per step, and log load +
temperature every step. Re-read actual position each step and re-command from there (see 4.3).

**Confirmed working:** this restored `elbow_flex` 69.54° → 53.27° with loads steady at 56-72 and temperature
flat at 35°C.

### 4.10 Don't let `move_joint` handle the lift-off

After contact, `Present_Load` is saturated at ~1004 from the press. The general-purpose `move_joint()` helper
reads that residual press as an obstruction and burns its entire retry budget (multi-second timeouts) on a move
that *cannot* be obstructed — retreating from the table is always free. This was the dominant cost per stamp
(~60s/stamp).

Fix: lift with a plain `bus.write` + wait-for-position loop, ignoring load entirely.
**⚠️ This change is implemented but NOT yet tested — see section 8.**

### 4.11 Assorted traps

- `move_joint()` **returns `True` even when it never converged** (there's an implicit `return True` after its
  outer loop). Don't trust its return value as proof of arrival; re-read position if it matters.
- `Torque_Limit` (RAM, addr 48) **cannot exceed** `Max_Torque_Limit` (EEPROM, addr 16).
- Wrap every read in a retry (`safe_read`): a dropped packet raises `"There is no status packet!"`. A single
  glitch once killed a run 2 stamps into 131.
- Beware bulk regex edits — replacing `bus.read(` → `safe_read(bus, ` once rewrote `safe_read`'s own body into
  infinite recursion.
- Make long runs resumable (`--start-row=N` + a persisted reference JSON). On resume, reuse the **original**
  anchor; re-deriving it from the current pose misaligns finished rows against remaining ones.
- `Torque_Limit` 600-900 is needed for real work; the default 350 is not enough to lift the forearm assembly.

---

## 5. How the drawing pipeline works

### Step 1 — reference image → coarse binary grid

Exact reproduction isn't the goal; a recognizable silhouette is.

A thin, noisy hand-sketch rasterizes badly at coarse resolution. **What worked:** draw a clean programmatic
silhouette with `cv2.rectangle/ellipse/circle`, run **Canny** edge detection to get an outline, then rasterize
to a small binary grid. Every "on" cell becomes one physical stamp, so keep it coarse.

To reduce rows, use **block-max-pooling** (`g[a:b].any(axis=0)`) — it preserves a silhouette far better than
nearest-neighbour downsampling.

The current Taj Mahal grid (`assets/taj_grid_compact.npy`, 7×30, 91 stamps), downsampled from an original
13×30 (131 stamps):

```
..............##..............
..###......########......###..
..#.#.....##......###....#.#..
..#.#...###.######.###...#.#..
..#.#...#............#...#.#..
.##.#####............#####.###
.#############################
```

Always print the grid as ASCII and eyeball it before running.

### Step 2 — map grid axes to joints

- **Columns → `shoulder_pan`.** Bidirectional and reliable. `COL_STEP_DEG = 0.42` ≈ **1.8 mm** per column at
  ~250 mm reach, so 30 columns ≈ **5.4 cm** wide.
- **Rows → `wrist_roll`** currently, at `ROW_STEP_DEG = 1.6`. **This is a known-bad choice** (see 4.1 and 4.7):
  it's one-way so rows can never be revisited, and rolling changes pen height. **Improving this is the main
  open design problem.**

### Step 3 — per stamp

`AnchoredDescender.stamp()`: jump to `table_h - 2.5°` → short local search downward until 2 consecutive load
readings > 300 → that's contact, the mark is made → lift 6° directly → done.

### Step 4 — verify with the cameras

This is a required part of the task, not optional. Use the `Cam` class, not a raw `cv2.VideoCapture` grab —
pencil marks need real exposure control:

```python
from cam import Cam                     # hardware/repeat/cam.py
c = Cam("/dev/video2", exposure=75)
c.auto_gain()
frame = c.stack(3).clip(0,255).astype(np.uint8)   # note: .stack(n), there is no .frame()
cv2.imwrite("shot.jpg", frame)
c.close()
```

**Gotcha:** the third-person camera is **fixed** — it shows one region of the desk regardless of where the pen
is. "I don't see the drawing" may mean the pen was never in that region, not that nothing was drawn. Crop and
upscale around the pen tip to judge mark quality; marks are small and faint at full-frame scale.

---

## 6. Codebase map

All paths relative to `/home/machine/so101-yash/`.

| File | Role |
|---|---|
| `hardware/probe.py` | **Read-only** diagnostic: pos / goal / limits / torque / load / voltage / **temperature** / moving, for all 6 joints. Your first stop. |
| `hardware/arm.py` | Guarded manual jog: `nudge <joint> <deg>` (clamped ≤5°), `relax`, `read`, `calib` |
| `hardware/draw/ik.py` | `make_kinematics()`, `solve_ik()` (iterated; rejects `elbow_flex` increases) |
| `hardware/draw/execute.py` | `load_bus()`, `ARM_JOINTS`, interpolated Cartesian motion |
| `hardware/draw/draw_shoulder_only.py` | **Core primitives**: `safe_read`, `setup`, `move_joint`, `lift`, and the **`AnchoredDescender`** class (the important one) |
| `hardware/draw/draw_taj2.py` | Raster driver: grid → rows/columns → stamps, with a `table_h` drift abort |
| `hardware/draw/assets/taj_grid_compact.npy` | The 7×30 Taj Mahal grid, 91 stamps |
| `hardware/repeat/cam.py` | `Cam` class: exposure control, `auto_gain()`, `stack(n)` |
| `.claude/skills/so101-draw/SKILL.md` | A Claude Code "skill" version of this knowledge, auto-loads on drawing tasks in this repo |

---

## 7. Exact current state of the hardware

As of the last reading, arm **torque off**, all temps **33-35°C** (cool and safe), in calibrated degrees:

```
shoulder_pan    ~12.97
shoulder_lift   ~59-61      (paper contact is at ~58.8-59.2; raw ~2700, comfortably mid-range)
elbow_flex      ~48.18
wrist_flex      ~-98.73
wrist_roll      ~-73.10
```

This is a **good working posture** — close to the original known-good drawing pose, well clear of all limits.

**Confirmed visually (camera photo):** the pen tip is on the paper and **makes real, visible marks**. A short
drawn line segment from test stamps is clearly visible next to the tip. The marking mechanism works.

---

## 8. What is done vs. what is left

**Working and verified:**
- Arm restored to a good posture from a fully collapsed state
- Pen reaches paper and makes visible marks (camera-confirmed)
- Contact detection is now reliable and matches independently measured ground truth
- The per-stamp drift bug is effectively solved (~0.01°/stamp)
- Camera capture with proper exposure works

**Implemented but NOT tested:**
- The direct-lift change in `AnchoredDescender.stamp()` (section 4.10). It was written, and the test run that
  would have confirmed both correctness and the speed improvement **was interrupted before it ran**. Verify
  this first — run ~8 stamps, confirm `table_h` stays stable and measure seconds per stamp.

**Known open problems:**
1. **Speed.** Before the direct-lift fix, stamps took ~60s each → 91 stamps ≈ 90 minutes. The fix should cut
   this substantially but that is unmeasured. Budget real time for a full run.
2. **The row axis is still `wrist_roll`, which is a bad choice** (one-way, and it changes pen height). This is
   the main structural design problem remaining. Options: find a bidirectional row axis, or accept it and keep
   total row travel small.
3. **No complete picture has ever been drawn.** Only individual test stamps and short line segments.
4. Earlier failed attempts left stray marks on the paper — don't mistake old marks for new output.

**Suggested next steps, in order:**
1. Verify the untested direct-lift change (8 stamps, check `table_h` stability + timing).
2. Run a small shape end-to-end first — e.g. a single row or a 10-stamp line — and photograph it. Prove the
   full loop before committing to 91 stamps.
3. Then run the full grid via `draw_taj2.py`, with `python -u`, logging to a file, with a monitor whose filter
   matches failure signatures (`ABORT|FAILED|Traceback|no contact`) and not just success lines.
4. Photograph the result and compare against the ASCII grid in section 5.

---

## 9. Quick-start commands

```bash
cd /home/machine/so101-yash/hardware/draw
PY=/home/machine/so101-yash/.venv/bin/python

# 0. Pre-flight — bus must be clear
ps aux | grep python | grep -v grep

# 1. Check arm state (read-only; NEVER while a drawing script is running)
timeout 20 $PY -u ../probe.py

# 2. Read positions in the calibrated-degree convention the scripts use
timeout 30 $PY -u -c "
from execute import load_bus, ARM_JOINTS
bus = load_bus(); bus.connect()
try:
    for j in ARM_JOINTS:
        print(j, round(bus.read('Present_Position', j),2),
              'raw', bus.read('Present_Position', j, normalize=False))
finally:
    bus.disconnect(disable_torque=False)"

# 3. Full drawing run (always -u, always backgrounded with a log)
nohup $PY -u draw_taj2.py > /tmp/taj.log 2>&1 &
tail -f /tmp/taj.log

# 4. Stop a stuck run SAFELY (never kill -9)
kill -INT <pid>
```

Noise filter for LeRobot's startup chatter: `2>&1 | grep -vE "^\[Genesis\]|^WARNING|^INFO"`
