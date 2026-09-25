"""Basic shapes using ONLY joints proven reliable today: shoulder_lift for height, shoulder_pan for a lateral
sweep (line), wrist_roll for a small circle (the pen tip is offset from the roll axis, so rolling the wrist
traces an arc). elbow_flex is held completely fixed throughout - every attempt to make it increase, isolated
or coordinated, at any position, with any torque_limit, has instantly hit near-max load with zero movement,
while it moves freely by hand (confirmed) and every other joint behaves normally. Treated as a fixed hardware
constraint of this joint, not something to keep solving for; this script routes around it entirely.

  draw_shoulder_only.py line
  draw_shoulder_only.py circle
"""
import sys, time
from execute import load_bus, ARM_JOINTS

SPEED, ACCEL = 40, 6
TORQUE_LIMIT = 600
LOAD_STOP = 400
SETTLE_S, TIMEOUT_S, TOL = 0.05, 3.0, 0.4


def safe_read(bus, reg, joint, tries=4, **kw):
    """A dropped serial packet ('There is no status packet!') is a communication glitch, not a fault - the
    same kind of transient event move_joint's load-spike retries already handle, just surfacing as an
    exception instead of a load reading. Confirmed for real: draw_taj.py crashed the whole run on exactly this
    after only 2/131 stamps, with the arm left safely torque-off by the finally block - annoying, not
    dangerous, but there is no reason a single dropped packet should end a long run."""
    for attempt in range(tries):
        try:
            return bus.read(reg, joint, **kw)
        except Exception as e:
            if attempt == tries - 1:
                raise
            print(f"    read glitch ({reg}/{joint}): {e!r}, retry {attempt + 1}/{tries}")
            time.sleep(0.15)


def setup(bus):
    cur = {j: safe_read(bus, "Present_Position", j) for j in ARM_JOINTS}
    for j in ARM_JOINTS:
        bus.write("Goal_Position", j, cur[j])
        bus.write("Acceleration", j, ACCEL, normalize=False)
        bus.write("Goal_Velocity", j, SPEED, normalize=False)
        bus.write("Torque_Limit", j, TORQUE_LIMIT, normalize=False)
        bus.write("Torque_Enable", j, 1, normalize=False)
    return cur


def move_joint(bus, joint, goal, tries=4, stall_retries=2):
    """A load spike does NOT immediately fail the move - it retries stall_retries times after a brief pause and
    re-reading current position first (distinguishes a real, repeatable fault like elbow_flex's - which fails
    every time, no exceptions found in this whole session - from a one-off bus/communication glitch, which a
    previous version of this function could not tell apart because it returned False on the very first spike,
    never actually using this retry loop)."""
    cmd = goal
    for attempt in range(tries):
        for stall in range(stall_retries + 1):
            bus.write("Goal_Position", joint, cmd)
            time.sleep(0.15)   # let the acceleration transient pass before sampling (a fresh move momentarily
                               # reads ~1000 on shoulder_pan even in free air - same transient the descender
                               # already learned to ignore; without this, every lateral move false-tripped here)
            t0 = time.time()
            spiked = False
            while time.time() - t0 < TIMEOUT_S:
                time.sleep(SETTLE_S)
                l = safe_read(bus, "Present_Load", joint, normalize=False) & 0x3FF
                if l > LOAD_STOP:
                    # confirm it is SUSTAINED, not a transient: a real jam (elbow_flex increase, pen drag) holds
                    # ~1000; an acceleration blip drops back within a few ms
                    if all((safe_read(bus, "Present_Load", joint, normalize=False) & 0x3FF) > LOAD_STOP
                           for _ in range(2)):
                        spiked = True
                        break
                p = safe_read(bus, "Present_Position", joint)
                if abs(p - cmd) < TOL:
                    break
            if not spiked:
                break
            print(f"    load spike on {joint} ({l}), retry {stall + 1}/{stall_retries}" +
                  (" - giving up" if stall == stall_retries else ""))
            time.sleep(0.3)
            cmd = safe_read(bus, "Present_Position", joint) + (goal - cmd) * 0.3   # small re-approach, not the full jump again
        else:
            return False
        err = goal - safe_read(bus, "Present_Position", joint)
        if abs(err) < 0.3:
            return True
        cmd = cmd + err
    return True


def descend_shoulder_lift(bus, step_deg=0.15, load_stop=150, max_steps=80, backoff_deg=0.35):
    """Pen-down height via shoulder_lift alone, increasing (matches every descent today: shoulder_lift
    increases as Z decreases toward the table).

    The contact transition is sharp (0 to ~1000 load within a fraction of a degree, confirmed repeatedly
    today), and load is only sampled every SETTLE_S, so detection always overshoots into a hard press, not a
    light touch (confirmed just now: a lateral sweep immediately jammed under that much pressure). So back off
    by backoff_deg after the load trips, to land on a light touch instead of a hard dig-in, before returning.
    """
    cur = safe_read(bus, "Present_Position", "shoulder_lift")
    for i in range(max_steps):
        cur += step_deg
        bus.write("Goal_Position", "shoulder_lift", cur)
        t0 = time.time()
        while time.time() - t0 < 1.5:
            time.sleep(SETTLE_S)
            l = safe_read(bus, "Present_Load", "shoulder_lift", normalize=False) & 0x3FF
            if l > load_stop:
                hit = safe_read(bus, "Present_Position", "shoulder_lift")
                target = hit - backoff_deg
                bus.write("Goal_Position", "shoulder_lift", target)
                time.sleep(0.3)
                final = safe_read(bus, "Present_Position", "shoulder_lift")
                fl = safe_read(bus, "Present_Load", "shoulder_lift", normalize=False) & 0x3FF
                print(f"  contact at {hit:.2f} (load {l}) -> backed off to {final:.2f} (load {fl})")
                return True
            p = safe_read(bus, "Present_Position", "shoulder_lift")
            if abs(p - cur) < 0.3:
                break
    print("  no contact found within max_steps")
    return False


def lift(bus, deg=4.0):
    cur = safe_read(bus, "Present_Position", "shoulder_lift")
    move_joint(bus, "shoulder_lift", cur - deg)


class AnchoredDescender:
    """Fixes a real drift bug found in draw_taj2.py's first corrected run: descend_shoulder_lift() always starts
    its search from wherever the PREVIOUS stamp's lift() happened to leave shoulder_lift, and lift()'s fixed
    -6deg is not a true inverse of whatever descend actually did to find contact - so small per-stamp errors
    compound. Confirmed directly: two different rows, at two very different wrist_roll angles, both drifted
    ~2.7-2.9deg PER STAMP (not per row) in the same direction, 91 stamps in from a fresh, verified-safe baseline
    - proving the bug is in the relative stamp cycle itself, not (only) the row-to-row wrist_roll geometry
    effect this run was originally built to reduce.

    Fix: never start a descent from "current position" after the first stamp. Instead remember table_h, the
    absolute height (in calibrated degrees) where contact was last found, and always jump shoulder_lift directly
    there minus a small clearance before each search - so lift()'s accuracy on any single stamp can no longer
    accumulate into the next one. A short local search around that anchor finds the actual contact each time
    (handles small real height changes, e.g. paper thickness or genuine row-to-row geometry) and updates table_h
    with a damped (EMA) step so a real geometry change is absorbed gradually, not overshot in one jump; if
    contact isn't found in the short local search (a bigger real height change), falls back to the original
    full-range incremental search and re-anchors table_h from that.
    """

    def __init__(self, step_deg=0.15, load_stop=300, lift_deg=6.0, pre_clearance_deg=2.5,
                 ema_alpha=0.3, step_settle_s=0.22, confirm=2):
        self.step_deg = step_deg
        self.load_stop = load_stop
        self.lift_deg = lift_deg
        self.pre_clearance_deg = pre_clearance_deg
        self.ema_alpha = ema_alpha
        self.step_settle_s = step_settle_s
        self.confirm = confirm
        self.table_h = None

    def _search(self, bus, start, max_steps):
        """Thresholds come from a measured descent profile, not guesswork: in free air load reads 0-36, spiking
        to ~104 only in the moments right after a commanded move (acceleration transient); real paper contact
        saturates at 1004 and STAYS there. So load_stop=300 sits in a wide empty gap, and requiring `confirm`
        consecutive readings above it rejects transients outright.

        The settle sleep before sampling is the important part. The previous version sampled immediately after
        writing the goal, which caught the acceleration transient of its own pre-position jump and reported
        "contact" without the joint having moved at all - producing phantom contacts that walked table_h
        downward a few tenths of a degree per stamp."""
        cur = start
        for i in range(max_steps):
            cur += self.step_deg
            bus.write("Goal_Position", "shoulder_lift", cur)
            time.sleep(self.step_settle_s)
            hits = 0
            for _ in range(self.confirm):
                l = safe_read(bus, "Present_Load", "shoulder_lift", normalize=False) & 0x3FF
                if l <= self.load_stop:
                    break
                hits += 1
            if hits >= self.confirm:
                return safe_read(bus, "Present_Position", "shoulder_lift")
        return None

    def stamp(self, bus):
        if self.table_h is None:
            start = safe_read(bus, "Present_Position", "shoulder_lift")
            hit = self._search(bus, start, max_steps=80)
            if hit is None:
                print("  AnchoredDescender: no contact found on first search")
                return False
            self.table_h = hit
        else:
            pre = self.table_h - self.pre_clearance_deg
            move_joint(bus, "shoulder_lift", pre)
            time.sleep(0.4)   # let the jump's transient die before the search samples load
            max_local_steps = int((self.pre_clearance_deg + 1.5) / self.step_deg)
            hit = self._search(bus, pre, max_steps=max_local_steps)
            if hit is None:
                print(f"  AnchoredDescender: local search missed (anchor {self.table_h:.2f}), "
                      f"falling back to full search")
                start = safe_read(bus, "Present_Position", "shoulder_lift")
                hit = self._search(bus, start, max_steps=80)
                if hit is None:
                    print("  AnchoredDescender: full fallback search also found no contact")
                    return False
            self.table_h = (1 - self.ema_alpha) * self.table_h + self.ema_alpha * hit
        # Lift straight off contact with a plain write, NOT move_joint. At this instant the joint is pressed
        # against paper and Present_Load is saturated at ~1004; move_joint reads that residual press as an
        # obstruction and burns its whole retry budget (multi-second timeouts) on a move that cannot be
        # obstructed - retreating from the table is always free. This was the dominant cost per stamp.
        # The old separate backoff step is gone: it never measurably moved the joint (a 0.35-0.8deg command
        # disappears into the steady-state shortfall), and lifting immediately shortens the press anyway.
        # Lift must fully clear the paper before any lateral move, or the pen grazes the surface and the
        # sideways move drags -> shoulder_pan load spikes to ~1000 and wastes its whole retry budget (measured:
        # a 2.5s timeout only lifted ~3deg, not enough clearance, and every subsequent pan move fought friction).
        # So: give it time to reach the full lift, and confirm we're back in free air (load < 100) before
        # returning - free air is the proof the pen is clear.
        target = hit - self.lift_deg
        bus.write("Goal_Position", "shoulder_lift", target)
        t0 = time.time()
        p, fl = hit, 999
        while time.time() - t0 < 5.0:
            time.sleep(0.1)
            p = safe_read(bus, "Present_Position", "shoulder_lift")
            fl = safe_read(bus, "Present_Load", "shoulder_lift", normalize=False) & 0x3FF
            if abs(p - target) < 0.5 and fl < 100:
                break
        print(f"  contact at {hit:.2f} -> lifted to {p:.2f} (load {fl}), table_h={self.table_h:.2f}")
        return True


def stamp(bus):
    """Touch down, mark a point, lift - no lateral motion while in contact. Dragging sideways even under a
    light touch immediately jams (confirmed: static friction between pen and paper - a real stick/breakaway
    effect, not a bug), so a continuous dragged line is not attempted; a series of individually stamped points
    sidesteps it entirely using only the touch/lift cycle already proven safe."""
    descend_shoulder_lift(bus)
    lift(bus, deg=6.0)
    p = safe_read(bus, "Present_Position", "shoulder_lift")
    l = safe_read(bus, "Present_Load", "shoulder_lift", normalize=False) & 0x3FF
    print(f"    after lift: shoulder_lift={p:.2f} load={l}")


def draw_line(bus, sweep_deg=8.0, n=8):
    start = safe_read(bus, "Present_Position", "shoulder_pan")
    for i in range(n + 1):
        p = start + sweep_deg * i / n
        ok = move_joint(bus, "shoulder_pan", p)
        print(f"  point {i}: shoulder_pan -> {p:.2f} {'ok' if ok else 'STOP'}")
        if not ok:
            return
        stamp(bus)


def draw_circle(bus, sweep_deg=350.0, n=16):
    """Stamped, not dragged - same reasoning as draw_line: rotating wrist_roll while the tip touches paper is
    still a drag (rotational instead of lateral), so step + stamp per point, not one continuous rotation."""
    start = safe_read(bus, "Present_Position", "wrist_roll")
    for i in range(n + 1):
        p = start + sweep_deg * i / n
        ok = move_joint(bus, "wrist_roll", p)
        print(f"  point {i}: wrist_roll -> {p:.2f} {'ok' if ok else 'STOP'}")
        if not ok:
            return
        stamp(bus)


if __name__ == "__main__":
    which = sys.argv[1] if len(sys.argv) > 1 else "line"
    bus = load_bus(); bus.connect()
    try:
        setup(bus)
        if which == "line":
            print("drawing line (stamped points, shoulder_pan sweep)...")
            draw_line(bus)
        elif which == "circle":
            print("drawing circle (stamped points, wrist_roll sweep)...")
            draw_circle(bus)
        print("lifting off...")
        lift(bus)
    finally:
        for j in ARM_JOINTS:
            bus.write("Torque_Enable", j, 0, normalize=False)
        bus.disconnect(disable_torque=False)
        print("torque off, disconnected")
