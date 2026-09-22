"""Minimal Feetech STS3215 bus helper for the SO-101 repeatability benchmark (RAM registers only, never EEPROM)."""
import time
import scservo_sdk as scs

PORT, BAUD = "/dev/ttyACM0", 1_000_000
JOINTS = {"shoulder_pan": 1, "shoulder_lift": 2, "elbow_flex": 3, "wrist_flex": 4, "wrist_roll": 5, "gripper": 6}
# name: (address, bytes)
R = {"min": (9, 2), "max": (11, 2), "max_torque": (16, 2), "P": (21, 1), "D": (22, 1), "I": (23, 1), "cw_dead": (26, 1),
     "ccw_dead": (27, 1), "mode": (33, 1), "torque_en": (40, 1), "accel": (41, 1), "goal": (42, 2), "speed": (46, 2),
     "torque_limit": (48, 2), "pos": (56, 2), "load": (60, 2), "volt": (62, 1), "temp": (63, 1), "moving": (66, 1), "current": (69, 2)}


def signmag(v, bit):
    return -(v & ((1 << bit) - 1)) if v & (1 << bit) else v


class Bus:
    def __init__(self):
        self.port, self.ph = scs.PortHandler(PORT), scs.PacketHandler(0)
        assert self.port.openPort() and self.port.setBaudRate(BAUD), f"cannot open {PORT}"

    def read(self, joint, reg, tries=3):
        i, (addr, n) = JOINTS[joint], R[reg]
        for _ in range(tries):
            v, res, _ = (self.ph.read1ByteTxRx if n == 1 else self.ph.read2ByteTxRx)(self.port, i, addr)
            if res == scs.COMM_SUCCESS:
                return signmag(v, 10) if reg == "load" else signmag(v, 15) if reg in ("pos", "current") else v
            time.sleep(0.01)
        raise IOError(f"read {joint}.{reg} failed")

    def write(self, joint, reg, v, tries=3):
        i, (addr, n) = JOINTS[joint], R[reg]
        assert addr >= 40, "EEPROM writes are not allowed here"
        for _ in range(tries):
            res, err = (self.ph.write1ByteTxRx if n == 1 else self.ph.write2ByteTxRx)(self.port, i, addr, int(v))
            if res == scs.COMM_SUCCESS and not err:
                return
            time.sleep(0.01)
        raise IOError(f"write {joint}.{reg}={v} failed")

    def write_eeprom(self, joint, reg, v, tries=3):
        """Deliberate EEPROM write, restricted to the P/I/D gains, torque off. Caller is responsible for restoring it."""
        assert reg in ("P", "I", "D"), "only the position-loop gains are allowed through here"
        assert not self.read(joint, "torque_en"), f"{joint}: torque must be off for an EEPROM write"
        i, (addr, n) = JOINTS[joint], R[reg]
        for _ in range(tries):
            res, err = (self.ph.write1ByteTxRx if n == 1 else self.ph.write2ByteTxRx)(self.port, i, addr, int(v))
            if res == scs.COMM_SUCCESS and not err:
                return
            time.sleep(0.01)
        raise IOError(f"eeprom write {joint}.{reg}={v} failed")

    def pos_avg(self, joint, n=7, dt=0.03):
        vals = []
        for _ in range(n):
            vals.append(self.read(joint, "pos")); time.sleep(dt)
        return sum(vals) / len(vals)

    def hold_all(self, torque_limit=350, accel=8, speed=150, gains=None):
        """Torque ON everywhere with goal = where each joint already is, so nothing moves. Returns saved originals.

        gains={joint: P} or {joint: {"P": .., "I": .., "D": ..}} temporarily sets position-loop gains (EEPROM) for
        those joints; release_all restores them.
        LeRobot's configure() leaves every motor at P=16 (anti-jitter for teleop); Feetech's factory value is 32. With
        I=0 the loop settles at error ~ load/P, so joints with a standing load (gripper spring, shoulder_lift gravity)
        fall ~24 counts short at P=16 and the servo never comes near any torque limit doing it.
        """
        saved = {}
        for j in JOINTS:
            saved[j] = {k: self.read(j, k) for k in ("accel", "speed", "torque_limit")}
            if gains and j in gains:
                g = gains[j] if isinstance(gains[j], dict) else {"P": gains[j]}
                self.write(j, "torque_en", 0)
                for k, v in g.items():
                    saved[j][k] = self.read(j, k)
                    self.write_eeprom(j, k, v)
            p = min(max(self.read(j, "pos"), self.read(j, "min")), self.read(j, "max"))
            self.write(j, "goal", p); self.write(j, "accel", accel); self.write(j, "speed", speed)
            self.write(j, "torque_limit", torque_limit); self.write(j, "torque_en", 1)
        return saved

    def release_all(self, saved=None, keep_hold=False):
        """Restore the saved registers. keep_hold=True re-enables torque afterwards (goal = where the joint is) so the
        arm does not sag between back-to-back runs; the gains still have to be written with torque off, which costs a
        few ms per joint."""
        for j in JOINTS:
            try:
                self.write(j, "torque_en", 0)
                for k, v in (saved or {}).get(j, {}).items():
                    (self.write_eeprom if k in ("P", "I", "D") else self.write)(j, k, v)
                if keep_hold:
                    self.write(j, "goal", self.read(j, "pos")); self.write(j, "torque_en", 1)
            except IOError as e:
                print("release:", e)

    def move(self, joint, goal, tol=15, timeout=6.0, settle=1.0, trim=False, trim_tries=4, trim_tol=0.5, trim_clamp=20,
             push_tries=3, push_tol=50, push_clamp=40):
        """Slow move of ONE joint. Raises if it cannot get there (something is in the way).

        The servo's position loop settles with a steady-state offset under friction/spring load (low peak load, just
        short of goal, not an obstruction). A stop short of goal is therefore not a stall by itself: while the motor is
        stopped within push_tol of goal, re-command goal + residual error (clamped to push_clamp) up to push_tries
        times. Only a stop further than push_tol away, or one that survives the pushes, raises.

        trim=True adds a finer closed-loop correction after arrival (down to trim_tol) for compensated runs.
        """
        cmd, peak = goal, 0
        for attempt in range(push_tries + 1):
            self.write(joint, "goal", cmd)
            t0, stopped = time.time(), 0
            while time.time() - t0 < timeout:
                time.sleep(0.05)
                peak = max(peak, abs(self.read(joint, "load")))
                if time.time() - t0 > 0.3 and not self.read(joint, "moving"):   # 'moving' lags the command by a few reads
                    stopped += 1
                    if stopped >= 2:
                        break
                else:
                    stopped = 0
            pos = self.read(joint, "pos")
            err = goal - pos
            if abs(err) <= tol:
                break
            if abs(err) > push_tol or attempt == push_tries:
                raise RuntimeError(f"{joint} stalled at {pos} going to {goal} (peak load {peak}, last cmd {cmd}, {attempt} pushes)")
            cmd = int(round(min(max(cmd + err, goal - push_clamp), goal + push_clamp)))
        time.sleep(settle)
        if trim:
            cmd = goal
            for _ in range(trim_tries):
                err = goal - self.pos_avg(joint, n=5)
                if abs(err) < trim_tol:
                    break
                cmd = int(round(min(max(cmd + err, goal - trim_clamp), goal + trim_clamp)))
                self.write(joint, "goal", cmd); time.sleep(0.6)
        return peak

    def close(self):
        self.port.closePort()
