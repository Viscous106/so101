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

    def pos_avg(self, joint, n=7, dt=0.03):
        vals = []
        for _ in range(n):
            vals.append(self.read(joint, "pos")); time.sleep(dt)
        return sum(vals) / len(vals)

    def hold_all(self, torque_limit=350, accel=8, speed=150):
        """Torque ON everywhere with goal = where each joint already is, so nothing moves. Returns saved originals."""
        saved = {}
        for j in JOINTS:
            saved[j] = {k: self.read(j, k) for k in ("accel", "speed", "torque_limit")}
            p = min(max(self.read(j, "pos"), self.read(j, "min")), self.read(j, "max"))
            self.write(j, "goal", p); self.write(j, "accel", accel); self.write(j, "speed", speed)
            self.write(j, "torque_limit", torque_limit); self.write(j, "torque_en", 1)
        return saved

    def release_all(self, saved=None):
        for j in JOINTS:
            try:
                self.write(j, "torque_en", 0)
                for k, v in (saved or {}).get(j, {}).items():
                    self.write(j, k, v)
            except IOError as e:
                print("release:", e)

    def move(self, joint, goal, tol=15, timeout=6.0, settle=1.0, trim=False, trim_tries=4, trim_tol=0.5, trim_clamp=20):
        """Slow move of ONE joint. Raises if it cannot get there (something is in the way).

        trim=True adds a closed-loop encoder correction after arrival: the servo's position loop settles with a
        steady-state offset under friction/spring load (this is what caused the shoulder_lift/gripper 'stalls' -
        low peak load, just short of goal, not an obstruction). Re-commanding the goal by the residual encoder
        error a few times cancels that offset instead of just accepting a wider tolerance for it.
        """
        self.write(joint, "goal", goal)
        t0, peak = time.time(), 0
        while time.time() - t0 < timeout:
            time.sleep(0.05)
            peak = max(peak, abs(self.read(joint, "load")))
            if not self.read(joint, "moving") and abs(self.read(joint, "pos") - goal) <= tol:
                break
        else:
            raise RuntimeError(f"{joint} stalled at {self.read(joint, 'pos')} going to {goal} (peak load {peak})")
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
