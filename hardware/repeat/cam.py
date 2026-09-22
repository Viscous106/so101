"""Fixed-exposure camera capture for the benchmark, raw YUYV (MJPG silently caps exposure at the frame period)."""
import cv2, numpy as np


class Cam:
    def __init__(self, dev="/dev/video0", exposure=5000, gain=0, size=(1280, 720)):
        self.dev = dev
        self.cap = cv2.VideoCapture(dev, cv2.CAP_V4L2)
        assert self.cap.isOpened(), f"cannot open {dev} (another app streaming it? check `lsof {dev}`)"
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"YUYV"))
        self.cap.set(3, size[0]); self.cap.set(4, size[1]); self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        self.cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 1); self.cap.set(cv2.CAP_PROP_EXPOSURE, exposure); self.cap.set(cv2.CAP_PROP_GAIN, gain)
        self.cap.set(cv2.CAP_PROP_AUTO_WB, 0)
        # the C270 keeps delivering auto-exposed frames for a while after the switch to manual; set twice, flush plenty
        self.cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 1); self.cap.set(cv2.CAP_PROP_EXPOSURE, exposure)
        for _ in range(15):
            ok, _ = self.cap.read()
        assert ok, f"{dev}: no frames"
        self.size = (int(self.cap.get(3)), int(self.cap.get(4)))

    def auto_gain(self, target=110, tol=20, tries=10):
        """Adjust gain (and, when gain is pinned at 0 or 255, the exposure) until the mean grey level lands near
        target. The C270's manual exposure has coarse steps and does not always take, so nothing is trusted: every
        change is verified on a fresh frame. Returns (gain, mean); the settings used are kept in .gain/.exposure."""
        gain, exposure = int(self.cap.get(cv2.CAP_PROP_GAIN)), int(self.cap.get(cv2.CAP_PROP_EXPOSURE))
        for _ in range(tries):
            g = cv2.cvtColor(self.stack(2, flush=4).clip(0, 255).astype(np.uint8), cv2.COLOR_BGR2GRAY)
            mean = float(g.mean())
            if abs(mean - target) <= tol:
                break
            if (gain == 0 and mean > target) or (gain == 255 and mean < target):
                exposure = max(1, int(exposure * (0.5 if mean > target else 2)))
                self.cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 1); self.cap.set(cv2.CAP_PROP_EXPOSURE, exposure)
            else:
                gain = int(min(255, max(0, gain + (target - mean) * 1.2)))
                self.cap.set(cv2.CAP_PROP_GAIN, gain)
            for _ in range(8):
                self.cap.read()
        self.gain, self.exposure, self.mean = gain, exposure, mean
        return gain, mean

    def stack(self, n=5, flush=3):
        """Mean of n frames as float32 BGR. Flushes frames exposed before/during the last move."""
        for _ in range(flush):
            self.cap.read()
        acc = None
        for _ in range(n):
            ok, f = self.cap.read()
            assert ok, f"{self.dev}: camera read failed"
            acc = f.astype(np.float32) if acc is None else acc + f
        return acc / n

    def close(self):
        self.cap.set(cv2.CAP_PROP_AUTO_WB, 1); self.cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 3); self.cap.release()
