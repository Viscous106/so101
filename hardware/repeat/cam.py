"""Wrist camera in raw YUYV with a fixed 0.5 s exposure (MJPG silently caps exposure at the frame period)."""
import cv2, numpy as np


class Cam:
    def __init__(self, exposure=5000, gain=0):
        self.cap = cv2.VideoCapture("/dev/video0", cv2.CAP_V4L2)
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"YUYV"))
        self.cap.set(3, 1280); self.cap.set(4, 720); self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        self.cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 1); self.cap.set(cv2.CAP_PROP_EXPOSURE, exposure); self.cap.set(cv2.CAP_PROP_GAIN, gain)
        self.cap.set(cv2.CAP_PROP_AUTO_WB, 0)
        for _ in range(5):
            self.cap.read()

    def stack(self, n=5, flush=3):
        """Mean of n frames as float32 BGR. Flushes frames exposed before/during the last move."""
        for _ in range(flush):
            self.cap.read()
        acc = None
        for _ in range(n):
            ok, f = self.cap.read()
            assert ok, "camera read failed"
            acc = f.astype(np.float32) if acc is None else acc + f
        return acc / n

    def close(self):
        self.cap.set(cv2.CAP_PROP_AUTO_WB, 1); self.cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 3); self.cap.release()
