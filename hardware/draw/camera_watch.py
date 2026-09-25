"""Keep a fresh fixed-camera view available without sharing the serial bus."""
import pathlib
import signal
import sys
import time

import cv2
import numpy as np

from camera_devices import DESK_CAMERA

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1] / "repeat"))
from cam import Cam


def main():
    def interrupt(signum, frame):
        raise KeyboardInterrupt()
    signal.signal(signal.SIGTERM, interrupt)
    destination = pathlib.Path("/tmp/so101-camera-live")
    destination.mkdir(exist_ok=True)
    cam = None
    try:
        cam = Cam(DESK_CAMERA, exposure=40, gain=30)
        cam.auto_gain(target=115, tries=3)
        print(f"Camera ready gain={cam.gain}, exposure={cam.exposure}", flush=True)
        while True:
            frame = cam.stack(1, flush=1).clip(0, 255).astype(np.uint8)
            tmp = destination / "desk.tmp.jpg"
            if not cv2.imwrite(str(tmp), frame, [cv2.IMWRITE_JPEG_QUALITY, 95]):
                raise RuntimeError("Failed to save camera frame")
            tmp.replace(destination / "desk.jpg")
            time.sleep(0.15)
    except KeyboardInterrupt:
        pass
    finally:
        if cam is not None:
            cam.close()


if __name__ == "__main__":
    main()
