"""Read-only smoke test of the SO-101 follower through LeRobot's own Robot API (not the raw scservo scripts).

Connects, prints one observation (joint degrees + camera frame shapes), disconnects.
Reuses the calibration file already on this machine
(~/.cache/huggingface/lerobot/calibration/robots/so_follower/lab_follower.json), written by arm.py's `calib`
command. Sends no action, so the arm does not move.

connect(calibrate=False) always: with calibrate=True (the default), if LeRobot decides the on-device
calibration doesn't match its cached copy, it silently runs its interactive set_half_turn_homings() /
record_ranges_of_motion() flow, which rewrites homing-offset and position-limit EEPROM registers on the
real servos. That already happened once here by accident (see restore_calibration.py). Every script in
this repo that touches the real robot should connect with calibrate=False and fail loudly instead.
"""
from lerobot.robots.so_follower import SO101Follower, SOFollowerRobotConfig
from lerobot.cameras.opencv import OpenCVCameraConfig

config = SOFollowerRobotConfig(
    port="/dev/ttyACM0",
    id="lab_follower",
    cameras={
        "wrist": OpenCVCameraConfig(index_or_path="/dev/video0", width=1280, height=720, fps=10),
    },
)
robot = SO101Follower(config)
robot.connect(calibrate=False)
try:
    if not robot.is_calibrated:
        raise RuntimeError(
            "robot reports as not calibrated - refusing to proceed. "
            "Investigate with hardware/probe.py before calling connect(calibrate=True) or robot.calibrate()."
        )
    obs = robot.get_observation()
    for k, v in obs.items():
        print(f"{k:16s}", v.shape if hasattr(v, "shape") else v)
finally:
    robot.disconnect()
