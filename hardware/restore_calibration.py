"""One-off recovery: write lab_follower.json's homing offsets + range limits back to the servos' EEPROM.

Needed after an lerobot_probe.py bug ran the interactive SOFollower.calibrate() path (wrong calibration
directory: lerobot 0.6.1 keys calibration files by robot.name="so_follower", not "so101_follower", so it
never found lab_follower.json) which called set_half_turn_homings() and wiped Min/Max_Position_Limit to
[0, 4095]. Torque was off throughout so nothing moved; only EEPROM registers changed.
"""
import json
import pathlib

from lerobot.motors import Motor, MotorCalibration, MotorNormMode
from lerobot.motors.feetech import FeetechMotorsBus

cal_path = pathlib.Path.home() / ".cache/huggingface/lerobot/calibration/robots/so101_follower/lab_follower.json"
cal = json.loads(cal_path.read_text())

motors = {
    name: Motor(c["id"], "sts3215", MotorNormMode.RANGE_0_100 if name == "gripper" else MotorNormMode.DEGREES)
    for name, c in cal.items()
}
bus = FeetechMotorsBus(port="/dev/ttyACM0", motors=motors)
bus.connect()
try:
    bus.disable_torque()
    calibration = {
        name: MotorCalibration(
            id=c["id"], drive_mode=c["drive_mode"], homing_offset=c["homing_offset"],
            range_min=c["range_min"], range_max=c["range_max"],
        )
        for name, c in cal.items()
    }
    bus.write_calibration(calibration)
    print("wrote calibration back to servos:")
    for name in cal:
        print(f"  {name:14s} offset={bus.read('Homing_Offset', name, normalize=False):6d} "
              f"min={bus.read('Min_Position_Limit', name, normalize=False):5d} "
              f"max={bus.read('Max_Position_Limit', name, normalize=False):5d}")
finally:
    bus.disconnect()
