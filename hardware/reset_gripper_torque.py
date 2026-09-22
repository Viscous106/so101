"""Set the gripper's EEPROM Max_Torque_Limit.   reset_gripper_torque.py [value=500]

LeRobot's so_follower.py configure() writes Max_Torque_Limit=500 (50%) for the gripper "to avoid burnout" on every
connect(). It's a hard EEPROM ceiling that a RAM-level torque_limit write (bus.py's hold_all()) can never exceed.
It was raised to 1000 on 2026-09-22 on the (wrong) theory that it limited the gripper repeatability test; the real
load was a marker taped into the jaws, so the 500 burnout cap is the right value while the arm holds a pen.
"""
import sys
from lerobot.motors import Motor, MotorNormMode
from lerobot.motors.feetech import FeetechMotorsBus

value = int(sys.argv[1]) if len(sys.argv) > 1 else 500
bus = FeetechMotorsBus(port="/dev/ttyACM0", motors={"gripper": Motor(6, "sts3215", MotorNormMode.RANGE_0_100)})
bus.connect()
try:
    bus.disable_torque()
    print("before:", bus.read("Max_Torque_Limit", "gripper", normalize=False))
    bus.write("Max_Torque_Limit", "gripper", value)
    print("after: ", bus.read("Max_Torque_Limit", "gripper", normalize=False))
finally:
    bus.disconnect()
