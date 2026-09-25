"""Sanity-check forward kinematics against the real arm: read calibrated joint degrees, print gripper_frame_link
pose. No motion - read-only. Run this by hand while moving the arm (torque off) to see if the numbers track reality
(Z should rise when you lift the arm, X/Y should track pan/reach) before trusting IK for anything.
"""
from lerobot.robots.so_follower import SO101Follower, SOFollowerRobotConfig
from lerobot.model.kinematics import RobotKinematics

URDF = "hardware/urdf/so-arm100/Simulation/SO101/so101_new_calib.urdf"
JOINT_ORDER = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper"]

config = SOFollowerRobotConfig(port="/dev/ttyACM0", id="lab_follower")
robot = SO101Follower(config)
robot.connect(calibrate=False)
assert robot.is_calibrated, "not calibrated - stopping"
kin = RobotKinematics(URDF, target_frame_name="gripper_frame_link", joint_names=JOINT_ORDER)
try:
    obs = robot.get_observation()
    deg = [obs[f"{j}.pos"] for j in JOINT_ORDER]
    T = kin.forward_kinematics(deg)
    print("joint degrees:", dict(zip(JOINT_ORDER, [round(d, 1) for d in deg])))
    print("gripper_frame_link pose (metres, URDF frame):")
    print("  xyz:", T[:3, 3].round(4).tolist())
    print("  rotation matrix:\n", T[:3, :3].round(3))
finally:
    robot.disconnect()
