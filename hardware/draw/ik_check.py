"""Sanity check for hardware/draw/ik.py: does it recover the arm's actual current pose from a bad seed, and does
it correctly report failure (not silently return garbage) for a target the arm cannot reach? Read-only, no motion.
"""
import numpy as np
from lerobot.robots.so_follower import SO101Follower, SOFollowerRobotConfig
from ik import make_kinematics, solve_ik, JOINT_ORDER

config = SOFollowerRobotConfig(port="/dev/ttyACM0", id="lab_follower")
robot = SO101Follower(config)
robot.connect(calibrate=False)
assert robot.is_calibrated
kin = make_kinematics()
try:
    obs = robot.get_observation()
    deg = np.array([obs[f"{j}.pos"] for j in JOINT_ORDER])
    T = kin.forward_kinematics(deg)
    print("actual joint degrees:", deg.round(2))
    print("FK pose xyz (m):     ", T[:3, 3].round(4))

    seed = deg.copy(); seed[:5] += 15
    solved, ok, err = solve_ik(kin, seed, T)
    print(f"\nrecover-current-pose test: converged={ok} err={err * 1000:.2f}mm")
    print("solved degrees:", solved.round(2), " (need not match actual exactly - redundant IK - but FK must match)")

    T_far = T.copy(); T_far[0, 3] += 5.0   # 5m away - the arm's reach is ~25cm, this cannot be reached
    solved2, ok2, err2 = solve_ik(kin, deg, T_far)
    print(f"\nunreachable-target test: converged={ok2} (must be False) err={err2 * 1000:.1f}mm (must be large)")
    assert not ok2, "solve_ik must NOT report convergence for an unreachable target"
    print("PASS: correctly reported failure instead of returning a false solution")
finally:
    robot.disconnect()
