"""IK helper for the SO-101 drawing pipeline, wrapping LeRobot's placo-based RobotKinematics correctly.

RobotKinematics.inverse_kinematics() is a single incremental solver step (built for a real-time control loop
where the target moves a little each tick), not a batch solve - a single call from a seed 15deg off barely
moves the joints at all (see hardware/draw/ik_check.py). Solving for a genuinely new target requires calling it
repeatedly, feeding its own output back as the next seed, until the resulting end-effector position converges.
Verified 2026-09-22: from a 15deg-off seed, converges to <1mm position error within ~5 iterations.
"""
import pathlib
import numpy as np
from lerobot.model.kinematics import RobotKinematics

URDF = str(pathlib.Path(__file__).resolve().parent.parent / "urdf/so-arm100/Simulation/SO101/so101_new_calib.urdf")
JOINT_ORDER = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper"]


def make_kinematics(urdf=URDF, joint_order=JOINT_ORDER):
    return RobotKinematics(urdf, target_frame_name="gripper_frame_link", joint_names=joint_order)


ELBOW_IDX = JOINT_ORDER.index("elbow_flex")


def solve_ik(kin, seed_deg, target_pose, tol_m=0.001, max_iters=20, orientation_weight=0.01, max_elbow_increase=0.05):
    """seed_deg: current/previous joint degrees (len 5 or 6, gripper preserved if present).
    target_pose: 4x4 desired gripper_frame_link pose.
    Returns (solved_deg, converged: bool, final_err_m).

    elbow_flex on this arm has a permanent one-direction hardware fault (confirmed 2026-09-22: increasing it
    instantly reports near-max load with ~0 real current and Moving=1 but zero actual motion - a dead H-bridge
    leg, not a software or mechanical-jam problem, reproduced at every position/torque_limit tried; the joint
    moves freely by hand and decreasing it always works). Any IK solution that increases elbow_flex beyond
    max_elbow_increase (degrees) is rejected here - converged=False - so callers never send it, the same way an
    unreachable target is rejected, rather than commanding a move that will silently do nothing but report a
    fault-driven "load spike" that looks like table contact.
    """
    seed = np.asarray(seed_deg, dtype=float)
    deg = seed.copy()
    err = np.inf
    for _ in range(max_iters):
        deg = kin.inverse_kinematics(deg, target_pose, orientation_weight=orientation_weight)
        if deg[ELBOW_IDX] - seed[ELBOW_IDX] > max_elbow_increase:
            return deg, False, float(np.linalg.norm(kin.forward_kinematics(deg)[:3, 3] - target_pose[:3, 3]))
        achieved = kin.forward_kinematics(deg)
        new_err = float(np.linalg.norm(achieved[:3, 3] - target_pose[:3, 3]))
        if abs(err - new_err) < 1e-6 and new_err > tol_m:
            break  # stalled short of tol - a real IK failure (unreachable/near-singular), not slow convergence
        err = new_err
        if err < tol_m:
            return deg, True, err
    return deg, False, err
