"""Live 3D mirror of the real SO-101: read real joint angles, drive a Genesis simulation to match.

  genesis_mirror.py

Read-only on the real arm - it only calls robot.get_observation() (LeRobot's calibrated degrees), never
send_action(), so this cannot move the real hardware. The simulated robot's DOFs are set directly with
set_dofs_position (kinematic, no physics/actuator lag) each frame, so the sim is a pure visual mirror of
whatever the real arm is doing, moved by hand or by any other script.

Joint sign/zero convention: the URDF's neutral pose (all DOFs = 0) is the CAD design pose, which is NOT the
same as LeRobot's calibrated 0 (homing_offset, set during `arm.py calib` / lerobot-calibrate). No offset
between the two has been measured yet - see hardware/fk_check.py's self-collision warning at neutral and the
TODO below. Until that offset is measured this mirror can be directionally/qualitatively wrong even though
each individual axis moves the right way (confirmed in hardware/fk_check.py: nudging shoulder_lift moved the
FK-predicted height the expected way).
"""
import signal, sys, time
import numpy as np
import genesis as gs
from lerobot.robots.so_follower import SO101Follower, SOFollowerRobotConfig

# SO101Follower.connect() enables torque as a side effect of configure() (rewriting P/I/D etc. under a
# torque_disabled() context that unconditionally re-enables on exit, not "restore prior state") - this has been
# harmless in every other script today only because they all call disconnect() cleanly, which explicitly disables
# torque again (disable_torque_on_disconnect=True). SIGTERM (e.g. from `timeout`, or a process manager) does not
# raise a catchable exception by default, so it skips `finally:` entirely and leaves torque on. Convert it to a
# normal KeyboardInterrupt-shaped exit here so this script is safe to kill by any means.
signal.signal(signal.SIGTERM, lambda *_: (_ for _ in ()).throw(KeyboardInterrupt))

URDF = "hardware/urdf/so-arm100/Simulation/SO101/so101_new_calib.urdf"
JOINT_ORDER = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper"]
# LeRobot reports the first 5 in degrees but the gripper as 0-100% of its calibrated travel (RANGE_0_100 norm
# mode, config_so_follower.py) - the URDF's gripper joint is a radian hinge (limits below, read from the URDF
# itself), so it needs its own conversion instead of the shared np.radians(deg) used for the rest.
GRIPPER_RAD_RANGE = (-0.174533, 1.74533)
HZ = 20

gs.init(backend=gs.gpu)
scene = gs.Scene(show_viewer=True, viewer_options=gs.options.ViewerOptions(camera_pos=(0.4, 0.4, 0.3), camera_lookat=(0.1, 0, 0.05)))
scene.add_entity(gs.morphs.Plane())
robot = scene.add_entity(gs.morphs.URDF(file=URDF, fixed=True))
scene.build()
dof_idx = [robot.get_joint(name).dofs_idx_local[0] for name in JOINT_ORDER]

config = SOFollowerRobotConfig(port="/dev/ttyACM0", id="lab_follower")
real = SO101Follower(config)
real.connect(calibrate=False)
assert real.is_calibrated, "real arm reports as not calibrated - stopping, will not mirror unverified data"
print("mirroring live - Ctrl-C to stop. Real arm connect(calibrate=False): no motion commands are ever sent to it.")

errors = 0
try:
    while True:
        t0 = time.time()
        try:
            obs = real.get_observation()
            rad = [np.radians(obs[f"{j}.pos"]) for j in JOINT_ORDER[:-1]]
            lo, hi = GRIPPER_RAD_RANGE
            rad.append(lo + (hi - lo) * obs["gripper.pos"] / 100)
            robot.set_dofs_position(np.array(rad), dof_idx)
            scene.step()
            errors = 0
        except Exception as e:
            # a transient serial hiccup (e.g. another process briefly opening the same port) should not kill an
            # otherwise-fine mirror; a persistent one should not spin forever either.
            errors += 1
            print(f"read/render error ({errors}/10): {e!r}", flush=True)
            if errors >= 10:
                raise
            time.sleep(0.2)
            continue
        dt = time.time() - t0
        if dt < 1 / HZ:
            time.sleep(1 / HZ - dt)
except KeyboardInterrupt:
    pass
finally:
    try:
        real.disconnect()
        print("disconnected (real arm untouched - read-only the whole time)")
    except Exception as e:
        print(f"disconnect() itself failed ({e!r}) - torque may still be on; run `arm.py relax` to release it.")
