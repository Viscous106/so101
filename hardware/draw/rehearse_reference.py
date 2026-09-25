"""Offline MuJoCo kinematic rehearsal; never opens a motor or camera device.

The pen offset is an input, not a measurement. A successful rehearsal is NOT
permission to execute: camera-measured calibration and held-out checks are
required before these joint targets can be used on hardware.
"""
import argparse
import json
import pathlib

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import mujoco
import numpy as np

from draw_reference import JOINTS, ROOT, save_json


MODEL = ROOT / "hardware/urdf/so-arm100/Simulation/SO101/so101_new_calib.xml"
CALIBRATION = pathlib.Path.home() / ".cache/huggingface/lerobot/calibration/robots/so_follower/lab_follower.json"


class Rehearsal:
    def __init__(self, pen_offset=(0, 0, 0), calibration=None):
        self.model = mujoco.MjModel.from_xml_path(str(MODEL))
        self.data = mujoco.MjData(self.model)
        self.site = self.model.site("gripperframe").id
        self.addresses = [int(self.model.joint(j).qposadr[0]) for j in JOINTS]
        self.dofs = [int(self.model.joint(j).dofadr[0]) for j in JOINTS]
        rotation = np.zeros(9)
        mujoco.mju_quat2Mat(rotation, self.model.site_quat[self.site])
        self.model.site_pos[self.site] += rotation.reshape(3, 3) @ np.asarray(pen_offset)
        if calibration is None:
            calibration = json.loads(CALIBRATION.read_text())
        self.limits = []
        for name in JOINTS:
            cal = calibration[name]
            midpoint = (cal["range_min"] + cal["range_max"]) / 2
            limits = (np.array([cal["range_min"] + 24, cal["range_max"] - 24]) - midpoint) * 360 / 4095
            self.limits.append(limits)
            # The model's nominal +/-95 wrist range differs from this arm's
            # encoder calibration. This changes model bounds, not hardware.
            self.model.jnt_range[self.model.joint(name).id] = np.deg2rad(limits)
        self.limits = np.asarray(self.limits)

    def forward(self, degrees):
        degrees = np.asarray(degrees, dtype=float)
        if degrees.shape != (5,) or not np.isfinite(degrees).all():
            raise ValueError("Expected five finite calibrated joint angles")
        self.data.qpos[self.addresses] = np.deg2rad(degrees)
        mujoco.mj_forward(self.model, self.data)
        return self.data.site_xpos[self.site].copy()

    def jacobian(self, degrees):
        self.forward(degrees)
        jac = np.zeros((3, self.model.nv))
        mujoco.mj_jacSite(self.model, self.data, jac, None, self.site)
        return jac[:, self.dofs] * np.pi / 180

    def solve(self, target, seed, active=(0, 1, 2), bounds=None, tolerance=1e-5):
        q = np.array(seed, dtype=float)
        active = list(active)
        bounds = self.limits if bounds is None else np.asarray(bounds)
        if np.any(q < bounds[:, 0]) or np.any(q > bounds[:, 1]):
            raise ValueError("IK seed outside the permitted envelope")
        for _ in range(100):
            error = np.asarray(target) - self.forward(q)
            if np.linalg.norm(error) < tolerance:
                return q
            jac = self.jacobian(q)[:, active]
            delta = np.linalg.lstsq(jac, error, rcond=None)[0]
            delta *= min(1.0, 1.0 / max(np.max(np.abs(delta)), 1e-12))
            q[active] += delta
            q = np.clip(q, bounds[:, 0], bounds[:, 1])
        raise RuntimeError(f"Nominal IK did not converge: {np.linalg.norm(error)*1000:.3f} mm")

    def surface_point(self, q, height):
        q = np.array(q, dtype=float)
        for _ in range(40):
            position = self.forward(q)
            error = height - position[2]
            if abs(error) < 1e-6:
                return q, position
            derivative = self.jacobian(q)[2, 1]
            if abs(derivative) < 1e-5:
                raise RuntimeError("Height control near singularity")
            q[1] += np.clip(error / derivative, -1, 1)
            if not self.limits[1, 0] <= q[1] <= self.limits[1, 1]:
                raise RuntimeError("Surface correction exceeds shoulder limit")
        raise RuntimeError("Nominal surface correction failed")


def rehearse(plan, state, out, pen_offset, width, height):
    model = Rehearsal(pen_offset)
    base = np.array([state["base"][j] for j in JOINTS])
    base[1] = state["first_contact"]
    origin = model.forward(base)
    jac = model.jacobian(base)
    x_axis = jac[:, 0].copy()
    x_axis[2] = 0
    x_axis /= np.linalg.norm(x_axis)
    y_axis = np.cross([0, 0, 1], x_axis)
    row_axis = jac[:, 2] - jac[:, 1] * jac[2, 2] / jac[2, 1]
    if np.dot(y_axis, row_axis) < 0:
        y_axis *= -1
    basis = np.stack([x_axis, y_axis])
    bounds = model.limits.copy()
    envelope = np.array([10, 12, 20, 0, 0])
    bounds[:, 0] = np.maximum(bounds[:, 0], base - envelope)
    bounds[:, 1] = np.minimum(bounds[:, 1], base + envelope)
    old, requested, achieved, rows = [], [], [], []
    seed = base.copy()
    for point in plan["points"]:
        legacy = base.copy()
        legacy[0] += (point["x"] - 15) * plan["pan_step"]
        legacy[4] -= point["row"] * plan["roll_step"]
        _, position = model.surface_point(legacy, origin[2])
        old.append(basis @ (position - origin))
        target = origin + x_axis * (point["x"] - 15) * width / 30 + y_axis * point["row"] * height / 10
        seed = model.solve(target, seed, bounds=bounds)
        actual = model.forward(seed)
        requested.append(basis @ (target - origin))
        achieved.append(basis @ (actual - origin))
        rows.append({"point": point, "nominal_target_m": target.tolist(),
                     "nominal_joints_deg": dict(zip(JOINTS, seed.tolist())),
                     "nominal_error_m": float(np.linalg.norm(actual - target))})
    old, requested, achieved = map(np.asarray, (old, requested, achieved))
    roll_axis = jac[:, 4] - jac[:, 1] * jac[2, 4] / jac[2, 1]
    report = {
        "mode": "offline_kinematics_only",
        "hardware_execution_allowed": False,
        "reason": "Unmeasured pen offset, camera mapping and joint-zero errors; no held-out hardware validation",
        "pen_offset_assumed_m": list(pen_offset),
        "legacy_span_mm": (np.ptp(old, axis=0) * 1000).tolist(),
        "candidate_span_mm": (np.ptp(achieved, axis=0) * 1000).tolist(),
        "max_nominal_ik_error_mm": max(p["nominal_error_m"] for p in rows) * 1000,
        "height_compensated_elbow_mm_per_degree": (row_axis * 1000).tolist(),
        "height_compensated_roll_mm_per_degree": (roll_axis * 1000).tolist(),
        "joint_envelope_deg": dict(zip(JOINTS, bounds.tolist())),
        "points": rows,
    }
    out.mkdir(parents=True, exist_ok=True)
    save_json(out / "rehearsal.json", report)
    figure, axes = plt.subplots(1, 2, figsize=(11, 4), constrained_layout=True)
    for ax, xy, title in zip(axes, (old, achieved), ("Old wrist-roll rows", "Candidate pan + elbow, height compensated")):
        ax.scatter(xy[:, 0] * 1000, xy[:, 1] * 1000, s=14, color="#176a80")
        ax.set_title(title)
        ax.set_aspect("equal")
        ax.set_xlim(-33, 33)
        ax.set_ylim(-5, 28)
        ax.set_xlabel("Nominal model x (mm)")
        ax.set_ylabel("Nominal model y (mm)")
        ax.grid(alpha=0.2)
    figure.suptitle("MuJoCo kinematic rehearsal, NOT a physical drawing or calibrated digital twin")
    figure.savefig(out / "comparison.png", dpi=150)
    plt.close(figure)
    print(json.dumps({k: v for k, v in report.items() if k != "points"}, indent=2))
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source", type=pathlib.Path, default=ROOT / "hardware/draw/runs/pasteboard_taj_verified")
    parser.add_argument("--out", type=pathlib.Path, default=ROOT / "hardware/draw/runs/mujoco_rehearsal")
    parser.add_argument("--pen-offset", type=float, nargs=3, default=(0, 0, 0), metavar=("X", "Y", "Z"))
    parser.add_argument("--width-mm", type=float, default=54)
    parser.add_argument("--height-mm", type=float, default=22)
    args = parser.parse_args()
    if not (0 < args.width_mm <= 60 and 0 < args.height_mm <= 25):
        parser.error("Rehearsal dimensions must be within 60 x 25 mm")
    if not np.isfinite(args.pen_offset).all() or np.linalg.norm(args.pen_offset) > 0.12:
        parser.error("Pen offset must be finite and no longer than 120 mm")
    plan = json.loads((args.source / "plan.json").read_text())
    state = json.loads((args.source / "state.json").read_text())
    rehearse(plan, state, args.out, args.pen_offset, args.width_mm / 1000, args.height_mm / 1000)


if __name__ == "__main__":
    main()
