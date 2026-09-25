"""Parallel offline contact-plane fits; no motor/camera access or execution."""
import concurrent.futures
import itertools
import json
import pathlib
import time

import cv2
import numpy as np
from scipy.optimize import lsq_linear

from draw_reference import JOINTS, ROOT, save_json
from rehearse_reference import Rehearsal

OUT = ROOT / "hardware/draw/runs/reference_OjPuJYdomotd/model_fit"


def worker(task):
    bias, poses, held_out = task
    sim = Rehearsal()
    correction = np.array([0, *bias, 0], dtype=float)
    heights, rotations = [], []
    for q in poses:
        point = sim.forward(np.asarray(q) + correction)
        heights.append(point[2])
        rotations.append(sim.data.site_xmat[sim.site].reshape(3, 3)[2].copy())
    design = np.column_stack([rotations, -np.ones(len(poses))])
    train = ~np.asarray(held_out)
    fit = lsq_linear(design[train], -np.asarray(heights)[train],
                     bounds=([-.12]*3 + [-.2], [.12]*3 + [.5]))
    residual = design @ fit.x + heights
    singular = np.linalg.svd(design[train], compute_uv=False)
    sim2 = Rehearsal(fit.x[:3])
    q = np.array([31.16, 50.29, 63.38, -99.34, -71.16])
    jac = sim2.jacobian(q + correction)
    delta = np.linalg.solve(jac[:, :3], [0, 0, .001])
    return {"joint_bias_assumed_deg": correction.tolist(), "pen_offset_fitted_m": fit.x[:3].tolist(),
            "plane_z_m": float(fit.x[3]), "train_rms_mm": float(np.sqrt(np.mean(residual[train]**2))*1000),
            "held_out_rms_mm": float(np.sqrt(np.mean(residual[~train]**2))*1000),
            "design_singular_values": singular.tolist(),
            "condition_number": float(singular[0]/max(singular[-1], 1e-15)),
            "nominal_vertical_1mm_delta_deg": delta.tolist()}


def main():
    OUT.mkdir(parents=True, exist_ok=True)
    source = ROOT / "hardware/draw/runs/paper_taj_final/telemetry.jsonl"
    poses, held_out = [], []
    corrupt = 0
    for line in source.read_text().splitlines():
        try:
            record = json.loads(line)
        except (ValueError, TypeError):
            corrupt += 1
            continue
        if record.get("event") == "final_stamp" and "contact_pose" in record:
            poses.append([record["contact_pose"][j] for j in JOINTS])
            held_out.append(record["point"]["row"] in (2, 5, 8))
    if len(poses) < 30 or not any(held_out):
        raise RuntimeError("Insufficient recorded contact poses")
    biases = list(itertools.product((-4, 0, 4), repeat=3))
    started = time.monotonic()
    with concurrent.futures.ProcessPoolExecutor(max_workers=4) as pool:
        results = list(pool.map(worker, [(bias, poses, held_out) for bias in biases]))
    results.sort(key=lambda r: r["held_out_rms_mm"])
    report = {"mode": "contact_geometry_only", "workers": 4, "candidates": len(results),
              "seconds": time.monotonic()-started, "observations": len(poses),
              "corrupt_log_lines_skipped": corrupt, "hardware_execution_allowed": False,
              "limitations": ["No fitted dynamics", "Contact plane alone does not validate XY mapping",
                              "Poorly observable offsets must not be interpreted as physical measurements"],
              "results": results}
    save_json(OUT / "fits.json", report)
    print(json.dumps({k:v for k,v in report.items() if k != "results"}, indent=2))
    print("BEST", json.dumps(results[:3], indent=2))
    print("VERTICAL DELTA RANGE", np.min([r["nominal_vertical_1mm_delta_deg"] for r in results], axis=0),
          np.max([r["nominal_vertical_1mm_delta_deg"] for r in results], axis=0))
    image = cv2.imread(str(OUT.parent / "contact_lab/gentle_pan_after/desk.jpg"))
    if image is not None:
        crop = image[360:515, 750:1050]
        crop = cv2.resize(crop, None, fx=3, fy=3)
        cv2.imwrite(str(OUT / "test_crop.jpg"), cv2.convertScaleAbs(crop, alpha=1.7, beta=20))


if __name__ == "__main__":
    main()
