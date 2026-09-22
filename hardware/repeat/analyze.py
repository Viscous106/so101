"""Analyse one collect.py run: image registration -> repeatability numbers.  analyze.py runs/<dir>"""
import json, sys, pathlib
import cv2, numpy as np

run = pathlib.Path(sys.argv[1])
meta = json.loads((run / "meta.json").read_text())
caps = [json.loads(l) for l in open(run / "captures.jsonl")]
PEN = np.array([755.0, 335.0])                      # where we report displacement: the marker, on the table plane
STEP_DEG = 360 / 4096


def load(i):
    g = cv2.imread(str(run / f"{i:03d}.png"), cv2.IMREAD_UNCHANGED).astype(np.float32) / 256
    g = cv2.GaussianBlur(g, (0, 0), 1.5)
    return g - cv2.GaussianBlur(g, (0, 0), 20)      # band-pass: drop illumination gradients, keep edges


mask = np.full((720, 1280), 255, np.uint8)
mask[:25], mask[-25:], mask[:, :25], mask[:, -25:] = 0, 0, 0, 0
cv2.fillPoly(mask, [np.array([[300, 720], [480, 440], [830, 440], [1010, 720]])], 0)   # gripper jaws move with the camera
soft = cv2.GaussianBlur(mask.astype(np.float32) / 255, (0, 0), 8)
ref = load(0)


def register(img, init=None):
    W = np.eye(2, 3, dtype=np.float32) if init is None else init.copy()
    if init is None:
        (sx, sy), _ = cv2.phaseCorrelate(ref * soft, img * soft, cv2.createHanningWindow((1280, 720), cv2.CV_32F))
        W[0, 2], W[1, 2] = sx, sy
    cc, W = cv2.findTransformECC(ref, img, W, cv2.MOTION_EUCLIDEAN, (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 300, 1e-8), mask, 5)
    d = W[:, :2] @ PEN + W[:, 2] - PEN
    return d, float(np.degrees(np.arctan2(W[1, 0], W[0, 0]))), float(cc), W


prev = None
for cdict in caps:
    img = load(cdict["idx"])
    try:
        d, rot, cc, W = register(img)
    except cv2.error:
        d, rot, cc, W = register(img, prev)          # large shift: start from the previous solution
    prev = W
    cdict.update(dx=float(d[0]), dy=float(d[1]), rot=rot, cc=cc)

D = lambda rows: np.array([[r["dx"], r["dy"]] for r in rows])
static = [c for c in caps if c["phase"] == "static"]
stair = [c for c in caps if c["phase"] == "stair"]
rep = [c for c in caps if c["phase"] == "repeat"]
end = [c for c in caps if c["phase"] == "static_end"]

# motion axis in the image + pixels per encoder step, from the staircase
sd = D(stair); sp = np.array([c["pos"] for c in stair])
u = np.linalg.svd(sd - sd.mean(0))[2][0]
if (sd @ u)[-1] < (sd @ u)[0]: u = -u
v = np.array([-u[1], u[0]])
slope, icpt = np.polyfit(sp, sd @ u, 1)
resid = sd @ u - (slope * sp + icpt)
r2 = 1 - resid.var() / (sd @ u).var()

out = {"joint": meta["joint"], "P": meta["P"], "delta_steps": meta["delta"], "n_pairs": meta["n_pairs"],
       "px_per_step": slope, "stair_r2": r2, "stair_resid_rms_px": float(np.sqrt((resid ** 2).mean())), "axis": u.tolist(),
       "noise_floor_px": {"along": float((D(static) @ u).std(ddof=1)), "across": float((D(static) @ v).std(ddof=1))},
       "drift_px": float((D(end) @ u).mean() - (D(static) @ u).mean()) if end else None}
for name, sign in (("from_above", -1), ("from_below", +1)):
    rows = [c for c in rep if c["approach"] == sign]
    s, pos = D(rows) @ u, np.array([c["pos"] for c in rows])
    out[name] = {"n": len(rows), "cam_mean_px": float(s.mean()), "cam_std_px": float(s.std(ddof=1)), "cam_range_px": float(np.ptp(s)),
                 "across_std_px": float((D(rows) @ v).std(ddof=1)), "enc_mean": float(pos.mean()), "enc_std": float(pos.std(ddof=1)),
                 "enc_minus_goal": float(pos.mean() - meta["P"])}
a, bl = out["from_above"], out["from_below"]
cam_gap_px = a["cam_mean_px"] - bl["cam_mean_px"]
enc_gap = a["enc_mean"] - bl["enc_mean"]
cam_gap_steps = cam_gap_px / slope
to_mm = lambda steps, reach=250: abs(steps) * STEP_DEG * np.pi / 180 * reach
out["bidirectional"] = {"camera_px": cam_gap_px, "camera_equiv_steps": cam_gap_steps, "camera_equiv_deg": cam_gap_steps * STEP_DEG,
                        "encoder_steps": enc_gap, "encoder_deg": enc_gap * STEP_DEG, "hidden_from_encoder_steps": cam_gap_steps - enc_gap,
                        "mm_at_250mm_reach": to_mm(cam_gap_steps)}
uni_steps = max(a["cam_std_px"], bl["cam_std_px"]) / abs(slope)
out["unidirectional"] = {"std_px": max(a["cam_std_px"], bl["cam_std_px"]), "std_equiv_steps": uni_steps, "std_deg": uni_steps * STEP_DEG,
                         "mm_at_250mm_reach_1sigma": to_mm(uni_steps)}
out["min_cc"] = min(c["cc"] for c in caps)
(run / "results.json").write_text(json.dumps({"summary": out, "captures": caps}, indent=1))
print(json.dumps(out, indent=1))
