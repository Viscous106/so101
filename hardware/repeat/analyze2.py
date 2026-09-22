"""Analyse one collect.py run (v2: robust registration, rotation metric for wrist_roll).  analyze2.py runs/<dir>"""
import json, sys, pathlib
import cv2, numpy as np

run = pathlib.Path(sys.argv[1])
meta = json.loads((run / "meta.json").read_text())
caps = [json.loads(l) for l in open(run / "captures.jsonl")]
PEN, CENTER, STEP_DEG = np.array([755.0, 335.0]), np.array([640.0, 360.0]), 360 / 4096
USE_ROT = meta["joint"] == "wrist_roll"            # rolling the wrist rotates the image; everything else mostly shifts it


def load(i):
    g = cv2.imread(str(run / f"{i:03d}.png"), cv2.IMREAD_UNCHANGED).astype(np.float32) / 256
    g = cv2.GaussianBlur(g, (0, 0), 1.5)
    return g - cv2.GaussianBlur(g, (0, 0), 20)


mask = np.full((720, 1280), 255, np.uint8)
mask[:25], mask[-25:], mask[:, :25], mask[:, -25:] = 0, 0, 0, 0
# everything rigidly attached to the camera must be masked out of the registration: the jaws, and (since
# 2026-09-22) the marker taped into them. collect_v2 records the polygon it was run with in meta["mask_poly"].
JAWS_ONLY = [[300, 720], [480, 440], [830, 440], [1010, 720]]
cv2.fillPoly(mask, [np.array(meta.get("mask_poly", JAWS_ONLY), np.int32)], 0)
CRIT = (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 400, 1e-8)
ref = load(0)
small = lambda im: cv2.resize(im, None, fx=0.25, fy=0.25, interpolation=cv2.INTER_AREA)
ref_s, mask_s = small(ref), cv2.resize(mask, None, fx=0.25, fy=0.25, interpolation=cv2.INTER_NEAREST)


def rot_about(deg, c=CENTER):
    t = np.radians(deg); R = np.array([[np.cos(t), -np.sin(t)], [np.sin(t), np.cos(t)]])
    return np.hstack([R, (c - R @ c)[:, None]]).astype(np.float32)


def ecc(img, img_s, W0):
    try:
        Ws = W0.copy(); Ws[:, 2] /= 4
        _, Ws = cv2.findTransformECC(ref_s, img_s, Ws, cv2.MOTION_EUCLIDEAN, CRIT, mask_s, 5)      # coarse
        W = Ws.copy(); W[:, 2] *= 4
        cc, W = cv2.findTransformECC(ref, img, W, cv2.MOTION_EUCLIDEAN, CRIT, mask, 5)             # fine
        return float(cc), W
    except cv2.error:
        return -1.0, W0


prev = np.eye(2, 3, dtype=np.float32)
for c in caps:
    img = load(c["idx"]); img_s = small(img)
    enc_deg = (c["pos"] - meta["P"]) * STEP_DEG
    inits = [np.eye(2, 3, dtype=np.float32), prev, rot_about(enc_deg), rot_about(-enc_deg)]
    cc, W = max((ecc(img, img_s, W0) for W0 in inits), key=lambda r: r[0])
    prev = W
    d = W[:, :2] @ PEN + W[:, 2] - PEN
    c.update(dx=float(d[0]), dy=float(d[1]), rot=float(np.degrees(np.arctan2(W[1, 0], W[0, 0]))), cc=cc)

D = lambda rows: np.array([[r["dx"], r["dy"]] for r in rows])
ph = lambda name: [c for c in caps if c["phase"] == name]
static, stair, rep, end = ph("static"), [c for c in ph("stair") if c["cc"] > 0.8], ph("repeat"), ph("static_end")
failed = [c["idx"] for c in caps if c["cc"] <= 0.8]
sd = D(stair); u = np.linalg.svd(sd - sd.mean(0))[2][0]
if (sd @ u)[-1] < (sd @ u)[0]: u = -u
meas = (lambda rows: np.array([r["rot"] for r in rows])) if USE_ROT else (lambda rows: D(rows) @ u)
unit = "deg of image rotation" if USE_ROT else "px"
sp = np.array([c["pos"] for c in stair])
slope, icpt = np.polyfit(sp, meas(stair), 1)
resid = meas(stair) - (slope * sp + icpt)
to_steps = lambda x: x / slope
mm = lambda steps, reach=250: abs(steps) * STEP_DEG * np.pi / 180 * reach

S = {"joint": meta["joint"], "compensated": meta.get("compensated", False), "P": meta["P"], "measure": unit, "per_step": slope,
     "stair_r2": 1 - resid.var() / meas(stair).var(), "stair_resid_rms_steps": float(np.sqrt((resid ** 2).mean()) / abs(slope)),
     "noise_floor_steps": float(meas(static).std(ddof=1) / abs(slope)), "failed_registrations": failed, "stair_points_used": len(stair), "min_cc_repeat": min(c["cc"] for c in rep)}
hid_all = []
for name, sign in (("from_above", -1), ("from_below", +1)):
    rows = [c for c in rep if c["approach"] == sign]
    cam = to_steps(meas(rows)); pos = np.array([c["pos"] for c in rows]) - meta["P"]; hid = cam - pos; hid_all.append(hid)
    S[name] = {"n": len(rows), "cam_mean": cam.mean(), "cam_std": cam.std(ddof=1), "enc_mean": pos.mean(), "enc_std": pos.std(ddof=1),
               "hidden_mean": hid.mean(), "hidden_std": hid.std(ddof=1)}
a, bl = S["from_above"], S["from_below"]
gap_cam, gap_enc = a["cam_mean"] - bl["cam_mean"], a["enc_mean"] - bl["enc_mean"]
S["reversal_error"] = {"camera_steps": gap_cam, "camera_deg": gap_cam * STEP_DEG, "encoder_steps": gap_enc,
                       "hidden_steps": gap_cam - gap_enc, "mm_at_250mm": mm(gap_cam)}
allcam = to_steps(meas(rep)); allpos = np.array([c["pos"] for c in rep]) - meta["P"]
S["all_arrivals"] = {"cam_std_steps": allcam.std(ddof=1), "cam_std_deg": allcam.std(ddof=1) * STEP_DEG, "cam_range_steps": float(np.ptp(allcam)),
                     "mm_at_250mm_1sigma": mm(allcam.std(ddof=1)), "enc_std_steps": allpos.std(ddof=1),
                     "hidden_std_steps": np.concatenate(hid_all).std(ddof=1), "corr_cam_enc": float(np.corrcoef(allcam, allpos)[0, 1]) if allpos.std() > 0 else None}
S = json.loads(json.dumps(S, default=float))
(run / "results2.json").write_text(json.dumps({"summary": S, "captures": caps}, indent=1))
rnd = lambda o: {k: (round(v, 3) if isinstance(v, float) else v) for k, v in o.items()} if isinstance(o, dict) else (round(o, 4) if isinstance(o, float) else o)
for k, v in S.items(): print(f"{k:22s}", rnd(v))
