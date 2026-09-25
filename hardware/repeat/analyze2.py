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

# reference frame: the one most representative of all captures nominally at P (static/repeat/static_end - collect_v2
# always passes goal=P for these), by median pairwise correlation to the others on a small masked downsample - cheap
# (no ECC) yet catches a contaminated frame that a mean-brightness test misses. Frame 0 was blindly used before; a
# person walked into frame 0 of the 2026-09-22 16:00 shoulder_lift run (median corr to the rest: 0.94, rank 37/44)
# and that alone lost 42/47 registrations.
at_p = [c["idx"] for c in caps if c["phase"] != "stair"]
small_mask = cv2.resize(mask, (160, 90), interpolation=cv2.INTER_NEAREST) > 0
def small(i):
    g = cv2.resize(cv2.imread(str(run / f"{i:03d}.png"), cv2.IMREAD_UNCHANGED).astype(np.float32), (160, 90), interpolation=cv2.INTER_AREA)
    g = g[small_mask]; return g - g.mean()
vecs = np.array([small(i) for i in at_p])
corr = (vecs @ vecs.T) / np.outer(np.linalg.norm(vecs, axis=1), np.linalg.norm(vecs, axis=1))
med_corr = np.array([np.median(np.delete(corr[k], k)) for k in range(len(at_p))])
ref_idx = at_p[int(np.argmax(med_corr))]
ref = load(ref_idx)
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
    # phase-correlation translation as an extra starting point: ECC from identity diverges on some frames even when
    # the images are near-identical (shoulder_lift run 2026-09-22 16:00 lost 42/47 captures that way)
    (sx, sy), _ = cv2.phaseCorrelate(ref * (mask / 255.0), img * (mask / 255.0))
    pc = np.array([[1, 0, sx], [0, 1, sy]], dtype=np.float32)
    inits = [np.eye(2, 3, dtype=np.float32), prev, pc, rot_about(enc_deg), rot_about(-enc_deg)]
    cc, W = max((ecc(img, img_s, W0) for W0 in inits), key=lambda r: r[0])
    prev = W
    d = W[:, :2] @ PEN + W[:, 2] - PEN
    c.update(dx=float(d[0]), dy=float(d[1]), rot=float(np.degrees(np.arctan2(W[1, 0], W[0, 0]))), cc=cc)

D = lambda rows: np.array([[r["dx"], r["dy"]] for r in rows]) if rows else np.zeros((0, 2))
ph = lambda name: [c for c in caps if c["phase"] == name]
static, stair = ph("static"), [c for c in ph("stair") if c["cc"] > 0.8]
rep, end = [c for c in ph("repeat") if c["cc"] > 0.8], [c for c in ph("static_end") if c["cc"] > 0.8]
failed = [c["idx"] for c in caps if c["cc"] <= 0.8]
if len(stair) < 2:
    print(f"stair phase: only {len(stair)}/{len(ph('stair'))} points passed cc>0.8 (reference_idx={ref_idx}) - "
          "cannot fit px-per-step. Registration is failing on this joint's stair-phase motion specifically, not "
          "just the reference frame; needs a wider search or a different mask, not attempted here.")
    (run / "results2.json").write_text(json.dumps({"summary": {"joint": meta["joint"], "reference_idx": ref_idx,
        "stair_points_used": len(stair), "failed_registrations": failed}, "captures": caps}, indent=1))
    sys.exit(1)
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
     "noise_floor_steps": float(meas(static).std(ddof=1) / abs(slope)), "failed_registrations": failed, "stair_points_used": len(stair),
     "reference_idx": ref_idx, "min_cc_repeat": min((c["cc"] for c in rep), default=None)}
hid_all = []
for name, sign in (("from_above", -1), ("from_below", +1)):
    rows = [c for c in rep if c["approach"] == sign]
    if len(rows) < 2:
        S[name] = {"n": len(rows)}; hid_all.append(np.zeros(0)); continue
    cam = to_steps(meas(rows)); pos = np.array([c["pos"] for c in rows]) - meta["P"]; hid = cam - pos; hid_all.append(hid)
    S[name] = {"n": len(rows), "cam_mean": cam.mean(), "cam_std": cam.std(ddof=1), "enc_mean": pos.mean(), "enc_std": pos.std(ddof=1),
               "hidden_mean": hid.mean(), "hidden_std": hid.std(ddof=1)}
a, bl = S["from_above"], S["from_below"]
if "cam_mean" in a and "cam_mean" in bl:
    gap_cam, gap_enc = a["cam_mean"] - bl["cam_mean"], a["enc_mean"] - bl["enc_mean"]
    S["reversal_error"] = {"camera_steps": gap_cam, "camera_deg": gap_cam * STEP_DEG, "encoder_steps": gap_enc,
                           "hidden_steps": gap_cam - gap_enc, "mm_at_250mm": mm(gap_cam)}
else:
    S["reversal_error"] = None
if len(rep) >= 2:
    allcam = to_steps(meas(rep)); allpos = np.array([c["pos"] for c in rep]) - meta["P"]
    S["all_arrivals"] = {"cam_std_steps": allcam.std(ddof=1), "cam_std_deg": allcam.std(ddof=1) * STEP_DEG, "cam_range_steps": float(np.ptp(allcam)),
                         "mm_at_250mm_1sigma": mm(allcam.std(ddof=1)), "enc_std_steps": allpos.std(ddof=1),
                         "hidden_std_steps": np.concatenate(hid_all).std(ddof=1) if sum(len(h) for h in hid_all) > 1 else None,
                         "corr_cam_enc": float(np.corrcoef(allcam, allpos)[0, 1]) if allpos.std() > 0 else None}
else:
    S["all_arrivals"] = None
S = json.loads(json.dumps(S, default=float))
(run / "results2.json").write_text(json.dumps({"summary": S, "captures": caps}, indent=1))
rnd = lambda o: {k: (round(v, 3) if isinstance(v, float) else v) for k, v in o.items()} if isinstance(o, dict) else (round(o, 4) if isinstance(o, float) else o)
for k, v in S.items(): print(f"{k:22s}", rnd(v))
