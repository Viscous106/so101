"""Analyse a collect_v2.py run from the fixed THIRD-PERSON camera (t###.png): track the tool, not the scene.

  analyze3.py runs/<dir>

The C270 is static, so the arm's jaws + marker are the moving object. The red marker tip is found in the *most
representative* at-P frame (see pick_reference below, not blindly frame 0 - a person walked into frame 0 of the
2026-09-22 shoulder_lift run) by shape/size (a compact blob of ~5000-20000px not touching the frame border, nearest
the orange jaws - a plain "biggest red blob" or "nearest orange" test each pick a wrong background object on at
least one of the three 2026-09-22 runs). A tool mask (convex hull of orange + red near the tip, dilated) restricts
ECC registration (euclidean) to the tool, so people and the window behind the arm do not matter. Numbers come out in
the same units as analyze2.py: px per encoder step from the stair, then reversal error / scatter of repeat arrivals.
"""
import json, sys, pathlib
import cv2, numpy as np

run = pathlib.Path(sys.argv[1])
meta = json.loads((run / "meta.json").read_text())
caps = [json.loads(l) for l in open(run / "captures.jsonl")]
STEP_DEG = 360 / 4096
assert (run / "t000.png").exists(), "no third-person frames in this run"


def find_tip(bgr):
    """(x,y,w,h) of the marker's red tip, or None. Candidates: relaxed red, 5000-20000px (measured 10600-11050
    across three runs), aspect 0.4-2.5, not touching the frame border (rules out the tan table-edge trim and other
    warm-toned background objects, which are otherwise often bigger and nearer the jaws than the true tip)."""
    H, W = bgr.shape[:2]
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    orange = cv2.inRange(hsv, (5, 120, 120), (25, 255, 255))
    red = cv2.inRange(hsv, (0, 80, 60), (10, 255, 255)) | cv2.inRange(hsv, (165, 80, 60), (180, 255, 255))
    n, _, stats, cents = cv2.connectedComponentsWithStats(red)

    def ok(i):
        x, y, w, h = stats[i, :4]
        return 5000 <= stats[i, 4] <= 20000 and 0.4 <= w / max(1, h) <= 2.5 and x > 2 and y > 2 and x + w < W - 2 and y + h < H - 2

    cands = [i for i in range(1, n) if ok(i)]
    if not cands:
        return None, orange, red
    dist = cv2.distanceTransform(255 - orange, cv2.DIST_L2, 3)
    i = min(cands, key=lambda i: dist[int(cents[i][1]), int(cents[i][0])])
    return tuple(int(v) for v in stats[i, :4]), orange, red


def pick_reference():
    """The best reference frame among the ones nominally at P (static/repeat/static_end - collect_v2 always passes
    goal=P for these; only stair frames are at P+s): the one with highest median pairwise correlation to the others
    on a small downsample, i.e. least likely to have a person or a lighting change in it that the rest do not share.
    Cheap (no ECC) and does not need find_tip first. A mean-brightness proxy is not enough: on the 2026-09-22 runs
    it picked frames with the right average level but a different, wrong scene."""
    idxs = [c["idx"] for c in caps if c["phase"] != "stair"]
    def small(i):
        g = cv2.resize(cv2.imread(str(run / f"t{i:03d}.png"), cv2.IMREAD_UNCHANGED).astype(np.float32), (160, 90), interpolation=cv2.INTER_AREA)
        return g.ravel() - g.mean()
    vecs = np.array([small(i) for i in idxs])
    corr = (vecs @ vecs.T) / np.outer(np.linalg.norm(vecs, axis=1), np.linalg.norm(vecs, axis=1))
    med_corr = np.array([np.median(np.delete(corr[k], k)) for k in range(len(idxs))])
    return idxs[int(np.argmax(med_corr))]


ref_idx = pick_reference()
# collect_v2 only saves a colour frame (reference_third.jpg) for idx 0, needed for find_tip's hue test; every other
# frame is saved as grayscale-only (t###.png). Tip position is measured on frame 0's colour image regardless of
# which frame ref_idx picks for the actual ECC registration below - the roi/tool built around it have generous
# margins and the tip does not move enough between frames of the same run to matter.
tip_bgr = cv2.imread(str(run / "reference_third.jpg"))
H, W = tip_bgr.shape[:2]
tip, orange, red = find_tip(tip_bgr)
assert tip is not None, "no red marker tip found (checked frame 0 only; rerun failed if the true reference frame differs)"
x, y, w, h = tip
TIP = np.array([x + w / 2, y + h])                      # bottom of the red tip, full-frame pixels
x0, x1 = max(0, x - 220), min(W, x + w + 220)
y0, y1 = max(0, y - 420), min(H, y + h + 40)            # marker body and the jaws holding it are above the tip
roi = (slice(y0, y1), slice(x0, x1))
# tool mask = convex hull of the orange (jaws) and red (tip) pixels inside the ROI: it covers the white-taped marker
# between them without picking up the bright window behind the arm (a brightness test did)
pts = cv2.findNonZero((orange | red)[roi])
tool = np.zeros((y1 - y0, x1 - x0), np.uint8)
cv2.fillConvexPoly(tool, cv2.convexHull(pts), 255)
tool = cv2.dilate(tool, np.ones((15, 15), np.uint8))
tool[:8], tool[-8:], tool[:, :8], tool[:, -8:] = 0, 0, 0, 0
CENTER = TIP - np.array([x0, y0])                       # report displacement at the pen tip (ROI coordinates)
cv2.imwrite(str(run / "tool_mask_third.png"), np.dstack([tool] * 3) // 2 + tip_bgr[roi] // 2)


def load(i):
    g = cv2.imread(str(run / f"t{i:03d}.png"), cv2.IMREAD_UNCHANGED).astype(np.float32)[roi] / 256
    g = cv2.GaussianBlur(g, (0, 0), 1.2)
    return g - cv2.GaussianBlur(g, (0, 0), 15)


CRIT = (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 400, 1e-8)
ref = load(ref_idx)
prev = np.eye(2, 3, dtype=np.float32)
for c in caps:
    img = load(c["idx"])
    (sx, sy), _ = cv2.phaseCorrelate(ref * (tool / 255.0), img * (tool / 255.0))
    pc = np.array([[1, 0, sx], [0, 1, sy]], dtype=np.float32)
    best = (-1.0, prev)
    for W0 in (np.eye(2, 3, dtype=np.float32), prev, pc):
        try:
            cc, Wm = cv2.findTransformECC(ref, img, W0.copy(), cv2.MOTION_EUCLIDEAN, CRIT, tool, 5)
            if cc > best[0]:
                best = (float(cc), Wm)
        except cv2.error:
            pass
    cc, Wm = best
    prev = Wm if cc > 0 else prev
    d = Wm[:, :2] @ CENTER + Wm[:, 2] - CENTER
    c.update(dx=float(d[0]), dy=float(d[1]), rot=float(np.degrees(np.arctan2(Wm[1, 0], Wm[0, 0]))), cc=cc)

D = lambda rows: np.array([[r["dx"], r["dy"]] for r in rows]) if rows else np.zeros((0, 2))
ph = lambda name: [c for c in caps if c["phase"] == name]
static, stair, rep = ph("static"), [c for c in ph("stair") if c["cc"] > 0.7], ph("repeat")
failed = [c["idx"] for c in caps if c["cc"] <= 0.7]
sd = D(stair); u = np.linalg.svd(sd - sd.mean(0))[2][0]
if (sd @ u)[-1] < (sd @ u)[0]: u = -u
meas = lambda rows: D(rows) @ u
sp = np.array([c["pos"] for c in stair])
slope, icpt = np.polyfit(sp, meas(stair), 1)
resid = meas(stair) - (slope * sp + icpt)
to_steps = lambda v: v / slope
mm = lambda steps, reach=250: abs(steps) * STEP_DEG * np.pi / 180 * reach

S = {"joint": meta["joint"], "camera": "third-person", "P": meta["P"], "roi": [int(x0), int(y0), int(x1), int(y1)], "tip_px": TIP.tolist(), "reference_idx": ref_idx,
     "per_step_px": slope, "stair_r2": 1 - resid.var() / meas(stair).var(), "stair_resid_rms_steps": float(np.sqrt((resid ** 2).mean()) / abs(slope)),
     "noise_floor_steps": float(meas(static).std(ddof=1) / abs(slope)), "failed_registrations": failed, "min_cc_repeat": min(c["cc"] for c in rep)}
hid_all = []
for name, sign in (("from_above", -1), ("from_below", +1)):
    rows = [c for c in rep if c["approach"] == sign and c["cc"] > 0.7]
    if len(rows) < 2:
        S[name] = {"n": len(rows)}; hid_all.append(np.zeros(0)); continue
    cam = to_steps(meas(rows)); pos = np.array([c["pos"] for c in rows]) - meta["P"]; hid = cam - pos; hid_all.append(hid)
    S[name] = {"n": len(rows), "cam_mean": cam.mean(), "cam_std": cam.std(ddof=1), "enc_mean": pos.mean(), "enc_std": pos.std(ddof=1),
               "hidden_mean": hid.mean(), "hidden_std": hid.std(ddof=1)}
a, bl = S["from_above"], S["from_below"]
if "cam_mean" in a and "cam_mean" in bl:
    gap_cam, gap_enc = a["cam_mean"] - bl["cam_mean"], a["enc_mean"] - bl["enc_mean"]
    S["reversal_error"] = {"camera_steps": gap_cam, "camera_deg": gap_cam * STEP_DEG, "encoder_steps": gap_enc, "hidden_steps": gap_cam - gap_enc, "mm_at_250mm": mm(gap_cam)}
else:
    S["reversal_error"] = None
good = [c for c in rep if c["cc"] > 0.7]
if len(good) >= 2:
    allcam = to_steps(meas(good)); allpos = np.array([c["pos"] for c in good]) - meta["P"]
    S["all_arrivals"] = {"cam_std_steps": allcam.std(ddof=1), "cam_std_deg": allcam.std(ddof=1) * STEP_DEG, "cam_range_steps": float(np.ptp(allcam)),
                         "mm_at_250mm_1sigma": mm(allcam.std(ddof=1)), "enc_std_steps": allpos.std(ddof=1),
                         "hidden_std_steps": np.concatenate(hid_all).std(ddof=1) if sum(len(h) for h in hid_all) > 1 else None,
                         "corr_cam_enc": float(np.corrcoef(allcam, allpos)[0, 1]) if allpos.std() > 0 else None}
else:
    S["all_arrivals"] = None
S = json.loads(json.dumps(S, default=float))
(run / "results3.json").write_text(json.dumps({"summary": S, "captures": caps}, indent=1))
rnd = lambda o: {k: (round(v, 3) if isinstance(v, float) else v) for k, v in o.items()} if isinstance(o, dict) else (round(o, 4) if isinstance(o, float) else o)
for k, v in S.items(): print(f"{k:22s}", rnd(v))
