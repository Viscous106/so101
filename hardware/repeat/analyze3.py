"""Analyse a collect_v2.py run from the fixed THIRD-PERSON camera (t###.png): track the tool, not the scene.

  analyze3.py runs/<dir>

The C270 is static, so the arm's jaws + marker are the moving object. Frame 0 defines a region of interest around
the jaws (leftmost orange blob, extended upward to take in the marker) and a tool mask (orange + red + the marker's
white tape, dilated); every later frame is registered to frame 0 with ECC (euclidean) using only the tool pixels,
so people and the window behind the arm do not matter. Numbers come out in the same units as analyze2.py:
px per encoder step from the stair, then reversal error / scatter of the repeat arrivals in steps and degrees.
"""
import json, sys, pathlib
import cv2, numpy as np

run = pathlib.Path(sys.argv[1])
meta = json.loads((run / "meta.json").read_text())
caps = [json.loads(l) for l in open(run / "captures.jsonl")]
STEP_DEG = 360 / 4096
assert (run / "t000.png").exists(), "no third-person frames in this run"

ref_bgr = cv2.imread(str(run / "reference_third.jpg"))
H, W = ref_bgr.shape[:2]

# --- tool segmentation on the colour reference frame ---------------------------------------------------------------
hsv = cv2.cvtColor(ref_bgr, cv2.COLOR_BGR2HSV)
orange = cv2.inRange(hsv, (5, 120, 120), (25, 255, 255))
red = cv2.inRange(hsv, (0, 120, 100), (5, 255, 255)) | cv2.inRange(hsv, (170, 120, 100), (180, 255, 255))
# anchor on the marker's red tip (the largest red blob): it is the point that draws, and it is rigid with the jaws
# for every joint including wrist_flex, unlike the orange forearm that fills the frame when the camera is close.
n, lab, stats, cents = cv2.connectedComponentsWithStats(red)
reds = [(stats[i, cv2.CC_STAT_AREA], i) for i in range(1, n) if stats[i, cv2.CC_STAT_AREA] > 150]
assert reds, "no red marker tip found in reference_third.jpg"
tip_i = max(reds)[1]
x, y, w, h = (int(stats[tip_i, k]) for k in (cv2.CC_STAT_LEFT, cv2.CC_STAT_TOP, cv2.CC_STAT_WIDTH, cv2.CC_STAT_HEIGHT))
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
cv2.imwrite(str(run / "tool_mask_third.png"), np.dstack([tool] * 3) // 2 + ref_bgr[roi] // 2)


def load(i):
    g = cv2.imread(str(run / f"t{i:03d}.png"), cv2.IMREAD_UNCHANGED).astype(np.float32)[roi] / 256
    g = cv2.GaussianBlur(g, (0, 0), 1.2)
    return g - cv2.GaussianBlur(g, (0, 0), 15)


CRIT = (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 400, 1e-8)
ref = load(0)
prev = np.eye(2, 3, dtype=np.float32)
for c in caps:
    img = load(c["idx"])
    best = (-1.0, prev)
    for W0 in (np.eye(2, 3, dtype=np.float32), prev):
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

D = lambda rows: np.array([[r["dx"], r["dy"]] for r in rows])
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

S = {"joint": meta["joint"], "camera": "third-person", "P": meta["P"], "roi": [int(x0), int(y0), int(x1), int(y1)], "tip_px": TIP.tolist(),
     "per_step_px": slope, "stair_r2": 1 - resid.var() / meas(stair).var(), "stair_resid_rms_steps": float(np.sqrt((resid ** 2).mean()) / abs(slope)),
     "noise_floor_steps": float(meas(static).std(ddof=1) / abs(slope)), "failed_registrations": failed, "min_cc_repeat": min(c["cc"] for c in rep)}
hid_all = []
for name, sign in (("from_above", -1), ("from_below", +1)):
    rows = [c for c in rep if c["approach"] == sign and c["cc"] > 0.7]
    cam = to_steps(meas(rows)); pos = np.array([c["pos"] for c in rows]) - meta["P"]; hid = cam - pos; hid_all.append(hid)
    S[name] = {"n": len(rows), "cam_mean": cam.mean(), "cam_std": cam.std(ddof=1), "enc_mean": pos.mean(), "enc_std": pos.std(ddof=1),
               "hidden_mean": hid.mean(), "hidden_std": hid.std(ddof=1)}
a, bl = S["from_above"], S["from_below"]
gap_cam, gap_enc = a["cam_mean"] - bl["cam_mean"], a["enc_mean"] - bl["enc_mean"]
S["reversal_error"] = {"camera_steps": gap_cam, "camera_deg": gap_cam * STEP_DEG, "encoder_steps": gap_enc, "hidden_steps": gap_cam - gap_enc, "mm_at_250mm": mm(gap_cam)}
good = [c for c in rep if c["cc"] > 0.7]
allcam = to_steps(meas(good)); allpos = np.array([c["pos"] for c in good]) - meta["P"]
S["all_arrivals"] = {"cam_std_steps": allcam.std(ddof=1), "cam_std_deg": allcam.std(ddof=1) * STEP_DEG, "cam_range_steps": float(np.ptp(allcam)),
                     "mm_at_250mm_1sigma": mm(allcam.std(ddof=1)), "enc_std_steps": allpos.std(ddof=1),
                     "hidden_std_steps": np.concatenate(hid_all).std(ddof=1), "corr_cam_enc": float(np.corrcoef(allcam, allpos)[0, 1]) if allpos.std() > 0 else None}
S = json.loads(json.dumps(S, default=float))
(run / "results3.json").write_text(json.dumps({"summary": S, "captures": caps}, indent=1))
rnd = lambda o: {k: (round(v, 3) if isinstance(v, float) else v) for k, v in o.items()} if isinstance(o, dict) else (round(o, 4) if isinstance(o, float) else o)
for k, v in S.items(): print(f"{k:22s}", rnd(v))
