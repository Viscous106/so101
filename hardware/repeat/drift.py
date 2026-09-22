"""Is the camera-minus-encoder residual random scatter or slow drift?  drift.py"""
import json, glob, numpy as np

for pat in ("runs/shoulder_pan_2*", "runs/shoulder_pan_comp_2*", "runs/wrist_roll_2*"):
    r = json.load(open(sorted(glob.glob(pat + "/results2.json"))[-1])); S, caps = r["summary"], r["captures"]
    k, P, rot = S["per_step"], S["P"], S["joint"] == "wrist_roll"
    sd = np.array([[c["dx"], c["dy"]] for c in caps if c["phase"] == "stair" and c["cc"] > 0.8])
    uu = np.linalg.svd(sd - sd.mean(0))[2][0]
    if (sd @ uu)[-1] < (sd @ uu)[0]: uu = -uu
    m = lambda c: (c["rot"] if rot else c["dx"] * uu[0] + c["dy"] * uu[1]) / k
    rows = [c for c in caps if c["cc"] > 0.8]
    t = np.array([(c["t"] - rows[0]["t"]) / 60 for c in rows]); hid = np.array([m(c) - (c["pos"] - P) for c in rows])
    rep = np.array([c["phase"] == "repeat" for c in rows]); ap = np.array([c["approach"] for c in rows])[rep]
    a, b = np.polyfit(t[rep], hid[rep], 1); det = hid[rep] - (a * t[rep] + b)
    tag = " (compensated)" if S["compensated"] else ""
    print(f"\n{S['joint']}{tag}: hidden residual = camera - encoder, in steps")
    print("  by minute:", " ".join(f"{mm}m:{hid[(t >= mm) & (t < mm + 1)].mean():+.2f}" for mm in range(int(t.max()) + 1) if ((t >= mm) & (t < mm + 1)).any()))
    print(f"  repeat phase: drift {a:+.3f} steps/min; hidden std raw {hid[rep].std(ddof=1):.2f} -> detrended {det.std(ddof=1):.2f} (encoder rounding alone = 0.29)")
    print(f"  direction-dependent hidden gap after detrending: {det[ap == -1].mean() - det[ap == 1].mean():+.2f} steps")
