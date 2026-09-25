"""Photo -> simplified line-art strokes ready for the drawing pipeline.

  vectorize.py <image> [out.json]

Pipeline: GrabCut isolates the subject from the background (a real photo's background is far too dense to
trace); bilateral filter + Canny gets internal features (face, collar, shirt) without fabric-texture noise;
the mask's own outer contour gives the silhouette. Each resulting line is simplified with approxPolyDP (a
robot arm should not trace every pixel of a Canny edge - that is thousands of near-collinear micro-segments)
and short noise fragments are dropped. Strokes are greedily reordered to reduce pen-up travel between them.

Output: {"strokes": [[[x,y],...], ...], "image_size": [w,h]} in source-image pixel coordinates - the next
stage (not this file) maps those into the real drawing plane.
"""
import json, sys
import cv2
import numpy as np

BOTTOM_TRIM_FRAC = 0.04   # drop the bottom strip - flowerpot-rim noise behind the hands, not the subject
SIMPLIFY_EPS = 1.5        # px, approxPolyDP tolerance
MIN_STROKE_LEN = 15       # px, drop fragments shorter than this (Canny speckle, not a real feature)
CANNY_LO, CANNY_HI = 40, 110


def segment(img):
    h, w = img.shape[:2]
    mask = np.zeros((h, w), np.uint8)
    # GrabCut is sensitive here: a few px of rect difference changed whether a background glass panel near the
    # shoulder got pulled into the foreground GMM - (20,10,w-40,h-20) is the exact rect verified clean.
    rect = (20, 10, w - 40, h - 20)
    bgd, fgd = np.zeros((1, 65), np.float64), np.zeros((1, 65), np.float64)
    cv2.grabCut(img, mask, rect, bgd, fgd, 8, cv2.GC_INIT_WITH_RECT)
    fg = np.where((mask == cv2.GC_FGD) | (mask == cv2.GC_PR_FGD), 255, 0).astype(np.uint8)
    # (7,7)/(5,5): a 9x9 close was verified to bridge the silhouette across the gap to a piece of the background
    # building near the shoulder, merging it into "largest component" - this exact size was checked clean.
    fg = cv2.morphologyEx(fg, cv2.MORPH_CLOSE, np.ones((7, 7), np.uint8))
    fg = cv2.morphologyEx(fg, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))
    n, lab, stats, _ = cv2.connectedComponentsWithStats(fg)
    biggest = 1 + int(np.argmax(stats[1:, cv2.CC_STAT_AREA]))
    fg = np.where(lab == biggest, 255, 0).astype(np.uint8)
    fg[int(h * (1 - BOTTOM_TRIM_FRAC)):, :] = 0
    return fg


def line_art(img, mask):
    smooth = cv2.bilateralFilter(img, 9, 60, 60)
    gray = cv2.cvtColor(smooth, cv2.COLOR_BGR2GRAY)
    edges = cv2.Canny(gray, CANNY_LO, CANNY_HI)
    edges[mask == 0] = 0
    return edges


def extract_strokes(edges, mask):
    """approxPolyDP needs a single ordered contour, not a scattered edge map - findContours on the *edges*
    themselves (not just the mask) gives each connected line its own ordered point sequence, which is the
    representation a pen can actually follow. Closed contours (the silhouette) simplify with closed=True."""
    strokes = []
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    for c in contours:
        approx = cv2.approxPolyDP(c, SIMPLIFY_EPS, closed=True)
        if cv2.arcLength(c, True) >= MIN_STROKE_LEN:
            strokes.append(("silhouette", approx.reshape(-1, 2)))
    contours, _ = cv2.findContours(edges, cv2.RETR_LIST, cv2.CHAIN_APPROX_NONE)
    for c in contours:
        if cv2.arcLength(c, False) < MIN_STROKE_LEN:
            continue
        approx = cv2.approxPolyDP(c, SIMPLIFY_EPS, closed=False)
        strokes.append(("feature", approx.reshape(-1, 2)))
    return strokes


def reorder(strokes):
    """Greedy nearest-endpoint ordering to cut pen-up travel; a stroke may be walked in either direction."""
    remaining = list(strokes)
    ordered = [remaining.pop(0)]
    while remaining:
        tail = ordered[-1][1][-1]
        best_i, best_flip, best_d = 0, False, np.inf
        for i, (_, pts) in enumerate(remaining):
            for flip, end in ((False, pts[0]), (True, pts[-1])):
                d = np.hypot(*(end - tail))
                if d < best_d:
                    best_i, best_flip, best_d = i, flip, d
        kind, pts = remaining.pop(best_i)
        ordered.append((kind, pts[::-1] if best_flip else pts))
    return ordered


if __name__ == "__main__":
    src, out = sys.argv[1], sys.argv[2] if len(sys.argv) > 2 else "strokes.json"
    img = cv2.imread(src)
    assert img is not None, f"cannot read {src}"
    mask = segment(img)
    edges = line_art(img, mask)
    strokes = extract_strokes(edges, mask)
    strokes = reorder(strokes)

    vis = np.full_like(img, 255)
    for kind, pts in strokes:
        cv2.polylines(vis, [pts.reshape(-1, 1, 2)], kind == "silhouette", (0, 0, 0), 1, cv2.LINE_AA)
    cv2.imwrite("assets/strokes_preview.png", vis)

    total_pts = sum(len(pts) for _, pts in strokes)
    total_len = sum(cv2.arcLength(pts.reshape(-1, 1, 2), False) for _, pts in strokes)
    print(f"{len(strokes)} strokes, {total_pts} points, {total_len:.0f}px total path length")
    json.dump({"strokes": [pts.tolist() for _, pts in strokes], "image_size": [img.shape[1], img.shape[0]]}, open(out, "w"))
    print("wrote", out)
