"""Trace the supplied cartoon as connected pen outlines, without hardware."""
import argparse
import pathlib
import shutil

import cv2
import numpy as np

from draw_reference import ROOT, save_json

OUT = ROOT / "hardware/draw/runs/reference_OjPuJYdomotd"


def arc(cx, cy, radius, start=0, end=360, count=40):
    angles = np.deg2rad(np.linspace(start, end, count))
    return np.column_stack((cx + radius * np.cos(angles), cy + radius * np.sin(angles))).tolist()


def rounded_rect(left, top, right, bottom, radius):
    points = []
    for x, y, start in ((right-radius, top+radius, -90), (right-radius, bottom-radius, 0),
                        (left+radius, bottom-radius, 90), (left+radius, top+radius, 180)):
        points += arc(x, y, radius, start, start+90, 10)
    return points + [points[0]]


def paths():
    return [
        ("head", rounded_rect(59, 33, 109, 70, 8)),
        ("face", rounded_rect(65, 39, 103, 64, 4)),
        ("left_eye", arc(75, 51, 3.4)),
        ("right_eye", arc(94, 51, 3.4)),
        ("antenna_stem", [[84, 33], [84, 25]]),
        ("antenna", arc(84, 19, 6)),
        ("left_ear", [[59, 41]] + arc(57, 47, 6, -90, -180, 8)
         + [[51, 55]] + arc(57, 55, 6, 180, 90, 8) + [[59, 61]]),
        ("right_ear", [[109, 41]] + arc(111, 47, 6, -90, 0, 8)
         + [[117, 55]] + arc(111, 55, 6, 0, 90, 8) + [[109, 61]]),
        ("left_neck", [[77, 70], [77, 75]]),
        ("right_neck", [[92, 70], [92, 75]]),
        ("body", rounded_rect(56, 75, 112, 122, 10)),
        ("chest", arc(84, 98, 12)),
        ("left_shoulder", arc(52, 88, 6, 65, 295, 26)),
        ("right_shoulder", arc(116, 88, 6, -115, 115, 26)),
        ("left_arm_claw", [[49, 94], [46, 103]] + arc(40, 115, 11, -58, -180, 22)),
        ("left_claw_inner", arc(40, 115, 6, 90, -180, 25)),
        ("right_arm_claw", [[119, 94], [122, 103]] + arc(128, 115, 11, -122, 0, 22)),
        ("right_claw_inner", arc(128, 115, 6, 90, 360, 25)),
        ("left_leg", [[69, 122], [69, 136], [78, 136], [78, 122]]),
        ("right_leg", [[90, 122], [90, 136], [99, 136], [99, 122]]),
        ("left_foot", [[59, 144], [59, 142]] + arc(67, 142, 8, 180, 270, 10)
         + [[75, 134]] + arc(75, 142, 8, 270, 360, 10) + [[83, 144], [59, 144]]),
        ("right_foot", [[85, 144], [85, 142]] + arc(93, 142, 8, 180, 270, 10)
         + [[101, 134]] + arc(101, 142, 8, 270, 360, 10) + [[109, 144], [85, 144]]),
    ]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--reference", type=pathlib.Path, default=pathlib.Path("/tmp/so101-new-reference.png"))
    args = parser.parse_args()
    OUT.mkdir(parents=True, exist_ok=True)
    shutil.copyfile(args.reference, OUT / "reference.png")
    strokes = [{"name": name, "points": points} for name, points in paths()]
    save_json(OUT / "outline.json", {"source": "https://www.pasteboard.co/OjPuJYdomotd.png",
              "mode": "monochrome_vector_trace", "hardware_execution_allowed": False,
              "reason": "Contact and geometric validation not yet passed", "strokes": strokes})
    preview = np.full((640, 688, 3), 248, np.uint8)
    for stroke in strokes:
        points = np.round(np.asarray(stroke["points"]) * 4).astype(np.int32)
        cv2.polylines(preview, [points], False, (45, 39, 30), 3, cv2.LINE_AA)
    cv2.imwrite(str(OUT / "outline_preview.png"), preview)
    print(f"Prepared {len(strokes)} continuous outlines; hardware execution remains disabled")


if __name__ == "__main__":
    main()
