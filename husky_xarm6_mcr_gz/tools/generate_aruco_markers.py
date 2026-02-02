#!/usr/bin/env python3
"""
Generate ArUco marker PNG images for Gazebo textures or printing.

Example:
  ros2 run husky_xarm6_mcr_gz generate_aruco_markers \
    --dict DICT_4X4_50 --ids 0 1 --size_px 800 --border_bits 1

By default, writes to:
  ./models/tree_fiducials/materials/textures/
"""

from __future__ import annotations

import os
import argparse
from typing import List

try:
    import cv2
except Exception as e:
    raise RuntimeError(
        "OpenCV not found. Install python3-opencv (apt) or opencv-contrib-python (pip)."
    ) from e


# Map friendly dict names to OpenCV constants
ARUCO_DICT_MAP = {
    "DICT_4X4_50": cv2.aruco.DICT_4X4_50,
    "DICT_4X4_100": cv2.aruco.DICT_4X4_100,
    "DICT_4X4_250": cv2.aruco.DICT_4X4_250,
    "DICT_4X4_1000": cv2.aruco.DICT_4X4_1000,
    "DICT_5X5_50": cv2.aruco.DICT_5X5_50,
    "DICT_5X5_100": cv2.aruco.DICT_5X5_100,
    "DICT_5X5_250": cv2.aruco.DICT_5X5_250,
    "DICT_5X5_1000": cv2.aruco.DICT_5X5_1000,
    "DICT_6X6_50": cv2.aruco.DICT_6X6_50,
    "DICT_6X6_100": cv2.aruco.DICT_6X6_100,
    "DICT_6X6_250": cv2.aruco.DICT_6X6_250,
    "DICT_6X6_1000": cv2.aruco.DICT_6X6_1000,
    "DICT_7X7_50": cv2.aruco.DICT_7X7_50,
    "DICT_7X7_100": cv2.aruco.DICT_7X7_100,
    "DICT_7X7_250": cv2.aruco.DICT_7X7_250,
    "DICT_7X7_1000": cv2.aruco.DICT_7X7_1000,
    "DICT_ARUCO_ORIGINAL": cv2.aruco.DICT_ARUCO_ORIGINAL,
    "DICT_APRILTAG_16h5": cv2.aruco.DICT_APRILTAG_16h5,
    "DICT_APRILTAG_25h9": cv2.aruco.DICT_APRILTAG_25h9,
    "DICT_APRILTAG_36h10": cv2.aruco.DICT_APRILTAG_36h10,
    "DICT_APRILTAG_36h11": cv2.aruco.DICT_APRILTAG_36h11,
}


def default_out_dir() -> str:
    return os.path.dirname(os.path.abspath(__file__)).replace(
        "tools", "models/tree_fiducials/materials/textures"
    )


def ensure_aruco_available() -> None:
    if not hasattr(cv2, "aruco"):
        raise RuntimeError(
            "Your OpenCV build does not include cv2.aruco. "
            "Install opencv-contrib-python (pip) or a system OpenCV build with aruco."
        )


def generate_marker_png(
    aruco_dict_name: str,
    marker_id: int,
    size_px: int,
    border_bits: int,
    out_path: str,
) -> None:
    ensure_aruco_available()

    if aruco_dict_name not in ARUCO_DICT_MAP:
        raise ValueError(
            f"Unknown dict '{aruco_dict_name}'. Choose from: {', '.join(ARUCO_DICT_MAP.keys())}"
        )

    dict_id = ARUCO_DICT_MAP[aruco_dict_name]
    aruco_dict = cv2.aruco.getPredefinedDictionary(dict_id)

    # Create image (single channel)
    img = cv2.aruco.drawMarker(aruco_dict, marker_id, size_px, borderBits=border_bits)

    os.makedirs(os.path.dirname(out_path), exist_ok=True)
    ok = cv2.imwrite(out_path, img)
    if not ok:
        raise RuntimeError(f"Failed to write image to {out_path}")


def parse_ids(ids: List[int] | None, start: int, count: int) -> List[int]:
    if ids and len(ids) > 0:
        return ids
    return list(range(start, start + count))


def main() -> int:
    ap = argparse.ArgumentParser(description="Generate ArUco marker PNGs.")
    ap.add_argument("--dict", dest="dict_name", default="DICT_4X4_50",
                    help=f"Aruco dictionary. Options: {', '.join(ARUCO_DICT_MAP.keys())}")
    ap.add_argument("--ids", type=int, nargs="*", default=None,
                    help="Marker IDs to generate. If omitted, uses --start and --count.")
    ap.add_argument("--start", type=int, default=0, help="Start ID if --ids not provided.")
    ap.add_argument("--count", type=int, default=2, help="How many IDs if --ids not provided.")
    ap.add_argument("--size_px", type=int, default=800, help="Marker size in pixels (square).")
    ap.add_argument("--border_bits", type=int, default=1, help="Border bits (usually 1).")
    ap.add_argument("--out_dir", type=str, default="", help="Output directory. Defaults to package textures dir.")
    ap.add_argument("--prefix", type=str, default="aruco", help="Filename prefix.")
    args = ap.parse_args()

    ids = parse_ids(args.ids, args.start, args.count)
    out_dir = args.out_dir if args.out_dir else default_out_dir()

    print(f"Dictionary: {args.dict_name}")
    print(f"IDs: {ids}")
    print(f"Output dir: {out_dir}")
    print(f"size_px={args.size_px}, border_bits={args.border_bits}\n")

    for mid in ids:
        out_path = os.path.join(out_dir, f"{args.prefix}_{args.dict_name}_id{mid:04d}.png")
        generate_marker_png(args.dict_name, mid, args.size_px, args.border_bits, out_path)
        print(f"Wrote {out_path}")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
