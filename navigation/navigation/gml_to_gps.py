#!/usr/bin/env python3
"""CLI tool: convert a GML map file from local ROS coordinates to GPS lat/lon.

Usage (after colcon build):
    ros2 run navigation gml_to_gps --input maps/main.gml --output maps/main_gps.gml

Or directly (no ROS install required):
    python3 gml_to_gps.py --input maps/main.gml --output maps/main_gps.gml

All calibration arguments are optional and default to the values configured in
global_planner.py.  Pass --help for the full option list.
"""

import argparse
import os
import sys

# Allow running directly without a colcon install
sys.path.insert(0, os.path.dirname(__file__))
import simple_gps_util


# ── Default calibration values (mirror global_planner.py) ──────────────────
_DEFAULT_ANCHOR_GPS = [38.43390310669193, -78.86210645708111]
_DEFAULT_ANCHOR_LOCAL = [1.038224697113037, 5.543598651885986]
_DEFAULT_TEST_GPS = [
    38.43155945255256, -78.86153456400355,  # B side of Chesapeake Hall
    38.43270016761392, -78.85989794981512,  # Festival
    38.43303883146722, -78.8621951163814,   # Astronomy Park
    38.43942688381657, -78.8751250905651,   # The Quad (inaccurate. Needs recalibration once main is mapped with the LiDAR)
]
_DEFAULT_TEST_LOCAL = [
    249.616943359375,   93.49036407470703,   # B side of Chesapeake Hall
    105.86905670166016, 217.34902954101562,  # Festival
    98.11779022216797,  13.654234886169434,  # Astronomy Park
    -446.297607421875, -1209.2774658203125,  # The Quad (inaccurate. Needs recalibration once main is mapped with the LiDAR)
]


def _pairs(flat):
    """Convert a flat list [a, b, c, d, ...] to [(a, b), (c, d), ...]."""
    return [(flat[i], flat[i + 1]) for i in range(0, len(flat), 2)]


def _derive_output_path(input_path):
    base, ext = os.path.splitext(input_path)
    return base + "_gps" + ext


def main():
    parser = argparse.ArgumentParser(
        description="Convert a GML map from local ROS coordinates to GPS lat/lon."
    )
    parser.add_argument(
        "--input", "-i",
        required=True,
        help="Path to the source .gml file (local ROS coordinates)",
    )
    parser.add_argument(
        "--output", "-o",
        default=None,
        help="Path to write the converted .gml file. "
             "Defaults to <input_stem>_gps.gml in the same directory.",
    )
    parser.add_argument(
        "--anchor-gps",
        nargs=2, type=float, metavar=("LAT", "LON"),
        default=_DEFAULT_ANCHOR_GPS,
        help="GPS coordinates (lat lon) of the map anchor point",
    )
    parser.add_argument(
        "--anchor-local",
        nargs=2, type=float, metavar=("X", "Y"),
        default=_DEFAULT_ANCHOR_LOCAL,
        help="Local map coordinates (x y) of the map anchor point",
    )
    parser.add_argument(
        "--test-gps",
        nargs="+", type=float, metavar="VAL",
        default=_DEFAULT_TEST_GPS,
        help="Flat list of test-point GPS coords: lat1 lon1 lat2 lon2 ...",
    )
    parser.add_argument(
        "--test-local",
        nargs="+", type=float, metavar="VAL",
        default=_DEFAULT_TEST_LOCAL,
        help="Flat list of test-point local coords: x1 y1 x2 y2 ...",
    )
    args = parser.parse_args()

    if len(args.test_gps) % 2 != 0 or len(args.test_local) % 2 != 0:
        parser.error("--test-gps and --test-local must each have an even number of values")
    if len(args.test_gps) != len(args.test_local):
        parser.error("--test-gps and --test-local must have the same number of values")

    output_path = args.output or _derive_output_path(args.input)

    # Compute heading offset from the calibration test points
    anchor_theta = simple_gps_util.calibrate_util_multi(
        _pairs(args.test_local),
        args.anchor_local,
        _pairs(args.test_gps),
        args.anchor_gps,
    )
    n_pts = len(args.test_gps) // 2
    print(f"Calibrated heading offset: {anchor_theta:.6f}°  ({n_pts} test point(s))")

    simple_gps_util.convert_gml_to_gps(
        input_path=args.input,
        output_path=output_path,
        anchor_gps=args.anchor_gps,
        anchor_local=args.anchor_local,
        anchor_theta=anchor_theta,
    )
    print(f"Written: {output_path}")


if __name__ == "__main__":
    main()
