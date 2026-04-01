#!/usr/bin/env python3
"""CLI tool: convert a GML map file from local ROS coordinates to GPS lat/lon.

Usage (after colcon build):
    ros2 run navigation gml_to_gps --input maps/main.gml --output maps/main_gps.gml

Or directly (no ROS install required):
    python3 gml_to_gps.py --input maps/main.gml --output maps/main_gps.gml

Calibration landmarks are loaded from a YAML config file. Pass --help for the
full option list.
"""

import argparse
import os
import sys

# Allow running directly without a colcon install
sys.path.insert(0, os.path.dirname(__file__))
import simple_gps_util

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
        "--config-dir",
        default=os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "maps")),
        help="Directory containing the landmark calibration YAML file.",
    )
    parser.add_argument(
        "--config-file",
        default="main_graph_config.yaml",
        help="Landmark calibration YAML file name.",
    )
    args = parser.parse_args()

    output_path = args.output or _derive_output_path(args.input)
    config_path = simple_gps_util.resolve_config_path(args.config_dir, args.config_file)
    test_points_local, test_points_gps = simple_gps_util.load_landmark_calibration(
        config_path
    )

    # Compute calibration parameters from the landmark test points
    ref_lat, ref_lon, cx_local, cy_local, cx_gps, cy_gps, theta = simple_gps_util.calibrate_with_landmarks(
        test_points_local,
        test_points_gps,
    )
    print(f"Loaded {len(test_points_gps)} landmark point(s) from {config_path}")
    print(f"Calibrated with landmarks: rotation {theta:.6f}°")

    simple_gps_util.convert_gml_to_gps_landmarks(
        input_path=args.input,
        output_path=output_path,
        ref_lat=ref_lat, ref_lon=ref_lon,
        cx_local=cx_local, cy_local=cy_local,
        cx_gps=cx_gps, cy_gps=cy_gps,
        theta_degrees=theta,
    )
    print(f"Written: {output_path}")


if __name__ == "__main__":
    main()
