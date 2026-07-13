#!/usr/bin/env python3
"""Print the GPS-to-map conversion used by MOLA GPS relocalization."""

import argparse
import math
import os
import re
import sys
from pathlib import Path
from typing import Any, List, Optional, Tuple

try:
    import yaml
except ImportError:  # pragma: no cover - PyYAML is present in the ROS env
    yaml = None


REPO_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_CONFIG_DIR = REPO_ROOT / "navigation" / "maps"
DEFAULT_CONFIG_FILE = "with_gps2_adjusted_route.yaml"


def _load_simple_gps_util():
    source_navigation_dir = REPO_ROOT / "navigation"
    if source_navigation_dir.exists():
        sys.path.insert(0, str(source_navigation_dir))
    from navigation import simple_gps_util

    return simple_gps_util


def _load_landmark_calibration(
    config_path: str,
) -> Tuple[List[Tuple[float, float]], List[Tuple[float, float]]]:
    if yaml is None:
        raise RuntimeError("PyYAML is required for fallback calibration loading")

    with open(config_path, "r", encoding="utf-8") as config_stream:
        config = yaml.safe_load(config_stream) or {}

    landmarks = config.get("landmarks")
    if not isinstance(landmarks, list) or len(landmarks) < 2:
        raise ValueError("calibration config must define at least 2 landmarks")

    local_points: List[Tuple[float, float]] = []
    gps_points: List[Tuple[float, float]] = []
    for index, landmark in enumerate(landmarks):
        try:
            local = landmark["local"]
            gps = landmark["gps"]
            local_points.append((float(local["x"]), float(local["y"])))
            gps_points.append(
                (float(gps["latitude"]), float(gps["longitude"]))
            )
        except (KeyError, TypeError, ValueError) as exc:
            raise ValueError(
                f"invalid landmark entry at index {index}"
            ) from exc

    return local_points, gps_points


def _mdeglon(lat0: float) -> float:
    lat0rad = math.radians(lat0)
    return (
        111415.13 * math.cos(lat0rad)
        - 94.55 * math.cos(3.0 * lat0rad)
        - 0.12 * math.cos(5.0 * lat0rad)
    )


def _mdeglat(lat0: float) -> float:
    lat0rad = math.radians(lat0)
    return (
        111132.09
        - 566.05 * math.cos(2.0 * lat0rad)
        + 1.20 * math.cos(4.0 * lat0rad)
        - 0.002 * math.cos(6.0 * lat0rad)
    )


def _latlon2xy(
    lat: float,
    lon: float,
    lat0: float,
    lon0: float,
) -> Tuple[float, float]:
    x = (lon - lon0) * _mdeglon(lat0)
    y = (lat - lat0) * _mdeglat(lat0)
    return x, y


def _calibrate_with_landmarks(
    local_points: List[Tuple[float, float]],
    gps_points: List[Tuple[float, float]],
) -> Tuple[float, ...]:
    if len(local_points) != len(gps_points):
        raise ValueError("local and GPS landmark counts differ")
    if len(local_points) < 2:
        raise ValueError("at least 2 landmarks are required")

    ref_lat = sum(lat for lat, _ in gps_points) / len(gps_points)
    ref_lon = sum(lon for _, lon in gps_points) / len(gps_points)
    gps_meters = [
        _latlon2xy(lat, lon, ref_lat, ref_lon)
        for lat, lon in gps_points
    ]

    cx_local = sum(x for x, _ in local_points) / len(local_points)
    cy_local = sum(y for _, y in local_points) / len(local_points)
    cx_gps = sum(x for x, _ in gps_meters) / len(gps_meters)
    cy_gps = sum(y for _, y in gps_meters) / len(gps_meters)

    dot_sum = 0.0
    cross_sum = 0.0
    for gps_point, local_point in zip(gps_meters, local_points):
        gps_dx = gps_point[0] - cx_gps
        gps_dy = gps_point[1] - cy_gps
        local_dx = local_point[0] - cx_local
        local_dy = local_point[1] - cy_local
        dot_sum += gps_dx * local_dx + gps_dy * local_dy
        cross_sum += gps_dx * local_dy - gps_dy * local_dx

    if abs(dot_sum) < 1e-12 and abs(cross_sum) < 1e-12:
        raise ValueError("landmark calibration is degenerate")

    theta_degrees = math.degrees(math.atan2(cross_sum, dot_sum))
    return (
        ref_lat,
        ref_lon,
        cx_local,
        cy_local,
        cx_gps,
        cy_gps,
        theta_degrees,
    )


def _gps_to_local(
    lat: float,
    lon: float,
    ref_lat: float,
    ref_lon: float,
    cx_local: float,
    cy_local: float,
    cx_gps: float,
    cy_gps: float,
    theta_degrees: float,
) -> Tuple[float, float]:
    x_m, y_m = _latlon2xy(lat, lon, ref_lat, ref_lon)
    dx = x_m - cx_gps
    dy = y_m - cy_gps

    theta_rad = math.radians(theta_degrees)
    cos_t = math.cos(theta_rad)
    sin_t = math.sin(theta_rad)
    rot_dx = dx * cos_t - dy * sin_t
    rot_dy = dx * sin_t + dy * cos_t
    return rot_dx + cx_local, rot_dy + cy_local


def _log_token(value: Any) -> str:
    text = str(value).strip()
    if not text:
        return "none"
    return re.sub(r"\s+", "_", text)


def _format_optional_float(value: Optional[float], precision: int) -> str:
    if value is None:
        return "none"
    try:
        number = float(value)
    except (TypeError, ValueError):
        return "none"
    if not math.isfinite(number):
        return str(number).lower()
    return f"{number:.{precision}f}"


def _resolve_config_path(args: argparse.Namespace) -> str:
    if args.config_path:
        return os.path.abspath(os.path.expanduser(args.config_path))
    return os.path.abspath(
        os.path.join(
            os.path.expanduser(args.config_dir),
            args.config_file,
        )
    )


def main() -> int:
    parser = argparse.ArgumentParser(
        description=(
            "Convert one GPS lat/lon through the same landmark calibration "
            "math used by the MOLA auto-localization supervisor."
        )
    )
    parser.add_argument("--lat", type=float, required=True, help="Latitude")
    parser.add_argument("--lon", type=float, required=True, help="Longitude")
    parser.add_argument(
        "--config-path",
        default="",
        help=(
            "Explicit landmark YAML path. Overrides --config-dir and "
            "--config-file."
        ),
    )
    parser.add_argument(
        "--config-dir",
        default=str(DEFAULT_CONFIG_DIR),
        help="Directory containing the landmark calibration YAML.",
    )
    parser.add_argument(
        "--config-file",
        default=DEFAULT_CONFIG_FILE,
        help="Landmark calibration YAML file.",
    )
    parser.add_argument(
        "--map-frame",
        default="map",
        help="Map frame label to print for comparison with supervisor logs.",
    )
    args = parser.parse_args()

    config_path = _resolve_config_path(args)
    helper_warning = ""
    try:
        simple_gps_util = _load_simple_gps_util()
    except ImportError as exc:
        simple_gps_util = None
        helper_warning = str(exc)

    if simple_gps_util is not None:
        local_points, gps_points = simple_gps_util.load_landmark_calibration(
            config_path
        )
        calibration = simple_gps_util.calibrate_with_landmarks(
            local_points,
            gps_points,
        )
        map_x, map_y = simple_gps_util.gps_to_local(
            args.lat,
            args.lon,
            *calibration,
        )
        meters_per_degree_lat = simple_gps_util.mdeglat(calibration[0])
        meters_per_degree_lon = simple_gps_util.mdeglon(calibration[0])
        method = "navigation.simple_gps_util.gps_to_local"
    else:
        local_points, gps_points = _load_landmark_calibration(config_path)
        calibration = _calibrate_with_landmarks(local_points, gps_points)
        map_x, map_y = _gps_to_local(args.lat, args.lon, *calibration)
        meters_per_degree_lat = _mdeglat(calibration[0])
        meters_per_degree_lon = _mdeglon(calibration[0])
        method = "supervisor_built_in_landmark_calibration_gps_to_local"

    (
        ref_lat,
        ref_lon,
        cx_local,
        cy_local,
        cx_gps,
        cy_gps,
        theta_degrees,
    ) = calibration

    print(f"config={config_path}")
    print(f"method={method}")
    if helper_warning:
        print(f"helper_warning={helper_warning}")
    print(f"landmarks={len(gps_points)}")
    print(f"map_x={map_x:.6f}")
    print(f"map_y={map_y:.6f}")
    print(
        "GPS_CONVERSION "
        f"topic=debug_cli frame_id=none "
        f"lat={args.lat:.12f} lon={args.lon:.12f} alt=none status=none "
        f"cov_x=none cov_y=none cov_z=none cov_type=none "
        f"-> map_x={_format_optional_float(map_x, 3)} "
        f"map_y={_format_optional_float(map_y, 3)} "
        f"accepted=true reason=debug_script "
        f"method={method} "
        f"source=landmark_calibration config={_log_token(config_path)} "
        f"map_frame={_log_token(args.map_frame)} units=m scale=1.0 "
        f"utm_zone=none origin_lat={ref_lat:.12f} "
        f"origin_lon={ref_lon:.12f} cx_local={cx_local:.6f} "
        f"cy_local={cy_local:.6f} cx_gps={cx_gps:.6f} "
        f"cy_gps={cy_gps:.6f} rotation_deg={theta_degrees:.9f} "
        f"meters_per_degree_lat={meters_per_degree_lat:.6f} "
        f"meters_per_degree_lon={meters_per_degree_lon:.6f}"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
