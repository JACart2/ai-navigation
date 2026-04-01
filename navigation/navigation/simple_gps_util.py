#!/usr/bin/env python
import math
import os
import numpy as np
import yaml

"""
This file is a modified version of the AlvinXY method produced by WHOI
Modified by Joshua Sun in Spring 2026.
"""


def resolve_config_path(config_dir, config_file):
    """Build an absolute config path from a directory and file name."""
    if not config_dir:
        raise ValueError("Calibration config directory must not be empty")
    if not config_file:
        raise ValueError("Calibration config file name must not be empty")
    return os.path.abspath(os.path.join(config_dir, config_file))


def load_landmark_calibration(config_path):
    """Load landmark calibration pairs from a YAML config file."""
    with open(config_path, "r", encoding="utf-8") as config_stream:
        config = yaml.safe_load(config_stream) or {}

    landmarks = config.get("landmarks")
    if not isinstance(landmarks, list) or len(landmarks) < 2:
        raise ValueError(
            f"Calibration config '{config_path}' must define at least 2 landmarks"
        )

    local_points = []
    gps_points = []

    for index, landmark in enumerate(landmarks):
        try:
            local = landmark["local"]
            gps = landmark["gps"]
            local_points.append((float(local["x"]), float(local["y"])))
            gps_points.append((float(gps["latitude"]), float(gps["longitude"])))
        except (KeyError, TypeError, ValueError) as exc:
            raise ValueError(
                f"Invalid landmark entry at index {index} in '{config_path}'"
            ) from exc

    return local_points, gps_points


def convert_graph_to_local(
    graph,
    graph_coordinate_format,
    ref_lat,
    ref_lon,
    cx_local,
    cy_local,
    cx_gps,
    cy_gps,
    theta_degrees,
):
    """Ensure graph nodes expose local ROS coordinates in the ``pos`` attribute."""
    fmt = graph_coordinate_format.lower()
    if fmt == "ros":
        for node in graph.nodes:
            if "pos" not in graph.nodes[node]:
                raise ValueError(
                    f"ROS graph node '{node}' is missing required 'pos' coordinates"
                )
        return graph

    if fmt != "gps":
        raise ValueError(
            f"Unsupported graph coordinate format '{graph_coordinate_format}'"
        )

    for node in graph.nodes:
        node_data = graph.nodes[node]
        if "lat" not in node_data or "lon" not in node_data:
            raise ValueError(
                f"GPS graph node '{node}' is missing 'lat'/'lon' coordinates"
            )

        local_x, local_y = gps_to_local(
            node_data["lat"],
            node_data["lon"],
            ref_lat,
            ref_lon,
            cx_local,
            cy_local,
            cx_gps,
            cy_gps,
            theta_degrees,
        )
        node_data["pos"] = [local_x, local_y]

    return graph


def latlon2xy(lat, lon, lat0, lon0):
    x = (lon - lon0) * mdeglon(lat0)
    y = (lat - lat0) * mdeglat(lat0)
    return x, y


def xy2latlon(x, y, lat0, lon0):
    lon = x / mdeglon(lat0) + lon0
    lat = y / mdeglat(lat0) + lat0
    return lat, lon


def mdeglon(lat0):
    lat0rad = math.radians(lat0)
    return (
        111415.13 * math.cos(lat0rad)
        - 94.55 * math.cos(3.0 * lat0rad)
        - 0.12 * math.cos(5.0 * lat0rad)
    )


def mdeglat(lat0):
    lat0rad = math.radians(lat0)
    return (
        111132.09
        - 566.05 * math.cos(2.0 * lat0rad)
        + 1.20 * math.cos(4.0 * lat0rad)
        - 0.002 * math.cos(6.0 * lat0rad)
    )


def calibrate_with_landmarks(test_points_local, test_points_gps):
    """Calibrate GPS transformation using a cloud of landmark points without a fixed anchor.

    Uses Procrustes analysis to find the optimal rotation and translation that maps
    the local ROS coordinates to GPS coordinates.

    Args:
        test_points_local: List of (x, y) tuples in local ROS frame
        test_points_gps: List of (lat, lon) tuples for the same points in GPS frame

    Returns:
        Tuple: (ref_lat, ref_lon, cx_local, cy_local, cx_gps, cy_gps, theta_degrees)
        - ref_lat, ref_lon: Reference point for lat/lon to meter conversions
        - cx_local, cy_local: Centroid of local points
        - cx_gps, cy_gps: Centroid of GPS points in meters
        - theta_degrees: Rotation angle in degrees
    """
    if len(test_points_local) != len(test_points_gps):
        raise ValueError("test_points_local and test_points_gps must have the same length")
    if len(test_points_local) < 2:
        raise ValueError("At least 2 landmark points are required for calibration")

    # Use centroid of GPS points as reference for lat/lon conversions
    ref_lat = sum(lat for lat, lon in test_points_gps) / len(test_points_gps)
    ref_lon = sum(lon for lat, lon in test_points_gps) / len(test_points_gps)

    # Convert all GPS points to meters relative to reference
    gps_meters = []
    for lat, lon in test_points_gps:
        x, y = latlon2xy(lat, lon, ref_lat, ref_lon)
        gps_meters.append((x, y))

    # Compute centroids
    cx_local = sum(x for x, y in test_points_local) / len(test_points_local)
    cy_local = sum(y for x, y in test_points_local) / len(test_points_local)
    cx_gps = sum(x for x, y in gps_meters) / len(gps_meters)
    cy_gps = sum(y for x, y in gps_meters) / len(gps_meters)

    # Center the points
    local_centered = np.array([(x - cx_local, y - cy_local) for x, y in test_points_local])
    gps_centered = np.array([(x - cx_gps, y - cy_gps) for x, y in gps_meters])

    # Procrustes analysis: find rotation R such that local_centered ≈ gps_centered @ R
    # Actually, we want local = R @ gps + T, so R such that local_centered ≈ R @ gps_centered
    H = gps_centered.T @ local_centered  # Note: gps first for the orientation
    U, s, Vt = np.linalg.svd(H)
    R = Vt.T @ U.T

    # Ensure it's a rotation (det >= 0)
    if np.linalg.det(R) < 0:
        Vt[-1] *= -1
        R = Vt.T @ U.T

    # Extract rotation angle
    theta_degrees = math.degrees(math.atan2(R[1, 0], R[0, 0]))

    return ref_lat, ref_lon, cx_local, cy_local, cx_gps, cy_gps, theta_degrees


def gps_to_local(lat, lon, ref_lat, ref_lon, cx_local, cy_local, cx_gps, cy_gps, theta_degrees):
    """Convert GPS coordinates to local ROS coordinates using landmark-based calibration.

    Args:
        lat, lon: GPS coordinates
        ref_lat, ref_lon: Reference point for conversions
        cx_local, cy_local: Centroid of local points
        cx_gps, cy_gps: Centroid of GPS points in meters
        theta_degrees: Rotation angle

    Returns:
        (x, y): Local coordinates
    """
    # Convert GPS to meters relative to reference
    x_m, y_m = latlon2xy(lat, lon, ref_lat, ref_lon)
    
    # Subtract GPS centroid
    dx = x_m - cx_gps
    dy = y_m - cy_gps
    
    # Rotate by theta
    theta_rad = math.radians(theta_degrees)
    cos_t = math.cos(theta_rad)
    sin_t = math.sin(theta_rad)
    rot_dx = dx * cos_t - dy * sin_t
    rot_dy = dx * sin_t + dy * cos_t
    
    # Add local centroid
    local_x = rot_dx + cx_local
    local_y = rot_dy + cy_local
    
    return local_x, local_y


def local_to_gps(x, y, ref_lat, ref_lon, cx_local, cy_local, cx_gps, cy_gps, theta_degrees):
    """Convert local ROS coordinates to GPS coordinates using landmark-based calibration.

    Args:
        x, y: Local coordinates
        ref_lat, ref_lon: Reference point for conversions
        cx_local, cy_local: Centroid of local points
        cx_gps, cy_gps: Centroid of GPS points in meters
        theta_degrees: Rotation angle

    Returns:
        (lat, lon): GPS coordinates
    """
    # Subtract local centroid
    dx = x - cx_local
    dy = y - cy_local
    
    # Rotate by -theta
    theta_rad = math.radians(-theta_degrees)
    cos_t = math.cos(theta_rad)
    sin_t = math.sin(theta_rad)
    rot_dx = dx * cos_t - dy * sin_t
    rot_dy = dx * sin_t + dy * cos_t
    
    # Add GPS centroid
    x_m = rot_dx + cx_gps
    y_m = rot_dy + cy_gps
    
    # Convert to lat/lon
    lat, lon = xy2latlon(x_m, y_m, ref_lat, ref_lon)
    
    return lat, lon


def convert_gml_to_gps_landmarks(input_path, output_path, ref_lat, ref_lon, cx_local, cy_local, cx_gps, cy_gps, theta_degrees):
    """Convert a GML map file from local ROS coordinates to GPS using landmark-based calibration.

    Args:
        input_path: Path to input GML file
        output_path: Path to output GML file
        ref_lat, ref_lon, cx_local, cy_local, cx_gps, cy_gps, theta_degrees: Calibration parameters
    """
    import networkx as nx

    graph = nx.read_gml(input_path)

    for node in graph.nodes:
        pos = graph.nodes[node]["pos"]
        local_x, local_y = pos[0], pos[1]

        lat, lon = local_to_gps(local_x, local_y, ref_lat, ref_lon, cx_local, cy_local, cx_gps, cy_gps, theta_degrees)

        graph.nodes[node]["lat"] = lat
        graph.nodes[node]["lon"] = lon
        del graph.nodes[node]["pos"]

    nx.write_gml(graph, output_path)
    return graph
