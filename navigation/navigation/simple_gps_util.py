#!/usr/bin/env python
import math
from geometry_msgs.msg import Point

"""
This file is a modified version of the AlvinXY method produced by WHOI
Modified by Joshua Sun in Spring 2026.
"""


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


def heading_correction(origin_x, origin_y, angle, point):
    """The map is not always oriented the same, this is to correct the map
    heading by a certain number of degrees

    Args:
        origin_x: Origin pos x of the map typically 0
        origin_y: Origin pos y of the map typically 0
        angle (degrees): Adjust the heading of the point
        point (Point message): The point to correct by the angle around the origin
    """
    sin_ang = math.sin(math.radians(angle))
    cos_ang = math.cos(math.radians(angle))

    # Translate point to origin before rotation
    point.x -= origin_x
    point.y -= origin_y

    # Rotate the point
    new_x = point.x * cos_ang - point.y * sin_ang
    new_y = point.x * sin_ang + point.y * cos_ang

    # Reproject point back out
    point.x = new_x + origin_x
    point.y = new_y + origin_y

    return point


def calibrate_util(test_point_local, map_origin_local, test_point_gps, map_origin_gps):
    """A calibration utility for finding the heading angle between the GPS frame and the
    local map frame using two known point correspondences.

    Computes the angle analytically via atan2 — the difference in bearing between the
    two points in GPS space vs. local map space. This is exact (no search granularity
    error) and limited only by the precision of the input coordinates.

    Args:
        test_point_local: Selected test point tuple in X, Y
        map_origin_local: The PCD map's origin tuple in X, Y
        test_point_gps: The same selected test point tuple but in Latitude, Longitude tuple
        map_origin_gps: The same map origin but a Latitude, Longitude tuple

    Returns:
        Heading offset angle in degrees
    """
    map_lat, map_lon = map_origin_gps[0], map_origin_gps[1]

    # Vector from anchor to test point in GPS space (meters)
    gps_x, gps_y = latlon2xy(
        test_point_gps[0], test_point_gps[1], map_lat, map_lon
    )
    gps_dx = gps_x - 0  # relative to GPS-frame anchor (always 0,0 from latlon2xy)
    gps_dy = gps_y - 0

    # Vector from anchor to test point in local map space
    local_dx = test_point_local[0] - map_origin_local[0]
    local_dy = test_point_local[1] - map_origin_local[1]

    # Bearing of the vector in each frame
    gps_bearing   = math.degrees(math.atan2(gps_dy, gps_dx))
    local_bearing = math.degrees(math.atan2(local_dy, local_dx))

    # The rotation needed to align GPS frame to local map frame
    angle = (local_bearing - gps_bearing) % 360

    return angle


def calibrate_util_multi(test_points_local, map_origin_local, test_points_gps, map_origin_gps):
    """Multi-point calibration using the circular mean of per-pair atan2 angles.

    Each test-point pair contributes one angle estimate via calibrate_util.
    A circular mean (sin/cos averaging) is then used to combine them correctly,
    avoiding the wrap-around error that plain arithmetic mean produces near 0°/360°.
    More spread-out test points reduce the impact of any single imprecise GPS reading.

    Args:
        test_points_local: List of (X, Y) tuples in local map frame
        map_origin_local:  Map anchor tuple in X, Y
        test_points_gps:   List of (lat, lon) tuples for the same test points
        map_origin_gps:    Map anchor as (lat, lon)

    Returns:
        Heading offset angle in degrees (circular mean across all pairs)
    """
    if len(test_points_local) != len(test_points_gps):
        raise ValueError(
            f"test_points_local ({len(test_points_local)} pts) and "
            f"test_points_gps ({len(test_points_gps)} pts) must be the same length"
        )
    if len(test_points_local) == 0:
        raise ValueError("At least one test point pair is required")

    # Single-point fast path — no averaging needed
    if len(test_points_local) == 1:
        return calibrate_util(
            test_points_local[0], map_origin_local,
            test_points_gps[0],   map_origin_gps
        )

    # Accumulate unit vectors on the unit circle (circular mean)
    sin_sum = 0.0
    cos_sum = 0.0
    for pt_local, pt_gps in zip(test_points_local, test_points_gps):
        angle_rad = math.radians(
            calibrate_util(pt_local, map_origin_local, pt_gps, map_origin_gps)
        )
        sin_sum += math.sin(angle_rad)
        cos_sum += math.cos(angle_rad)

    return math.degrees(math.atan2(sin_sum, cos_sum)) % 360


def convert_gml_to_gps(input_path, output_path, anchor_gps, anchor_local, anchor_theta):
    """Convert a GML map file from local ROS coordinates to GPS (lat/lon) coordinates.

    Each node's pos [x, y] is transformed using the same pipeline as the runtime
    local→GPS conversion in global_planner.py:
        1. Subtract anchor_local to re-centre on the GPS anchor
        2. Rotate by -anchor_theta to align with the GPS frame
        3. Convert metre offset to lat/lon via xy2latlon

    The output GML replaces the two-value 'pos' list attribute with separate 'lat'
    and 'lon' float attributes on every node.  All edges and non-pos node attributes
    (label, active, weight, …) are preserved exactly as-is.

    Args:
        input_path:   Path to the source .gml file (local ROS coordinates)
        output_path:  Path to write the converted .gml file
        anchor_gps:   (lat, lon) of the map anchor point
        anchor_local: (x, y) of the same anchor in local map frame
        anchor_theta: Heading offset in degrees (from calibrate_util_multi)

    Returns:
        The converted NetworkX DiGraph
    """
    import networkx as nx

    graph = nx.read_gml(input_path)
    anchor_lat, anchor_lon = anchor_gps[0], anchor_gps[1]

    for node in graph.nodes:
        pos = graph.nodes[node]["pos"]
        local_x, local_y = pos[0], pos[1]

        # Step 1: subtract anchor_local to re-centre on the GPS anchor origin
        pt = Point()
        pt.x = local_x - anchor_local[0]
        pt.y = local_y - anchor_local[1]

        # Step 2: rotate by -anchor_theta to align with the GPS frame
        pt = heading_correction(0, 0, -anchor_theta, pt)

        # Step 3: convert metre offset to lat/lon
        lat, lon = xy2latlon(pt.x, pt.y, anchor_lat, anchor_lon)

        graph.nodes[node]["lat"] = lat
        graph.nodes[node]["lon"] = lon
        del graph.nodes[node]["pos"]

    nx.write_gml(graph, output_path)
    return graph
