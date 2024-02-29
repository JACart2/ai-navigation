#!/usr/bin/env python
import math
import numpy
from geometry_msgs.msg import Point

"""
This file is a modified version of the AlvinXY method produced by WHOI
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
    """A calibration utility for finding the optimal heading angle for use.

    Args:
        test_point_local: Selected test point tuple in X, Y
        map_origin_local: The PCD map's origin tuple in X, Y
        test_point_gps: The same selected test point tuple but in Latitude, Longitude tuple
        map_origin_gps: The same map origin but a Latitude, Longitude tuple
    """
    min_err_angle = 360
    min_err = 999999

    # Test point in X, Y
    point_x, point_y = test_point_local[0], test_point_local[1]

    # Same test point but in GPS coordinates
    lat_x, lat_y = test_point_gps[0], test_point_gps[1]

    # Map Origin in X, Y
    map_x, map_y = map_origin_local[0], map_origin_local[1]

    # Same origin but in Latitude Longitude
    map_lat, map_lon = map_origin_gps[0], map_origin_gps[1]

    # Step degree of 0.1 is arbitrary and is indeed a magic number
    for angle in numpy.arange(0, 360, 0.1):
        # Get a fresh point for each heading adjustment
        local_point = Point()
        local_x, local_y = latlon2xy(lat_x, lat_y, map_lat, map_lon)
        local_point.x = local_x
        local_point.y = local_y

        # Calculate the heading using the step angle and point
        corrected_point = heading_correction(map_x, map_y, angle, local_point)

        # Calculate the error this heading gives
        dist_err = math.sqrt(
            (corrected_point.x - point_x) ** 2 + (corrected_point.y - point_y) ** 2
        )
        if dist_err < min_err:
            min_err = dist_err
            min_err_angle = angle

    return min_err_angle
