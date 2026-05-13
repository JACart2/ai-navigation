"""Pure-Python helper for off-path detection.

Kept free of ROS dependencies so the logic can be unit-tested in isolation.
"""
import math


def compute_off_path_distance(
    cur_x, cur_y, cx, cy, prev_idx, search_back=20, search_fwd=50
):
    """Find the nearest spline-sample to (cur_x, cur_y) within a window
    centered on prev_idx, and return (min_dist, new_idx).

    The window is anchored on the cart's previously-known closest index, not
    on pure_pursuit's look-ahead target. With spline ds=0.1 m, the default
    window covers 2 m behind and 5 m ahead of the cart's last known position
    on the path, which is sufficient for any single-tick movement at typical
    cart speeds while keeping the per-tick cost O(70).
    """
    n = len(cx)
    if n == 0:
        return float("inf"), prev_idx
    # Clamp prev_idx into [0, n-1] before forming the window, so a stale
    # prev_idx past the end of the path still produces a meaningful search.
    start = max(0, min(n - 1, prev_idx - search_back))
    end = min(n, max(start + 1, prev_idx + search_fwd))
    min_dist = float("inf")
    min_idx = start
    for i in range(start, end):
        dx = cx[i] - cur_x
        dy = cy[i] - cur_y
        d = math.hypot(dx, dy)
        if d < min_dist:
            min_dist = d
            min_idx = i
    return min_dist, min_idx
