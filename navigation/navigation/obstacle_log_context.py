"""Human-readable spatial context for obstacle anomaly events."""

import math
from typing import Iterable


def format_obstacle_context(
    obstacles: Iterable,
    frame_id: str = "",
    directional_labels: bool = False,
) -> str:
    """Summarize obstacle positions using distance to the obstacle surface."""
    measurements = []
    for obstacle in obstacles:
        try:
            x = float(obstacle.pos.point.x)
            y = float(obstacle.pos.point.y)
            radius = max(0.0, float(obstacle.radius))
        except (AttributeError, TypeError, ValueError):
            continue
        if not (math.isfinite(x) and math.isfinite(y) and math.isfinite(radius)):
            continue
        surface_distance = max(0.0, math.hypot(x, y) - radius)
        measurements.append((surface_distance, x, y))

    if not measurements:
        return ""

    measurements.sort(key=lambda item: item[0])
    nearest_distance, nearest_x, nearest_y = measurements[0]
    farthest_distance = measurements[-1][0]
    if directional_labels:
        nearest_obstacle = (
            f"({nearest_x:.2f}m ahead,{abs(nearest_y):.2f}m "
            f"{'left' if nearest_y >= 0.0 else 'right'})"
        )
    else:
        nearest_obstacle = f"({nearest_x:.2f},{nearest_y:.2f})m"

    return (
        f"nearest_surface={nearest_distance:.2f}m, "
        f"farthest_surface={farthest_distance:.2f}m, "
        f"within_1m={sum(distance <= 1.0 for distance, _, _ in measurements)}, "
        f"within_2m={sum(distance <= 2.0 for distance, _, _ in measurements)}, "
        f"within_4m={sum(distance <= 4.0 for distance, _, _ in measurements)}, "
        f"nearest_obstacle={nearest_obstacle}"
    )
