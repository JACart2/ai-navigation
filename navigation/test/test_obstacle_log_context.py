"""Tests for spatial context included in obstacle anomaly events."""

from types import SimpleNamespace

from navigation.obstacle_log_context import format_obstacle_context


def _obstacle(x, y, radius=0.0):
    return SimpleNamespace(
        pos=SimpleNamespace(point=SimpleNamespace(x=x, y=y)),
        radius=radius,
    )


def test_dense_distant_field_explains_that_nothing_is_nearby():
    obstacles = [_obstacle(distance, 0.0, 0.2) for distance in range(5, 11)]

    context = format_obstacle_context(obstacles, "base_link")

    assert "nearest_surface=4.80m" in context
    assert "farthest_surface=9.80m" in context
    assert "within_1m=0" in context
    assert "within_2m=0" in context
    assert "within_4m=0" in context
    assert "proximity=distant" in context
    assert "frame=base_link" in context


def test_near_field_reports_counts_and_nearest_position():
    obstacles = [
        _obstacle(1.5, 0.0, 0.5),
        _obstacle(0.0, 3.0, 0.25),
        _obstacle(6.0, 0.0),
    ]

    context = format_obstacle_context(obstacles, "base_link")

    assert "nearest_surface=1.00m" in context
    assert "within_1m=1" in context
    assert "within_2m=1" in context
    assert "within_4m=2" in context
    assert "nearest_center=(1.50,0.00)m" in context
    assert "proximity=immediate" in context


def test_missing_positions_are_reported_without_crashing():
    assert format_obstacle_context([object()], "velodyne") == (
        "distance_context=unavailable, frame=velodyne"
    )
