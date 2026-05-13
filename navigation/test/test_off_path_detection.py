"""Pure-Python tests for off_path_detector.compute_off_path_distance.

These tests intentionally avoid any ROS imports so they can run in CI or on a
dev box without a ROS2 install. They validate the detection invariants that
the prior windowed-search implementation got wrong:

  1. The search window must be anchored on the cart's previous index, not the
     pure-pursuit look-ahead. (Was the headline bug.)
  2. The returned min_idx must monotonically follow the cart along the path
     across successive calls.
  3. The function must not silently miss the true nearest point when the cart
     lags or jumps within the window's reach.
  4. Edge cases: empty path, cart at start, cart at end.

A debounce simulator at the bottom exercises the strike-count logic that
local_planner uses on top of this function.
"""
import math
import sys
import os

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "navigation"))
from off_path_detector import compute_off_path_distance  # noqa: E402


def _straight_path(length_m=20.0, ds=0.1):
    """A straight path along the x-axis at y=0 from x=0 to x=length_m."""
    n = int(length_m / ds) + 1
    cx = [i * ds for i in range(n)]
    cy = [0.0 for _ in range(n)]
    return cx, cy


def _curved_path():
    """A quarter circle of radius 10 centered at (0,0), 0->pi/2."""
    n = 200
    cx = [10 * math.cos(i * (math.pi / 2) / (n - 1)) for i in range(n)]
    cy = [10 * math.sin(i * (math.pi / 2) / (n - 1)) for i in range(n)]
    return cx, cy


# 1. Cart sitting exactly on path returns ~0 distance.
def test_on_path_is_zero_distance():
    cx, cy = _straight_path()
    dist, idx = compute_off_path_distance(5.0, 0.0, cx, cy, prev_idx=50)
    assert dist < 1e-9
    assert idx == 50


# 2. Cart laterally offset by 0.3 m -> distance ~0.3 m.
def test_lateral_offset_detected():
    cx, cy = _straight_path()
    dist, _ = compute_off_path_distance(5.0, 0.3, cx, cy, prev_idx=50)
    assert abs(dist - 0.3) < 1e-6


# 3. Cart laterally offset by 0.6 m exceeds a 0.5 m threshold.
def test_lateral_offset_exceeds_threshold():
    cx, cy = _straight_path()
    dist, _ = compute_off_path_distance(5.0, 0.6, cx, cy, prev_idx=50)
    assert dist > 0.5


# 4. min_idx slides forward as the cart advances.
def test_index_slides_forward_along_path():
    cx, cy = _straight_path()
    idx = 0
    last = -1
    for x in [0.5, 1.0, 2.5, 4.0, 7.5, 10.0]:
        _, idx = compute_off_path_distance(x, 0.0, cx, cy, prev_idx=idx)
        assert idx > last
        last = idx
    assert idx > 90  # well into the path


# 5. Regression for the headline bug: when the look-ahead target races ahead
#    of the cart, the *cart-centered* search still finds the true nearest
#    point. Simulate by holding prev_idx near the cart while the cart stays
#    put -- contrast: an old target_ind-anchored search at idx+30 would have
#    failed this.
def test_cart_centered_window_finds_true_nearest_when_lookahead_runs_ahead():
    cx, cy = _straight_path(length_m=30.0)
    # Cart is at x=5, y=0.6 -> truly off path by 0.6m at index 50.
    # prev_idx tracks the cart, NOT the pure-pursuit look-ahead.
    dist, idx = compute_off_path_distance(5.0, 0.6, cx, cy, prev_idx=50)
    assert abs(dist - 0.6) < 1e-6
    assert idx == 50


# 6. Across multiple ticks where the cart progresses 0.5 m per tick, the
#    index advances by ~5 samples per tick (ds=0.1).
def test_index_advances_one_step_per_movement():
    cx, cy = _straight_path()
    idx = 0
    last_idx = idx
    for tick in range(1, 11):
        x = tick * 0.5
        _, idx = compute_off_path_distance(x, 0.0, cx, cy, prev_idx=idx)
        # Each tick advances 5 samples on the spline.
        assert idx - last_idx >= 4
        assert idx - last_idx <= 6
        last_idx = idx


# 7. Curved path: cart on the inside of the curve by 0.4 m -> distance ~0.4.
def test_curved_path_inner_offset():
    cx, cy = _curved_path()
    # Point at angle pi/4 on a radius-10 arc is (7.071, 7.071).
    # Move 0.4 m toward origin (along the radius) -> radius 9.6.
    angle = math.pi / 4
    r = 9.6
    cart_x = r * math.cos(angle)
    cart_y = r * math.sin(angle)
    # Index ~100 corresponds to angle pi/4 in a 200-sample path 0..pi/2.
    dist, _ = compute_off_path_distance(cart_x, cart_y, cx, cy, prev_idx=100)
    assert abs(dist - 0.4) < 0.05  # small discretization error from sampling


# 8. Empty path -> returns infinity, prev_idx echoed.
def test_empty_path_returns_inf():
    dist, idx = compute_off_path_distance(0.0, 0.0, [], [], prev_idx=7)
    assert dist == float("inf")
    assert idx == 7


# 9. prev_idx beyond path length -> clamps to end without crashing.
def test_prev_idx_past_end_clamps():
    cx, cy = _straight_path(length_m=5.0)
    n = len(cx)
    dist, idx = compute_off_path_distance(5.0, 0.0, cx, cy, prev_idx=n + 100)
    assert idx < n
    assert dist < 0.5


# 10. Window does NOT wrap; cart far behind the window is reported as the
#     window-edge distance (this is by design -- the strike counter in
#     local_planner will then trip off_path even from a bad initial pose).
def test_cart_outside_window_reports_edge_distance():
    cx, cy = _straight_path(length_m=20.0)
    # prev_idx=100 -> window is [80, 150]. Cart at x=0 is ~8m behind window.
    dist, idx = compute_off_path_distance(0.0, 0.0, cx, cy, prev_idx=100)
    # Closest sample in window is at x=8 (idx=80).
    assert abs(dist - 8.0) < 1e-6
    assert idx == 80


# --- Debounce simulator ------------------------------------------------------
# This is not testing compute_off_path_distance directly, but rather the
# strike-counter pattern that local_planner wraps around it. The pattern is
# simple enough to inline so a regression in local_planner is easy to spot.

def _simulate_strike_loop(distances, threshold, strikes_required):
    strikes = 0
    aborted = False
    for d in distances:
        if d > threshold:
            strikes += 1
        else:
            strikes = 0
        if strikes >= strikes_required:
            aborted = True
            break
    return aborted, strikes


def test_debounce_requires_consecutive_strikes():
    # Single spike does not trip.
    aborted, _ = _simulate_strike_loop([0.1, 0.1, 0.9, 0.1, 0.1], 0.5, 3)
    assert not aborted


def test_debounce_trips_on_three_consecutive():
    aborted, _ = _simulate_strike_loop([0.1, 0.9, 0.9, 0.9, 0.1], 0.5, 3)
    assert aborted


def test_debounce_resets_on_in_threshold_sample():
    # Two strikes, recovery, two more strikes -> no abort (resets to 0).
    aborted, _ = _simulate_strike_loop([0.9, 0.9, 0.1, 0.9, 0.9], 0.5, 3)
    assert not aborted
