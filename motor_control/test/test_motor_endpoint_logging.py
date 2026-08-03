"""Tests for accurate motor speed and Arduino-command log context."""

from types import SimpleNamespace

from motor_control.motor_endpoint import MotorEndpoint
from anomaly_msg.msg import AnomalyMsg


def test_speed_context_keeps_physical_speed_and_throttle_separate(monkeypatch):
    """Throttle units must never be presented as measured road speed."""
    node = SimpleNamespace(
        vel_curr=1.25,
        last_vel_curr_received_monotonic=98.5,
        last_arduino_throttle_command=62,
        last_arduino_brake_command=0,
    )
    monkeypatch.setattr("motor_control.motor_endpoint.time.monotonic", lambda: 100.0)

    context = MotorEndpoint._speed_context_for_log(node)

    assert "estimated_cart_speed=1.25m/s" in context
    assert "speed_source=/estimate_twist" in context
    assert "speed_measurement_age=1.50s" in context
    assert "last_arduino_throttle=62/255" in context
    assert "last_arduino_brake=0/255" in context
    assert "km/h" not in context


def test_speed_context_reports_missing_measurement():
    """A missing physical-speed estimate must be explicit."""
    node = SimpleNamespace(
        last_vel_curr_received_monotonic=None,
        last_arduino_throttle_command=0,
        last_arduino_brake_command=255,
    )

    context = MotorEndpoint._speed_context_for_log(node)

    assert "estimated_cart_speed=unavailable" in context
    assert "last_arduino_throttle=0/255" in context
    assert "last_arduino_brake=255/255" in context


def test_collision_braking_logs_only_on_state_edges():
    """Repeated braking commands belong to one incident, not many anomalies."""
    logged = []
    node = SimpleNamespace(
        vel_planned=None,
        angle_planned=None,
        obstacle_distance=-1,
        collision_braking_active=False,
        brake_time_used=0,
        full_stop_count=0,
        state=0,
        brake=0,
        stopping_time=0,
        new_vel=False,
        log_header=lambda message: None,
        log_aad=lambda importance, message: logged.append((importance, message)),
        _speed_context_for_log=lambda: "estimated_cart_speed=0.20m/s",
        report_state_change=lambda: None,
    )
    braking_command = SimpleNamespace(vel=-2.37, angle=0.0)
    clear_command = SimpleNamespace(vel=1.0, angle=0.0)

    MotorEndpoint.vel_angle_planned_callback(node, braking_command)
    MotorEndpoint.vel_angle_planned_callback(node, braking_command)
    MotorEndpoint.vel_angle_planned_callback(node, clear_command)

    assert len(logged) == 2
    assert logged[0][0] == AnomalyMsg.ERROR
    assert "braking active" in logged[0][1]
    assert logged[1] == (
        AnomalyMsg.INFO,
        "Collision avoidance braking cleared",
    )
