"""Tests for accurate motor speed and Arduino-command log context."""

from types import SimpleNamespace

from motor_control.motor_endpoint import MotorEndpoint


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
