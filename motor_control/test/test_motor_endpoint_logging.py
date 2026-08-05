"""Tests for accurate motor speed and Arduino-command log context."""

from types import SimpleNamespace

from motor_control.motor_endpoint import MotorEndpoint
from anomaly_msg.msg import AnomalyMsg


def test_collision_braking_does_not_duplicate_upstream_anomaly():
    """The endpoint must act on a stop without reporting the incident again."""
    logged = []
    node = SimpleNamespace(
        vel_planned=None,
        angle_planned=None,
        obstacle_distance=-1,
        brake_time_used=0,
        full_stop_count=0,
        state=0,
        brake=0,
        stopping_time=0,
        new_vel=False,
        log_header=lambda message: None,
        log_aad=lambda importance, message: logged.append((importance, message)),
        report_state_change=lambda: None,
    )
    braking_command = SimpleNamespace(vel=-2.37, angle=0.0)
    clear_command = SimpleNamespace(vel=1.0, angle=0.0)

    MotorEndpoint.vel_angle_planned_callback(node, braking_command)
    MotorEndpoint.vel_angle_planned_callback(node, braking_command)
    MotorEndpoint.vel_angle_planned_callback(node, clear_command)

    assert logged == []
    assert node.obstacle_distance == -1


def test_anomaly_telemetry_is_plain_text_with_only_motor_specific_context():
    """Routine telemetry must use readable text and omit unwanted metadata."""
    node = SimpleNamespace(
        angle_planned=-12.5,
        last_arduino_steering_command=73,
        last_vel_curr_received_monotonic=98.0,
        vel_curr=2.0,
        wheel_base=2.4003,
        estimated_yaw_rate=-0.3,
        last_arduino_throttle_command=70,
        last_arduino_brake_command=4,
        last_heartbeat_time=197.0,
    )
    message = MotorEndpoint._anomaly_telemetry_message(node)

    assert message == (
        "Motor telemetry: requested_steering=12.50 deg right, "
        "estimated_steering=19.80 deg right, arduino_steering_command=73, "
        "arduino_throttle_command=70, arduino_brake_command=4"
    )
    assert not message.startswith("{")
    assert "event" not in message
    assert "heartbeat age" not in message
    assert "motion measurement age" not in message


def test_steering_angle_format_includes_direction():
    assert MotorEndpoint._format_steering_angle(8.25) == "8.25 deg left"
    assert MotorEndpoint._format_steering_angle(-8.25) == "8.25 deg right"
    assert MotorEndpoint._format_steering_angle(0.0) == "0.00 deg straight"


def test_publish_anomaly_telemetry_uses_separate_source():
    """Routine telemetry must not consume the motor event rate-limit bucket."""
    published = []
    node = SimpleNamespace(
        _anomaly_telemetry_message=lambda: "Motor telemetry: test message",
        log_aad=lambda importance, message, node_name=None: published.append(
            (importance, message, node_name)
        ),
    )

    MotorEndpoint.publish_anomaly_telemetry(node)

    assert published == [
        (
            AnomalyMsg.INFO,
            "Motor telemetry: test message",
            "motor_endpoint_telemetry",
        )
    ]
