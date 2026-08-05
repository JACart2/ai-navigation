"""Tests for accurate motor speed and Arduino-command log context."""

import json
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


def test_anomaly_telemetry_contains_only_motor_specific_context(monkeypatch):
    """Routine telemetry must omit fields already logged by upstream nodes."""
    node = SimpleNamespace(
        angle_planned=-12.5,
        last_arduino_steering_command=73,
        last_vel_curr_received_monotonic=98.0,
        estimated_yaw_rate=-0.3,
        last_arduino_throttle_command=70,
        last_arduino_brake_command=4,
        last_heartbeat_time=197.0,
    )
    monkeypatch.setattr("motor_control.motor_endpoint.time.monotonic", lambda: 100.0)
    monkeypatch.setattr("motor_control.motor_endpoint.time.time", lambda: 200.0)

    payload = MotorEndpoint._anomaly_telemetry_payload(node)

    assert payload["requested_steering_deg"] == -12.5
    assert payload["arduino_steering_command"] == 73
    assert payload["estimated_yaw_rate_rad_s"] == -0.3
    assert payload["motion_measurement_age_s"] == 2.0
    assert payload["heartbeat_age_s"] == 3.0
    assert "arduino_heartbeat" not in payload
    assert "requested_speed_mps" not in payload
    assert "estimated_speed_mps" not in payload
    assert "obstacle_distance_m" not in payload
    assert "drive_state" not in payload
    assert "control_mode" not in payload
    assert json.loads(json.dumps(payload))["event"] == "motor_steering_telemetry"


def test_publish_anomaly_telemetry_uses_separate_source():
    """Routine telemetry must not consume the motor event rate-limit bucket."""
    published = []
    node = SimpleNamespace(
        _anomaly_telemetry_payload=lambda: {"event": "motor_steering_telemetry"},
        log_aad=lambda importance, message, node_name=None: published.append(
            (importance, json.loads(message), node_name)
        ),
    )

    MotorEndpoint.publish_anomaly_telemetry(node)

    assert published == [
        (
            AnomalyMsg.INFO,
            {"event": "motor_steering_telemetry"},
            "motor_endpoint_telemetry",
        )
    ]
