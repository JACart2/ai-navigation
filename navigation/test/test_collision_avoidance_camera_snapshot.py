"""Test event-driven camera snapshots for collision avoidance AAD logs."""

import threading
from types import SimpleNamespace

from sensor_msgs.msg import Image

from navigation.collision_avoidance_aad_log import CollisionAvoidanceAADLog


class _Clock:
    def __init__(self, nanoseconds):
        self.nanoseconds = nanoseconds

    def now(self):
        return SimpleNamespace(nanoseconds=self.nanoseconds)


def _image(frame_id):
    message = Image()
    message.header.frame_id = frame_id
    return message


def test_camera_callback_keeps_latest_frame_per_explicit_source():
    """Each explicit camera source retains only its newest frame."""
    fake_node = SimpleNamespace(
        _camera_lock=threading.Lock(),
        _latest_camera_frames={},
        get_clock=lambda: _Clock(10),
    )
    first_front = _image("front_optical")
    latest_front = _image("front_optical")
    rear = _image("rear_optical")

    CollisionAvoidanceAADLog.camera_callback(fake_node, "front", first_front)
    fake_node.get_clock = lambda: _Clock(20)
    CollisionAvoidanceAADLog.camera_callback(fake_node, "rear", rear)
    fake_node.get_clock = lambda: _Clock(30)
    CollisionAvoidanceAADLog.camera_callback(fake_node, "front", latest_front)

    assert set(fake_node._latest_camera_frames) == {"front", "rear"}
    assert fake_node._latest_camera_frames["front"] == (30, latest_front)
    assert fake_node._latest_camera_frames["rear"] == (20, rear)


def test_stop_snapshot_publishes_only_fresh_frames_in_fixed_order():
    """A stop snapshot contains fresh cameras in configured source order."""
    calls = []
    front = _image("front_optical")
    rear = _image("rear_optical")
    stale = _image("stale_optical")
    fake_node = SimpleNamespace(
        CAMERA_FRAME_MAX_AGE_SECONDS=2.0,
        CAMERA_SOURCES=("front", "rear", "stale"),
        enable_camera_capture=True,
        _camera_lock=threading.Lock(),
        _latest_camera_frames={
            "front": (9_000_000_000, front),
            "rear": (9_500_000_000, rear),
            "stale": (7_000_000_000, stale),
        },
        anomaly_camera_pub=object(),
        get_clock=lambda: _Clock(10_000_000_000),
        anomaly_logging=lambda *args, **kwargs: calls.append((args, kwargs)),
    )

    CollisionAvoidanceAADLog._publish_stop_camera_snapshot(fake_node)

    assert [call[1]["image"] for call in calls] == [front, rear]
    assert [call[1]["header"].frame_id for call in calls] == [
        "camera:front",
        "camera:rear",
    ]


def test_periodic_snapshot_publishes_pre_event_context():
    """Periodic camera publication uses the same bounded snapshot path."""
    calls = []
    front = _image("front_optical")
    fake_node = SimpleNamespace(
        CAMERA_FRAME_MAX_AGE_SECONDS=2.0,
        CAMERA_SOURCES=("front",),
        enable_camera_capture=True,
        _camera_lock=threading.Lock(),
        _latest_camera_frames={"front": (9_500_000_000, front)},
        anomaly_camera_pub=object(),
        get_clock=lambda: _Clock(10_000_000_000),
        anomaly_logging=lambda *args, **kwargs: calls.append((args, kwargs)),
    )

    CollisionAvoidanceAADLog._publish_periodic_camera_snapshot(fake_node)

    assert len(calls) == 1
    assert calls[0][1]["image"] is front
    assert calls[0][0][0] == (
        "Camera frame captured for periodic context; camera=front"
    )


def test_disabled_camera_capture_returns_without_touching_camera_state():
    """The explicit opt-out path does not read or publish camera data."""
    fake_node = SimpleNamespace(
        enable_camera_capture=False,
        anomaly_camera_pub=None,
    )

    CollisionAvoidanceAADLog._publish_stop_camera_snapshot(fake_node)


def test_repeated_stop_true_only_requests_one_camera_snapshot():
    """Repeated stop levels publish only state transitions."""
    snapshots = []
    logs = []
    fake_node = SimpleNamespace(
        _last_stop_state=False,
        _publish_stop_camera_snapshot=lambda: snapshots.append(True),
        anomaly_logging=lambda *args, **kwargs: logs.append((args, kwargs)),
    )
    stop = SimpleNamespace(
        sender_id=SimpleNamespace(data="collision_detector"),
        stop=True,
        distance=0.5,
        header=SimpleNamespace(),
    )

    CollisionAvoidanceAADLog.stop_callback(fake_node, stop)
    CollisionAvoidanceAADLog.stop_callback(fake_node, stop)

    assert snapshots == [True]
    assert len(logs) == 1

    stop.stop = False
    CollisionAvoidanceAADLog.stop_callback(fake_node, stop)
    CollisionAvoidanceAADLog.stop_callback(fake_node, stop)
    stop.stop = True
    CollisionAvoidanceAADLog.stop_callback(fake_node, stop)
    assert snapshots == [True, True]
    assert len(logs) == 3


def test_initial_clear_stop_level_is_not_published_as_a_transition():
    """An initial false level is ignored until a real stop has occurred."""
    snapshots = []
    logs = []
    fake_node = SimpleNamespace(
        _last_stop_state=False,
        _publish_stop_camera_snapshot=lambda: snapshots.append(True),
        anomaly_logging=lambda *args, **kwargs: logs.append((args, kwargs)),
    )
    stop = SimpleNamespace(
        sender_id=SimpleNamespace(data="collision_detector"),
        stop=False,
        distance=0.0,
        header=SimpleNamespace(),
    )

    CollisionAvoidanceAADLog.stop_callback(fake_node, stop)

    assert snapshots == []
    assert logs == []
