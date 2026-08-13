import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
import threading
import json
import re

from sensor_msgs.msg import Image
from std_msgs.msg import String
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import Float32, String
from motor_control_interface.msg import VelAngle
from navigation_interface.msg import Stop
from std_msgs.msg import Header
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, qos_profile_sensor_data
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
import yaml

from anomaly_msg.msg import AnomalyMsg


class CollisionAvoidanceAADLog(Node):

    def __init__(self):
        super().__init__('general_aad_log')

        self.CAMERA_FRAME_MAX_AGE_SECONDS = 2.0
        self.CAMERA_SOURCES = ("front", "rear")
        self.enable_camera_capture = bool(
            self.declare_parameter("enable_camera_capture", True).value
        )
        self.camera_publish_period_seconds = max(
            0.2,
            float(
                self.declare_parameter(
                    "camera_publish_period_seconds",
                    1.0,
                ).value
            ),
        )
        self.MOVING_LOG_PERIOD = 5
        self.LOCALIZATION_HEALTH_LOG_PERIOD = 5
        self.MOLA_BAD_ICP_QUALITY_THRESHOLD = 0.2
        self.MOLA_BAD_DROPPED_FRAMES_THRESHOLD = 0.4

        self.get_logger().info("Creating subscribers")
        # --- Subscribers ---

        # Use minimal QoS for camera - drop frames if can't keep up
        camera_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1  # Only keep latest frame
        )

        self.camera_0_sub = None
        self.camera_1_sub = None
        if self.enable_camera_capture:
            self.camera_0_sub = self.create_subscription(
                Image,
                '/zed_front/zed_node_0/rgb/color/rect/image',
                lambda msg: self.camera_callback("front", msg),
                camera_qos
            )

            self.camera_1_sub = self.create_subscription(
                Image,
                '/zed_rear/zed_node_1/rgb/color/rect/image',
                lambda msg: self.camera_callback("rear", msg),
                camera_qos
            )

        self.stop_sub = self.create_subscription(
            Stop,
            '/stop',
            self.stop_callback,
            10
        )

        self.localization_health_sub = self.create_subscription(
            DiagnosticArray,
            '/alignment_status',
            self.localization_health_callback,
            10
        )

        self.mola_diagnostics_sub = self.create_subscription(
            String,
            '/mola_diagnostics/lidar_odom/status',
            self.mola_diagnostics_callback,
            10
        )
        self.get_logger().info("Finished creating subscribers")
        self.get_logger().info("Creating publishers")

        # --- Publisher ---
        self.anomaly_camera_pub = None
        if self.enable_camera_capture:
            self.anomaly_camera_pub = self.create_publisher(
                AnomalyMsg,
                '/ai_anomaly_logging',
                camera_qos
            )

        # Text anomaly publisher remains active when camera capture is off.
        self.anomaly_log_pub = self.create_publisher(
            AnomalyMsg,
            '/ai_anomaly_logging',
            10
        )
        self.get_logger().info("Finished creating publishers")

        self.last_localization_health_pub_time = self.get_clock().now()
        self.last_localization_health_signature = ""
<<<<<<< HEAD
=======
        self.last_speed = 0.0
        self.last_commanded_speed_mps = None
>>>>>>> mola_main_sum26
        self._camera_lock = threading.Lock()
        self._latest_camera_frames = {}
        self._last_stop_state = False
        self.camera_publish_timer = None
        if self.enable_camera_capture:
            self.camera_publish_timer = self.create_timer(
                self.camera_publish_period_seconds,
                self._publish_periodic_camera_snapshot,
            )
        self.get_logger().info(
            "AAD camera capture is "
            f"{'enabled' if self.enable_camera_capture else 'disabled'}; "
            f"publish_period={self.camera_publish_period_seconds:.2f}s."
        )

    # --- Callbacks ---

    def camera_callback(self, source: str, img_msg: Image):
        """Keep only the latest in-memory frame for each camera."""
        received_ns = self.get_clock().now().nanoseconds
        with self._camera_lock:
            self._latest_camera_frames[source] = (received_ns, img_msg)

    def _publish_stop_camera_snapshot(self):
        """Publish at most one fresh frame per camera for a new stop event."""
        CollisionAvoidanceAADLog._publish_camera_snapshot(self, "stop event")

    def _publish_periodic_camera_snapshot(self):
        """Refresh AAD's bounded pre-event camera history."""
        CollisionAvoidanceAADLog._publish_camera_snapshot(self, "periodic context")

    def _publish_camera_snapshot(self, reason):
        """Publish at most one fresh frame per configured camera."""
        if not self.enable_camera_capture or self.anomaly_camera_pub is None:
            return

        now_ns = self.get_clock().now().nanoseconds
        max_age_ns = int(self.CAMERA_FRAME_MAX_AGE_SECONDS * 1e9)

        with self._camera_lock:
            snapshot = [
                (source, cached)
                for source in self.CAMERA_SOURCES
                if (cached := self._latest_camera_frames.get(source)) is not None
                and (now_ns - cached[0]) <= max_age_ns
            ]

        for source, (_, img_msg) in snapshot:
            event_header = Header()
            event_header.stamp = img_msg.header.stamp
            event_header.frame_id = f"camera:{source}"
            camera_name = "passenger" if source == "rear" else source
            if reason == "periodic context":
                message = f"Periodic context image; camera={camera_name}"
            else:
                message = f"Camera frame captured for {reason}; camera={camera_name}"
            self.anomaly_logging(
                message,
                AnomalyMsg.INFO,
                header=event_header,
                msg_type=AnomalyMsg.IMAGE,
                image=img_msg,
                publisher=self.anomaly_camera_pub,
            )

    def stop_callback(self, stop_msg: Stop):
        new_stop_state = bool(stop_msg.stop)
        if new_stop_state == self._last_stop_state:
            return

        self._last_stop_state = new_stop_state
        sender = str(stop_msg.sender_id.data).strip()

        if new_stop_state:
            self._publish_stop_camera_snapshot()
            self.anomaly_logging(
                "Stop signal received, obstacle  "
                f"distance={self._format_number(stop_msg.distance)}m",
                AnomalyMsg.ERROR,
                header=stop_msg.header,
            )
        else:
            self.anomaly_logging(
                f"Collision detector: stop signal cleared",
                AnomalyMsg.INFO,
                header=stop_msg.header,
            )

<<<<<<< HEAD
=======
    def speed_callback(self, msg: Float32):
        if abs(self.last_speed - msg.data) > 0.1:
            self.last_speed = msg.data

            self.anomaly_logging(
                f"Planner target speed changed to {msg.data:.2f} m/s",
                AnomalyMsg.INFO,
                frame_id="collision_avoidance_frame",
            )

    def commanded_speed_callback(self, msg: VelAngle):
        # Negative /nav_cmd velocities encode obstacle distance, not reverse speed.
        commanded_speed_mps = max(0.0, msg.vel)
        if (
            self.last_commanded_speed_mps is None
            or abs(self.last_commanded_speed_mps - commanded_speed_mps) > 0.1
        ):
            self.last_commanded_speed_mps = commanded_speed_mps
            self.anomaly_logging(
                f"Motor commanded speed changed to {commanded_speed_mps:.2f} m/s",
                AnomalyMsg.INFO,
                frame_id="collision_avoidance_frame",
            )

    def estimated_speed_callback(self, msg: TwistStamped):
        speed_mps = msg.twist.linear.x
        now = self.get_clock().now()
        if (
            now - self.last_moving_pub_time
        ).nanoseconds <= self.MOVING_LOG_PERIOD * 1e9:
            return

        commanded_speed = (
            "unavailable"
            if self.last_commanded_speed_mps is None
            else f"{self.last_commanded_speed_mps:.2f} m/s"
        )
        self.anomaly_logging(
            f"Speed status: commanded={commanded_speed}, "
            f"estimated={speed_mps:.2f} m/s",
            AnomalyMsg.INFO,
            frame_id="collision_avoidance_frame",
        )
        self.last_moving_pub_time = now

>>>>>>> mola_main_sum26
    def localization_health_callback(self, msg: DiagnosticArray):
        if not msg.status:
            return

        status = msg.status[0]
        values = {value.key: value.value for value in status.values}
        signature = (
            f"{status.level}|{status.message}|"
            f"{values.get('failure_category', '')}|"
            f"{values.get('recovery_state', '')}|"
            f"{values.get('reinitialization_requested', '')}"
        )

        now = self.get_clock().now()
        should_publish_periodic = (
            now - self.last_localization_health_pub_time
        ).nanoseconds > self.LOCALIZATION_HEALTH_LOG_PERIOD * 1e9
        should_publish_change = signature != self.last_localization_health_signature
        if not should_publish_periodic and not should_publish_change:
            return

        self.anomaly_logging(
            self._format_legacy_localization_health(status, values),
            self._diagnostic_level_to_anomaly_importance(status.level),
            header=msg.header,
            frame_id="localization_health_frame",
        )
        self.last_localization_health_pub_time = now
        self.last_localization_health_signature = signature

    def mola_diagnostics_callback(self, msg: String):
        fields = self._parse_mola_diagnostics(msg.data)
        severity, reasons = self._mola_diagnostics_importance(fields)
        signature = (
            f"mola|{severity}|"
            f"{fields.get('active', '')}|"
            f"{fields.get('icp_quality', '')}|"
            f"{fields.get('dropped_frames_ratio', '')}|"
            f"{';'.join(reasons)}|"
            f"{msg.data[:120]}"
        )

        now = self.get_clock().now()
        should_publish_periodic = (
            now - self.last_localization_health_pub_time
        ).nanoseconds > self.LOCALIZATION_HEALTH_LOG_PERIOD * 1e9
        should_publish_change = signature != self.last_localization_health_signature
        if not should_publish_periodic and not should_publish_change:
            return

        self.anomaly_logging(
            self._format_mola_localization_health(fields, reasons, msg.data),
            severity,
            frame_id="mola_localization_health_frame",
        )
        self.last_localization_health_pub_time = now
        self.last_localization_health_signature = signature

    def _diagnostic_level_to_anomaly_importance(self, level):
        if level >= DiagnosticStatus.ERROR:
            return AnomalyMsg.ERROR
        if level >= DiagnosticStatus.WARN:
            return AnomalyMsg.WARNING
        return AnomalyMsg.INFO

    def _format_legacy_localization_health(self, status, values):
        fitness = self._format_maybe_numeric_value(
            values.get('fitness_score', 'unknown'),
            key='fitness_score',
        )
        threshold = self._format_maybe_numeric_value(
            values.get('effective_score_threshold', values.get('score_threshold', 'unknown')),
            key='effective_score_threshold',
        )
        consecutive_rejected = self._format_maybe_numeric_value(
            values.get('consecutive_rejected_updates', 'unknown'),
            key='consecutive_rejected_updates',
        )
        fields = [
            "source=legacy_alignment_status",
            f"status={status.message}",
            f"level={status.level}",
            f"fitness={fitness}",
            f"threshold={threshold}",
            f"failure_category={values.get('failure_category', 'unknown')}",
            f"recovery_state={values.get('recovery_state', 'unknown')}",
            f"reinit_requested={values.get('reinitialization_requested', 'unknown')}",
            f"reinit_reason={values.get('reinitialization_request_reason', 'unknown')}",
            f"consecutive_rejected={consecutive_rejected}",
        ]
        return "Localization health: " + ", ".join(fields)

    def _format_mola_localization_health(self, fields, reasons, raw_text):
        if not fields:
            raw = " ".join(str(raw_text).split())
            if len(raw) > 200:
                raw = raw[:197] + "..."
            return f"MOLA localization health: source=mola_diagnostics, raw={raw}"

        summary_fields = [
            f"status={'unhealthy' if reasons else 'healthy'}",
            f"icp_quality={self._format_field_value(fields, 'icp_quality', 'unknown')}",
            f"icp_quality_threshold={self._format_number(self.MOLA_BAD_ICP_QUALITY_THRESHOLD)}",
            f"too_many_dropped_frames={any('dropped frame' in reason for reason in reasons)}",
        ]
        if reasons:
            summary_fields.append(f"reasons={'; '.join(reasons)}")
        return "MOLA localization health: " + ", ".join(summary_fields)

    def _mola_diagnostics_importance(self, fields):
        reasons = []
        active = self._mola_field(fields, "active")
        if active is False or active == 0.0:
            reasons.append("MOLA diagnostics report inactive")

        icp_quality = self._mola_float(fields, "icp_quality")
        if (
            icp_quality is not None
            and icp_quality < self.MOLA_BAD_ICP_QUALITY_THRESHOLD
        ):
            reasons.append(
                f"ICP quality {self._format_number(icp_quality)} below "
                f"{self._format_number(self.MOLA_BAD_ICP_QUALITY_THRESHOLD)}"
            )

        dropped_frames_ratio = self._mola_float(fields, "dropped_frames_ratio")
        if (
            dropped_frames_ratio is not None
            and dropped_frames_ratio > self.MOLA_BAD_DROPPED_FRAMES_THRESHOLD
        ):
            reasons.append(
                f"dropped frame ratio {self._format_number(dropped_frames_ratio)} above "
                f"{self._format_number(self.MOLA_BAD_DROPPED_FRAMES_THRESHOLD)}"
            )

        if active is False or active == 0.0:
            return AnomalyMsg.ERROR, reasons
        if reasons:
            return AnomalyMsg.WARNING, reasons
        return AnomalyMsg.INFO, reasons

    def _parse_mola_diagnostics(self, text):
        fields = {}
        stripped = str(text).strip()
        if not stripped:
            return fields

        parsed = None
        try:
            parsed = json.loads(stripped)
        except json.JSONDecodeError:
            try:
                parsed = yaml.safe_load(stripped)
            except yaml.YAMLError:
                parsed = None

        if isinstance(parsed, dict):
            self._flatten_mola_diagnostics(parsed, fields)

        for key, value in re.findall(
            r"([A-Za-z_][\w./-]*)\s*[:=]\s*([^\s,;]+)",
            stripped,
        ):
            fields[self._normalize_mola_key(key)] = self._normalize_mola_value(value)

        return fields

    def _flatten_mola_diagnostics(self, value, out, prefix=""):
        if isinstance(value, dict):
            for key, child in value.items():
                next_prefix = f"{prefix}.{key}" if prefix else str(key)
                self._flatten_mola_diagnostics(child, out, next_prefix)
            return

        if prefix:
            out[self._normalize_mola_key(prefix)] = self._normalize_mola_value(value)

    def _mola_field(self, fields, wanted):
        wanted_norm = self._normalize_mola_key(wanted)
        for key, value in fields.items():
            if key == wanted_norm or key.endswith("." + wanted_norm):
                return value
        return None

    def _mola_float(self, fields, wanted):
        value = self._mola_field(fields, wanted)
        if isinstance(value, bool):
            return 1.0 if value else 0.0
        if isinstance(value, (float, int)):
            return float(value)
        if isinstance(value, str):
            try:
                return float(value)
            except ValueError:
                return None
        return None

    def _normalize_mola_key(self, key):
        return str(key).strip().lower().replace("-", "_").replace("/", ".")

    def _normalize_mola_value(self, value):
        if isinstance(value, bool):
            return value
        if isinstance(value, (float, int)):
            return float(value)
        text = str(value).strip().strip("'\"")
        lowered = text.lower()
        if lowered in {"1", "true", "yes", "on", "active", "enabled"}:
            return True
        if lowered in {"0", "false", "no", "off", "inactive", "disabled"}:
            return False
        try:
            return float(text)
        except ValueError:
            return text

    def _format_number(self, value, decimals=3):
        formatted = f"{float(value):.{decimals}f}"
        if "." in formatted:
            formatted = formatted.rstrip("0").rstrip(".")
        return formatted

    def _is_gps_key(self, key):
        normalized = str(key).strip().lower().replace("-", "_")
        if "gps" in normalized:
            return True

        tokens = [token for token in re.split(r"[._]", normalized) if token]
        gps_tokens = {
            "lat",
            "lon",
            "lng",
            "latitude",
            "longitude",
            "utm_easting",
            "utm_northing",
        }
        return any(token in gps_tokens for token in tokens)

    def _format_maybe_numeric_value(self, value, key=""):
        if value is None:
            return "unknown"
        if isinstance(value, bool):
            return value
        if key and self._is_gps_key(key):
            return value
        if isinstance(value, (float, int)):
            return self._format_number(value)
        if isinstance(value, str):
            stripped = value.strip()
            try:
                return self._format_number(float(stripped))
            except ValueError:
                return value
        return value

    def _format_field_value(self, fields, key, default="unknown"):
        value = self._mola_field(fields, key)
        if value is None:
            return default
        return self._format_maybe_numeric_value(value, key=key)

    def anomaly_logging(
        self,
        message,
        severity,
        header=None,
        frame_id="collision_avoidance_frame",
        msg_type=AnomalyMsg.TEXT,
        image=None,
        data_type="",
        data=None,
        publisher=None,
    ):
        anomaly = AnomalyMsg()
        if header is not None:
            anomaly.header = header
        else:
            anomaly.header = Header()
            anomaly.header.stamp = self.get_clock().now().to_msg()

        if not anomaly.header.frame_id:
            anomaly.header.frame_id = frame_id

        anomaly.node_name = self.get_name()
        anomaly.importance = severity
        anomaly.type = msg_type
        anomaly.msg = message
        if image is not None:
            anomaly.image = image
        if data_type:
            anomaly.data_type = data_type
        if data is not None:
            anomaly.data = list(data)

        (publisher or self.anomaly_log_pub).publish(anomaly)
    

def main(args=None):
    rclpy.init(args=args)
    node = CollisionAvoidanceAADLog()
    
    # Use multiple callbacks without allowing image processing to block logs.
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    
    try:
        executor.spin()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
