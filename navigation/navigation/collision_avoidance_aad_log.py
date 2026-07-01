import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
import threading
from queue import Queue

from sensor_msgs.msg import Image
from std_msgs.msg import Float32
from navigation_interface.msg import Stop
from std_msgs.msg import Header
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, qos_profile_sensor_data
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus

from anomaly_msg.msg import AnomalyMsg


class CollisionAvoidanceAADLog(Node):

    def __init__(self):
        super().__init__('collision_avoidance_aad_log')

        self.IMG_PUBLISH_PERIOD = 5
        self.MOVING_LOG_PERIOD = 5
        self.MOVING_SPEED_THRESHOLD_MPS = 0.1
        self.LOCALIZATION_HEALTH_LOG_PERIOD = 5

        self.get_logger().info("Creating subscribers")
        # --- Subscribers ---

        # Use minimal QoS for camera - drop frames if can't keep up
        camera_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1  # Only keep latest frame
        )

        self.camera_0_sub = self.create_subscription(
            Image,
            '/zed_front/zed_node_0/rgb/color/rect/image',
            self.camera_callback,
            camera_qos
        )

        self.camera_1_sub = self.create_subscription(
            Image,
            '/zed_rear/zed_node_1/rgb/color/rect/image',
            self.camera_callback,
            camera_qos
        )

        self.speed_sub = self.create_subscription(
            Float32,
            '/speed',
            self.speed_callback,
            10
        )

        self.estimated_speed_sub = self.create_subscription(
            Float32,
            '/estimated_vel_mps',
            self.estimated_speed_callback,
            10
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
        self.get_logger().info("Finished creating subscribers")
        self.get_logger().info("Creating publishers")

        # --- Publisher ---
        self.anomaly_camera_pub = self.create_publisher(
            AnomalyMsg,
            '/ai_anomaly_logging',
            camera_qos
        )

                # --- Publisher ---
        self.anomaly_log_pub = self.create_publisher(
            AnomalyMsg,
            '/ai_anomaly_logging',
            10
        )
        self.get_logger().info("Finished creating publishers")

        self.last_pub_time = self.get_clock().now()
        self.last_moving_pub_time = self.get_clock().now()
        self.last_localization_health_pub_time = self.get_clock().now()
        self.last_localization_health_signature = ""
        self.last_speed = 0.0

        # --- Async processing setup ---
        self.image_queue = Queue(maxsize=1)  # Only keep latest image
        self.processing_thread = threading.Thread(target=self._process_images, daemon=True)
        self.processing_thread.start()

    def _process_images(self):
        """Background thread that processes images without blocking the callback"""
        while rclpy.ok():
            try:
                # Block until an image is available
                img_msg = self.image_queue.get(timeout=0.1)
                
                now = self.get_clock().now()
                
                # Only process if enough time has passed
                if (now - self.last_pub_time).nanoseconds > self.IMG_PUBLISH_PERIOD * 1e9:
                    self.anomaly_logging(
                        "Camera frame received.",
                        AnomalyMsg.INFO,
                        header=img_msg.header,
                        msg_type=AnomalyMsg.IMAGE,
                        image=img_msg,
                        publisher=self.anomaly_camera_pub,
                    )
                    self.last_pub_time = now
                    
            except:
                # Queue timeout - continue waiting
                pass

    # --- Callbacks ---

    def camera_callback(self, img_msg: Image):
        """Callback returns immediately - just queues the image"""
        try:
            # Non-blocking: put latest image, discard old one if queue full
            self.image_queue.put_nowait(img_msg)
        except:
            # Queue full - that's okay, we'll process the next frame
            pass

    def stop_callback(self, stop_msg: Stop):
        if stop_msg.stop:
            self.anomaly_logging(
                f"Stop signal received from {stop_msg.sender_id.data}; distance={stop_msg.distance:.2f}",
                AnomalyMsg.WARNING,
                header=stop_msg.header,
            )
        else:
            self.anomaly_logging(
                f"Stop signal cleared by {stop_msg.sender_id.data}",
                AnomalyMsg.INFO,
                header=stop_msg.header,
            )

    def speed_callback(self, msg: Float32):
        if abs(self.last_speed - msg.data) > 0.1:
            self.last_speed = msg.data

            self.anomaly_logging(
                f"Planner target speed changed to {msg.data:.2f} km/h",
                AnomalyMsg.INFO,
                frame_id="collision_avoidance_frame",
            )

    def estimated_speed_callback(self, msg: Float32):
        if abs(msg.data) < self.MOVING_SPEED_THRESHOLD_MPS:
            return

        now = self.get_clock().now()
        if (
            now - self.last_moving_pub_time
        ).nanoseconds <= self.MOVING_LOG_PERIOD * 1e9:
            return

        self.anomaly_logging(
            f"The cart is moving at {msg.data:.2f} m/s",
            AnomalyMsg.INFO,
            frame_id="collision_avoidance_frame",
        )
        self.last_moving_pub_time = now

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
            self._format_localization_health(status, values),
            self._diagnostic_level_to_anomaly_importance(status.level),
            header=msg.header,
            frame_id="localization_health_frame",
        )
        self.last_localization_health_pub_time = now
        self.last_localization_health_signature = signature

    def _diagnostic_level_to_anomaly_importance(self, level):
        if level >= DiagnosticStatus.ERROR:
            return AnomalyMsg.ERROR
        if level >= DiagnosticStatus.WARN:
            return AnomalyMsg.WARNING
        return AnomalyMsg.INFO

    def _format_localization_health(self, status, values):
        fields = [
            f"status={status.message}",
            f"level={status.level}",
            f"fitness={values.get('fitness_score', 'unknown')}",
            f"threshold={values.get('effective_score_threshold', values.get('score_threshold', 'unknown'))}",
            f"failure_category={values.get('failure_category', 'unknown')}",
            f"recovery_state={values.get('recovery_state', 'unknown')}",
            f"reinit_requested={values.get('reinitialization_requested', 'unknown')}",
            f"reinit_reason={values.get('reinitialization_request_reason', 'unknown')}",
            f"consecutive_rejected={values.get('consecutive_rejected_updates', 'unknown')}",
        ]
        return "Localization health: " + ", ".join(fields)

    def anomaly_logging(
        self,
        message,
        severity,
        header=None,
        frame_id="collision_avoidance_frame",
        msg_type=AnomalyMsg.TEXT,
        image=None,
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

        (publisher or self.anomaly_log_pub).publish(anomaly)
    

def main(args=None):
    rclpy.init(args=args)
    node = CollisionAvoidanceAADLog()
    
    # Use MultiThreadedExecutor to allow background thread to work
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    
    try:
        executor.spin()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
