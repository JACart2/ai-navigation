import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
import threading
from queue import Queue

from sensor_msgs.msg import Image
from std_msgs.msg import Float32
from navigation_interface.msg import Stop
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, qos_profile_sensor_data

from anomaly_msg.msg import AnomalyLog


ANOMALY_INFO = "INFO"
ANOMALY_WARNING = "WARNING"


class CollisionAvoidanceAADLog(Node):

    def __init__(self):
        super().__init__('collision_avoidance_aad_log')

        self.IMG_PUBLISH_PERIOD = 15

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

        self.stop_sub = self.create_subscription(
            Stop,
            '/stop',
            self.stop_callback,
            10
        )
        self.get_logger().info("Finished creating subscribers")
        self.get_logger().info("Creating publishers")

        # --- Publisher ---
        self.anomaly_camera_pub = self.create_publisher(
            AnomalyLog,
            '/ai_anomaly_logging',
            camera_qos
        )

                # --- Publisher ---
        self.anomaly_log_pub = self.create_publisher(
            AnomalyLog,
            '/ai_anomaly_logging',
            10
        )
        self.get_logger().info("Finished creating publishers")

        self.last_pub_time = self.get_clock().now()
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
                    # Create and publish anomaly with full image data
                    anomaly = AnomalyLog()
                    anomaly.stamp = img_msg.header.stamp
                    anomaly.node_name = self.get_name()
                    anomaly.source = "camera"
                    anomaly.description = f"[{ANOMALY_INFO}] Camera frame received."
                    anomaly.topic_name = "/zed_front_or_rear/rgb/color/rect/image"
                    anomaly.data_type = "image"
                    anomaly.data = list(img_msg.data)

                    self.anomaly_camera_pub.publish(anomaly)
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

    def publish_text_anomaly(self, message: str, severity: str, topic_name: str):
        anomaly = AnomalyLog()
        anomaly.stamp = self.get_clock().now().to_msg()
        anomaly.node_name = self.get_name()
        anomaly.source = "collision_avoidance"
        anomaly.description = f"[{severity}] {message}"
        anomaly.topic_name = topic_name
        anomaly.data_type = "text"
        anomaly.data = list(message.encode("utf-8"))
        self.anomaly_log_pub.publish(anomaly)

    def stop_callback(self, stop_msg: Stop):
        if stop_msg.stop:
            self.publish_text_anomaly(
                "Stop signal received.",
                ANOMALY_WARNING,
                "/stop"
            )

    def speed_callback(self, msg: Float32):
        if abs(self.last_speed - msg.data) > 0.1:
            self.last_speed = msg.data

            self.publish_text_anomaly(
                f"The speed of the cart is {msg.data}",
                ANOMALY_INFO,
                "/speed"
            )
    

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
