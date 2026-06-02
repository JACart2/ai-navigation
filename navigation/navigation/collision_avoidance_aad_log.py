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

from anomaly_msg.msg import AnomalyMsg


class CollisionAvoidanceAADLog(Node):

    def __init__(self):
        super().__init__('collision_avoidance_aad_log')

        self.IMG_PUBLISH_PERIOD = 5

        self.get_logger().info("Creating subscribers")
        # --- Subscribers ---

        # Use reliable QoS for camera frames and anomaly publishing.
        camera_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
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
            AnomalyMsg,
            '/ai_anomaly_logging',
            camera_qos
        )

                # --- Publisher ---
        self.anomaly_log_pub = self.create_publisher(
            AnomalyMsg,
            '/ai_anomaly_logging',
            camera_qos
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
                    anomaly = AnomalyMsg()
                    anomaly.header = img_msg.header
                    anomaly.node_name = self.get_name()
                    anomaly.importance = AnomalyMsg.INFO
                    anomaly.type = AnomalyMsg.IMAGE
                    anomaly.msg = "Camera frame received."
                    anomaly.image = img_msg  # Full image is safe here - no blocking

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

    def stop_callback(self, stop_msg: Stop):
        anomaly = AnomalyMsg()
        anomaly.header = stop_msg.header
        anomaly.node_name = self.get_name()
        anomaly.type = AnomalyMsg.TEXT

        if stop_msg.stop:
            anomaly.importance = AnomalyMsg.WARNING
            anomaly.msg = "Stop signal received."

            self.anomaly_log_pub.publish(anomaly)

    def speed_callback(self, msg: Float32):
        if abs(self.last_speed - msg.data) > 0.1:
            self.last_speed = msg.data

            anomaly = AnomalyMsg()

            anomaly.header = Header()
            anomaly.header.stamp = self.get_clock().now().to_msg()
            anomaly.header.frame_id = "collision_avoidance_frame"

            anomaly.node_name = self.get_name()
            anomaly.importance = AnomalyMsg.INFO
            anomaly.type = AnomalyMsg.TEXT
            anomaly.msg = f"The speed of the cart is {msg.data}"

            self.anomaly_log_pub.publish(anomaly)
    

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