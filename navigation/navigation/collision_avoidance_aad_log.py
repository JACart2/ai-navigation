import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from std_msgs.msg import Float32
from navigation_interface.msg import Stop
from std_msgs.msg import Header
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from anomaly_msg.msg import AnomalyMsg


class CollisionAvoidanceAADLog(Node):

    def __init__(self):
        super().__init__('collision_avoidance_anomaly_log')

        self.IMG_PUBLISH_PERIOD = 0.5 

        self.get_logger().info("Creating subsribers")
        # --- Subscribers ---


        camera_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.camera_sub = self.create_subscription(
            Image,
            '/zed_front/zed_node_0/rgb/color/rect/image',
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
        self.get_logger().info("Finished creating subsribers")
        self.get_logger().info("Creating publishers")

        # --- Publisher ---
        self.anomaly_pub = self.create_publisher(
            AnomalyMsg,
            '/ai_anomaly_logging',
            10
        )
        self.get_logger().info("Finished creating publishers")

        self.last_image = None
        self.last_speed = 0.0

        self.timer = self.create_timer(
            self.IMG_PUBLISH_PERIOD, 
            self.timer_callback)

    # --- Callbacks ---

    def camera_callback(self, img_msg: Image):
        anomaly = AnomalyMsg()
        anomaly.header = img_msg.header
        anomaly.node_name = self.get_name()
        anomaly.importance = AnomalyMsg.INFO
        anomaly.type = AnomalyMsg.IMAGE
        anomaly.msg = "Camera frame received."
        anomaly.image = img_msg

        self.last_image = anomaly

    def stop_callback(self, stop_msg: Stop):
        anomaly = AnomalyMsg()
        anomaly.header = stop_msg.header
        anomaly.node_name = self.get_name()
        anomaly.type = AnomalyMsg.MSG

        if stop_msg.stop:
            anomaly.importance = AnomalyMsg.WARNING
            anomaly.msg = "Stop signal received."

            self.anomaly_pub.publish(anomaly)

    def speed_callback(self, msg: Float32):
        if abs(self.last_speed - msg.data) > 0.1:
            self.last_speed = msg.data

            anomaly = AnomalyMsg()

            anomaly.header = Header()
            anomaly.header.stamp = self.get_clock().now().to_msg()
            anomaly.header.frame_id = "collision_avoidance_frame"

            anomaly.node_name = self.get_name()
            anomaly.importance = AnomalyMsg.INFO
            anomaly.type = AnomalyMsg.MSG
            anomaly.msg = f"The speed of the cart is {msg.data}"

            self.anomaly_pub.publish(anomaly)

    def timer_callback(self):
        if self.last_image: 
            self.get_logger().info("Publishing image")


            self.anomaly_pub.publish(self.last_image)
            self.last_image = None
        else:
            self.get_logger().info("Issue with previous image")
    

def main(args=None):
    rclpy.init(args=args)
    node = CollisionAvoidanceAADLog()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()