import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from std_msgs.msg import Bool, Float32
from navigation_interface.msg import Stop
from std_msgs.msg import Header

from anomaly_msg.msg import AnomalyMsg


class CollisionAvoidanceAADLog(Node):

    def __init__(self):
        super().__init__('collision_avoidance_anomaly_log')

        self.IMG_PUBLISH_SPEED = 2

        # --- Subscribers ---
        self.camera_sub = self.create_subscription(
            Image,
            '/zed_front/zed_node_0/rgb/color/rect/image',
            self.camera_callback,
            10
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

        # --- Publishers ---
        self.anomaly_pub = self.create_publisher(
            AnomalyMsg,
            '/ai_anomaly_logging',
            10
        )

        self.speed_pub = self.create_publisher(
            Float32,
            '/speed_out',
            10
        )

        self.stop_pub = self.create_publisher(
            Stop,
            '/stop_out',
            10
        )
        self.last_image = None
        self.last_speed = 0.0

        self.timer = self.create_timer(
            self.IMG_PUBLISH_SPEED, 
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

        if stop_msg.stop:
            anomaly.importance = AnomalyMsg.WARNING
            anomaly.msg = "Stop signal received."

            self.anomaly_pub.publish(anomaly)

    def speed_callback(self, msg: Float32):
        if abs(self.last_speed - msg.data) > 1e-3:
            self.last_speed = msg.data

            anomaly = AnomalyMsg()

            anomaly.header = Header()
            anomaly.header.stamp = self.get_clock().now().to_msg()
            anomaly.header.frame_id = "collision_avoidance_frame"

            anomaly.node_name = self.get_name()
            anomaly.importance = AnomalyMsg.INFO
            anomaly.msg = f"The speed of the cart is {msg.data}"

            self.anomaly_pub.publish(anomaly)

    def timer_callback():
        if self.last_image: 
            self.anomaly_pub.publish(last_image)
    

def main(args=None):
    rclpy.init(args=args)
    node = CollisionAvoidanceAADLog()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()