import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from tf2_ros import TransformBroadcaster


class MolaOdomToTf(Node):
    def __init__(self) -> None:
        super().__init__("mola_odom_to_tf")

        self.declare_parameter("odom_topic", "/lidar_odometry/pose")
        self.declare_parameter("parent_frame", "map")
        self.declare_parameter("child_frame", "base_link")
        self.declare_parameter("publish_rate_hz", 20.0)
        self.declare_parameter("use_msg_stamp", False)

        parent_param = self.get_parameter("parent_frame").get_parameter_value()
        child_param = self.get_parameter("child_frame").get_parameter_value()
        stamp_param = self.get_parameter("use_msg_stamp").get_parameter_value()
        rate_param = (
            self.get_parameter("publish_rate_hz").get_parameter_value()
        )
        topic_param = self.get_parameter("odom_topic").get_parameter_value()

        self.parent_frame = parent_param.string_value
        self.child_frame = child_param.string_value
        self.use_msg_stamp = stamp_param.bool_value
        publish_rate_hz = rate_param.double_value
        odom_topic = topic_param.string_value

        if publish_rate_hz <= 0.0:
            self.get_logger().warn(
                "publish_rate_hz must be positive; using default 20.0 Hz"
            )
            publish_rate_hz = 20.0

        self.latest_odom = None
        self.logged_first_odom = False
        self.tf_broadcaster = TransformBroadcaster(self)

        self.create_subscription(Odometry, odom_topic, self._odom_callback, 10)
        self.create_timer(1.0 / publish_rate_hz, self._publish_tf)

        self.get_logger().info(
            f"Bridging Odometry '{odom_topic}' to TF "
            f"at {publish_rate_hz:.1f} Hz"
        )

    def _odom_callback(self, msg: Odometry) -> None:
        self.latest_odom = msg
        if not self.logged_first_odom:
            self.get_logger().info("Received first MOLA odometry message")
            self.logged_first_odom = True

    def _publish_tf(self) -> None:
        if self.latest_odom is None:
            return

        msg = self.latest_odom
        parent_frame = msg.header.frame_id or self.parent_frame
        child_frame = msg.child_frame_id or self.child_frame

        transform = TransformStamped()
        transform.header.frame_id = parent_frame
        transform.child_frame_id = child_frame
        if self.use_msg_stamp:
            transform.header.stamp = msg.header.stamp
        else:
            transform.header.stamp = self.get_clock().now().to_msg()
        transform.transform.translation.x = msg.pose.pose.position.x
        transform.transform.translation.y = msg.pose.pose.position.y
        transform.transform.translation.z = msg.pose.pose.position.z
        transform.transform.rotation = msg.pose.pose.orientation

        self.tf_broadcaster.sendTransform(transform)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MolaOdomToTf()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
