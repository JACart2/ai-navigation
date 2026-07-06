import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped


class PclPoseRelay(Node):
    def __init__(self) -> None:
        super().__init__("pcl_pose_relay")
        self.publisher_ = self.create_publisher(
            PoseWithCovarianceStamped,
            "/pcl_pose",
            10,
        )
        self.subscription = self.create_subscription(
            Odometry,
            "/lidar_odometry/pose",
            self._pose_callback,
            10,
        )

    def _pose_callback(self, msg: Odometry) -> None:
        out = PoseWithCovarianceStamped()
        out.header = msg.header
        out.pose = msg.pose
        self.publisher_.publish(out)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PclPoseRelay()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
