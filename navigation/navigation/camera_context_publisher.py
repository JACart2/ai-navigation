"""Publish low-frequency camera frames on the anomaly logging topic."""

import rclpy
from anomaly_msg.msg import AnomalyMsg
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image as ROSImage


DEFAULT_CAMERA_TOPICS = [
    "/zed_front/zed_node_0/rgb/color/rect/image",
    "/zed_rear/zed_node_1/rgb/color/rect/image",
]


def _friendly_source_label(topic: str) -> str:
    """Return a human-readable camera label for a configured topic."""
    lowered = topic.lower()
    if "front" in lowered:
        return "Front camera"
    if "rear" in lowered:
        return "Rear camera"
    return topic


class CameraContextPublisher(Node):
    """Republish the latest frame from each camera as an ``AnomalyMsg``."""

    def __init__(self) -> None:
        super().__init__("camera_context_publisher")

        self.declare_parameter("raw_input_topic", "/ai_anomaly_logging")
        self.declare_parameter("camera_topics", DEFAULT_CAMERA_TOPICS)
        self.declare_parameter("publish_period_sec", 10.0)

        self.raw_input_topic = (
            self.get_parameter("raw_input_topic").get_parameter_value().string_value
        )
        self.camera_topics = [
            topic.strip()
            for topic in self.get_parameter("camera_topics")
            .get_parameter_value()
            .string_array_value
            if topic.strip()
        ]
        self.publish_period_sec = max(
            0.5,
            self.get_parameter("publish_period_sec")
            .get_parameter_value()
            .double_value,
        )

        best_effort_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self._latest_frames: dict[str, ROSImage] = {}
        self.anomaly_pub = self.create_publisher(
            AnomalyMsg, self.raw_input_topic, 10
        )
        self.camera_subscriptions = [
            self.create_subscription(
                ROSImage,
                topic,
                lambda msg, topic=topic: self._camera_callback(msg, topic),
                best_effort_qos,
            )
            for topic in self.camera_topics
        ]
        self.publish_timer = self.create_timer(
            self.publish_period_sec, self._publish_latest_frames
        )

        self.get_logger().info(
            "Camera context publisher started with "
            f"raw_input_topic={self.raw_input_topic}, "
            f"camera_topics={self.camera_topics}, "
            f"publish_period_sec={self.publish_period_sec}"
        )

    def _camera_callback(self, msg: ROSImage, topic: str) -> None:
        """Cache the latest frame seen on a configured camera topic."""
        self._latest_frames[topic] = msg

    def _publish_latest_frames(self) -> None:
        """Publish the latest cached frame from every configured camera."""
        for topic, image_msg in list(self._latest_frames.items()):
            anomaly = AnomalyMsg()
            anomaly.header = image_msg.header
            anomaly.node_name = self.get_name()
            anomaly.importance = AnomalyMsg.INFO
            anomaly.type = AnomalyMsg.IMAGE
            anomaly.msg = f"{_friendly_source_label(topic)} image received"
            anomaly.image = image_msg
            anomaly.data_type = ""
            anomaly.data = []
            self.anomaly_pub.publish(anomaly)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CameraContextPublisher()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
