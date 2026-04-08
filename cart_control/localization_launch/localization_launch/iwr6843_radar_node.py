import struct
import threading
import time
from pathlib import Path

import rclpy
from builtin_interfaces.msg import Duration
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from serial import Serial
from std_msgs.msg import Header
from visualization_msgs.msg import Marker, MarkerArray


class Iwr6843RadarNode(Node):
    MAGIC_WORD = b"\x02\x01\x04\x03\x06\x05\x08\x07"
    TLV_TYPE_DETECTED_POINTS = 1
    TLV_TYPE_SIDE_INFO = 7

    def __init__(self):
        super().__init__("iwr6843_radar_node")

        self.declare_parameter(
            "cli_port",
            "/dev/serial/by-id/usb-Silicon_Labs_CP2105_Dual_USB_to_UART_Bridge_Controller_011D1A9D-if00-port0",
        )
        self.declare_parameter(
            "data_port",
            "/dev/serial/by-id/usb-Silicon_Labs_CP2105_Dual_USB_to_UART_Bridge_Controller_011D1A9D-if01-port0",
        )
        self.declare_parameter(
            "cfg_file",
            "/home/collision-avoidance/ti/mmwave_sdk_03_06_02_00-LTS/packages/ti/demo/xwr68xx/mmw/profiles/profile_2d.cfg",
        )
        self.declare_parameter("frame_id", "radar_link")
        self.declare_parameter("cli_baud", 115200)
        self.declare_parameter("data_baud", 921600)
        self.declare_parameter("marker_lifetime_sec", 0.25)
        self.declare_parameter("marker_scale", 0.18)

        self.cli_port = self.get_parameter("cli_port").value
        self.data_port = self.get_parameter("data_port").value
        self.cfg_file = self.get_parameter("cfg_file").value
        self.frame_id = self.get_parameter("frame_id").value
        self.cli_baud = int(self.get_parameter("cli_baud").value)
        self.data_baud = int(self.get_parameter("data_baud").value)
        self.marker_lifetime_sec = float(
            self.get_parameter("marker_lifetime_sec").value
        )
        self.marker_scale = float(self.get_parameter("marker_scale").value)

        self.points_pub = self.create_publisher(PointCloud2, "/radar/points", 10)
        self.markers_pub = self.create_publisher(MarkerArray, "/radar/markers", 10)

        self._data_serial = None
        self._running = True
        self._reader_thread = None
        self._buffer = bytearray()

        self._configure_radar()
        self._start_reader()

    def _configure_radar(self):
        cfg_path = Path(self.cfg_file)
        if not cfg_path.is_file():
            raise FileNotFoundError(f"Radar config file not found: {cfg_path}")

        self.get_logger().info(f"Configuring radar using {cfg_path}")
        with Serial(self.cli_port, self.cli_baud, timeout=1) as cli:
            time.sleep(1.0)
            for raw_line in cfg_path.read_text().splitlines():
                line = raw_line.strip()
                if not line or line.startswith("%"):
                    continue
                cli.write((line + "\n").encode())
                cli.flush()
                time.sleep(0.2)
                response = cli.read_all().decode(errors="ignore").strip()
                if response:
                    self.get_logger().debug(response)

    def _start_reader(self):
        self._data_serial = Serial(self.data_port, self.data_baud, timeout=0.1)
        self._reader_thread = threading.Thread(target=self._read_loop, daemon=True)
        self._reader_thread.start()
        self.get_logger().info(
            f"Radar data reader started on {self.data_port} at {self.data_baud} baud"
        )

    def _read_loop(self):
        while self._running:
            try:
                chunk = self._data_serial.read(4096)
                if chunk:
                    self._buffer.extend(chunk)
                    self._drain_packets()
                else:
                    time.sleep(0.01)
            except Exception as exc:
                self.get_logger().error(f"Radar read loop stopped: {exc}")
                break

    def _drain_packets(self):
        while True:
            start = self._buffer.find(self.MAGIC_WORD)
            if start < 0:
                if len(self._buffer) > len(self.MAGIC_WORD):
                    del self._buffer[:-len(self.MAGIC_WORD)]
                return

            if start > 0:
                del self._buffer[:start]

            if len(self._buffer) < 40:
                return

            try:
                _, version, total_packet_len, _, frame_number, _, num_detected_obj, num_tlvs, _ = struct.unpack_from(
                    "<Q8I", self._buffer, 0
                )
            except struct.error:
                return

            if total_packet_len <= 0:
                del self._buffer[: len(self.MAGIC_WORD)]
                continue

            if len(self._buffer) < total_packet_len:
                return

            packet = bytes(self._buffer[:total_packet_len])
            del self._buffer[:total_packet_len]

            try:
                self._handle_packet(packet, version, frame_number, num_detected_obj, num_tlvs)
            except Exception as exc:
                self.get_logger().warn(f"Skipping malformed radar packet: {exc}")

    def _handle_packet(
        self, packet: bytes, version: int, frame_number: int, num_detected_obj: int, num_tlvs: int
    ):
        del version
        offset = 40
        points = []
        side_info = []

        for _ in range(num_tlvs):
            if offset + 8 > len(packet):
                break
            tlv_type, tlv_length = struct.unpack_from("<II", packet, offset)
            offset += 8
            tlv_value_len = tlv_length - 8
            tlv_end = offset + tlv_value_len
            if tlv_end > len(packet):
                break

            tlv_payload = packet[offset:tlv_end]
            offset = tlv_end

            if tlv_type == self.TLV_TYPE_DETECTED_POINTS:
                points = self._parse_detected_points(tlv_payload, num_detected_obj)
            elif tlv_type == self.TLV_TYPE_SIDE_INFO:
                side_info = self._parse_side_info(tlv_payload, num_detected_obj)

        enriched_points = self._merge_side_info(points, side_info)
        self.points_pub.publish(self._build_pointcloud(enriched_points))
        self.markers_pub.publish(self._build_markers(enriched_points, frame_number))

    def _parse_detected_points(self, payload: bytes, count: int):
        points = []
        point_struct = struct.Struct("<ffff")
        max_count = min(count, len(payload) // point_struct.size)
        for i in range(max_count):
            x, y, z, velocity = point_struct.unpack_from(payload, i * point_struct.size)
            points.append(
                {
                    "x": x,
                    "y": y,
                    "z": z,
                    "velocity": velocity,
                    "snr": 0.0,
                    "noise": 0.0,
                }
            )
        return points

    def _parse_side_info(self, payload: bytes, count: int):
        info = []
        info_struct = struct.Struct("<HH")
        max_count = min(count, len(payload) // info_struct.size)
        for i in range(max_count):
            snr, noise = info_struct.unpack_from(payload, i * info_struct.size)
            info.append({"snr": float(snr), "noise": float(noise)})
        return info

    def _merge_side_info(self, points, side_info):
        for point, info in zip(points, side_info):
            point["snr"] = info["snr"]
            point["noise"] = info["noise"]
        return points

    def _build_pointcloud(self, points):
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = self.frame_id

        fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name="velocity", offset=12, datatype=PointField.FLOAT32, count=1),
            PointField(name="snr", offset=16, datatype=PointField.FLOAT32, count=1),
            PointField(name="noise", offset=20, datatype=PointField.FLOAT32, count=1),
        ]

        point_step = 24
        data = bytearray()
        for point in points:
            data.extend(
                struct.pack(
                    "<ffffff",
                    point["x"],
                    point["y"],
                    point["z"],
                    point["velocity"],
                    point["snr"],
                    point["noise"],
                )
            )

        msg = PointCloud2()
        msg.header = header
        msg.height = 1
        msg.width = len(points)
        msg.fields = fields
        msg.is_bigendian = False
        msg.point_step = point_step
        msg.row_step = point_step * len(points)
        msg.is_dense = True
        msg.data = bytes(data)
        return msg

    def _build_markers(self, points, frame_number: int):
        markers = MarkerArray()

        delete_marker = Marker()
        delete_marker.header.frame_id = self.frame_id
        delete_marker.header.stamp = self.get_clock().now().to_msg()
        delete_marker.ns = "radar_points"
        delete_marker.action = Marker.DELETEALL
        markers.markers.append(delete_marker)

        lifetime = Duration()
        lifetime.sec = int(self.marker_lifetime_sec)
        lifetime.nanosec = int((self.marker_lifetime_sec % 1.0) * 1e9)

        now = self.get_clock().now().to_msg()
        for index, point in enumerate(points):
            marker = Marker()
            marker.header.frame_id = self.frame_id
            marker.header.stamp = now
            marker.ns = "radar_points"
            marker.id = frame_number * 1000 + index
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position.x = point["x"]
            marker.pose.position.y = point["y"]
            marker.pose.position.z = point["z"]
            marker.pose.orientation.w = 1.0
            marker.scale.x = self.marker_scale
            marker.scale.y = self.marker_scale
            marker.scale.z = self.marker_scale
            speed = abs(point["velocity"])
            marker.color.a = 0.95
            marker.color.r = min(1.0, speed / 3.0)
            marker.color.g = max(0.1, 1.0 - min(1.0, speed / 3.0))
            marker.color.b = 0.2
            marker.lifetime = lifetime
            markers.markers.append(marker)

        return markers

    def destroy_node(self):
        self._running = False
        if self._reader_thread is not None:
            self._reader_thread.join(timeout=1.0)
        if self._data_serial is not None:
            try:
                self._data_serial.close()
            except Exception:
                pass
        super().destroy_node()


def main():
    rclpy.init()
    node = Iwr6843RadarNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
