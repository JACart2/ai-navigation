#!/usr/bin/env python3

import math
import socket

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, NavSatStatus


def nmea_checksum_ok(sentence: str) -> bool:
    sentence = sentence.strip()
    if not sentence.startswith("$") or "*" not in sentence:
        return False

    body, checksum = sentence[1:].split("*", 1)
    checksum = checksum[:2]

    calc = 0
    for ch in body:
        calc ^= ord(ch)

    try:
        expected = int(checksum, 16)
    except ValueError:
        return False

    return calc == expected


def strip_checksum(sentence: str) -> str:
    sentence = sentence.strip()
    if "*" in sentence:
        return sentence.split("*", 1)[0]
    return sentence


def parse_nmea_coord(value: str, hemi: str):
    if not value or not hemi:
        return math.nan

    raw = float(value)

    degrees = int(raw // 100)
    minutes = raw - degrees * 100
    decimal = degrees + minutes / 60.0

    if hemi in ("S", "W"):
        decimal *= -1.0

    return decimal


class VelodyneGpsUdpDriver(Node):
    def __init__(self):
        super().__init__("velodyne_gps_udp_driver")

        self.declare_parameter("host", "0.0.0.0")
        self.declare_parameter("port", 8308)
        self.declare_parameter("output_topic", "/velodyne_fix")
        self.declare_parameter("frame_id", "gps")

        self.host = self.get_parameter("host").value
        self.port = int(self.get_parameter("port").value)
        self.frame_id = self.get_parameter("frame_id").value
        output_topic = self.get_parameter("output_topic").value

        self.pub = self.create_publisher(NavSatFix, output_topic, 10)

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind((self.host, self.port))
        self.sock.setblocking(False)

        self.timer = self.create_timer(0.02, self.poll_udp)

        self.get_logger().info(
            f"Listening for Velodyne GPS UDP packets on {self.host}:{self.port}; "
            f"publishing {output_topic}"
        )

    def poll_udp(self):
        for _ in range(20):
            try:
                data, addr = self.sock.recvfrom(2048)
            except BlockingIOError:
                return
            except Exception as exc:
                self.get_logger().warn(f"UDP receive error: {exc}")
                return

            sentence = self.extract_nmea(data)
            if not sentence:
                continue

            if not nmea_checksum_ok(sentence):
                self.get_logger().warn(f"Ignoring NMEA with bad checksum: {sentence}")
                continue

            self.handle_sentence(sentence)

    def extract_nmea(self, data: bytes):
        start = data.find(b"$")
        if start < 0:
            return None

        end = len(data)
        for i in range(start, len(data)):
            b = data[i]
            if b in (0x0D, 0x0A):
                end = i
                break
            if b < 0x20 or b > 0x7E:
                end = i
                break

        try:
            return data[start:end].decode("ascii").strip()
        except UnicodeDecodeError:
            return None

    def make_fix(self, status_value, lat, lon, alt=math.nan, variance=None):
        msg = NavSatFix()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id

        msg.status.status = status_value
        msg.status.service = NavSatStatus.SERVICE_GPS

        msg.latitude = lat
        msg.longitude = lon
        msg.altitude = alt

        if status_value >= NavSatStatus.STATUS_FIX and variance is not None:
            msg.position_covariance = [
                variance, 0.0, 0.0,
                0.0, variance, 0.0,
                0.0, 0.0, variance * 4.0,
            ]
            msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_APPROXIMATED
        elif status_value >= NavSatStatus.STATUS_FIX:
            msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_UNKNOWN
        else:
            msg.position_covariance = [
                1e12, 0.0, 0.0,
                0.0, 1e12, 0.0,
                0.0, 0.0, 4e12,
            ]
            msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_APPROXIMATED

        return msg

    def handle_sentence(self, sentence: str):
        clean = strip_checksum(sentence)
        parts = clean.split(",")

        if not parts:
            return

        kind = parts[0]

        try:
            if kind in ("$GPRMC", "$GNRMC"):
                self.handle_rmc(parts)
            elif kind in ("$GPGGA", "$GNGGA"):
                self.handle_gga(parts)
        except Exception as exc:
            self.get_logger().warn(f"Failed to parse NMEA sentence {sentence}: {exc}")

    def handle_rmc(self, parts):
        # $GPRMC,time,status,lat,N/S,lon,E/W,...
        if len(parts) < 7:
            return

        status_char = parts[2]
        lat = parse_nmea_coord(parts[3], parts[4])
        lon = parse_nmea_coord(parts[5], parts[6])

        if not math.isfinite(lat) or not math.isfinite(lon):
            return

        status_value = (
            NavSatStatus.STATUS_FIX
            if status_char == "A"
            else NavSatStatus.STATUS_NO_FIX
        )

        self.pub.publish(self.make_fix(status_value, lat, lon))

    def handle_gga(self, parts):
        # $GPGGA,time,lat,N/S,lon,E/W,quality,num_sats,hdop,altitude,M,...
        if len(parts) < 10:
            return

        lat = parse_nmea_coord(parts[2], parts[3])
        lon = parse_nmea_coord(parts[4], parts[5])

        if not math.isfinite(lat) or not math.isfinite(lon):
            return

        quality = int(parts[6]) if parts[6] else 0
        hdop = float(parts[8]) if parts[8] else math.nan
        alt = float(parts[9]) if parts[9] else math.nan

        status_value = (
            NavSatStatus.STATUS_FIX
            if quality > 0
            else NavSatStatus.STATUS_NO_FIX
        )

        variance = None
        if math.isfinite(hdop):
            variance = max((hdop * 5.0) ** 2, 1.0)

        self.pub.publish(self.make_fix(status_value, lat, lon, alt, variance))


def main(args=None):
    rclpy.init(args=args)
    node = VelodyneGpsUdpDriver()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
