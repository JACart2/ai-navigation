#!/usr/bin/env python3

"""Minimal NMEA serial driver for Garmin GPS 18x USB.

Publishes sensor_msgs/NavSatFix on the `fix` topic from a serial NMEA stream.
"""

from __future__ import annotations

import math
from typing import Optional

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, NavSatStatus

try:
    import serial
except ImportError as exc:  # pragma: no cover - import error is runtime only
    serial = None
    _SERIAL_IMPORT_ERROR = exc
else:
    _SERIAL_IMPORT_ERROR = None


def _nmea_checksum_ok(sentence: str) -> bool:
    if not sentence.startswith("$") or "*" not in sentence:
        return False
    payload, checksum_text = sentence[1:].split("*", 1)
    checksum = 0
    for char in payload:
        checksum ^= ord(char)
    try:
        expected = int(checksum_text.strip()[:2], 16)
    except ValueError:
        return False
    return checksum == expected


def _parse_degrees(value: str, direction: str) -> Optional[float]:
    if not value:
        return None
    try:
        raw = float(value)
    except ValueError:
        return None

    degrees = int(raw // 100)
    minutes = raw - degrees * 100
    decimal = degrees + minutes / 60.0
    if direction in ("S", "W"):
        decimal = -decimal
    return decimal


def _covariance_for_fix(fix_quality: int, hdop: Optional[float]) -> tuple[list[float], int]:
    # Approximate the same quality-to-error mapping used by nmea_navsat_driver.
    quality_map = {
        -1: (1_000_000.0, NavSatStatus.STATUS_NO_FIX),
        0: (1_000_000.0, NavSatStatus.STATUS_NO_FIX),
        1: (4.0, NavSatStatus.STATUS_FIX),
        2: (0.1, NavSatStatus.STATUS_SBAS_FIX),
        4: (0.02, NavSatStatus.STATUS_GBAS_FIX),
        5: (4.0, NavSatStatus.STATUS_GBAS_FIX),
        9: (3.0, NavSatStatus.STATUS_GBAS_FIX),
    }
    default_epe, status = quality_map.get(fix_quality, quality_map[-1])
    if hdop is None or math.isnan(hdop):
        hdop = 1.0
    lon_std_dev = default_epe
    lat_std_dev = default_epe
    alt_std_dev = default_epe * 2.0
    covariance = [0.0] * 9
    covariance[0] = (hdop * lon_std_dev) ** 2
    covariance[4] = (hdop * lat_std_dev) ** 2
    covariance[8] = (hdop * alt_std_dev) ** 2
    return covariance, status


class GarminGps18xDriver(Node):
    def __init__(self) -> None:
        super().__init__("garmin_gps18x_driver")

        self._port = self.declare_parameter("port", "/dev/ttyUSB0").value
        self._baud = int(self.declare_parameter("baud", 4800).value)
        self._frame_id = self.declare_parameter("frame_id", "gps").value
        self._use_rmc = bool(self.declare_parameter("use_rmc", False).value)
        self._time_ref_source = self.declare_parameter("time_ref_source", "gps").value

        self._fix_pub = self.create_publisher(NavSatFix, "fix", 10)

        if serial is None:
            raise RuntimeError(f"pyserial is not available: {_SERIAL_IMPORT_ERROR}")

        self._serial = serial.Serial(port=self._port, baudrate=self._baud, timeout=1.0)
        self.get_logger().info(
            f"Reading Garmin GPS NMEA from {self._port} at {self._baud} baud"
        )

    def _publish_fix_from_gga(self, fields: list[str]) -> None:
        if len(fields) < 10:
            return

        fix = NavSatFix()
        fix.header.stamp = self.get_clock().now().to_msg()
        fix.header.frame_id = self._frame_id
        fix.status.service = NavSatStatus.SERVICE_GPS

        try:
            fix_quality = int(fields[6]) if fields[6] else -1
        except ValueError:
            fix_quality = -1

        hdop = None
        if fields[8]:
            try:
                hdop = float(fields[8])
            except ValueError:
                hdop = None

        covariance, status = _covariance_for_fix(fix_quality, hdop)
        fix.status.status = status
        fix.position_covariance = covariance
        fix.position_covariance_type = NavSatFix.COVARIANCE_TYPE_APPROXIMATED

        latitude = _parse_degrees(fields[2], fields[3])
        longitude = _parse_degrees(fields[4], fields[5])
        if latitude is None or longitude is None:
            self.get_logger().warn("Skipping GGA sentence with invalid coordinates")
            return

        fix.latitude = latitude
        fix.longitude = longitude

        altitude = float("nan")
        if fields[9]:
            try:
                altitude = float(fields[9])
            except ValueError:
                altitude = float("nan")
        if len(fields) > 11 and fields[11]:
            try:
                altitude += float(fields[11])
            except ValueError:
                pass
        fix.altitude = altitude

        self._fix_pub.publish(fix)

    def _publish_fix_from_rmc(self, fields: list[str]) -> None:
        if len(fields) < 7:
            return

        fix = NavSatFix()
        fix.header.stamp = self.get_clock().now().to_msg()
        fix.header.frame_id = self._frame_id
        fix.status.service = NavSatStatus.SERVICE_GPS

        fix.status.status = NavSatStatus.STATUS_FIX if fields[2] == "A" else NavSatStatus.STATUS_NO_FIX
        fix.position_covariance_type = NavSatFix.COVARIANCE_TYPE_UNKNOWN

        latitude = _parse_degrees(fields[3], fields[4])
        longitude = _parse_degrees(fields[5], fields[6])
        if latitude is None or longitude is None:
            self.get_logger().warn("Skipping RMC sentence with invalid coordinates")
            return

        fix.latitude = latitude
        fix.longitude = longitude
        fix.altitude = float("nan")

        self._fix_pub.publish(fix)

    def spin(self) -> None:
        try:
            while rclpy.ok():
                raw = self._serial.readline()
                if not raw:
                    continue

                try:
                    line = raw.decode("utf-8", errors="ignore").strip()
                except Exception:
                    continue

                if "$" not in line or "*" not in line:
                    continue

                # Some USB adapters prepend non-NMEA bytes. Extract each NMEA sentence
                # that starts at a '$' and ends at the checksum delimiter.
                for fragment in line.split("$"):
                    if "*" not in fragment:
                        continue
                    sentence = "$" + fragment
                    if not _nmea_checksum_ok(sentence):
                        continue

                    payload = sentence[1: sentence.index("*")]
                    fields = payload.split(",")
                    sentence_type = fields[0][2:] if len(fields[0]) >= 5 else fields[0]

                    if sentence_type == "GGA":
                        self._publish_fix_from_gga(fields)
                    elif self._use_rmc and sentence_type == "RMC":
                        self._publish_fix_from_rmc(fields)
        finally:
            self._serial.close()


def main(args=None) -> int:
    rclpy.init(args=args)
    node = GarminGps18xDriver()
    try:
        node.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return 0
