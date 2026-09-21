#!/usr/bin/env python3

import argparse
import csv
import json
import math
import sys
from pathlib import Path

import numpy as np
import rclpy
from geometry_msgs.msg import PointStamped
from rclpy.node import Node


NAVIGATION_PYTHON_DIR = Path(__file__).resolve().parents[1] / "navigation"
sys.path.insert(0, str(NAVIGATION_PYTHON_DIR))

import simple_gps_util  # noqa: E402


CSV_FIELDS = [
    "point_id",
    "label",
    "current_x",
    "current_y",
    "target_x",
    "target_y",
]


def is_enabled(value):
    if value is None or value == "":
        return True

    return str(value).strip().lower() not in {
        "0",
        "false",
        "no",
        "off",
        "disabled",
    }


def load_anchors(geojson_path, calibration):
    data = json.loads(
        Path(geojson_path).read_text(encoding="utf-8")
    )

    anchors = []

    for index, feature in enumerate(data.get("features", [])):
        properties = feature.get("properties") or {}

        if not is_enabled(properties.get("enabled")):
            continue

        geometry = feature.get("geometry") or {}
        coordinates = geometry.get("coordinates") or []

        if geometry.get("type") != "Point" or len(coordinates) < 2:
            print(
                f"Skipping feature {index}: invalid Point geometry",
                file=sys.stderr,
            )
            continue

        longitude = float(coordinates[0])
        latitude = float(coordinates[1])

        current_x, current_y = simple_gps_util.gps_to_local(
            latitude,
            longitude,
            *calibration,
        )

        point_id = (
            properties.get("point_id")
            or properties.get("label")
            or f"anchor_{index}"
        )

        label = (
            properties.get("label")
            or properties.get("point_id")
            or f"Anchor {index}"
        )

        anchors.append(
            {
                "point_id": str(point_id),
                "label": str(label),
                "latitude": latitude,
                "longitude": longitude,
                "current_x": float(current_x),
                "current_y": float(current_y),
            }
        )

    return anchors


def load_existing_records(csv_path):
    path = Path(csv_path)

    if not path.is_file() or path.stat().st_size == 0:
        return []

    with path.open(newline="", encoding="utf-8") as stream:
        return list(csv.DictReader(stream))


def save_records(csv_path, records):
    path = Path(csv_path)
    path.parent.mkdir(parents=True, exist_ok=True)

    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(
            stream,
            fieldnames=CSV_FIELDS,
        )
        writer.writeheader()
        writer.writerows(records)


def fit_rigid_transform(records):
    if len(records) < 2:
        raise ValueError(
            "At least two point pairs are required. "
            "Three or more are strongly recommended."
        )

    current = np.array(
        [
            [float(row["current_x"]), float(row["current_y"])]
            for row in records
        ],
        dtype=float,
    )

    target = np.array(
        [
            [float(row["target_x"]), float(row["target_y"])]
            for row in records
        ],
        dtype=float,
    )

    current_centroid = current.mean(axis=0)
    target_centroid = target.mean(axis=0)

    current_centered = current - current_centroid
    target_centered = target - target_centroid

    covariance = current_centered.T @ target_centered
    u_matrix, _, v_transpose = np.linalg.svd(covariance)

    rotation = v_transpose.T @ u_matrix.T

    if np.linalg.det(rotation) < 0:
        v_transpose[-1, :] *= -1
        rotation = v_transpose.T @ u_matrix.T

    translation = (
        target_centroid
        - rotation @ current_centroid
    )

    predicted = (rotation @ current.T).T + translation
    residual_vectors = target - predicted
    residual_distances = np.linalg.norm(
        residual_vectors,
        axis=1,
    )

    rotation_rad = math.atan2(
        rotation[1, 0],
        rotation[0, 0],
    )
    rotation_deg = math.degrees(rotation_rad)

    rms_error = float(
        math.sqrt(
            np.mean(residual_distances ** 2)
        )
    )
    max_error = float(np.max(residual_distances))

    return {
        "rotation": rotation,
        "rotation_rad": rotation_rad,
        "rotation_deg": rotation_deg,
        "translation_x": float(translation[0]),
        "translation_y": float(translation[1]),
        "rms_error": rms_error,
        "max_error": max_error,
        "predicted": predicted,
        "residual_vectors": residual_vectors,
        "residual_distances": residual_distances,
    }


def write_fit_yaml(path, records, fit):
    rotation = fit["rotation"]
    cosine = float(rotation[0, 0])
    sine = float(rotation[1, 0])

    lines = [
        "# Transform direction:",
        "# current QGIS/GPS-derived map coordinates -> target MOLA map coordinates",
        "",
        "transform_type: rigid_2d",
        f"point_count: {len(records)}",
        f"rotation_deg: {fit['rotation_deg']:.12f}",
        f"rotation_rad: {fit['rotation_rad']:.12f}",
        f"translation_x_m: {fit['translation_x']:.12f}",
        f"translation_y_m: {fit['translation_y']:.12f}",
        f"rms_error_m: {fit['rms_error']:.12f}",
        f"max_error_m: {fit['max_error']:.12f}",
        "",
        "homogeneous_matrix:",
        (
            f"  - [{cosine:.12f}, "
            f"{-sine:.12f}, "
            f"{fit['translation_x']:.12f}]"
        ),
        (
            f"  - [{sine:.12f}, "
            f"{cosine:.12f}, "
            f"{fit['translation_y']:.12f}]"
        ),
        "  - [0.0, 0.0, 1.0]",
        "",
        "residuals:",
    ]

    for index, row in enumerate(records):
        residual = fit["residual_vectors"][index]
        distance = fit["residual_distances"][index]

        lines.extend(
            [
                f"  - point_id: {json.dumps(row['point_id'])}",
                f"    error_x_m: {float(residual[0]):.12f}",
                f"    error_y_m: {float(residual[1]):.12f}",
                f"    error_distance_m: {float(distance):.12f}",
            ]
        )

    output_path = Path(path)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_path.write_text(
        "\n".join(lines) + "\n",
        encoding="utf-8",
    )


def print_fit(records, fit):
    print()
    print("=" * 68)
    print("RIGID 2D ALIGNMENT RESULT")
    print("=" * 68)
    print(
        "Transform direction: "
        "current QGIS/GPS position -> target MOLA position"
    )
    print()
    print(
        f"Rotation:      {fit['rotation_deg']:+.9f} degrees"
    )
    print(
        f"Translation X: {fit['translation_x']:+.6f} m"
    )
    print(
        f"Translation Y: {fit['translation_y']:+.6f} m"
    )
    print(
        f"RMS error:     {fit['rms_error']:.6f} m"
    )
    print(
        f"Maximum error: {fit['max_error']:.6f} m"
    )
    print()
    print("Per-point residuals:")

    for index, row in enumerate(records):
        residual = fit["residual_vectors"][index]
        distance = fit["residual_distances"][index]

        print(
            f"  {row['point_id']}: "
            f"dx={residual[0]:+.3f} m, "
            f"dy={residual[1]:+.3f} m, "
            f"distance={distance:.3f} m"
        )


class AlignmentCollector(Node):
    def __init__(
        self,
        anchors,
        existing_records,
        csv_path,
        fit_output_path,
        topic,
    ):
        super().__init__("alignment_transform_collector")

        self.anchors = anchors
        self.records = existing_records
        self.csv_path = csv_path
        self.fit_output_path = fit_output_path
        self.done = False

        if len(self.records) > len(self.anchors):
            raise RuntimeError(
                "CSV has more rows than available anchors."
            )

        for index, record in enumerate(self.records):
            expected_id = self.anchors[index]["point_id"]

            if record["point_id"] != expected_id:
                raise RuntimeError(
                    "Existing CSV order does not match GeoJSON order. "
                    f"Row {index + 1}: expected {expected_id}, "
                    f"found {record['point_id']}"
                )

        self.subscription = self.create_subscription(
            PointStamped,
            topic,
            self.handle_click,
            10,
        )

        self.get_logger().info(
            f"Loaded {len(self.anchors)} enabled anchors"
        )
        self.get_logger().info(
            f"Already recorded: {len(self.records)}"
        )

        self.print_anchor_order()

        if len(self.records) == len(self.anchors):
            self.finish()
        else:
            self.print_next_anchor()

    def print_anchor_order(self):
        print()
        print("CLICK ORDER")
        print("-" * 68)

        for index, anchor in enumerate(self.anchors, start=1):
            status = (
                "DONE"
                if index <= len(self.records)
                else "WAITING"
            )

            print(
                f"{index:02d}. {anchor['point_id']} "
                f"({anchor['label']}) "
                f"current=({anchor['current_x']:.3f}, "
                f"{anchor['current_y']:.3f}) "
                f"[{status}]"
            )

        print("-" * 68)

    def print_next_anchor(self):
        anchor = self.anchors[len(self.records)]

        print()
        print("=" * 68)
        print(
            f"NEXT: click the MOLA-map corner for "
            f"{anchor['point_id']} ({anchor['label']})"
        )
        print(
            f"Current anchor position: "
            f"x={anchor['current_x']:.3f}, "
            f"y={anchor['current_y']:.3f}"
        )
        print("=" * 68)
        print()

    def handle_click(self, message):
        if self.done:
            return

        if len(self.records) >= len(self.anchors):
            self.finish()
            return

        if message.header.frame_id not in {"", "map", "/map"}:
            self.get_logger().warning(
                "Clicked point frame is "
                f"{message.header.frame_id!r}, expected 'map'"
            )

        anchor = self.anchors[len(self.records)]

        row = {
            "point_id": anchor["point_id"],
            "label": anchor["label"],
            "current_x": f"{anchor['current_x']:.12f}",
            "current_y": f"{anchor['current_y']:.12f}",
            "target_x": f"{message.point.x:.12f}",
            "target_y": f"{message.point.y:.12f}",
        }

        self.records.append(row)
        save_records(self.csv_path, self.records)

        self.get_logger().info(
            f"Recorded {anchor['point_id']}: "
            f"current=({anchor['current_x']:.3f}, "
            f"{anchor['current_y']:.3f}) "
            f"target=({message.point.x:.3f}, "
            f"{message.point.y:.3f})"
        )

        if len(self.records) == len(self.anchors):
            self.finish()
        else:
            self.print_next_anchor()

    def finish(self):
        if self.done:
            return

        fit = fit_rigid_transform(self.records)
        write_fit_yaml(
            self.fit_output_path,
            self.records,
            fit,
        )
        print_fit(self.records, fit)

        print()
        print(f"Measurements: {self.csv_path}")
        print(f"Transform fit: {self.fit_output_path}")
        print()

        self.done = True


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--geojson", required=True)
    parser.add_argument("--calibration", required=True)
    parser.add_argument("--output-csv", required=True)
    parser.add_argument("--output-fit", required=True)
    parser.add_argument(
        "--topic",
        default="/clicked_point",
    )

    args, ros_args = parser.parse_known_args()

    local_points, gps_points = (
        simple_gps_util.load_landmark_calibration(
            args.calibration
        )
    )

    calibration = simple_gps_util.calibrate_with_landmarks(
        local_points,
        gps_points,
    )

    anchors = load_anchors(
        args.geojson,
        calibration,
    )

    if len(anchors) < 2:
        raise RuntimeError(
            "At least two enabled anchors are required."
        )

    existing_records = load_existing_records(
        args.output_csv
    )

    rclpy.init(args=ros_args)

    node = AlignmentCollector(
        anchors=anchors,
        existing_records=existing_records,
        csv_path=args.output_csv,
        fit_output_path=args.output_fit,
        topic=args.topic,
    )

    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.25)
    except KeyboardInterrupt:
        print()
        print(
            "Stopped. Existing measurements were preserved."
        )
    finally:
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
