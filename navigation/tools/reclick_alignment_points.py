#!/usr/bin/env python3

import argparse
import csv
from pathlib import Path

import rclpy
from geometry_msgs.msg import PointStamped
from rclpy.node import Node


class ReclickCollector(Node):
    def __init__(self, csv_path, point_ids, topic):
        super().__init__("reclick_alignment_points")

        self.csv_path = Path(csv_path)
        self.point_ids = point_ids
        self.index = 0
        self.done = False

        with self.csv_path.open(
            newline="",
            encoding="utf-8",
        ) as stream:
            self.rows = list(csv.DictReader(stream))
            self.fieldnames = list(self.rows[0].keys())

        available = {row["point_id"] for row in self.rows}

        missing = [
            point_id
            for point_id in self.point_ids
            if point_id not in available
        ]

        if missing:
            raise RuntimeError(
                f"Points missing from CSV: {missing}"
            )

        self.subscription = self.create_subscription(
            PointStamped,
            topic,
            self.handle_click,
            10,
        )

        self.print_next()

    def print_next(self):
        point_id = self.point_ids[self.index]

        row = next(
            row for row in self.rows
            if row["point_id"] == point_id
        )

        print()
        print("=" * 68)
        print(
            f"CLICK THE MOLA CORNER FOR: "
            f"{point_id} ({row['label']})"
        )
        print(
            f"Current anchor: "
            f"x={float(row['current_x']):.3f}, "
            f"y={float(row['current_y']):.3f}"
        )
        print("=" * 68)
        print()

    def save(self):
        with self.csv_path.open(
            "w",
            newline="",
            encoding="utf-8",
        ) as stream:
            writer = csv.DictWriter(
                stream,
                fieldnames=self.fieldnames,
            )
            writer.writeheader()
            writer.writerows(self.rows)

    def handle_click(self, message):
        point_id = self.point_ids[self.index]

        row = next(
            row for row in self.rows
            if row["point_id"] == point_id
        )

        old_x = row["target_x"]
        old_y = row["target_y"]

        row["target_x"] = f"{message.point.x:.12f}"
        row["target_y"] = f"{message.point.y:.12f}"

        self.save()

        print(
            f"Updated {point_id}: "
            f"old=({float(old_x):.3f}, {float(old_y):.3f}) "
            f"new=({message.point.x:.3f}, "
            f"{message.point.y:.3f})"
        )

        self.index += 1

        if self.index >= len(self.point_ids):
            self.done = True
            print()
            print("Remeasurement complete.")
        else:
            self.print_next()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--csv", required=True)
    parser.add_argument(
        "--points",
        nargs="+",
        required=True,
    )
    parser.add_argument(
        "--topic",
        default="/clicked_point",
    )

    args, ros_args = parser.parse_known_args()

    rclpy.init(args=ros_args)

    node = ReclickCollector(
        csv_path=args.csv,
        point_ids=args.points,
        topic=args.topic,
    )

    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.25)
    except KeyboardInterrupt:
        print("Stopped. Completed measurements were saved.")
    finally:
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
