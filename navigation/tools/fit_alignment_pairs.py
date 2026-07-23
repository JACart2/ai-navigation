#!/usr/bin/env python3

import argparse
import csv
import math
from pathlib import Path

import numpy as np


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--pairs", required=True)
    parser.add_argument("--output", required=True)
    args = parser.parse_args()

    pairs_path = Path(args.pairs)
    output_path = Path(args.output)

    if not pairs_path.is_file():
        raise SystemExit(f"Pairs CSV not found: {pairs_path}")

    with pairs_path.open(newline="", encoding="utf-8") as stream:
        rows = list(csv.DictReader(stream))

    if len(rows) < 3:
        raise SystemExit(
            f"Need at least 3 point pairs; found {len(rows)}"
        )

    current = np.array(
        [
            [float(row["current_x"]), float(row["current_y"])]
            for row in rows
        ],
        dtype=float,
    )

    target = np.array(
        [
            [float(row["target_x"]), float(row["target_y"])]
            for row in rows
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

    translation = target_centroid - rotation @ current_centroid

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

    rms_error = math.sqrt(
        float(np.mean(residual_distances ** 2))
    )
    max_error = float(np.max(residual_distances))

    cosine = float(rotation[0, 0])
    sine = float(rotation[1, 0])

    lines = [
        "# Trusted rigid transform",
        "# Direction: QGIS/GPS map coordinates -> MOLA map coordinates",
        "",
        "transform_type: rigid_2d",
        f"point_count: {len(rows)}",
        f"rotation_deg: {rotation_deg:.12f}",
        f"rotation_rad: {rotation_rad:.12f}",
        f"translation_x_m: {float(translation[0]):.12f}",
        f"translation_y_m: {float(translation[1]):.12f}",
        f"rms_error_m: {rms_error:.12f}",
        f"max_error_m: {max_error:.12f}",
        "",
        "homogeneous_matrix:",
        (
            f"  - [{cosine:.12f}, "
            f"{-sine:.12f}, "
            f"{float(translation[0]):.12f}]"
        ),
        (
            f"  - [{sine:.12f}, "
            f"{cosine:.12f}, "
            f"{float(translation[1]):.12f}]"
        ),
        "  - [0.0, 0.0, 1.0]",
        "",
        "residuals:",
    ]

    for index, row in enumerate(rows):
        dx = float(residual_vectors[index, 0])
        dy = float(residual_vectors[index, 1])
        distance = float(residual_distances[index])

        lines.extend(
            [
                f"  - point_id: {row['point_id']!r}",
                f"    error_x_m: {dx:.12f}",
                f"    error_y_m: {dy:.12f}",
                f"    error_distance_m: {distance:.12f}",
            ]
        )

    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_path.write_text(
        "\n".join(lines) + "\n",
        encoding="utf-8",
    )

    print("=== TRUSTED ALIGNMENT FIT ===")
    print(f"Points:        {len(rows)}")
    print(f"Rotation:      {rotation_deg:+.9f} degrees")
    print(f"Translation X: {translation[0]:+.6f} m")
    print(f"Translation Y: {translation[1]:+.6f} m")
    print(f"RMS error:     {rms_error:.6f} m")
    print(f"Maximum error: {max_error:.6f} m")
    print()
    print("Residuals:")

    for index, row in enumerate(rows):
        print(
            f"  {row['point_id']}: "
            f"{residual_distances[index]:.3f} m"
        )

    print()
    print(f"Saved: {output_path}")


if __name__ == "__main__":
    main()
