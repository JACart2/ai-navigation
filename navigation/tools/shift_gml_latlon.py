#!/usr/bin/env python3

import argparse
import math
import re
from pathlib import Path


EARTH_RADIUS_M = 6_378_137.0

LAT_PATTERN = re.compile(
    r"^(?P<indent>\s*)lat\s+(?P<value>[-+0-9.eE]+)\s*$"
)

LON_PATTERN = re.compile(
    r"^(?P<indent>\s*)lon\s+(?P<value>[-+0-9.eE]+)\s*$"
)


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Translate every lat/lon node in a GML file."
    )

    parser.add_argument("--input", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--east-m", required=True, type=float)
    parser.add_argument("--north-m", required=True, type=float)

    args = parser.parse_args()

    input_path = args.input.resolve()
    output_path = args.output.resolve()

    if not input_path.exists():
        raise FileNotFoundError(input_path)

    if input_path == output_path:
        raise ValueError("Input and output files must be different.")

    lines = input_path.read_text(encoding="utf-8").splitlines()

    latitudes = []

    for line in lines:
        match = LAT_PATTERN.match(line)

        if match:
            latitudes.append(float(match.group("value")))

    if not latitudes:
        raise ValueError("No latitude fields were found.")

    mean_latitude = sum(latitudes) / len(latitudes)
    mean_latitude_rad = math.radians(mean_latitude)

    latitude_delta = math.degrees(
        args.north_m / EARTH_RADIUS_M
    )

    longitude_delta = math.degrees(
        args.east_m
        / (
            EARTH_RADIUS_M
            * math.cos(mean_latitude_rad)
        )
    )

    shifted_lines = []
    latitude_count = 0
    longitude_count = 0

    for line in lines:
        latitude_match = LAT_PATTERN.match(line)

        if latitude_match:
            old_value = float(latitude_match.group("value"))
            new_value = old_value + latitude_delta

            shifted_lines.append(
                f"{latitude_match.group('indent')}lat "
                f"{new_value:.14f}"
            )

            latitude_count += 1
            continue

        longitude_match = LON_PATTERN.match(line)

        if longitude_match:
            old_value = float(longitude_match.group("value"))
            new_value = old_value + longitude_delta

            shifted_lines.append(
                f"{longitude_match.group('indent')}lon "
                f"{new_value:.14f}"
            )

            longitude_count += 1
            continue

        shifted_lines.append(line)

    if latitude_count != longitude_count:
        raise ValueError(
            "Latitude and longitude counts do not match: "
            f"{latitude_count} lat versus {longitude_count} lon"
        )

    output_path.parent.mkdir(parents=True, exist_ok=True)

    output_path.write_text(
        "\n".join(shifted_lines) + "\n",
        encoding="utf-8",
    )

    print("=== GML translation complete ===")
    print(f"Input: {input_path}")
    print(f"Output: {output_path}")
    print(f"Nodes shifted: {latitude_count}")
    print(f"Mean latitude: {mean_latitude:.10f}")
    print(f"East shift: {args.east_m:.3f} m")
    print(f"North shift: {args.north_m:.3f} m")
    print(f"Latitude delta: {latitude_delta:.12f} degrees")
    print(f"Longitude delta: {longitude_delta:.12f} degrees")


if __name__ == "__main__":
    main()
