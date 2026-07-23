#!/usr/bin/env python3

import argparse
import math
import sys
from pathlib import Path

import yaml


COLLECTION_KEYS = {
    "local_points",
    "map_points",
    "ros_points",
    "cartesian_points",
    "local_landmarks",
}

POINT_KEYS = {
    "local",
    "local_point",
    "map",
    "map_point",
    "ros",
    "ros_point",
    "xy",
    "cartesian",
}

SCALAR_PAIRS = (
    ("local_x", "local_y"),
    ("map_x", "map_y"),
    ("ros_x", "ros_y"),
)


def is_number(value):
    return isinstance(value, (int, float)) and not isinstance(value, bool)


def transform_xy(x, y, cosine, sine, tx, ty):
    return (
        cosine * x - sine * y + tx,
        sine * x + cosine * y + ty,
    )


def transform_point(value, cosine, sine, tx, ty):
    if (
        isinstance(value, list)
        and len(value) >= 2
        and is_number(value[0])
        and is_number(value[1])
    ):
        value[0], value[1] = transform_xy(
            float(value[0]),
            float(value[1]),
            cosine,
            sine,
            tx,
            ty,
        )
        return True

    if (
        isinstance(value, dict)
        and is_number(value.get("x"))
        and is_number(value.get("y"))
    ):
        value["x"], value["y"] = transform_xy(
            float(value["x"]),
            float(value["y"]),
            cosine,
            sine,
            tx,
            ty,
        )
        return True

    return False


def transform_structure(obj, cosine, sine, tx, ty):
    transformed_count = 0

    if isinstance(obj, dict):
        handled_scalar_keys = set()

        for x_key, y_key in SCALAR_PAIRS:
            if (
                is_number(obj.get(x_key))
                and is_number(obj.get(y_key))
            ):
                obj[x_key], obj[y_key] = transform_xy(
                    float(obj[x_key]),
                    float(obj[y_key]),
                    cosine,
                    sine,
                    tx,
                    ty,
                )

                transformed_count += 1
                handled_scalar_keys.update({x_key, y_key})

        for key, value in list(obj.items()):
            if key in handled_scalar_keys:
                continue

            normalized_key = str(key).strip().lower()

            if normalized_key in COLLECTION_KEYS and isinstance(value, list):
                for point in value:
                    if transform_point(
                        point,
                        cosine,
                        sine,
                        tx,
                        ty,
                    ):
                        transformed_count += 1
                    else:
                        transformed_count += transform_structure(
                            point,
                            cosine,
                            sine,
                            tx,
                            ty,
                        )

            elif normalized_key in POINT_KEYS:
                if transform_point(
                    value,
                    cosine,
                    sine,
                    tx,
                    ty,
                ):
                    transformed_count += 1
                else:
                    transformed_count += transform_structure(
                        value,
                        cosine,
                        sine,
                        tx,
                        ty,
                    )

            else:
                transformed_count += transform_structure(
                    value,
                    cosine,
                    sine,
                    tx,
                    ty,
                )

    elif isinstance(obj, list):
        for item in obj:
            transformed_count += transform_structure(
                item,
                cosine,
                sine,
                tx,
                ty,
            )

    return transformed_count


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--input", required=True)
    parser.add_argument("--fit", required=True)
    parser.add_argument("--output", required=True)
    args = parser.parse_args()

    input_path = Path(args.input)
    fit_path = Path(args.fit)
    output_path = Path(args.output)

    calibration_data = yaml.safe_load(
        input_path.read_text(encoding="utf-8")
    )
    fit_data = yaml.safe_load(
        fit_path.read_text(encoding="utf-8")
    )

    rotation_deg = float(fit_data["rotation_deg"])
    translation_x = float(fit_data["translation_x_m"])
    translation_y = float(fit_data["translation_y_m"])

    rotation_rad = math.radians(rotation_deg)
    cosine = math.cos(rotation_rad)
    sine = math.sin(rotation_rad)

    transformed_count = transform_structure(
        calibration_data,
        cosine,
        sine,
        translation_x,
        translation_y,
    )

    if transformed_count == 0:
        top_level_keys = (
            list(calibration_data)
            if isinstance(calibration_data, dict)
            else []
        )

        raise SystemExit(
            "No local calibration coordinates were recognized. "
            f"Top-level keys: {top_level_keys}"
        )

    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_path.write_text(
        yaml.safe_dump(
            calibration_data,
            sort_keys=False,
        ),
        encoding="utf-8",
    )

    print("=== CALIBRATION TRANSFORM COMPLETE ===")
    print(f"Input:              {input_path}")
    print(f"Fit:                {fit_path}")
    print(f"Output:             {output_path}")
    print(f"Coordinates changed: {transformed_count}")
    print(f"Rotation:           {rotation_deg:+.9f} degrees")
    print(f"Translation X:      {translation_x:+.6f} m")
    print(f"Translation Y:      {translation_y:+.6f} m")

    navigation_python = Path(__file__).resolve().parents[1] / "navigation"
    sys.path.insert(0, str(navigation_python))

    import simple_gps_util

    local_points, gps_points = (
        simple_gps_util.load_landmark_calibration(
            str(output_path)
        )
    )

    print()
    print("Validation passed:")
    print(f"  Local landmarks: {len(local_points)}")
    print(f"  GPS landmarks:   {len(gps_points)}")


if __name__ == "__main__":
    main()
