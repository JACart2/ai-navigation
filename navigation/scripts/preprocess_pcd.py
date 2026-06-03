#!/usr/bin/env python3
"""Thin an ASCII PCD file by keeping every Nth point.

This is intentionally simple and streaming-friendly so it can handle very large
maps without loading them into memory.
"""

from __future__ import annotations

import argparse
from pathlib import Path


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("input", type=Path, help="Input ASCII .pcd file")
    parser.add_argument("output", type=Path, help="Output ASCII .pcd file")
    parser.add_argument(
        "--stride",
        type=int,
        default=8,
        help="Keep every Nth point from the DATA section (default: 8)",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if args.stride < 1:
        raise SystemExit("--stride must be >= 1")

    lines = args.input.read_text().splitlines()

    header: list[str] = []
    data_idx = None
    for idx, line in enumerate(lines):
        header.append(line)
        if line.strip().upper() == "DATA ascii".upper():
            data_idx = idx
            break

    if data_idx is None:
        raise SystemExit("Only ASCII PCD files with a DATA ascii header are supported")

    data_lines = lines[data_idx + 1 :]
    kept = data_lines[:: args.stride]

    updated_header: list[str] = []
    for line in header:
        if line.startswith("WIDTH "):
            updated_header.append(f"WIDTH {len(kept)}")
        elif line.startswith("POINTS "):
            updated_header.append(f"POINTS {len(kept)}")
        else:
            updated_header.append(line)

    args.output.write_text("\n".join(updated_header + kept) + "\n")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
