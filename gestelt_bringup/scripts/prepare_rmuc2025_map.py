#!/usr/bin/env python3
"""Prepare the RMUC 2025 PCD map with the Gazebo field as the authority.

The downloaded PCD contains a small number of points outside the physical
field fence.  Gazebo's STL is the physical boundary used by the simulator,
so crop the PCD to the same x/y extents and retain every interior obstacle.
"""

from __future__ import annotations

import argparse
from pathlib import Path
import sys

import numpy as np


SCRIPT_DIR = Path(__file__).resolve().parent
WORKSPACE_DIR = SCRIPT_DIR.parents[1]
BRINGUP_DIR = SCRIPT_DIR.parent

DEFAULT_SOURCE = WORKSPACE_DIR / "RM2025_national.pcd"
DEFAULT_STL = (
    BRINGUP_DIR
    / "simulation"
    / "models"
    / "rmuc2025_field"
    / "meshes"
    / "RMUC2025_national.stl"
)
DEFAULT_OUTPUT = BRINGUP_DIR / "simulation" / "fake_maps" / "rmuc2025.pcd"


def parse_pcd_header(path: Path) -> tuple[dict[str, list[str]], int]:
    header: dict[str, list[str]] = {}
    with path.open("rb") as stream:
        while True:
            raw_line = stream.readline()
            if not raw_line:
                raise ValueError(f"PCD header is incomplete: {path}")
            line = raw_line.decode("ascii").strip()
            if not line or line.startswith("#"):
                continue
            fields = line.split()
            header[fields[0]] = fields[1:]
            if fields[0] == "DATA":
                if fields[1].lower() != "binary":
                    raise ValueError("Only binary PCD input is supported")
                return header, stream.tell()


def pcd_numeric_dtype(
    header: dict[str, list[str]]
) -> tuple[np.dtype, int]:
    names = header["FIELDS"]
    sizes = [int(value) for value in header["SIZE"]]
    types = header["TYPE"]
    counts = [int(value) for value in header["COUNT"]]

    type_codes = {
        ("F", 4): "f4",
        ("F", 8): "f8",
        ("I", 1): "i1",
        ("I", 2): "i2",
        ("I", 4): "i4",
        ("I", 8): "i8",
        ("U", 1): "u1",
        ("U", 2): "u2",
        ("U", 4): "u4",
        ("U", 8): "u8",
    }
    formats = []
    for type_name, size, count in zip(types, sizes, counts):
        try:
            base_type = type_codes[(type_name, size)]
        except KeyError as error:
            raise ValueError(
                f"Unsupported PCD field type {type_name}{size}"
            ) from error
        formats.append(base_type if count == 1 else (base_type, (count,)))

    return np.dtype(
        {
            "names": names,
            "formats": formats,
            "offsets": np.cumsum([0] + [size * count for size, count in
                                        zip(sizes, counts)][:-1]).tolist(),
            "itemsize": sum(size * count for size, count in
                            zip(sizes, counts)),
        }
    ), sum(size * count for size, count in zip(sizes, counts))


def load_pcd_xyz(path: Path) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    header, data_offset = parse_pcd_header(path)
    point_count = int(header["POINTS"][0])
    dtype, point_step = pcd_numeric_dtype(header)

    expected_size = data_offset + point_count * point_step
    if path.stat().st_size < expected_size:
        raise ValueError(
            f"PCD is truncated: expected at least {expected_size} bytes, "
            f"got {path.stat().st_size}"
        )

    points = np.memmap(
        path,
        dtype=dtype,
        mode="r",
        offset=data_offset,
        shape=(point_count,),
    )
    return (
        points["x"].astype(np.float64),
        points["y"].astype(np.float64),
        points["z"].astype(np.float64),
    )


def load_ascii_stl_bounds(path: Path) -> np.ndarray:
    minimum = np.full(3, np.inf, dtype=np.float64)
    maximum = np.full(3, -np.inf, dtype=np.float64)
    vertex_count = 0

    with path.open("r", encoding="ascii", errors="strict") as stream:
        for line in stream:
            fields = line.split()
            if len(fields) != 4 or fields[0] != "vertex":
                continue
            vertex = np.array([float(value) for value in fields[1:]], dtype=float)
            minimum = np.minimum(minimum, vertex)
            maximum = np.maximum(maximum, vertex)
            vertex_count += 1

    if vertex_count < 3 or not np.isfinite(minimum).all():
        raise ValueError(f"No valid vertices found in ASCII STL: {path}")
    return np.vstack((minimum, maximum))


def write_binary_pcd(
    path: Path, points: np.ndarray, translation_xy: tuple[float, float]
) -> None:
    points = points.astype("<f4", copy=False)
    header = (
        "# .PCD v0.7 - Point Cloud Data file format\n"
        "VERSION 0.7\n"
        "FIELDS x y z\n"
        "SIZE 4 4 4\n"
        "TYPE F F F\n"
        "COUNT 1 1 1\n"
        f"WIDTH {len(points)}\n"
        "HEIGHT 1\n"
        "VIEWPOINT 0 0 0 1 0 0 0\n"
        f"POINTS {len(points)}\n"
        "DATA binary\n"
    )
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary_path = path.with_suffix(path.suffix + ".tmp")
    with temporary_path.open("wb") as stream:
        stream.write(header.encode("ascii"))
        points.tofile(stream)
    temporary_path.replace(path)

    print(
        f"Applied Gazebo ENU translation: "
        f"x={translation_xy[0]:+.3f}, y={translation_xy[1]:+.3f}"
    )


def prepare_map(
    source: Path,
    stl: Path,
    output: Path,
    translation_xy: tuple[float, float],
    boundary_tolerance: float,
) -> None:
    x, y, z = load_pcd_xyz(source)
    stl_bounds = load_ascii_stl_bounds(stl)
    x_min, y_min = stl_bounds[0, :2]
    x_max, y_max = stl_bounds[1, :2]

    crop_mask = (
        (x >= x_min - boundary_tolerance)
        & (x <= x_max + boundary_tolerance)
        & (y >= y_min - boundary_tolerance)
        & (y <= y_max + boundary_tolerance)
    )
    x = np.clip(x[crop_mask], x_min, x_max)
    y = np.clip(y[crop_mask], y_min, y_max)
    z = z[crop_mask]

    x += translation_xy[0]
    y += translation_xy[1]
    points = np.column_stack((x, y, z))
    write_binary_pcd(output, points, translation_xy)

    output_x_min = float(points[:, 0].min())
    output_x_max = float(points[:, 0].max())
    output_y_min = float(points[:, 1].min())
    output_y_max = float(points[:, 1].max())
    expected_x_min = x_min + translation_xy[0]
    expected_x_max = x_max + translation_xy[0]
    expected_y_min = y_min + translation_xy[1]
    expected_y_max = y_max + translation_xy[1]

    print(f"Source PCD: {source}")
    print(f"STL boundary: {stl}")
    print(f"Output PCD: {output}")
    print(f"Points: {len(points):,} (removed {int((~crop_mask).sum()):,})")
    print(
        "Output x/y bounds: "
        f"x=[{output_x_min:.6f}, {output_x_max:.6f}], "
        f"y=[{output_y_min:.6f}, {output_y_max:.6f}]"
    )
    print(
        "STL x/y size: "
        f"{x_max - x_min:.6f} x {y_max - y_min:.6f} m"
    )

    tolerance = 2.0 * boundary_tolerance
    for actual, expected in (
        (output_x_min, expected_x_min),
        (output_x_max, expected_x_max),
        (output_y_min, expected_y_min),
        (output_y_max, expected_y_max),
    ):
        if abs(actual - expected) > tolerance:
            raise RuntimeError(
                f"Prepared PCD boundary mismatch: {actual} != {expected}"
            )


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Crop and translate the RMUC 2025 navigation PCD map."
    )
    parser.add_argument("--source", type=Path, default=DEFAULT_SOURCE)
    parser.add_argument("--stl", type=Path, default=DEFAULT_STL)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    parser.add_argument(
        "--translate-x",
        type=float,
        default=3.0,
        help="Gazebo map translation along x [m]",
    )
    parser.add_argument(
        "--translate-y",
        type=float,
        default=-12.0,
        help="Gazebo map translation along y [m]",
    )
    parser.add_argument(
        "--boundary-tolerance",
        type=float,
        default=1.0e-5,
        help="Tolerance for float32 points sampled on the STL boundary [m]",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    try:
        prepare_map(
            args.source,
            args.stl,
            args.output,
            (args.translate_x, args.translate_y),
            args.boundary_tolerance,
        )
    except (OSError, ValueError, RuntimeError) as error:
        print(f"error: {error}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
