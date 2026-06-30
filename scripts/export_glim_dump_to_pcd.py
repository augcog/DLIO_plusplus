#!/usr/bin/env python3
"""Export a GLIM dump directory to a binary PCD map.

This is a fallback for checkouts where the expected ``glim_dump_to_pcd`` ROS
executable is not installed. It reads GLIM submap compact point bins and writes
points in the dump's world frame using each submap's ``T_world_origin``.
"""

from __future__ import annotations

import argparse
import math
import re
import sys
from pathlib import Path
from typing import Iterable, Optional

import numpy as np


def parse_matrix(lines: list[str], key: str) -> np.ndarray:
    for idx, line in enumerate(lines):
        if line.strip() == f"{key}:":
            rows = []
            for row in lines[idx + 1 : idx + 5]:
                values = [float(v) for v in row.strip().split()]
                if len(values) != 4:
                    raise ValueError(f"bad {key} matrix row: {row!r}")
                rows.append(values)
            return np.asarray(rows, dtype=np.float64)
    raise ValueError(f"{key} not found")


def submap_dirs(dump_dir: Path) -> list[Path]:
    dirs = []
    for path in dump_dir.iterdir():
        if path.is_dir() and re.fullmatch(r"\d+", path.name):
            if (path / "data.txt").is_file() and (path / "points_compact.bin").is_file():
                dirs.append(path)
    return sorted(dirs, key=lambda p: int(p.name))


def point_count(path: Path) -> int:
    size = path.stat().st_size
    if size % 12 != 0:
        raise ValueError(f"{path} size {size} is not divisible by 12")
    return size // 12


def read_submap(path: Path) -> tuple[np.ndarray, np.ndarray]:
    n = point_count(path / "points_compact.bin")
    points = np.fromfile(path / "points_compact.bin", dtype=np.float32).reshape(n, 3)
    intensity_path = path / "intensities_compact.bin"
    if intensity_path.is_file() and intensity_path.stat().st_size == n * 4:
        intensities = np.fromfile(intensity_path, dtype=np.float32)
    else:
        intensities = np.zeros(n, dtype=np.float32)
    return points, intensities


def transformed_chunks(
    dirs: Iterable[Path],
    voxel_size: float,
    stride: int,
) -> Iterable[np.ndarray]:
    seen: Optional[set[tuple[int, int, int]]] = set() if voxel_size > 0.0 else None
    for idx, path in enumerate(dirs, start=1):
        lines = (path / "data.txt").read_text(encoding="utf-8", errors="replace").splitlines()
        transform = parse_matrix(lines, "T_world_origin")
        points, intensities = read_submap(path)
        if stride > 1:
            points = points[::stride]
            intensities = intensities[::stride]
        if points.size == 0:
            continue

        world = points.astype(np.float64) @ transform[:3, :3].T + transform[:3, 3]
        if seen is not None:
            keys = np.floor(world / voxel_size).astype(np.int64)
            keep_indices = []
            for i, key in enumerate(keys):
                item = (int(key[0]), int(key[1]), int(key[2]))
                if item in seen:
                    continue
                seen.add(item)
                keep_indices.append(i)
            if not keep_indices:
                continue
            keep = np.asarray(keep_indices, dtype=np.int64)
            world = world[keep]
            intensities = intensities[keep]

        out = np.empty(world.shape[0], dtype=[("x", "<f4"), ("y", "<f4"), ("z", "<f4"), ("intensity", "<f4")])
        out["x"] = world[:, 0].astype(np.float32)
        out["y"] = world[:, 1].astype(np.float32)
        out["z"] = world[:, 2].astype(np.float32)
        out["intensity"] = intensities.astype(np.float32)
        print(f"[export_glim_dump_to_pcd] {idx}: {path.name} -> {len(out)} points", flush=True)
        yield out


def write_header(handle, count: int) -> None:
    header = (
        "# .PCD v0.7 - Point Cloud Data file format\n"
        "VERSION 0.7\n"
        "FIELDS x y z intensity\n"
        "SIZE 4 4 4 4\n"
        "TYPE F F F F\n"
        "COUNT 1 1 1 1\n"
        f"WIDTH {count}\n"
        "HEIGHT 1\n"
        "VIEWPOINT 0 0 0 1 0 0 0\n"
        f"POINTS {count}\n"
        "DATA binary\n"
    )
    handle.write(header.encode("ascii"))


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("dump_dir", type=Path)
    parser.add_argument("output_pcd", type=Path)
    parser.add_argument("--voxel-size", type=float, default=0.0)
    parser.add_argument("--stride", type=int, default=1)
    args = parser.parse_args()

    if args.voxel_size < 0.0 or not math.isfinite(args.voxel_size):
        parser.error("--voxel-size must be finite and >= 0")
    if args.stride < 1:
        parser.error("--stride must be >= 1")

    dirs = submap_dirs(args.dump_dir)
    if not dirs:
        raise SystemExit(f"no GLIM submap dirs found under {args.dump_dir}")

    chunks = []
    total = 0
    for chunk in transformed_chunks(dirs, args.voxel_size, args.stride):
        chunks.append(chunk)
        total += len(chunk)
    if total == 0:
        raise SystemExit("export produced zero points")

    args.output_pcd.parent.mkdir(parents=True, exist_ok=True)
    tmp = args.output_pcd.with_suffix(args.output_pcd.suffix + ".tmp")
    with tmp.open("wb") as handle:
        write_header(handle, total)
        for chunk in chunks:
            chunk.tofile(handle)
    tmp.replace(args.output_pcd)
    print(f"[export_glim_dump_to_pcd] wrote {total} points to {args.output_pcd}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
