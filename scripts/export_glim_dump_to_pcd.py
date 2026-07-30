#!/usr/bin/env python3
"""Export a GLIM dump directory to a binary PCD map.

This is a fallback for checkouts where the expected ``glim_dump_to_pcd`` ROS
executable is not installed. It reads GLIM submap compact point bins and
composes each submap's ``T_world_origin``.

[P2 FIX 2026-07-10] Frame correctness: GLIM's graph lives in its own WORLD
frame, related to the Atlas local-ENU frame by the ``T_world_utm`` alignment
the gnss_global module fits and saves into the dump. GICP localization
consumes Atlas ENU poses (odom-init seed, GT snap, cross-check) DIRECTLY as
map-frame poses, so the map it loads must be genuinely ENU. This exporter now
defaults to ``--frame enu``: it loads ``<dump>/T_world_utm.txt`` and applies
its inverse, so the written PCD is in the Atlas local-ENU frame. When
exporting an ENU map, leave GICP's ``localization/utm_transform_path`` EMPTY —
the old world-frame workflow only worked to the extent T_world_utm happened
to be near identity.
"""

from __future__ import annotations

import argparse
import datetime
import math
import re
import shutil
import sys
from pathlib import Path
from typing import Iterable, Optional

import numpy as np


# [P2 FIX 2026-07-14] ENU datum single source of truth. prep_bag writes the
# resolved origin into the dump dir as enu_origin.txt; the exporter reads it as
# the default and CANONICALIZES it so the manifest carries a fixed-precision
# form (GICP then compares numerically with tolerance, not by raw string). This
# kills both the false-accept on a typo'd free-text origin and the false-reject
# on whitespace/precision differences.
ENU_ORIGIN_FILENAME = "enu_origin.txt"
WGS84_A_M = 6378137.0
WGS84_F = 1.0 / 298.257223563


def parse_enu_origin(text: str) -> tuple[float, float, float]:
    """Parse 'lat,lon,alt' (comma- or whitespace-separated) into floats."""
    parts = [p for p in re.split(r"[\s,]+", text.strip()) if p]
    if len(parts) != 3:
        raise ValueError(f"expected 'lat_deg,lon_deg,alt_m', got {text!r}")
    lat, lon, alt = (float(p) for p in parts)
    if not all(math.isfinite(v) for v in (lat, lon, alt)):
        raise ValueError(f"non-finite ENU origin: {text!r}")
    if not (-90.0 <= lat <= 90.0) or not (-180.0 <= lon <= 180.0):
        raise ValueError(f"ENU origin out of range (lat/lon): {text!r}")
    return lat, lon, alt


def canonical_enu_origin(text: str) -> str:
    """Fixed-precision canonical form written to the manifest."""
    lat, lon, alt = parse_enu_origin(text)
    return f"{lat:.8f},{lon:.8f},{alt:.3f}"


def lla_to_ecef(origin: tuple[float, float, float]) -> np.ndarray:
    """Convert WGS84 latitude/longitude/altitude to ECEF metres."""
    lat_deg, lon_deg, alt_m = origin
    lat = math.radians(lat_deg)
    lon = math.radians(lon_deg)
    e2 = WGS84_F * (2.0 - WGS84_F)
    sin_lat = math.sin(lat)
    cos_lat = math.cos(lat)
    sin_lon = math.sin(lon)
    cos_lon = math.cos(lon)
    radius = WGS84_A_M / math.sqrt(1.0 - e2 * sin_lat * sin_lat)
    return np.asarray(
        [
            (radius + alt_m) * cos_lat * cos_lon,
            (radius + alt_m) * cos_lat * sin_lon,
            (radius * (1.0 - e2) + alt_m) * sin_lat,
        ],
        dtype=np.float64,
    )


def ecef_R_enu(origin: tuple[float, float, float]) -> np.ndarray:
    """Return the rotation that maps local ENU vectors into ECEF."""
    lat = math.radians(origin[0])
    lon = math.radians(origin[1])
    sin_lat = math.sin(lat)
    cos_lat = math.cos(lat)
    sin_lon = math.sin(lon)
    cos_lon = math.cos(lon)
    return np.asarray(
        [
            [-sin_lon, -sin_lat * cos_lon, cos_lat * cos_lon],
            [cos_lon, -sin_lat * sin_lon, cos_lat * sin_lon],
            [0.0, cos_lat, sin_lat],
        ],
        dtype=np.float64,
    )


def enu_reanchor_transform(
    input_origin: tuple[float, float, float],
    output_origin: tuple[float, float, float],
) -> np.ndarray:
    """Return exact WGS84 ``T_output_enu_input_enu``.

    This is a datum conversion, not a fitted alignment: the rotation and
    translation are determined entirely by the two declared LLA origins.
    """
    input_ecef = lla_to_ecef(input_origin)
    output_ecef = lla_to_ecef(output_origin)
    R_ecef_input = ecef_R_enu(input_origin)
    R_ecef_output = ecef_R_enu(output_origin)

    transform = np.eye(4, dtype=np.float64)
    transform[:3, :3] = R_ecef_output.T @ R_ecef_input
    transform[:3, 3] = R_ecef_output.T @ (input_ecef - output_ecef)
    return transform


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


def load_world_utm(path: Path) -> np.ndarray:
    """Parse T_world_utm.txt (gnss_global's dump format) and validate it."""
    lines = path.read_text(encoding="utf-8", errors="replace").splitlines()
    mat = parse_matrix(lines, "T_world_utm")
    if not np.all(np.isfinite(mat)):
        raise ValueError(f"{path}: non-finite entries")
    if not np.allclose(mat[3], [0.0, 0.0, 0.0, 1.0], atol=1e-9):
        raise ValueError(f"{path}: bottom row is not [0 0 0 1]")
    R = mat[:3, :3]
    if not np.allclose(R @ R.T, np.eye(3), atol=1e-6):
        raise ValueError(f"{path}: rotation block is not orthonormal")
    return mat


def invert_se3(mat: np.ndarray) -> np.ndarray:
    out = np.eye(4)
    out[:3, :3] = mat[:3, :3].T
    out[:3, 3] = -mat[:3, :3].T @ mat[:3, 3]
    return out


def submap_dirs(dump_dir: Path) -> list[Path]:
    dirs = []
    for path in dump_dir.iterdir():
        if path.is_dir() and re.fullmatch(r"\d+", path.name):
            if (path / "data.txt").is_file() and (path / "points_compact.bin").is_file():
                dirs.append(path)
    return sorted(dirs, key=lambda p: int(p.name))


def parse_submap_range(text: str) -> tuple[int, int]:
    """Parse a half-open ``START:END`` submap-index range."""
    match = re.fullmatch(r"\s*(\d+)\s*:\s*(\d+)\s*", text)
    if match is None:
        raise ValueError(f"expected START:END, got {text!r}")
    start, end = (int(value) for value in match.groups())
    if end <= start:
        raise ValueError(
            f"submap range must be non-empty and half-open, got [{start}, {end})"
        )
    return start, end


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
    pcd_fields: str,
    pre_transform: Optional[np.ndarray] = None,
) -> Iterable[np.ndarray]:
    seen: Optional[set[tuple[int, int, int]]] = set() if voxel_size > 0.0 else None
    for idx, path in enumerate(dirs, start=1):
        lines = (path / "data.txt").read_text(encoding="utf-8", errors="replace").splitlines()
        transform = parse_matrix(lines, "T_world_origin")
        if pre_transform is not None:
            # [P2 FIX 2026-07-10] Compose ONCE per submap: output frame =
            # pre_transform (T_utm_world) applied on top of T_world_origin,
            # i.e. points land in the Atlas local-ENU frame.
            transform = pre_transform @ transform
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

        dtype = [("x", "<f4"), ("y", "<f4"), ("z", "<f4")]
        if pcd_fields == "xyzi":
            dtype.append(("intensity", "<f4"))
        out = np.empty(world.shape[0], dtype=dtype)
        out["x"] = world[:, 0].astype(np.float32)
        out["y"] = world[:, 1].astype(np.float32)
        out["z"] = world[:, 2].astype(np.float32)
        if pcd_fields == "xyzi":
            out["intensity"] = intensities.astype(np.float32)
        print(f"[export_glim_dump_to_pcd] {idx}: {path.name} -> {len(out)} points", flush=True)
        yield out


def write_header(handle, count: int, pcd_fields: str) -> None:
    if pcd_fields == "xyz":
        fields = "x y z"
        scalar_columns = "4 4 4"
        scalar_types = "F F F"
        scalar_counts = "1 1 1"
    else:
        fields = "x y z intensity"
        scalar_columns = "4 4 4 4"
        scalar_types = "F F F F"
        scalar_counts = "1 1 1 1"
    header = (
        "# .PCD v0.7 - Point Cloud Data file format\n"
        "VERSION 0.7\n"
        f"FIELDS {fields}\n"
        f"SIZE {scalar_columns}\n"
        f"TYPE {scalar_types}\n"
        f"COUNT {scalar_counts}\n"
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
    parser.add_argument(
        "--pcd-fields",
        choices=("xyz", "xyzi"),
        default="xyzi",
        help="PCD payload fields. Use 'xyz' for perception-ws-compatible deployment maps; "
        "'xyzi' preserves the historical dense-map output.",
    )
    parser.add_argument(
        "--submap-range",
        type=str,
        default="",
        metavar="START:END",
        help="Export only the half-open numeric submap range [START, END). Use this to "
        "export representative complete laps independently instead of accumulating "
        "an entire multi-lap session.",
    )
    parser.add_argument(
        "--submap-step",
        type=int,
        default=1,
        help="After --submap-range selection, export every Nth submap. This is useful "
        "for low-speed/stationary staging coverage where adjacent 10 Hz submaps are "
        "almost identical. Default 1 keeps every selected submap.",
    )
    parser.add_argument(
        "--frame",
        choices=("enu", "world"),
        default="enu",
        help="Output frame. 'enu' (default) applies inverse T_world_utm from the dump so the "
        "PCD matches the Atlas local-ENU poses GICP consumes directly; 'world' writes GLIM's "
        "raw world frame (legacy behavior — GICP GT/seed poses will be frame-mismatched "
        "unless T_world_utm happens to be identity).",
    )
    parser.add_argument(
        "--enu-origin",
        type=str,
        default="",
        help="OUTPUT map datum 'lat_deg,lon_deg,alt_m'. REQUIRED for --frame enu and recorded "
        "in the map manifest. If this differs from the GNSS coordinates used during mapping, "
        "also pass --gnss-enu-origin so the exporter performs an exact WGS84 ENU reanchor.",
    )
    parser.add_argument(
        "--gnss-enu-origin",
        type=str,
        default="",
        help="INPUT datum of the GNSS coordinates consumed by gnss_global. Defaults to "
        "--enu-origin for backward compatibility. When the trusted localization map uses "
        "another datum, this source origin plus --enu-origin determines the exact, non-fitted "
        "T_output_enu_input_enu conversion.",
    )
    parser.add_argument(
        "--allow-missing-origin",
        action="store_true",
        help="Escape hatch for legacy dumps whose origin is unknown: export without recording "
        "a datum (the manifest will carry an explicit UNSPECIFIED warning).",
    )
    parser.add_argument(
        "--transform-file",
        type=Path,
        default=None,
        help="Override path to T_world_utm.txt (default: <dump_dir>/T_world_utm.txt)",
    )
    args = parser.parse_args()

    if args.voxel_size < 0.0 or not math.isfinite(args.voxel_size):
        parser.error("--voxel-size must be finite and >= 0")

    # [P2 FIX 2026-07-14] Single source of truth: default the ENU origin from the
    # datum prep_bag recorded in the dump (<dump_dir>/enu_origin.txt) when the
    # operator did not pass one explicitly. Then parse/canonicalize whatever we
    # have so the manifest carries a validated fixed-precision datum.
    if args.frame == "enu" and not args.enu_origin:
        origin_file = args.dump_dir / ENU_ORIGIN_FILENAME
        if origin_file.is_file():
            args.enu_origin = origin_file.read_text(encoding="utf-8").strip()
            print(f"[export_glim_dump_to_pcd] ENU origin read from {origin_file}: {args.enu_origin}",
                  flush=True)
    if args.enu_origin:
        try:
            args.enu_origin = canonical_enu_origin(args.enu_origin)
        except ValueError as exc:
            parser.error(f"--enu-origin invalid: {exc}")
    if args.gnss_enu_origin:
        try:
            args.gnss_enu_origin = canonical_enu_origin(args.gnss_enu_origin)
        except ValueError as exc:
            parser.error(f"--gnss-enu-origin invalid: {exc}")
    elif args.enu_origin:
        args.gnss_enu_origin = args.enu_origin

    # [P3 FIX 2026-07-10] Provenance is ENFORCED, not just recorded: an ENU
    # map without its datum cannot be validated against the live adapter.
    if args.frame == "enu" and not args.enu_origin and not args.allow_missing_origin:
        parser.error(
            "--frame enu requires --enu-origin 'lat_deg,lon_deg,alt_m' (the adapter/prep_bag "
            "datum for this dataset). Use --allow-missing-origin ONLY for legacy dumps whose "
            "origin is unrecoverable."
        )
    if args.stride < 1:
        parser.error("--stride must be >= 1")
    if args.submap_step < 1:
        parser.error("--submap-step must be >= 1")

    dirs = submap_dirs(args.dump_dir)
    if not dirs:
        raise SystemExit(f"no GLIM submap dirs found under {args.dump_dir}")
    selected_submap_range: Optional[tuple[int, int]] = None
    if args.submap_range:
        try:
            selected_submap_range = parse_submap_range(args.submap_range)
        except ValueError as exc:
            parser.error(f"--submap-range invalid: {exc}")
        start, end = selected_submap_range
        dirs = [path for path in dirs if start <= int(path.name) < end]
        if not dirs:
            raise SystemExit(
                f"--submap-range [{start}, {end}) selected no valid submaps "
                f"under {args.dump_dir}"
            )
        print(
            f"[export_glim_dump_to_pcd] selected {len(dirs)} submaps in "
            f"half-open range [{start}, {end})",
            flush=True,
        )
    if args.submap_step > 1:
        dirs = dirs[:: args.submap_step]
        print(
            f"[export_glim_dump_to_pcd] submap step {args.submap_step}: "
            f"retained {len(dirs)} submaps",
            flush=True,
        )

    pre_transform: Optional[np.ndarray] = None
    enu_reanchor = np.eye(4, dtype=np.float64)
    if args.frame == "enu":
        tf_path = args.transform_file or (args.dump_dir / "T_world_utm.txt")
        if not tf_path.is_file():
            # Fail CLOSED: a silently world-framed "ENU" map poisons every
            # GT-anchored mechanism in GICP. A GNSS-less dump must be exported
            # with an explicit --frame world.
            raise SystemExit(
                f"--frame enu but {tf_path} does not exist (was the GNSS module enabled for this "
                "mapping run?). Re-run with --frame world ONLY if the map is genuinely meant to "
                "stay in GLIM's world frame."
            )
        T_world_utm = load_world_utm(tf_path)
        if not args.gnss_enu_origin:
            raise SystemExit(
                "--frame enu needs the GNSS input datum. Pass --gnss-enu-origin, "
                "or pass --enu-origin when input and output use the same datum."
            )
        input_origin = parse_enu_origin(args.gnss_enu_origin)
        output_origin = parse_enu_origin(args.enu_origin)
        enu_reanchor = enu_reanchor_transform(input_origin, output_origin)
        pre_transform = enu_reanchor @ invert_se3(T_world_utm)
        yaw_deg = math.degrees(math.atan2(T_world_utm[1, 0], T_world_utm[0, 0]))
        print(
            f"[export_glim_dump_to_pcd] frame=enu: applying inverse T_world_utm from {tf_path} "
            f"(world-utm offset: t=[{T_world_utm[0,3]:.2f}, {T_world_utm[1,3]:.2f}, "
            f"{T_world_utm[2,3]:.2f}] m, yaw={yaw_deg:.2f} deg). Leave GICP's "
            "localization/utm_transform_path EMPTY for this map.",
            flush=True,
        )
        if args.gnss_enu_origin != args.enu_origin:
            t = enu_reanchor[:3, 3]
            reanchor_yaw_deg = math.degrees(
                math.atan2(enu_reanchor[1, 0], enu_reanchor[0, 0]))
            print(
                "[export_glim_dump_to_pcd] datum reanchor: "
                f"input={args.gnss_enu_origin} output={args.enu_origin} "
                f"t=[{t[0]:.3f}, {t[1]:.3f}, {t[2]:.3f}] m "
                f"yaw={reanchor_yaw_deg:.6f} deg",
                flush=True,
            )
    else:
        print(
            "[export_glim_dump_to_pcd] frame=world (legacy): PCD stays in GLIM's world frame — "
            "GICP's Atlas ENU seeds/GT will be frame-mismatched unless T_world_utm ~= identity.",
            flush=True,
        )

    args.output_pcd.parent.mkdir(parents=True, exist_ok=True)
    tmp = args.output_pcd.with_suffix(args.output_pcd.suffix + ".tmp")
    data_tmp = args.output_pcd.with_suffix(args.output_pcd.suffix + ".data.tmp")

    # The PCD binary header must carry the final POINTS count, which is only
    # known after the whole dump is processed (voxel dedup / stride change it).
    # Rather than buffer every transformed chunk in RAM, stream each chunk to a
    # temp binary file as it is produced (one submap chunk resident at a time,
    # plus the voxel-dedup set when --voxel-size > 0), then prepend the header
    # and copy the data back out. Byte-identical output, bounded peak memory.
    total = 0
    success = False
    try:
        with data_tmp.open("wb") as data_handle:
            for chunk in transformed_chunks(
                dirs,
                args.voxel_size,
                args.stride,
                args.pcd_fields,
                pre_transform,
            ):
                chunk.tofile(data_handle)
                total += len(chunk)

        if total == 0:
            raise SystemExit("export produced zero points")

        with tmp.open("wb") as handle:
            write_header(handle, total, args.pcd_fields)
            with data_tmp.open("rb") as data_handle:
                shutil.copyfileobj(data_handle, handle, length=8 * 1024 * 1024)
        # [SELF-AUDIT FIX 2026-07-10] Manifest FIRST, then finalize the PCD:
        # provenance is mandatory now, so a manifest-write failure must not
        # leave a finished but unmanifested map behind.
        # [P2 FIX 2026-07-10b] Map manifest: record the frame and transform the
        # PCD was exported with, so the mapping->localization handoff is
        # auditable (reviewer requirement: the ENU conversion must be a
        # GUARANTEED pipeline step, not an operator convention).
        manifest = args.output_pcd.with_suffix(args.output_pcd.suffix + ".manifest.yaml")
        with manifest.open("w", encoding="utf-8") as mh:
            mh.write("# Map provenance manifest (written by export_glim_dump_to_pcd.py)\n")
            mh.write(f"exported_utc: {datetime.datetime.now(datetime.timezone.utc).isoformat()}\n")
            mh.write(f"source_dump: {args.dump_dir.resolve()}\n")
            mh.write(f"frame: {args.frame}\n")
            mh.write(f"points: {total}\n")
            mh.write(f"voxel_size: {args.voxel_size}\n")
            mh.write(f"stride: {args.stride}\n")
            mh.write(f"submap_step: {args.submap_step}\n")
            mh.write(f"pcd_fields: {args.pcd_fields}\n")
            if selected_submap_range is None:
                mh.write("submap_range: all\n")
            else:
                start, end = selected_submap_range
                mh.write(f"submap_range: \"{start}:{end}\"  # half-open [start, end)\n")
                mh.write(f"submap_start: {start}\n")
                mh.write(f"submap_end_exclusive: {end}\n")
            mh.write(f"selected_submaps: {len(dirs)}\n")
            if args.enu_origin:
                mh.write(f"enu_origin: {args.enu_origin}  # output map datum\n")
                mh.write(f"gnss_enu_origin: {args.gnss_enu_origin}  # mapping input datum\n")
            else:
                mh.write("enu_origin: UNSPECIFIED  # WARNING: record the adapter's\n")
                mh.write("#   local_enu_origin for this dataset — a live adapter with a\n")
                mh.write("#   different datum is silently incompatible with this map\n")
            if pre_transform is not None:
                mh.write(
                    "applied_transform: T_output_enu_input_enu * inverse(T_world_utm)\n")
                mh.write("T_world_utm:\n")
                for row in T_world_utm:
                    mh.write("  - [" + ", ".join(f"{v:.10f}" for v in row) + "]\n")
                mh.write("T_output_enu_input_enu:\n")
                for row in enu_reanchor:
                    mh.write("  - [" + ", ".join(f"{v:.10f}" for v in row) + "]\n")
                mh.write("gicp_note: leave localization/utm_transform_path EMPTY for this map\n")
            else:
                mh.write("applied_transform: none  # PCD is GLIM WORLD frame — NOT directly\n")
                mh.write("#   compatible with Atlas ENU seeds/GT in gicp localization\n")
        print(f"[export_glim_dump_to_pcd] manifest: {manifest}")
        tmp.replace(args.output_pcd)
        success = True
    finally:
        data_tmp.unlink(missing_ok=True)
        if not success:
            tmp.unlink(missing_ok=True)

    print(f"[export_glim_dump_to_pcd] wrote {total} points to {args.output_pcd}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
