#!/usr/bin/env python3
"""Build a compact, repeatability-filtered deployment map from session PCDs.

The perception-ws mapping pipeline does not deploy a naive union of all mapped
laps.  It first makes each input globally unique at a fixed voxel size,
then keeps:

* voxels observed by at least ``--min-sessions`` inside the driven corridor;
* the union outside that corridor, where distant static structure may only be
  visible from one lap/session.

Inputs must be binary PCDs in the same local-ENU frame and have provenance
manifests named ``<map>.pcd.manifest.yaml``.  The output is an XYZ-only binary
PCD suitable for the sparse map loader in GICP++.
"""

from __future__ import annotations

import argparse
import datetime
import hashlib
import math
import os
import re
from pathlib import Path
import numpy as np
import yaml
from scipy.spatial import cKDTree


PACK_BITS = 21
PACK_BIAS = 1 << (PACK_BITS - 1)


def parse_index_range(text: str) -> tuple[int, int]:
    match = re.fullmatch(r"\s*(\d+)\s*:\s*(\d+)\s*", text)
    if match is None:
        raise ValueError(f"expected START:END, got {text!r}")
    start, end = (int(value) for value in match.groups())
    if end <= start:
        raise ValueError(f"range must be non-empty and half-open, got [{start}, {end})")
    return start, end


def invert_se3(transform: np.ndarray) -> np.ndarray:
    output = np.eye(4, dtype=np.float64)
    output[:3, :3] = transform[:3, :3].T
    output[:3, 3] = -transform[:3, :3].T @ transform[:3, 3]
    return output


def matrix_from_manifest(manifest: dict, key: str) -> np.ndarray:
    matrix = np.asarray(manifest.get(key), dtype=np.float64)
    if matrix.shape != (4, 4) or not np.all(np.isfinite(matrix)):
        raise ValueError(f"manifest {key} must be a finite 4x4 matrix")
    if not np.allclose(matrix[3], [0.0, 0.0, 0.0, 1.0], atol=1e-8):
        raise ValueError(f"manifest {key} is not a homogeneous transform")
    return matrix


def read_manifest(
    pcd_path: Path, allow_missing_origin: bool = False
) -> tuple[Path, dict]:
    path = pcd_path.with_suffix(pcd_path.suffix + ".manifest.yaml")
    if not path.is_file():
        raise FileNotFoundError(f"required map provenance manifest not found: {path}")
    with path.open("r", encoding="utf-8") as handle:
        manifest = yaml.safe_load(handle)
    if not isinstance(manifest, dict):
        raise ValueError(f"{path}: expected a YAML mapping")
    if manifest.get("frame") != "enu":
        raise ValueError(f"{path}: input map frame must be 'enu'")
    matrix_from_manifest(manifest, "T_world_utm")
    matrix_from_manifest(manifest, "T_output_enu_input_enu")
    origin = str(manifest.get("enu_origin", "")).split("#", 1)[0].strip()
    if not origin or origin.startswith("UNSPECIFIED"):
        if allow_missing_origin:
            return path, manifest
        raise ValueError(f"{path}: input map must declare enu_origin")
    parts = [part for part in re.split(r"[\s,]+", origin) if part]
    if len(parts) != 3:
        raise ValueError(f"{path}: enu_origin must be lat,lon,alt, got {origin!r}")
    try:
        lat, lon, alt = (float(part) for part in parts)
    except ValueError as exc:
        raise ValueError(f"{path}: non-numeric enu_origin {origin!r}") from exc
    if not all(math.isfinite(value) for value in (lat, lon, alt)):
        raise ValueError(f"{path}: non-finite enu_origin {origin!r}")
    if not (-90.0 <= lat <= 90.0 and -180.0 <= lon <= 180.0):
        raise ValueError(f"{path}: enu_origin latitude/longitude out of range")
    return path, manifest


def source_export_summary(pcd_path: Path, manifest_path: Path, manifest: dict) -> dict:
    """Keep dump/range provenance even if a generated slice is later removed."""
    keys = (
        "source_dump",
        "submap_range",
        "submap_start",
        "submap_end_exclusive",
        "submap_step",
        "selected_submaps",
        "points",
        "voxel_size",
        "pcd_fields",
        "frame",
        "enu_origin",
        "gnss_enu_origin",
        "applied_transform",
    )
    summary = {
        "pcd": str(pcd_path.resolve()),
        "manifest": str(manifest_path.resolve()),
    }
    summary.update({key: manifest[key] for key in keys if key in manifest})
    summary.setdefault("submap_step", 1)
    return summary


def pcd_memmap(path: Path) -> tuple[np.memmap, dict]:
    """Open a binary PCD as a structured memory map."""
    header: dict[str, list[str]] = {}
    with path.open("rb") as handle:
        while True:
            raw = handle.readline()
            if not raw:
                raise ValueError(f"{path}: missing DATA line")
            try:
                line = raw.decode("ascii").strip()
            except UnicodeDecodeError as exc:
                raise ValueError(f"{path}: non-ASCII PCD header") from exc
            if not line or line.startswith("#"):
                continue
            parts = line.split()
            key = parts[0].upper()
            header[key] = parts[1:]
            if key == "DATA":
                data_offset = handle.tell()
                break

    if header.get("DATA") != ["binary"]:
        raise ValueError(f"{path}: only DATA binary PCD is supported")
    fields = header.get("FIELDS", [])
    sizes = [int(value) for value in header.get("SIZE", [])]
    types = header.get("TYPE", [])
    counts = [int(value) for value in header.get("COUNT", ["1"] * len(fields))]
    if not (len(fields) == len(sizes) == len(types) == len(counts)):
        raise ValueError(f"{path}: inconsistent FIELDS/SIZE/TYPE/COUNT header")
    if any(count != 1 for count in counts):
        raise ValueError(f"{path}: vector-valued PCD fields are not supported")
    for axis in ("x", "y", "z"):
        if axis not in fields:
            raise ValueError(f"{path}: missing {axis!r} field")
        index = fields.index(axis)
        if sizes[index] != 4 or types[index].upper() != "F":
            raise ValueError(f"{path}: {axis} must be a float32 scalar")

    offsets = np.cumsum([0] + sizes[:-1]).tolist()
    formats = []
    for scalar_type, size in zip(types, sizes):
        code = {
            ("F", 4): "<f4",
            ("F", 8): "<f8",
            ("I", 1): "<i1",
            ("I", 2): "<i2",
            ("I", 4): "<i4",
            ("I", 8): "<i8",
            ("U", 1): "<u1",
            ("U", 2): "<u2",
            ("U", 4): "<u4",
            ("U", 8): "<u8",
        }.get((scalar_type.upper(), size))
        if code is None:
            raise ValueError(f"{path}: unsupported PCD scalar {scalar_type}{size}")
        formats.append(code)
    dtype = np.dtype(
        {
            "names": fields,
            "formats": formats,
            "offsets": offsets,
            "itemsize": sum(sizes),
        }
    )
    try:
        points = int(header["POINTS"][0])
    except (KeyError, IndexError, ValueError) as exc:
        raise ValueError(f"{path}: invalid POINTS header") from exc
    expected_size = data_offset + points * dtype.itemsize
    if path.stat().st_size != expected_size:
        raise ValueError(
            f"{path}: byte size {path.stat().st_size} != header-implied {expected_size}"
        )
    return np.memmap(path, dtype=dtype, mode="r", offset=data_offset, shape=(points,)), header


def xyz_array(points: np.ndarray) -> np.ndarray:
    output = np.empty((len(points), 3), dtype=np.float32)
    output[:, 0] = points["x"]
    output[:, 1] = points["y"]
    output[:, 2] = points["z"]
    return output


def pack_voxels(points: np.ndarray, voxel_size: float) -> np.ndarray:
    voxels = np.floor(points / voxel_size).astype(np.int64)
    if np.any(voxels < -PACK_BIAS) or np.any(voxels >= PACK_BIAS):
        minimum = voxels.min(axis=0).tolist()
        maximum = voxels.max(axis=0).tolist()
        raise ValueError(
            f"voxel coordinates exceed signed {PACK_BITS}-bit packing range: "
            f"min={minimum} max={maximum}"
        )
    unsigned = (voxels + PACK_BIAS).astype(np.uint64)
    return (
        (unsigned[:, 0] << np.uint64(2 * PACK_BITS))
        | (unsigned[:, 1] << np.uint64(PACK_BITS))
        | unsigned[:, 2]
    )


def unique_session(path: Path, voxel_size: float) -> tuple[np.ndarray, np.ndarray, int]:
    mapped, _ = pcd_memmap(path)
    raw_points = len(mapped)
    points = xyz_array(mapped)
    del mapped
    keys = pack_voxels(points, voxel_size)
    unique_keys, first_indices = np.unique(keys, return_index=True)
    unique_points = points[first_indices]
    print(
        f"[consistent_pcd] {path}: raw={raw_points} "
        f"unique_{voxel_size:g}m={len(unique_keys)}",
        flush=True,
    )
    return unique_keys, unique_points, raw_points


def transformed_centerline(
    trajectory_path: Path,
    index_range: tuple[int, int],
    source_manifest: dict,
    sample_stride: int,
) -> np.ndarray:
    trajectory = np.loadtxt(trajectory_path, dtype=np.float64)
    if trajectory.ndim != 2 or trajectory.shape[1] < 4:
        raise ValueError(f"{trajectory_path}: expected stamp x y z ... rows")
    start, end = index_range
    if end > len(trajectory):
        raise ValueError(
            f"centerline range [{start}, {end}) exceeds {len(trajectory)} trajectory rows"
        )
    world_points = trajectory[start:end:sample_stride, 1:4]
    if (end - start - 1) % sample_stride:
        world_points = np.vstack([world_points, trajectory[end - 1, 1:4]])
    T_world_utm = matrix_from_manifest(source_manifest, "T_world_utm")
    T_output_input = matrix_from_manifest(
        source_manifest, "T_output_enu_input_enu"
    )
    T_output_world = T_output_input @ invert_se3(T_world_utm)
    output = world_points @ T_output_world[:3, :3].T + T_output_world[:3, 3]
    return output


def write_xyz_pcd(path: Path, points: np.ndarray) -> None:
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
    with path.open("wb") as handle:
        handle.write(header.encode("ascii"))
        points.astype("<f4", copy=False).tofile(handle)
        handle.flush()
        os.fsync(handle.fileno())


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(8 * 1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("output_pcd", type=Path)
    parser.add_argument("input_pcd", type=Path, nargs="+")
    parser.add_argument(
        "--coverage-pcd",
        type=Path,
        action="append",
        default=[],
        help="Additional preselected staging/pit coverage. These maps are globally "
        "deduplicated into the output but do not count toward cross-session support.",
    )
    parser.add_argument("--voxel-size", type=float, default=0.15)
    parser.add_argument("--min-sessions", type=int, default=2)
    parser.add_argument("--corridor-trajectory", type=Path, required=True)
    parser.add_argument(
        "--corridor-source-index",
        type=int,
        help="0-based input_pcd index whose GLIM trajectory/transform produced "
        "--corridor-trajectory; mandatory for multi-source builds.",
    )
    parser.add_argument(
        "--corridor-index-range",
        type=str,
        required=True,
        metavar="START:END",
        help="Half-open trajectory row range corresponding to the mapped representative laps.",
    )
    parser.add_argument("--corridor-radius", type=float, default=35.0)
    parser.add_argument("--centerline-stride", type=int, default=10)
    parser.add_argument(
        "--query-chunk",
        type=int,
        default=1_000_000,
        help="Number of union points per nearest-centerline query chunk.",
    )
    parser.add_argument(
        "--force",
        action="store_true",
        help="Replace an existing output PCD/manifest.",
    )
    parser.add_argument(
        "--allow-missing-origin",
        action="store_true",
        help="compatibility escape for legacy UNSPECIFIED origins; unsafe for deployment maps",
    )
    args = parser.parse_args()

    if len(args.input_pcd) < 2:
        parser.error("at least two input PCDs are required for a consistency map")
    if args.corridor_source_index is None:
        parser.error(
            "--corridor-source-index is mandatory when multiple source maps are used"
        )
    if not 0 <= args.corridor_source_index < len(args.input_pcd):
        parser.error("--corridor-source-index is outside the input_pcd list")
    if not math.isfinite(args.voxel_size) or args.voxel_size <= 0.0:
        parser.error("--voxel-size must be finite and > 0")
    if args.min_sessions < 2 or args.min_sessions > len(args.input_pcd):
        parser.error("--min-sessions must be between 2 and the number of input maps")
    if not math.isfinite(args.corridor_radius) or args.corridor_radius <= 0.0:
        parser.error("--corridor-radius must be finite and > 0")
    if args.centerline_stride < 1 or args.query_chunk < 1:
        parser.error("--centerline-stride and --query-chunk must be >= 1")
    try:
        corridor_range = parse_index_range(args.corridor_index_range)
    except ValueError as exc:
        parser.error(f"--corridor-index-range invalid: {exc}")

    all_input_paths = [path.resolve() for path in args.input_pcd + args.coverage_pcd]
    if len(set(all_input_paths)) != len(all_input_paths):
        parser.error("duplicate input/coverage PCD paths are not allowed")
    if args.output_pcd.resolve() in set(all_input_paths):
        parser.error("output PCD must not collide with an input or coverage PCD")

    manifest_pairs = [
        read_manifest(path, args.allow_missing_origin)
        for path in args.input_pcd
    ]
    reference_origin = str(
        manifest_pairs[0][1].get("enu_origin", "UNSPECIFIED")
    ).split("#", 1)[0].strip()
    coverage_manifest_pairs = [
        read_manifest(path, args.allow_missing_origin)
        for path in args.coverage_pcd
    ]
    for manifest_path, manifest in manifest_pairs[1:] + coverage_manifest_pairs:
        origin = str(manifest.get("enu_origin", "UNSPECIFIED")).split("#", 1)[0].strip()
        if origin != reference_origin:
            raise SystemExit(
                f"ENU datum mismatch: {manifest_path} has {origin!r}, "
                f"expected {reference_origin!r}"
            )

    output_manifest = args.output_pcd.with_suffix(
        args.output_pcd.suffix + ".manifest.yaml"
    )
    if not args.force and (args.output_pcd.exists() or output_manifest.exists()):
        raise SystemExit(
            f"output exists: {args.output_pcd} or {output_manifest}; use --force to replace"
        )
    args.output_pcd.parent.mkdir(parents=True, exist_ok=True)

    session_keys: list[np.ndarray] = []
    session_points: list[np.ndarray] = []
    raw_counts: list[int] = []
    session_unique_counts: list[int] = []
    for path in args.input_pcd:
        keys, points, raw_count = unique_session(path, args.voxel_size)
        session_keys.append(keys)
        session_points.append(points)
        raw_counts.append(raw_count)
        session_unique_counts.append(len(keys))

    all_keys = np.concatenate(session_keys)
    all_points = np.concatenate(session_points)
    union_keys, first_indices, support_counts = np.unique(
        all_keys, return_index=True, return_counts=True
    )
    union_count = len(union_keys)
    representatives = all_points[first_indices]
    del all_keys, all_points, session_keys, session_points, first_indices

    repeated = support_counts >= args.min_sessions
    consistent_count = int(np.count_nonzero(repeated))
    centerline = transformed_centerline(
        args.corridor_trajectory,
        corridor_range,
        manifest_pairs[args.corridor_source_index][1],
        args.centerline_stride,
    )
    tree = cKDTree(centerline[:, :2])
    keep = repeated.copy()
    outside_unique_count = 0
    for start in range(0, len(union_keys), args.query_chunk):
        end = min(start + args.query_chunk, len(union_keys))
        candidate = ~repeated[start:end]
        if not np.any(candidate):
            continue
        distances = tree.query(
            representatives[start:end][candidate, :2], k=1, workers=-1
        )[0]
        outside = distances > args.corridor_radius
        indices = np.flatnonzero(candidate)
        keep[start + indices[outside]] = True
        outside_unique_count += int(np.count_nonzero(outside))
        print(
            f"[consistent_pcd] corridor query {end}/{len(union_keys)}",
            flush=True,
        )

    output_keys = union_keys[keep]
    output_points = representatives[keep]
    coverage_unique_counts: list[int] = []
    coverage_added_counts: list[int] = []
    for coverage_index, coverage_path in enumerate(args.coverage_pcd):
        coverage_keys, coverage_points, _ = unique_session(
            coverage_path, args.voxel_size
        )
        coverage_unique_counts.append(len(coverage_keys))
        add = ~np.isin(coverage_keys, output_keys, assume_unique=True)
        coverage_added_counts.append(int(np.count_nonzero(add)))
        output_keys = np.concatenate([output_keys, coverage_keys[add]])
        output_points = np.concatenate([output_points, coverage_points[add]])
        # Keep output_keys unique before processing another coverage map.
        if coverage_index + 1 < len(args.coverage_pcd):
            order = np.argsort(output_keys)
            output_keys = output_keys[order]
            output_points = output_points[order]
    del representatives, keep, union_keys, support_counts, repeated, output_keys
    if len(output_points) == 0:
        raise SystemExit("consistency filter produced zero points")

    temporary_pcd = args.output_pcd.with_suffix(args.output_pcd.suffix + ".tmp")
    temporary_manifest = output_manifest.with_suffix(output_manifest.suffix + ".tmp")
    temporary_pcd.unlink(missing_ok=True)
    temporary_manifest.unlink(missing_ok=True)
    success = False
    try:
        write_xyz_pcd(temporary_pcd, output_points)
        output_bytes = temporary_pcd.stat().st_size
        output_sha256 = sha256_file(temporary_pcd)
        manifest = {
            "format": "consistent_pcd_v1",
            "built_utc": datetime.datetime.now(datetime.timezone.utc).isoformat(),
            "frame": "enu",
            "enu_origin": reference_origin,
            "pcd_fields": "xyz",
            "points": int(len(output_points)),
            "pcd_bytes": int(output_bytes),
            "pcd_sha256": output_sha256,
            "voxel_size": float(args.voxel_size),
            "source_maps": [str(path.resolve()) for path in args.input_pcd],
            "source_manifests": [
                str(path.resolve()) for path, _ in manifest_pairs
            ],
            "source_exports": [
                source_export_summary(pcd, manifest_path, source_manifest)
                for pcd, (manifest_path, source_manifest) in zip(
                    args.input_pcd, manifest_pairs
                )
            ],
            "coverage_maps": [str(path.resolve()) for path in args.coverage_pcd],
            "coverage_manifests": [
                str(path.resolve()) for path, _ in coverage_manifest_pairs
            ],
            "coverage_exports": [
                source_export_summary(pcd, manifest_path, source_manifest)
                for pcd, (manifest_path, source_manifest) in zip(
                    args.coverage_pcd, coverage_manifest_pairs
                )
            ],
            "source_raw_points": raw_counts,
            "source_unique_voxels": session_unique_counts,
            "union_voxels": int(union_count),
            "consistent_voxels": consistent_count,
            "minimum_sessions": int(args.min_sessions),
            "unique_voxels_kept_outside_corridor": outside_unique_count,
            "coverage_unique_voxels": coverage_unique_counts,
            "coverage_voxels_added": coverage_added_counts,
            "corridor_radius_m": float(args.corridor_radius),
            "corridor_trajectory": str(args.corridor_trajectory.resolve()),
            "corridor_source_index": int(args.corridor_source_index),
            "corridor_source_map": str(
                args.input_pcd[args.corridor_source_index].resolve()
            ),
            "corridor_source_manifest": str(
                manifest_pairs[args.corridor_source_index][0].resolve()
            ),
            "corridor_index_range": args.corridor_index_range,
            "corridor_centerline_stride": int(args.centerline_stride),
            "algorithm": (
                "global voxel union; require min-session support inside driven "
                "corridor; retain unique coverage outside"
            ),
            "gicp_note": "leave localization/utm_transform_path EMPTY for this local-ENU map",
        }
        # Keep the exact upstream transforms, so the driven centerline and map
        # frame remain independently auditable.
        corridor_manifest = manifest_pairs[args.corridor_source_index][1]
        manifest["T_world_utm"] = corridor_manifest["T_world_utm"]
        manifest["T_output_enu_input_enu"] = corridor_manifest[
            "T_output_enu_input_enu"
        ]
        manifest["source_transforms"] = [
            {
                "source_index": index,
                "pcd": str(pcd.resolve()),
                "T_world_utm": source_manifest["T_world_utm"],
                "T_output_enu_input_enu": source_manifest[
                    "T_output_enu_input_enu"
                ],
            }
            for index, (pcd, (_, source_manifest)) in enumerate(
                zip(args.input_pcd, manifest_pairs)
            )
        ]
        manifest["output_formula"] = (
            "consistent_voxels + unique_voxels_kept_outside_corridor "
            "+ globally-new coverage_voxels_added"
        )
        with temporary_manifest.open("w", encoding="utf-8") as handle:
            yaml.safe_dump(manifest, handle, sort_keys=False)
            handle.flush()
            os.fsync(handle.fileno())
        temporary_manifest.replace(output_manifest)
        temporary_pcd.replace(args.output_pcd)
        success = True
    finally:
        if not success:
            temporary_pcd.unlink(missing_ok=True)
            temporary_manifest.unlink(missing_ok=True)

    print(
        f"[consistent_pcd] wrote {len(output_points)} points "
        f"({consistent_count} repeated + {outside_unique_count} unique outside corridor "
        f"+ {sum(coverage_added_counts)} staging coverage) "
        f"to {args.output_pcd}",
        flush=True,
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
