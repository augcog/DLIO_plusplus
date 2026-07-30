#!/usr/bin/env python3
"""Generate dataset-independent full-run and per-lap GICP top-down plots.

The tool intentionally does not infer a dataset root or embed a map/run path.
Callers inject four independent inputs:

* ``--debug-bag``: localization output poses.
* ``--reference-bag``: the GNSS/reference trajectory.
* ``--localization-log``: optional GICP status and reset evidence.
* ``--map``: the PCD drawn in the background.

Topic names, reference-to-map transform, labels, lap detector thresholds, map
sampling, plot margin, and output directory are also command-line parameters.
This keeps one plotting implementation reusable across Laguna, Putnam, and
future datasets without borrowing another dataset's ENU origin or paths.

Typical injection pattern::

    python3 scripts/generate_gicp_topdown.py \
      --debug-bag "$DATASET_ROOT/gicp_result/<result>/debug_topics_bag" \
      --reference-bag "$DATASET_ROOT/prep_bag/<prepared-run>" \
      --localization-log "$DATASET_ROOT/gicp_result/<result>/localization.log" \
      --map "$DATASET_ROOT/maps/<map>.pcd" \
      --output-dir "$DATASET_ROOT/gicp_result/<result>/topdown" \
      --reference-topic /gnss \
      --reference-offset <map-x> <map-y> <map-z> \
      --run-label "<dataset> <run>" --map-label "<map name>"

The shell expands these paths before Python starts. The tool reads only the
injected locations and creates files only below ``--output-dir``.
"""

from __future__ import annotations

import argparse
import bisect
import hashlib
import json
import math
import re
from dataclasses import dataclass
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from rclpy.serialization import deserialize_message
from rosbag2_py import ConverterOptions, SequentialReader, StorageFilter, StorageOptions
from rosidl_runtime_py.utilities import get_message


DEFAULT_FINAL_TOPIC = "/gicp/localization/debug/final_pose"
DEFAULT_GUESS_TOPIC = "/gicp/localization/debug/initial_guess_pose"
DEFAULT_REFERENCE_TOPIC = "/gnss"
STATUS_RE = re.compile(
    r"\[[A-Z]+\] \[([0-9]+\.[0-9]+)\] \[[^\]]+\]: "
    r".*SCAN DEBUG \| status=([^ ]+) stamp=([0-9.]+)"
)
RESET_RE = re.compile(
    r"\[[A-Z]+\] \[([0-9]+\.[0-9]+)\] \[[^\]]+\]: .*"
    r"absolute state reset at \[([-+0-9.eE]+), ([-+0-9.eE]+), ([-+0-9.eE]+)\]"
)


@dataclass(frozen=True)
class Interval:
    index: int
    start: float
    end: float
    complete: bool
    kind: str


@dataclass(frozen=True)
class PlotConfig:
    """Injected labels and rendering choices shared by every output image."""

    run_label: str
    map_label: str
    trajectory_label: str
    reference_label: str
    reference_offset: tuple[float, float, float]
    reference_yaw_deg: float
    margin_m: float
    dpi: int


def arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Render a GICP trajectory, reference trajectory, and PCD from above.",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
        epilog=(
            "All paths are injected explicitly. The reference transform is applied as "
            "map_xyz = Rz(reference_yaw_deg) * reference_xyz + reference_offset. "
            "Pass the transform belonging to the selected map/dataset; never reuse an "
            "ENU offset from another site."
        ),
    )

    paths = parser.add_argument_group("input and output paths")
    paths.add_argument(
        "--debug-bag",
        required=True,
        type=Path,
        help="rosbag2 directory containing the final and initial-guess pose topics",
    )
    paths.add_argument(
        "--reference-bag",
        "--input-bag",
        dest="reference_bag",
        required=True,
        type=Path,
        help="rosbag2 directory containing the GNSS or other absolute reference topic",
    )
    paths.add_argument(
        "--localization-log",
        type=Path,
        help=(
            "optional GICP log with SCAN DEBUG status rows and absolute-reset rows; "
            "without it, final poses are used directly and reset markers are omitted"
        ),
    )
    paths.add_argument(
        "--map",
        dest="map_path",
        required=True,
        type=Path,
        help="binary PCD map; x/y/z may coexist with additional point fields",
    )
    paths.add_argument(
        "--output-dir",
        required=True,
        type=Path,
        help="destination outside the source repository for PNGs and trajectory_manifest.json",
    )

    topics = parser.add_argument_group("topic injection")
    topics.add_argument(
        "--final-pose-topic",
        default=DEFAULT_FINAL_TOPIC,
        help="accepted/final localization pose topic in --debug-bag",
    )
    topics.add_argument(
        "--initial-guess-topic",
        default=DEFAULT_GUESS_TOPIC,
        help="initial-guess pose topic used for rejected frames in --debug-bag",
    )
    topics.add_argument(
        "--reference-topic",
        default=DEFAULT_REFERENCE_TOPIC,
        help="PoseStamped, PoseWithCovarianceStamped, or Odometry-like reference topic",
    )

    transform = parser.add_argument_group("reference-to-map transform injection")
    transform.add_argument(
        "--reference-offset",
        nargs=3,
        type=float,
        metavar=("X", "Y", "Z"),
        default=(0.0, 0.0, 0.0),
        help="translation in meters after rotating the reference coordinates",
    )
    transform.add_argument(
        "--reference-yaw-deg",
        type=float,
        default=0.0,
        help="counter-clockwise yaw applied to reference XY before translation",
    )

    labels = parser.add_argument_group("plot labels")
    labels.add_argument("--run-label", default="GICP replay", help="run/dataset title")
    labels.add_argument("--map-label", default="PCD map", help="map legend/title label")
    labels.add_argument(
        "--trajectory-label",
        default="GICP trajectory",
        help="localization trajectory legend/title label",
    )
    labels.add_argument(
        "--reference-label",
        default="absolute reference",
        help="GNSS/reference trajectory legend label",
    )

    rendering = parser.add_argument_group("rendering parameters")
    rendering.add_argument(
        "--map-sample-count",
        type=int,
        default=900_000,
        help="deterministic maximum number of PCD points rendered",
    )
    rendering.add_argument(
        "--map-sample-seed",
        type=int,
        default=325,
        help="random seed used only for deterministic PCD subsampling",
    )
    rendering.add_argument(
        "--plot-margin-m",
        type=float,
        default=45.0,
        help="XY margin around all trajectory samples",
    )
    rendering.add_argument("--dpi", type=int, default=190, help="output PNG resolution")

    laps = parser.add_argument_group("automatic lap segmentation parameters")
    laps.add_argument(
        "--min-lap-duration-s",
        type=float,
        default=60.0,
        help="minimum time between two crossings accepted as one complete lap",
    )
    laps.add_argument(
        "--max-lap-duration-s",
        type=float,
        default=600.0,
        help="maximum time between two crossings accepted as one complete lap",
    )
    laps.add_argument(
        "--start-line-half-width-m",
        type=float,
        default=25.0,
        help="lateral half-width of each candidate start/finish line",
    )
    laps.add_argument(
        "--min-crossing-speed-mps",
        type=float,
        default=3.0,
        help="minimum forward speed for accepting a start/finish crossing",
    )
    laps.add_argument(
        "--min-partial-duration-s",
        type=float,
        default=5.0,
        help="minimum leading/trailing duration emitted as a partial segment",
    )
    laps.add_argument(
        "--no-lap-split",
        action="store_true",
        help="write only the full-run plot when the dataset is not a closed-course replay",
    )
    return parser.parse_args()


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def open_reader(path: Path, topics: list[str]):
    reader = SequentialReader()
    reader.open(StorageOptions(uri=str(path), storage_id=""), ConverterOptions("cdr", "cdr"))
    topic_types = {item.name: item.type for item in reader.get_all_topics_and_types()}
    missing = [topic for topic in topics if topic not in topic_types]
    if missing:
        raise RuntimeError(f"missing topics in {path}: {missing}")
    reader.set_filter(StorageFilter(topics=topics))
    return reader, {topic: get_message(topic_types[topic]) for topic in topics}


def stamped_position(message) -> tuple[float, float, float, float]:
    """Read a stamped position from common ROS pose container messages."""

    stamp = message.header.stamp.sec + message.header.stamp.nanosec * 1e-9
    pose = message.pose
    while not hasattr(pose, "position"):
        if not hasattr(pose, "pose"):
            raise RuntimeError(f"message {type(message).__name__} has no pose.position")
        pose = pose.pose
    return stamp, float(pose.position.x), float(pose.position.y), float(pose.position.z)


def read_debug_poses(path: Path, final_topic: str, guess_topic: str):
    reader, message_types = open_reader(path, [final_topic, guess_topic])
    series = {final_topic: [], guess_topic: []}
    while reader.has_next():
        topic, data, _ = reader.read_next()
        message = deserialize_message(data, message_types[topic])
        series[topic].append(stamped_position(message))
    return series[final_topic], series[guess_topic]


def read_reference(
    path: Path,
    topic_name: str,
    offset: tuple[float, float, float],
    yaw_deg: float,
):
    """Load a reference trajectory and place it in the selected map frame.

    The transform is deliberately injected by the caller because its values
    belong to the selected dataset/map contract. Rotation is applied before
    translation: ``map_xyz = Rz(yaw) * reference_xyz + offset``.
    """

    reader, message_types = open_reader(path, [topic_name])
    yaw = math.radians(yaw_deg)
    cosine = math.cos(yaw)
    sine = math.sin(yaw)
    result = []
    while reader.has_next():
        topic, data, _ = reader.read_next()
        message = deserialize_message(data, message_types[topic])
        stamp, x, y, z = stamped_position(message)
        result.append(
            (
                stamp,
                cosine * x - sine * y + offset[0],
                sine * x + cosine * y + offset[1],
                z + offset[2],
            )
        )
    result.sort(key=lambda row: row[0])
    return result


def parse_log(path: Path | None):
    """Parse optional log evidence emitted by the repository's GICP node."""

    if path is None:
        return [], []
    statuses = []
    reset_wall_rows = []
    for line in path.read_text(errors="replace").splitlines():
        match = STATUS_RE.search(line)
        if match:
            statuses.append((float(match.group(3)), match.group(2), float(match.group(1))))
        match = RESET_RE.search(line)
        if match:
            reset_wall_rows.append(
                (
                    float(match.group(1)),
                    float(match.group(2)),
                    float(match.group(3)),
                    float(match.group(4)),
                )
            )
    statuses.sort(key=lambda row: row[0])
    if reset_wall_rows and not statuses:
        raise RuntimeError(
            f"{path} has absolute-reset rows but no SCAN DEBUG rows to align them"
        )
    wall_sorted = sorted(statuses, key=lambda row: row[2])
    status_walls = np.asarray([row[2] for row in wall_sorted], dtype=np.float64)
    status_stamps = np.asarray([row[0] for row in wall_sorted], dtype=np.float64)
    resets = []
    for wall, x, y, z in reset_wall_rows:
        source_stamp = float(np.interp(wall, status_walls, status_stamps))
        resets.append((source_stamp, x, y, z))
    return statuses, resets


def closest_status(stamps: list[float], rows, stamp: float):
    if not stamps:
        return None
    index = bisect.bisect_left(stamps, stamp)
    candidates = [index]
    if index:
        candidates.append(index - 1)
    if index + 1 < len(stamps):
        candidates.append(index + 1)
    if not candidates:
        return None
    nearest = min(candidates, key=lambda item: abs(stamps[item] - stamp))
    if abs(stamps[nearest] - stamp) > 0.005:
        return None
    return rows[nearest][1]


def official_trajectory(finals, guesses, statuses):
    if len(finals) != len(guesses):
        raise RuntimeError(f"debug pose count mismatch: final={len(finals)} guess={len(guesses)}")
    status_stamps = [row[0] for row in statuses]
    official = []
    rejected = 0
    matched = 0
    for final, guess in zip(finals, guesses):
        if abs(final[0] - guess[0]) > 0.005:
            raise RuntimeError(f"debug pose stamp mismatch: {final[0]} vs {guess[0]}")
        status = closest_status(status_stamps, statuses, final[0])
        if status is not None:
            matched += 1
        if status is not None and not status.startswith("ok"):
            official.append(guess)
            rejected += 1
        else:
            official.append(final)
    return official, rejected, matched


def pcd_scalar_dtype(type_code: str, size: int) -> np.dtype:
    """Translate one PCD scalar declaration to a little-endian NumPy dtype."""

    codes = {
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
    }
    try:
        return np.dtype(codes[(type_code, size)])
    except KeyError as error:
        raise RuntimeError(f"unsupported PCD scalar type: TYPE={type_code} SIZE={size}") from error


def pcd_xyz_sample(path: Path, requested: int, seed: int):
    """Read a deterministic XYZ sample from a binary PCD with arbitrary fields."""

    with path.open("rb") as stream:
        header = []
        while True:
            line = stream.readline()
            if not line:
                raise RuntimeError("PCD header ended before DATA")
            header.append(line.decode("ascii", errors="strict").strip())
            if line.startswith(b"DATA "):
                data_offset = stream.tell()
                break
    values = {}
    for line in header:
        parts = line.split(maxsplit=1)
        if len(parts) == 2:
            values[parts[0]] = parts[1]
    fields = values.get("FIELDS", "").split()
    if not {"x", "y", "z"}.issubset(fields):
        raise RuntimeError(f"{path} must contain x, y, and z fields")
    if values.get("DATA") != "binary":
        raise RuntimeError(f"{path} uses DATA {values.get('DATA')}; only binary PCD is supported")

    sizes = [int(item) for item in values["SIZE"].split()]
    type_codes = values["TYPE"].split()
    counts = [int(item) for item in values.get("COUNT", " ".join(["1"] * len(fields))).split()]
    if not (len(fields) == len(sizes) == len(type_codes) == len(counts)):
        raise RuntimeError(f"inconsistent PCD FIELDS/SIZE/TYPE/COUNT declarations in {path}")
    if any(counts[fields.index(axis)] != 1 for axis in ("x", "y", "z")):
        raise RuntimeError(f"x, y, and z must be scalar PCD fields in {path}")

    dtype_fields = []
    for name, size, type_code, field_count in zip(fields, sizes, type_codes, counts):
        scalar = pcd_scalar_dtype(type_code, size)
        dtype_fields.append((name, scalar) if field_count == 1 else (name, scalar, (field_count,)))

    count = int(values["POINTS"])
    points = np.memmap(
        path,
        mode="r",
        dtype=np.dtype(dtype_fields),
        offset=data_offset,
        shape=(count,),
    )
    sample_count = min(requested, count)
    rng = np.random.default_rng(seed)
    indices = np.sort(rng.choice(count, size=sample_count, replace=False))
    sample = points[indices]
    xyz = np.column_stack((sample["x"], sample["y"], sample["z"]))
    return np.asarray(xyz, dtype=np.float64), count


def percentile(values: np.ndarray, q: float) -> float:
    return float(np.percentile(values, q))


def merge_crossings(crossings: list[tuple[float, float]], min_gap_s: float) -> list[float]:
    if not crossings:
        return []
    groups = [[crossings[0]]]
    for crossing in crossings[1:]:
        if crossing[0] - groups[-1][-1][0] < min_gap_s:
            groups[-1].append(crossing)
        else:
            groups.append([crossing])
    return [min(group, key=lambda item: item[1])[0] for group in groups]


def crossings_for_line(
    stamps: np.ndarray,
    xy: np.ndarray,
    anchor: np.ndarray,
    tangent: np.ndarray,
    line_half_width_m: float,
    min_speed_mps: float,
    min_lap_duration_s: float,
) -> list[float]:
    normal = np.asarray((-tangent[1], tangent[0]), dtype=np.float64)
    relative = xy - anchor
    along = relative @ tangent
    indices = np.flatnonzero((along[:-1] <= 0.0) & (along[1:] > 0.0))
    result = []
    for index in indices:
        denominator = float(along[index + 1] - along[index])
        dt = float(stamps[index + 1] - stamps[index])
        if denominator <= 1e-9 or dt <= 0.0:
            continue
        alpha = float(np.clip(-along[index] / denominator, 0.0, 1.0))
        point = xy[index] + alpha * (xy[index + 1] - xy[index])
        lateral = abs(float((point - anchor) @ normal))
        forward_speed = float((xy[index + 1] - xy[index]) @ tangent) / dt
        if lateral <= line_half_width_m and forward_speed >= min_speed_mps:
            result.append((float(stamps[index] + alpha * dt), lateral))
    return merge_crossings(result, min_lap_duration_s)


def detect_intervals(
    reference: np.ndarray,
    min_lap_duration_s: float = 60.0,
    max_lap_duration_s: float = 600.0,
    line_half_width_m: float = 25.0,
    min_speed_mps: float = 3.0,
    min_partial_duration_s: float = 5.0,
):
    """Find complete laps from repeated same-direction reference crossings."""

    stamps = reference[:, 0]
    xy = reference[:, 1:3]
    dt = np.gradient(stamps)
    velocity = np.gradient(xy, axis=0) / dt[:, None]
    speed = np.linalg.norm(velocity, axis=1)
    valid = np.flatnonzero(
        (speed >= min_speed_mps)
        & (np.arange(len(reference)) >= 5)
        & (np.arange(len(reference)) < len(reference) - 5)
    )
    if len(valid) > 180:
        valid = valid[np.linspace(0, len(valid) - 1, 180, dtype=np.int64)]

    best = None
    for candidate in valid:
        delta = xy[candidate + 5] - xy[candidate - 5]
        norm = float(np.linalg.norm(delta))
        if norm < 1e-6:
            continue
        tangent = delta / norm
        crossings = crossings_for_line(
            stamps,
            xy,
            xy[candidate],
            tangent,
            line_half_width_m,
            min_speed_mps,
            min_lap_duration_s,
        )
        if len(crossings) < 2:
            continue
        durations = np.diff(np.asarray(crossings))
        plausible = durations[
            (durations >= min_lap_duration_s) & (durations <= max_lap_duration_s)
        ]
        if not len(plausible):
            continue
        median = float(np.median(plausible))
        regular = int(
            np.count_nonzero(np.abs(plausible - median) <= max(10.0, 0.30 * median))
        )
        cv = float(np.std(plausible) / median) if len(plausible) > 1 else 0.0
        score = (regular, len(plausible), -cv, crossings[-1] - crossings[0])
        if best is None or score > best[0]:
            best = (score, crossings, xy[candidate].copy(), tangent.copy(), cv)

    if best is None:
        return [Interval(1, float(stamps[0]), float(stamps[-1]), False, "partial_run")], {
            "method": "whole-run fallback",
            "confidence": "partial",
            "crossing_times": [],
            "note": "No repeated same-direction crossing found.",
        }

    _, crossings, anchor, tangent, cv = best
    complete = [
        Interval(index + 1, start, end, True, "lap")
        for index, (start, end) in enumerate(zip(crossings[:-1], crossings[1:]))
        if min_lap_duration_s <= end - start <= max_lap_duration_s
    ]
    if not complete:
        return [Interval(1, float(stamps[0]), float(stamps[-1]), False, "partial_run")], {
            "method": "whole-run fallback",
            "confidence": "partial",
            "crossing_times": crossings,
            "note": "Crossings did not form a plausible lap.",
        }

    intervals = []
    partial_index = 1
    if complete[0].start - stamps[0] >= min_partial_duration_s:
        intervals.append(
            Interval(partial_index, float(stamps[0]), complete[0].start, False, "partial_segment")
        )
        partial_index += 1
    intervals.extend(complete)
    if stamps[-1] - complete[-1].end >= min_partial_duration_s:
        intervals.append(
            Interval(partial_index, complete[-1].end, float(stamps[-1]), False, "partial_segment")
        )
    confidence = "high" if len(complete) >= 2 and cv <= 0.20 else "medium"
    return intervals, {
        "method": "automatic repeated same-direction reference crossing",
        "confidence": confidence,
        "anchor_xy_m": [float(anchor[0]), float(anchor[1])],
        "tangent_xy": [float(tangent[0]), float(tangent[1])],
        "crossing_times": crossings,
        "interval_cv": cv,
        "note": "The injected reference trajectory is used after replay to segment laps.",
    }


def make_plot(
    map_points: np.ndarray,
    map_total: int,
    official: np.ndarray,
    reference: np.ndarray,
    resets: np.ndarray,
    output: Path,
    config: PlotConfig,
):
    x_all = np.concatenate((official[:, 1], reference[:, 1]))
    y_all = np.concatenate((official[:, 2], reference[:, 2]))
    margin = config.margin_m
    x_min, x_max = float(x_all.min() - margin), float(x_all.max() + margin)
    y_min, y_max = float(y_all.min() - margin), float(y_all.max() + margin)
    mask = (
        (map_points[:, 0] >= x_min)
        & (map_points[:, 0] <= x_max)
        & (map_points[:, 1] >= y_min)
        & (map_points[:, 1] <= y_max)
    )
    visible_map = map_points[mask]

    reference_t = reference[:, 0]
    valid = (official[:, 0] >= reference_t[0]) & (official[:, 0] <= reference_t[-1])
    official_scored = official[valid]
    if not len(official_scored):
        raise RuntimeError("localization and reference trajectories do not overlap in time")
    reference_x = np.interp(official_scored[:, 0], reference_t, reference[:, 1])
    reference_y = np.interp(official_scored[:, 0], reference_t, reference[:, 2])
    errors = np.hypot(
        official_scored[:, 1] - reference_x,
        official_scored[:, 2] - reference_y,
    )

    figure, axis = plt.subplots(figsize=(16, 11), constrained_layout=True)
    if len(visible_map):
        axis.scatter(
            visible_map[:, 0],
            visible_map[:, 1],
            s=0.12,
            c="#7c8794",
            alpha=0.20,
            linewidths=0,
            rasterized=True,
            label=f"{config.map_label} (sampled {len(visible_map):,})",
        )
    axis.plot(
        reference[:, 1],
        reference[:, 2],
        color="#d62728",
        linewidth=2.0,
        alpha=0.90,
        label=config.reference_label,
    )
    axis.plot(
        official[:, 1],
        official[:, 2],
        color="#12a33a",
        linewidth=1.15,
        alpha=0.94,
        label=config.trajectory_label,
    )
    if len(resets):
        axis.scatter(
            resets[:, 1],
            resets[:, 2],
            marker="X",
            s=70,
            c="#ffd21f",
            edgecolors="#222222",
            linewidths=0.8,
            zorder=8,
            label=f"absolute resets ({len(resets)})",
        )
    axis.scatter(
        [official[0, 1]],
        [official[0, 2]],
        marker="o",
        s=72,
        facecolors="white",
        edgecolors="#111111",
        linewidths=1.3,
        zorder=9,
        label="start",
    )
    axis.scatter(
        [official[-1, 1]],
        [official[-1, 2]],
        marker="s",
        s=64,
        facecolors="#4f7cff",
        edgecolors="#111111",
        linewidths=1.0,
        zorder=9,
        label="end",
    )
    axis.set_title(
        f"{config.run_label} — {config.trajectory_label} on {config.map_label}\n"
        f"full replay {official[-1, 0] - official[0, 0]:.1f}s | "
        f"reference XY error median {percentile(errors, 50):.2f}m, "
        f"P95 {percentile(errors, 95):.2f}m, max {errors.max():.2f}m"
    )
    axis.set_xlabel("Map X / East [m]")
    axis.set_ylabel("Map Y / North [m]")
    axis.set_xlim(x_min, x_max)
    axis.set_ylim(y_min, y_max)
    axis.set_aspect("equal", adjustable="box")
    axis.grid(True, alpha=0.22)
    axis.legend(loc="best", framealpha=0.93)
    axis.text(
        0.008,
        0.008,
        f"PCD total points: {map_total:,} | reference transform: "
        f"yaw {config.reference_yaw_deg:g} deg, offset "
        f"({config.reference_offset[0]:+.4f}, {config.reference_offset[1]:+.4f}, "
        f"{config.reference_offset[2]:+.4f}) m",
        transform=axis.transAxes,
        fontsize=8.5,
        color="#333333",
    )
    figure.savefig(output, dpi=config.dpi)
    plt.close(figure)
    return errors, len(visible_map)


def make_interval_plot(
    map_points: np.ndarray,
    map_total: int,
    official: np.ndarray,
    reference: np.ndarray,
    resets: np.ndarray,
    interval: Interval,
    global_bounds: tuple[float, float, float, float],
    output: Path,
    config: PlotConfig,
):
    include_end = interval.end >= reference[-1, 0] - 1e-6
    official_mask = (official[:, 0] >= interval.start) & (
        official[:, 0] <= interval.end if include_end else official[:, 0] < interval.end
    )
    reference_mask = (reference[:, 0] >= interval.start) & (
        reference[:, 0] <= interval.end
        if include_end
        else reference[:, 0] < interval.end
    )
    reset_mask = (resets[:, 0] >= interval.start) & (resets[:, 0] < interval.end)
    official_segment = official[official_mask]
    reference_segment = reference[reference_mask]
    reset_segment = resets[reset_mask]
    if len(official_segment) < 2 or len(reference_segment) < 2:
        raise RuntimeError(f"interval has insufficient samples: {interval}")

    reference_x = np.interp(
        official_segment[:, 0],
        reference_segment[:, 0],
        reference_segment[:, 1],
    )
    reference_y = np.interp(
        official_segment[:, 0],
        reference_segment[:, 0],
        reference_segment[:, 2],
    )
    errors = np.hypot(
        official_segment[:, 1] - reference_x,
        official_segment[:, 2] - reference_y,
    )
    x_min, x_max, y_min, y_max = global_bounds
    map_mask = (
        (map_points[:, 0] >= x_min)
        & (map_points[:, 0] <= x_max)
        & (map_points[:, 1] >= y_min)
        & (map_points[:, 1] <= y_max)
    )
    visible_map = map_points[map_mask]
    label = f"Lap {interval.index:03d}" if interval.complete else f"Partial {interval.index:03d}"

    figure, axis = plt.subplots(figsize=(16, 11), constrained_layout=True)
    axis.scatter(
        visible_map[:, 0],
        visible_map[:, 1],
        s=0.12,
        c="#7c8794",
        alpha=0.20,
        linewidths=0,
        rasterized=True,
        label=f"{config.map_label} (sampled {len(visible_map):,})",
    )
    axis.plot(
        reference_segment[:, 1],
        reference_segment[:, 2],
        color="#d62728",
        linewidth=2.0,
        alpha=0.90,
        label=config.reference_label,
    )
    axis.plot(
        official_segment[:, 1],
        official_segment[:, 2],
        color="#12a33a",
        linewidth=1.35,
        alpha=0.95,
        label=config.trajectory_label,
    )
    if len(reset_segment):
        axis.scatter(
            reset_segment[:, 1],
            reset_segment[:, 2],
            marker="X",
            s=74,
            c="#ffd21f",
            edgecolors="#222222",
            linewidths=0.8,
            zorder=8,
            label=f"absolute resets ({len(reset_segment)})",
        )
    axis.scatter(
        [official_segment[0, 1]],
        [official_segment[0, 2]],
        marker="o",
        s=72,
        facecolors="white",
        edgecolors="#111111",
        linewidths=1.3,
        zorder=9,
        label="lap start",
    )
    axis.scatter(
        [official_segment[-1, 1]],
        [official_segment[-1, 2]],
        marker="s",
        s=64,
        facecolors="#4f7cff",
        edgecolors="#111111",
        linewidths=1.0,
        zorder=9,
        label="lap end",
    )
    axis.set_title(
        f"{config.run_label} — {label} top-down on {config.map_label}\n"
        f"duration {interval.end - interval.start:.1f}s | samples {len(official_segment):,} | "
        f"reference XY median {percentile(errors, 50):.2f}m, "
        f"P95 {percentile(errors, 95):.2f}m, max {errors.max():.2f}m"
    )
    axis.set_xlabel("Map X / East [m]")
    axis.set_ylabel("Map Y / North [m]")
    axis.set_xlim(x_min, x_max)
    axis.set_ylim(y_min, y_max)
    axis.set_aspect("equal", adjustable="box")
    axis.grid(True, alpha=0.22)
    axis.legend(loc="best", framealpha=0.93)
    axis.text(
        0.008,
        0.008,
        f"PCD total points: {map_total:,} | interval [{interval.start:.3f}, {interval.end:.3f}]",
        transform=axis.transAxes,
        fontsize=8.5,
        color="#333333",
    )
    figure.savefig(output, dpi=config.dpi)
    plt.close(figure)
    return {
        "index": interval.index,
        "complete": interval.complete,
        "kind": interval.kind,
        "start": interval.start,
        "end": interval.end,
        "duration_s": interval.end - interval.start,
        "sample_count": len(official_segment),
        "reset_count": len(reset_segment),
        "reference_xy_error_m": {
            "median": percentile(errors, 50),
            "p90": percentile(errors, 90),
            "p95": percentile(errors, 95),
            "p99": percentile(errors, 99),
            "max": float(errors.max()),
        },
        "image": output.name,
        "image_sha256": sha256(output),
    }


def main() -> int:
    args = arguments()
    offset = tuple(args.reference_offset)
    if args.map_sample_count <= 0:
        raise RuntimeError("--map-sample-count must be positive")
    if args.plot_margin_m < 0.0:
        raise RuntimeError("--plot-margin-m must be non-negative")
    if args.dpi <= 0:
        raise RuntimeError("--dpi must be positive")
    if args.min_lap_duration_s <= 0.0:
        raise RuntimeError("--min-lap-duration-s must be positive")
    if args.max_lap_duration_s <= args.min_lap_duration_s:
        raise RuntimeError("--max-lap-duration-s must exceed --min-lap-duration-s")

    config = PlotConfig(
        run_label=args.run_label,
        map_label=args.map_label,
        trajectory_label=args.trajectory_label,
        reference_label=args.reference_label,
        reference_offset=offset,
        reference_yaw_deg=args.reference_yaw_deg,
        margin_m=args.plot_margin_m,
        dpi=args.dpi,
    )
    args.output_dir.mkdir(parents=True, exist_ok=True)

    finals, guesses = read_debug_poses(
        args.debug_bag,
        args.final_pose_topic,
        args.initial_guess_topic,
    )
    reference = read_reference(
        args.reference_bag,
        args.reference_topic,
        offset,
        args.reference_yaw_deg,
    )
    statuses, resets = parse_log(args.localization_log)
    official, rejected, matched = official_trajectory(finals, guesses, statuses)
    map_points, map_total = pcd_xyz_sample(
        args.map_path,
        args.map_sample_count,
        args.map_sample_seed,
    )

    official_array = np.asarray(official, dtype=np.float64)
    reference_array = np.asarray(reference, dtype=np.float64)
    resets_array = np.asarray(resets, dtype=np.float64).reshape((-1, 4))
    if len(official_array) < 2:
        raise RuntimeError("localization trajectory must contain at least two poses")
    if len(reference_array) < 2:
        raise RuntimeError("reference trajectory must contain at least two poses")

    image = args.output_dir / "full_topdown.png"
    errors, visible_map_count = make_plot(
        map_points,
        map_total,
        official_array,
        reference_array,
        resets_array,
        image,
        config,
    )
    x_all = np.concatenate((official_array[:, 1], reference_array[:, 1]))
    y_all = np.concatenate((official_array[:, 2], reference_array[:, 2]))
    global_bounds = (
        float(x_all.min() - config.margin_m),
        float(x_all.max() + config.margin_m),
        float(y_all.min() - config.margin_m),
        float(y_all.max() + config.margin_m),
    )
    if args.no_lap_split:
        intervals = []
        lap_detection = {
            "method": "disabled by --no-lap-split",
            "confidence": "n/a",
            "crossing_times": [],
        }
    else:
        intervals, lap_detection = detect_intervals(
            reference_array,
            min_lap_duration_s=args.min_lap_duration_s,
            max_lap_duration_s=args.max_lap_duration_s,
            line_half_width_m=args.start_line_half_width_m,
            min_speed_mps=args.min_crossing_speed_mps,
            min_partial_duration_s=args.min_partial_duration_s,
        )
    lap_items = []
    for interval in intervals:
        if interval.complete:
            name = f"lap_{interval.index:03d}.png"
        elif interval.kind == "partial_run":
            name = "lap_001_partial.png"
        else:
            name = f"partial_{interval.index:03d}.png"
        lap_items.append(
            make_interval_plot(
                map_points,
                map_total,
                official_array,
                reference_array,
                resets_array,
                interval,
                global_bounds,
                args.output_dir / name,
                config,
            )
        )
    manifest = {
        "schema_version": 2,
        "run": args.run_label,
        "image": image.name,
        "image_sha256": sha256(image),
        "inputs": {
            "debug_bag": str(args.debug_bag.resolve()),
            "reference_bag": str(args.reference_bag.resolve()),
            "localization_log": (
                str(args.localization_log.resolve()) if args.localization_log else None
            ),
            "map": str(args.map_path.resolve()),
            "map_total_points": map_total,
            "topics": {
                "final_pose": args.final_pose_topic,
                "initial_guess": args.initial_guess_topic,
                "reference": args.reference_topic,
            },
            "reference_to_map": {
                "operation": "map_xyz = Rz(yaw) * reference_xyz + offset",
                "offset_xyz_m": list(offset),
                "yaw_deg": args.reference_yaw_deg,
            },
        },
        "parameters": {
            "map_sample_count": args.map_sample_count,
            "map_sample_seed": args.map_sample_seed,
            "plot_margin_m": args.plot_margin_m,
            "dpi": args.dpi,
            "lap_split_enabled": not args.no_lap_split,
            "min_lap_duration_s": args.min_lap_duration_s,
            "max_lap_duration_s": args.max_lap_duration_s,
            "start_line_half_width_m": args.start_line_half_width_m,
            "min_crossing_speed_mps": args.min_crossing_speed_mps,
            "min_partial_duration_s": args.min_partial_duration_s,
        },
        "counts": {
            "debug_final_pose": len(finals),
            "debug_initial_guess_pose": len(guesses),
            "status_rows": len(statuses),
            "status_matched_poses": matched,
            "rejected_poses_rendered_from_initial_guess": rejected,
            "reference_samples": len(reference),
            "absolute_resets": len(resets),
            "sampled_map_points_visible": visible_map_count,
            "complete_laps": sum(item["complete"] for item in lap_items),
            "partial_segments": sum(not item["complete"] for item in lap_items),
        },
        "reference_xy_error_m": {
            "count": int(len(errors)),
            "median": percentile(errors, 50),
            "p90": percentile(errors, 90),
            "p95": percentile(errors, 95),
            "p99": percentile(errors, 99),
            "max": float(errors.max()),
        },
        "plot_contract": {
            "green": (
                "localization trajectory: accepted final pose, or initial guess for a "
                "rejected status when the optional log is available"
            ),
            "red": "injected reference topic after the injected reference-to-map transform",
            "yellow_x": "absolute state resets parsed from the optional localization log",
            "gray": "deterministic sample of the injected PCD map",
        },
        "lap_detection": lap_detection,
        "laps": lap_items,
    }
    manifest_path = args.output_dir / "trajectory_manifest.json"
    manifest_path.write_text(json.dumps(manifest, indent=2, sort_keys=True) + "\n")
    print(
        json.dumps(
            {
                "image": str(image),
                "manifest": str(manifest_path),
                **manifest["counts"],
                "errors": manifest["reference_xy_error_m"],
            },
            indent=2,
        )
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
