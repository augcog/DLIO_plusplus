#!/usr/bin/env python3
"""Prepare and validate a normalized AV-24 bag, then optionally run GLIM.

This script intentionally keeps the normalization boundary simple:

* ``adapter`` produces only the small Atlas-derived streams:
  ``/gps_p1/imu`` and ``/gps_p1/filtered_odom*``; with
  ``--record-gnss-pose`` it also emits the derived ``/gnss*`` pose streams.
* The raw bag's ``/luminar_front|left|right/points`` topics are copied
  directly into the normalized bag. They are not bridged or restamped by
  ``adapter``.
* Point One IMU timing comes from PCAP-decoded ``IMU_OUTPUT.p1_time``. The
  PCAP replay node writes that measurement time into the IMU stamp consumed by
  ``adapter``; this script does not run the older arrival-time de-jitter,
  interpolation, or fitted time-line repair logic.

Typical use:

    python3 scripts/prep_bag.py \\
      --input /media/roar/data1/rosbags/putnam/may_26/run_5/filtered/all \\
      --output /media/roar/data1/rosbags/putnam/prep_bag/run_5_normalized \\
      --dump-dir /media/roar/data1/rosbags/putnam/maps/run_5_glim
"""

from __future__ import annotations

import argparse
import glob
import json
import math
import os
import re
import shutil
import signal
import statistics
import struct
import subprocess
import sys
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Dict, Iterable, List, Optional, Sequence, Tuple

import rosbag2_py
from rclpy.serialization import deserialize_message
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, PointCloud2


POSE_IN = "/atlas/pose_filtered"
PCAP_IMU_TOPIC = "/atlas/imu_calibrated_pcap"

LIDAR_TOPICS = [
    "/luminar_front/points",
    "/luminar_left/points",
    "/luminar_right/points",
]

SMALL_TOPICS = [
    "/gps_p1/imu",
    "/gps_p1/filtered_odom",
    "/gps_p1/filtered_odom_rtk_fixed",
]

OPTIONAL_GNSS_TOPICS = [
    "/gnss",
    "/gnss_rtk_fixed",
]

CHECK_TOPIC_TYPES = {
    "/gps_p1/imu": Imu,
    "/gps_p1/filtered_odom": Odometry,
    "/gps_p1/filtered_odom_rtk_fixed": Odometry,
    "/gnss": PoseWithCovarianceStamped,
    "/gnss_rtk_fixed": PoseWithCovarianceStamped,
    "/luminar_front/points": PointCloud2,
    "/luminar_left/points": PointCloud2,
    "/luminar_right/points": PointCloud2,
}

GLIM_SYMPTOM_PATTERN = (
    r"large time gap between consecutive LiDAR|"
    r"large time difference between points and imu|"
    r"IndeterminantLinearSystemException|"
    r"segfault"
)


class PrepError(RuntimeError):
    pass


@dataclass
class RunningProcess:
    name: str
    cmd: List[str]
    proc: subprocess.Popen
    log_path: Path


@dataclass
class TopicStats:
    topic: str
    count: int = 0
    first_header: Optional[float] = None
    last_header: Optional[float] = None
    first_log: Optional[float] = None
    last_log: Optional[float] = None
    header_dts: List[float] = field(default_factory=list)
    log_dts: List[float] = field(default_factory=list)
    header_minus_log: List[float] = field(default_factory=list)
    rewinds: int = 0

    def add(self, header_sec: float, log_ns: int) -> None:
        log_sec = float(log_ns) * 1e-9
        if self.count == 0:
            self.first_header = header_sec
            self.first_log = log_sec
        else:
            assert self.last_header is not None
            assert self.last_log is not None
            header_dt = header_sec - self.last_header
            log_dt = log_sec - self.last_log
            self.header_dts.append(header_dt)
            self.log_dts.append(log_dt)
            if header_dt <= 0.0:
                self.rewinds += 1
        self.last_header = header_sec
        self.last_log = log_sec
        self.header_minus_log.append(header_sec - log_sec)
        self.count += 1


def die(message: str) -> None:
    raise PrepError(message)


def stamp_sec(stamp) -> float:
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


def serialized_header_stamp_sec(data: bytes) -> float:
    if len(data) < 12:
        die("serialized message is too small to contain std_msgs/Header")
    sec, nanosec = struct.unpack_from("<iI", data, 4)
    return float(sec) + float(nanosec) * 1e-9


def percentile(values: Sequence[float], pct: float) -> float:
    if not values:
        return math.nan
    ordered = sorted(values)
    if len(ordered) == 1:
        return ordered[0]
    pos = (len(ordered) - 1) * pct / 100.0
    lo = int(math.floor(pos))
    hi = int(math.ceil(pos))
    if lo == hi:
        return ordered[lo]
    return ordered[lo] * (hi - pos) + ordered[hi] * (pos - lo)


def summarize_seconds(values: Sequence[float]) -> str:
    if not values:
        return "n=0"
    return (
        f"n={len(values)} "
        f"median={statistics.median(values):.6f}s "
        f"p95={percentile(values, 95):.6f}s "
        f"max={max(values):.6f}s"
    )


def yaml_scalar(value) -> str:
    if isinstance(value, bool):
        return "true" if value else "false"
    if isinstance(value, (int, float)):
        return repr(value)
    return json.dumps(str(value))


def write_params(path: Path, params: Dict[str, object]) -> None:
    lines = ["/**:", "  ros__parameters:"]
    for key in sorted(params):
        lines.append(f"    {key}: {yaml_scalar(params[key])}")
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def resolve_existing_path(path_text: str, label: str) -> Path:
    path = Path(path_text).expanduser().resolve()
    if not path.exists():
        die(f"{label} does not exist: {path}")
    return path


def resolve_pcap_arg(path_text: str) -> Path:
    expanded = Path(path_text).expanduser()
    if any(ch in str(expanded) for ch in "*?["):
        matches = sorted(Path(p).resolve() for p in glob.glob(str(expanded)))
        matches = [m.resolve() for m in matches if m.is_file()]
        if not matches:
            die(f"--p1-imu-pcap glob did not match any files: {path_text}")
        if len(matches) > 1:
            joined = "\n  ".join(str(m) for m in matches)
            die(f"--p1-imu-pcap matched multiple files; pass one explicitly:\n  {joined}")
        return matches[0]
    return resolve_existing_path(path_text, "--p1-imu-pcap")


def discover_p1_pcap(input_path: Path) -> Optional[Path]:
    roots: List[Path] = []
    start = input_path.parent if input_path.is_file() else input_path
    cur = start.resolve()
    for _ in range(5):
        roots.append(cur)
        if cur.parent == cur:
            break
        cur = cur.parent

    seen = set()
    candidates: List[Path] = []
    for root in roots:
        for pattern in ("ins*.pcap", "*ins*.pcap", "pointone*.pcap", "*p1*.pcap"):
            for candidate in sorted(root.glob(pattern)):
                resolved = candidate.resolve()
                if resolved not in seen and resolved.is_file():
                    seen.add(resolved)
                    candidates.append(resolved)
        if candidates:
            break

    if not candidates:
        return None
    if len(candidates) > 1:
        joined = "\n  ".join(str(c) for c in candidates)
        die(
            "found multiple possible Point One PCAP files; pass --p1-imu-pcap explicitly:\n"
            f"  {joined}"
        )
    return candidates[0]


def ensure_output_path(path: Path, force: bool, label: str) -> None:
    if not path.exists():
        return
    if not force:
        die(f"{label} already exists: {path} (use --force to remove it first)")
    if path.is_dir():
        shutil.rmtree(path)
    else:
        path.unlink()


def require_ros_package(package: str) -> None:
    result = subprocess.run(
        ["ros2", "pkg", "prefix", package],
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
        text=True,
    )
    if result.returncode != 0:
        die(
            f"ROS package '{package}' is not visible. Source install/setup.bash "
            "from a workspace that built this repo first."
        )


def start_process(name: str, cmd: Sequence[str], log_path: Path) -> RunningProcess:
    log_path.parent.mkdir(parents=True, exist_ok=True)
    log_file = log_path.open("wb")
    proc = subprocess.Popen(
        list(cmd),
        stdout=log_file,
        stderr=subprocess.STDOUT,
        stdin=subprocess.DEVNULL,
        preexec_fn=os.setsid,
    )
    # The child owns the descriptor now; closing this copy prevents leaks.
    log_file.close()
    print(f"[prep_bag] started {name}: pid={proc.pid} log={log_path}")
    return RunningProcess(name=name, cmd=list(cmd), proc=proc, log_path=log_path)


def stop_process(rp: RunningProcess, sig: int = signal.SIGINT, timeout: float = 20.0) -> None:
    if rp.proc.poll() is not None:
        return
    try:
        os.killpg(rp.proc.pid, sig)
    except ProcessLookupError:
        return
    try:
        rp.proc.wait(timeout=timeout)
    except subprocess.TimeoutExpired:
        print(f"[prep_bag] {rp.name} did not stop after SIGINT; sending SIGTERM")
        try:
            os.killpg(rp.proc.pid, signal.SIGTERM)
        except ProcessLookupError:
            return
        try:
            rp.proc.wait(timeout=5.0)
        except subprocess.TimeoutExpired:
            print(f"[prep_bag] {rp.name} did not stop after SIGTERM; sending SIGKILL")
            try:
                os.killpg(rp.proc.pid, signal.SIGKILL)
            except ProcessLookupError:
                return
            rp.proc.wait(timeout=5.0)


def check_processes(processes: Iterable[RunningProcess]) -> None:
    for rp in processes:
        rc = rp.proc.poll()
        if rc is not None:
            die(f"{rp.name} exited early with code {rc}; see {rp.log_path}")


def sleep_checked(seconds: float, processes: Iterable[RunningProcess]) -> None:
    deadline = time.monotonic() + seconds
    while time.monotonic() < deadline:
        check_processes(processes)
        time.sleep(min(0.25, max(0.0, deadline - time.monotonic())))


def build_adapter_params(args: argparse.Namespace, pcap_path: Path, summary_path: Path) -> Dict[str, object]:
    local_origin = args.local_enu_origin
    ttl_path = args.local_enu_origin_ttl_path
    if local_origin and ttl_path:
        die("set only one of --local-enu-origin or --local-enu-origin-ttl-path")

    params: Dict[str, object] = {
        "use_sim_time": True,
        "pose_input_topic": args.pose_input_topic,
        "imu_input_topic": PCAP_IMU_TOPIC,
        "imu_stamp_mode": "p1",
        "publish_gnss_pose": bool(args.record_gnss_pose),
        "summary_output_path": str(summary_path),
        # One huge bin keeps the P1->ROS conversion as a constant offset and
        # avoids the piecewise interpolation path in P1ClockMapper.
        "p1_clock_bin_seconds": args.p1_clock_bin_seconds,
        "p1_like_threshold_sec": args.p1_like_threshold_sec,
        "imu_frame_id": args.imu_frame_id,
        "body_frame_id": args.body_frame_id,
        "odom_frame_id": args.odom_frame_id,
        "rtk_max_var_xy": args.rtk_max_var_xy,
        "rtk_max_var_z": args.rtk_max_var_z,
        "imu_input_reliability": "best_effort",
        "pose_input_reliability": "reliable",
        "imu_input_qos_depth": 1000,
        "pose_input_qos_depth": 10000,
        "local_enu_origin": local_origin,
        "local_enu_origin_ttl_path": ttl_path,
        # Keep sidecar empty. The PCAP replay node publishes decoded p1_time
        # directly in the incoming IMU stamp.
        "imu_p1_sidecar_path": "",
    }
    if not local_origin and not ttl_path:
        # Match adapter/config/adapter.yaml default explicitly so this script
        # remains stable even if a different params file is in the environment.
        params["local_enu_origin"] = "39.58227391,-86.74232215,260.4"
    if not pcap_path.exists():
        die(f"Point One PCAP does not exist: {pcap_path}")
    return params


def normalize_bag(args: argparse.Namespace, input_path: Path, output_path: Path, work_dir: Path) -> None:
    require_ros_package("adapter")
    ensure_output_path(output_path, args.force, "normalized output bag")

    if args.p1_imu_pcap:
        pcap_path = resolve_pcap_arg(args.p1_imu_pcap)
    else:
        pcap_path = discover_p1_pcap(input_path)
        if pcap_path is None:
            die(
                "could not auto-discover a Point One PCAP near the input bag. "
                "Pass --p1-imu-pcap /path/to/ins_*.pcap."
            )

    work_dir.mkdir(parents=True, exist_ok=True)
    adapter_params_path = work_dir / "adapter_params.yaml"
    pcap_params_path = work_dir / "p1_imu_pcap_params.yaml"
    adapter_summary_path = work_dir / "adapter_summary.txt"
    small_output_path = work_dir / "small_streams_bag"
    ensure_output_path(small_output_path, True, "temporary small-stream bag")

    write_params(
        pcap_params_path,
        {
            "pcap_path": str(pcap_path),
            "output_topic": PCAP_IMU_TOPIC,
            "clock_topic": "/clock",
            "frame_id": args.imu_frame_id,
            "pace_mode": "clock",
            "play_rate": args.rate,
            "max_clock_lag_sec": args.pcap_max_clock_lag_sec,
            "max_messages": 0,
        },
    )
    write_params(adapter_params_path, build_adapter_params(args, pcap_path, adapter_summary_path))

    record_topics = list(SMALL_TOPICS)
    if args.record_gnss_pose:
        record_topics += OPTIONAL_GNSS_TOPICS

    record_cmd = [
        "ros2",
        "bag",
        "record",
        "-o",
        str(small_output_path),
        "-s",
        "mcap",
        "--disable-keyboard-controls",
        "--topics",
        *record_topics,
    ]
    pcap_cmd = [
        "ros2",
        "run",
        "adapter",
        "p1_imu_pcap_replay_node.py",
        "--ros-args",
        "--params-file",
        str(pcap_params_path),
    ]
    adapter_cmd = [
        "ros2",
        "run",
        "adapter",
        "adapter",
        "--ros-args",
        "--params-file",
        str(adapter_params_path),
    ]
    play_topics = [args.pose_input_topic]
    play_cmd = [
        "ros2",
        "bag",
        "play",
        str(input_path),
        "--clock",
        str(args.clock_hz),
        "--rate",
        str(args.rate),
        "--read-ahead-queue-size",
        str(args.read_ahead_queue_size),
        "--disable-keyboard-controls",
        "--topics",
        *play_topics,
    ]

    processes: List[RunningProcess] = []
    play: Optional[RunningProcess] = None
    try:
        processes.append(start_process("p1_imu_pcap_replay", pcap_cmd, work_dir / "p1_imu_pcap_replay.log"))
        processes.append(start_process("adapter", adapter_cmd, work_dir / "adapter.log"))
        sleep_checked(args.startup_delay, processes)

        processes.append(start_process("ros2 bag record", record_cmd, work_dir / "record.log"))
        sleep_checked(args.startup_delay, processes)

        print(f"[prep_bag] playing raw bag with PCAP IMU timing: pcap={pcap_path}")
        play = start_process("ros2 bag play", play_cmd, work_dir / "play.log")
        while play.proc.poll() is None:
            check_processes(processes)
            time.sleep(1.0)
        if play.proc.returncode != 0:
            die(f"ros2 bag play failed with code {play.proc.returncode}; see {play.log_path}")
        print("[prep_bag] raw bag playback finished")

        time.sleep(args.post_play_drain_sec)
    finally:
        if play is not None:
            stop_process(play)
        for rp in reversed(processes):
            stop_process(rp)

    if not small_output_path.exists():
        die(f"record did not create small-stream bag: {small_output_path}")
    print(f"[prep_bag] small-stream bag written: {small_output_path}")
    merge_raw_lidar_with_small_streams(input_path, small_output_path, output_path, record_topics)
    shutil.rmtree(small_output_path, ignore_errors=True)
    print(f"[prep_bag] normalized bag written: {output_path}")
    print(f"[prep_bag] adapter summary: {adapter_summary_path}")


def open_bag_reader(path: Path) -> rosbag2_py.SequentialReader:
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(path), storage_id="mcap"),
        rosbag2_py.ConverterOptions(
            input_serialization_format="cdr",
            output_serialization_format="cdr",
        ),
    )
    return reader


def clone_topic_metadata(meta) -> rosbag2_py.TopicMetadata:
    return rosbag2_py.TopicMetadata(
        0,
        meta.name,
        meta.type,
        meta.serialization_format,
        getattr(meta, "offered_qos_profiles", []),
        getattr(meta, "type_description_hash", ""),
    )


def read_next(
    reader: rosbag2_py.SequentialReader,
    *,
    use_header_stamp_as_log_time: bool = False,
) -> Optional[Tuple[int, str, bytes]]:
    if not reader.has_next():
        return None
    topic, data, log_time = reader.read_next()
    if use_header_stamp_as_log_time:
        log_time = int(round(serialized_header_stamp_sec(data) * 1e9))
    return int(log_time), topic, data


def merge_raw_lidar_with_small_streams(
    raw_bag: Path,
    small_bag: Path,
    output_bag: Path,
    small_topics: Sequence[str],
) -> None:
    """Create the final normalized bag without replaying LiDAR.

    LiDAR messages are copied byte-for-byte from the raw bag with their
    original rosbag log_time. Only the Atlas-derived small streams come from
    the adapter recording. The temporary small-stream bag uses wall-time
    recording so it can subscribe before the first simulated clock tick; this
    final merge writes each small-stream message at its own header stamp.
    """
    print("[prep_bag] merging raw LiDAR messages with adapter small streams ...")
    ensure_output_path(output_bag, True, "merged normalized output bag")

    raw_reader = open_bag_reader(raw_bag)
    small_reader = open_bag_reader(small_bag)
    raw_meta = {meta.name: meta for meta in raw_reader.get_all_topics_and_types()}
    small_meta = {meta.name: meta for meta in small_reader.get_all_topics_and_types()}

    topics = list(LIDAR_TOPICS) + list(small_topics)
    missing = [topic for topic in LIDAR_TOPICS if topic not in raw_meta]
    missing += [topic for topic in small_topics if topic not in small_meta]
    if missing:
        die("cannot merge normalized bag; missing topics: " + ", ".join(missing))

    raw_reader.set_filter(rosbag2_py.StorageFilter(topics=list(LIDAR_TOPICS)))
    small_reader.set_filter(rosbag2_py.StorageFilter(topics=list(small_topics)))

    writer = rosbag2_py.SequentialWriter()
    writer.open(
        rosbag2_py.StorageOptions(uri=str(output_bag), storage_id="mcap"),
        rosbag2_py.ConverterOptions(
            input_serialization_format="cdr",
            output_serialization_format="cdr",
        ),
    )
    for topic in topics:
        meta = raw_meta.get(topic) or small_meta[topic]
        writer.create_topic(clone_topic_metadata(meta))

    counts = {topic: 0 for topic in topics}
    raw_next = read_next(raw_reader)
    small_next = read_next(small_reader, use_header_stamp_as_log_time=True)
    while raw_next is not None or small_next is not None:
        use_raw = small_next is None or (raw_next is not None and raw_next[0] <= small_next[0])
        if use_raw:
            assert raw_next is not None
            log_time, topic, data = raw_next
            writer.write(topic, data, log_time)
            counts[topic] += 1
            raw_next = read_next(raw_reader)
        else:
            assert small_next is not None
            log_time, topic, data = small_next
            writer.write(topic, data, log_time)
            counts[topic] += 1
            small_next = read_next(small_reader, use_header_stamp_as_log_time=True)
    writer.close()

    for topic in topics:
        print(f"[prep_bag]   merged {topic}: {counts[topic]}")


def compute_nearest_deltas(reference: Sequence[float], queries: Sequence[float]) -> List[float]:
    if not reference or not queries:
        return []
    ref = sorted(reference)
    out: List[float] = []
    import bisect

    for query in queries:
        idx = bisect.bisect_left(ref, query)
        candidates = []
        if idx < len(ref):
            candidates.append(abs(ref[idx] - query))
        if idx > 0:
            candidates.append(abs(ref[idx - 1] - query))
        if candidates:
            out.append(min(candidates))
    return out


def read_topic_timeline(bag_path: Path, topics: Sequence[str]) -> Dict[str, List[Tuple[float, float]]]:
    timeline = {topic: [] for topic in topics}
    reader = open_bag_reader(bag_path)
    filt = rosbag2_py.StorageFilter(topics=list(topics))
    reader.set_filter(filt)
    while reader.has_next():
        topic, data, log_time = reader.read_next()
        if topic in timeline:
            timeline[topic].append((serialized_header_stamp_sec(data), float(log_time) * 1e-9))
    return timeline


def compare_raw_lidar_timeline(raw_bag: Path, normalized_bag: Path, report_path: Path) -> bool:
    print("[prep_bag] comparing raw vs normalized LiDAR header timelines ...")
    raw = read_topic_timeline(raw_bag, LIDAR_TOPICS)
    normalized = read_topic_timeline(normalized_bag, LIDAR_TOPICS)

    ok = True
    lines = ["[prep_bag] raw vs normalized LiDAR timeline check"]
    for topic in LIDAR_TOPICS:
        raw_t = raw[topic]
        norm_t = normalized[topic]
        lines.append(f"[prep_bag]   {topic}: raw_count={len(raw_t)} normalized_count={len(norm_t)}")
        if len(raw_t) != len(norm_t):
            lines.append("[prep_bag]     ERROR: LiDAR message count changed")
            ok = False
            continue
        if not raw_t:
            lines.append("[prep_bag]     ERROR: no LiDAR messages found")
            ok = False
            continue

        header_diffs = [abs(a[0] - b[0]) for a, b in zip(raw_t, norm_t)]
        log_diffs = [b[1] - a[1] for a, b in zip(raw_t, norm_t)]
        max_header_diff = max(header_diffs)
        lines.append(
            f"[prep_bag]     header.stamp abs diff: {summarize_seconds(header_diffs)}"
        )
        lines.append(
            f"[prep_bag]     bag log_time normalized-raw: {summarize_seconds(log_diffs)}"
        )
        if max_header_diff > 1e-9:
            lines.append(
                f"[prep_bag]     ERROR: LiDAR header timeline changed by {max_header_diff:.9f}s"
            )
            ok = False

    lines.append(
        "[prep_bag]   NOTE: GLIM/GICP sensor synchronization uses message header/per-point "
        "times. rosbag storage log_time may change when a bag is played and recorded again; "
        "that affects playback pacing/start-offset style controls, not the LiDAR measurement stamp."
    )
    report_path.parent.mkdir(parents=True, exist_ok=True)
    report_path.write_text("\n".join(lines) + "\n", encoding="utf-8")
    for line in lines:
        print(line)
    print(f"[prep_bag] raw/normalized timeline report: {report_path}")
    return ok


def check_normalized_bag(args: argparse.Namespace, bag_path: Path, report_path: Path) -> bool:
    print("[prep_bag] checking normalized bag topic counts and timing ...")
    topics_to_check = LIDAR_TOPICS + SMALL_TOPICS
    if args.record_gnss_pose:
        topics_to_check += OPTIONAL_GNSS_TOPICS

    stats = {topic: TopicStats(topic) for topic in topics_to_check}
    header_samples = {topic: [] for topic in topics_to_check}

    reader = open_bag_reader(bag_path)
    filt = rosbag2_py.StorageFilter(topics=topics_to_check)
    reader.set_filter(filt)
    while reader.has_next():
        topic, data, log_time = reader.read_next()
        msg_type = CHECK_TOPIC_TYPES.get(topic)
        if msg_type is None:
            continue
        msg = deserialize_message(data, msg_type)
        header_sec = stamp_sec(msg.header.stamp)
        stats[topic].add(header_sec, int(log_time))
        if topic == "/gps_p1/imu" or len(header_samples[topic]) < args.nearest_sample_limit:
            header_samples[topic].append(header_sec)

    lines = ["[prep_bag] normalized bag check"]
    ok = True

    min_counts = {
        "/gps_p1/imu": args.min_imu_count,
        "/gps_p1/filtered_odom": args.min_odom_count,
        "/gps_p1/filtered_odom_rtk_fixed": args.min_rtk_count,
        "/luminar_front/points": args.min_lidar_count,
        "/luminar_left/points": args.min_lidar_count,
        "/luminar_right/points": args.min_lidar_count,
    }
    max_gap_by_topic = {
        "/gps_p1/imu": args.max_imu_gap_sec,
        "/gps_p1/filtered_odom": args.max_odom_gap_sec,
        "/gps_p1/filtered_odom_rtk_fixed": None,
        "/luminar_front/points": args.max_lidar_gap_sec,
        "/luminar_left/points": args.max_lidar_gap_sec,
        "/luminar_right/points": args.max_lidar_gap_sec,
        "/gnss": args.max_odom_gap_sec,
        "/gnss_rtk_fixed": None,
    }

    for topic in topics_to_check:
        st = stats[topic]
        min_count = min_counts.get(topic, 0)
        max_gap = max_gap_by_topic.get(topic)
        line = (
            f"[prep_bag]   {topic}: count={st.count} "
            f"header_dt=({summarize_seconds(st.header_dts)}) "
            f"log_dt=({summarize_seconds(st.log_dts)})"
        )
        lines.append(line)
        if st.count < min_count:
            lines.append(f"[prep_bag]     ERROR: count {st.count} < required {min_count}")
            ok = False
        if st.rewinds:
            lines.append(f"[prep_bag]     ERROR: {st.rewinds} non-increasing header.stamp intervals")
            ok = False
        if max_gap is not None and st.header_dts and max(st.header_dts) > max_gap:
            lines.append(
                f"[prep_bag]     ERROR: max header gap {max(st.header_dts):.6f}s "
                f"> threshold {max_gap:.6f}s"
            )
            ok = False

    imu_samples = header_samples.get("/gps_p1/imu", [])
    front_lidar_samples = header_samples.get("/luminar_front/points", [])
    in_coverage_lidar_samples = front_lidar_samples
    outside_imu_coverage = 0
    if imu_samples:
        imu_first = min(imu_samples)
        imu_last = max(imu_samples)
        in_coverage_lidar_samples = [
            sample for sample in front_lidar_samples if imu_first <= sample <= imu_last
        ]
        outside_imu_coverage = len(front_lidar_samples) - len(in_coverage_lidar_samples)
    nearest = compute_nearest_deltas(imu_samples, in_coverage_lidar_samples)
    lines.append(
        "[prep_bag]   nearest /gps_p1/imu to /luminar_front/points: "
        f"{summarize_seconds(nearest)}"
    )
    if outside_imu_coverage:
        lines.append(
            "[prep_bag]     NOTE: "
            f"{outside_imu_coverage} sampled front LiDAR frame(s) are outside IMU coverage"
        )
    if nearest and max(nearest) > args.max_lidar_imu_nearest_sec:
        lines.append(
            f"[prep_bag]     ERROR: nearest IMU/LiDAR delta {max(nearest):.6f}s "
            f"> threshold {args.max_lidar_imu_nearest_sec:.6f}s"
        )
        ok = False

    report_path.parent.mkdir(parents=True, exist_ok=True)
    report_path.write_text("\n".join(lines) + "\n", encoding="utf-8")
    for line in lines:
        print(line)
    print(f"[prep_bag] normalized bag check report: {report_path}")
    return ok


def run_glim(args: argparse.Namespace, normalized_bag: Path, dump_dir: Path, work_dir: Path) -> None:
    require_ros_package("glim_ros")
    if dump_dir.exists() and args.force:
        shutil.rmtree(dump_dir)
    dump_dir.mkdir(parents=True, exist_ok=True)

    log_path = dump_dir / "glim_rosbag.log"
    cmd = [
        "ros2",
        "run",
        "glim_ros",
        "glim_rosbag",
        str(normalized_bag),
        "--ros-args",
        "-p",
        f"dump_path:={dump_dir}",
    ]
    print("[prep_bag] running GLIM:", " ".join(cmd))
    with log_path.open("w", encoding="utf-8", errors="replace") as log_file:
        proc = subprocess.Popen(
            cmd,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            bufsize=1,
        )
        assert proc.stdout is not None
        for line in proc.stdout:
            sys.stdout.write(line)
            log_file.write(line)
        rc = proc.wait()
    print(f"[prep_bag] GLIM log: {log_path}")
    if rc != 0:
        die(f"glim_rosbag exited with code {rc}; see {log_path}")

    symptom_report = dump_dir / "glim_rosbag_symptoms.txt"
    rg = shutil.which("rg")
    if rg:
        result = subprocess.run(
            [rg, "-n", GLIM_SYMPTOM_PATTERN, str(log_path)],
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
        )
        symptom_text = result.stdout
    else:
        compiled = re.compile(GLIM_SYMPTOM_PATTERN)
        matches = []
        for lineno, line in enumerate(log_path.read_text(errors="replace").splitlines(), start=1):
            if compiled.search(line):
                matches.append(f"{lineno}:{line}")
        symptom_text = "\n".join(matches) + ("\n" if matches else "")

    symptom_report.write_text(symptom_text, encoding="utf-8")
    if symptom_text:
        print("[prep_bag] GLIM symptom grep matched:")
        print(symptom_text, end="")
        print(f"[prep_bag] symptom report: {symptom_report}")
        if not args.allow_glim_symptoms:
            die("GLIM log contained timing/solver/crash symptoms")
    else:
        print(f"[prep_bag] GLIM symptom grep clean: {symptom_report}")


def parse_args(argv: Optional[Sequence[str]] = None) -> argparse.Namespace:
    ap = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    ap.add_argument("--input", required=True, help="raw rosbag2 directory or .mcap file")
    ap.add_argument("--output", required=True, help="normalized rosbag2 output directory")
    ap.add_argument("--dump-dir", default="", help="GLIM dump dir; default: <output>_glim_dump")
    ap.add_argument("--work-dir", default="", help="prep logs/params dir; default: <output>_prep_logs")
    ap.add_argument("--p1-imu-pcap", default="", help="Point One INS/IMU PCAP; auto-discovered if omitted")
    ap.add_argument("--skip-normalize", action="store_true", help="use an existing --output bag")
    ap.add_argument("--skip-glim", action="store_true", help="only normalize and check the bag")
    ap.add_argument("--force", action="store_true", help="remove existing output/dump paths created by this script")

    ap.add_argument("--rate", type=float, default=1.0, help="ros2 bag play and PCAP replay rate")
    ap.add_argument("--clock-hz", type=float, default=200.0, help="ros2 bag play --clock frequency")
    ap.add_argument("--startup-delay", type=float, default=3.0, help="seconds to let recorder/nodes discover topics")
    ap.add_argument("--post-play-drain-sec", type=float, default=3.0, help="seconds to drain final messages after play")
    ap.add_argument("--read-ahead-queue-size", type=int, default=10000)
    ap.add_argument("--pcap-max-clock-lag-sec", type=float, default=1.0)

    ap.add_argument("--pose-input-topic", default=POSE_IN)
    ap.add_argument("--imu-frame-id", default="gps_antenna_top")
    ap.add_argument("--body-frame-id", default="gps_antenna_top")
    ap.add_argument("--odom-frame-id", default="map")
    ap.add_argument("--local-enu-origin", default="")
    ap.add_argument("--local-enu-origin-ttl-path", default="")
    ap.add_argument("--rtk-max-var-xy", type=float, default=1e-3)
    ap.add_argument("--rtk-max-var-z", type=float, default=5e-3)
    ap.add_argument("--p1-clock-bin-seconds", type=float, default=1.0e12)
    ap.add_argument("--p1-like-threshold-sec", type=float, default=100000000.0)
    ap.add_argument("--record-gnss-pose", action="store_true", help="also record /gnss and /gnss_rtk_fixed")

    ap.add_argument("--min-imu-count", type=int, default=100)
    ap.add_argument("--min-odom-count", type=int, default=10)
    ap.add_argument("--min-rtk-count", type=int, default=1)
    ap.add_argument("--min-lidar-count", type=int, default=10)
    ap.add_argument("--max-imu-gap-sec", type=float, default=0.1)
    ap.add_argument("--max-odom-gap-sec", type=float, default=0.5)
    ap.add_argument("--max-lidar-gap-sec", type=float, default=0.5)
    ap.add_argument("--max-lidar-imu-nearest-sec", type=float, default=1.0)
    ap.add_argument("--nearest-sample-limit", type=int, default=5000)
    ap.add_argument("--skip-raw-lidar-timeline-compare", action="store_true")
    ap.add_argument("--allow-glim-symptoms", action="store_true")
    return ap.parse_args(argv)


def main(argv: Optional[Sequence[str]] = None) -> int:
    try:
        args = parse_args(argv)
        input_path = resolve_existing_path(args.input, "--input")
        output_path = Path(args.output).expanduser().resolve()
        dump_dir = (
            Path(args.dump_dir).expanduser().resolve()
            if args.dump_dir
            else output_path.with_name(output_path.name + "_glim_dump")
        )
        work_dir = (
            Path(args.work_dir).expanduser().resolve()
            if args.work_dir
            else output_path.with_name(output_path.name + "_prep_logs")
        )

        if args.rate <= 0.0:
            die("--rate must be > 0")
        if args.clock_hz <= 0.0:
            die("--clock-hz must be > 0")

        if not args.skip_normalize:
            normalize_bag(args, input_path, output_path, work_dir)
        elif not output_path.exists():
            die(f"--skip-normalize was set but --output does not exist: {output_path}")

        check_report = work_dir / "normalized_bag_check.txt"
        if not check_normalized_bag(args, output_path, check_report):
            die("normalized bag check failed; GLIM was not run")
        if not args.skip_raw_lidar_timeline_compare:
            timeline_report = work_dir / "raw_vs_normalized_lidar_timeline.txt"
            if not compare_raw_lidar_timeline(input_path, output_path, timeline_report):
                die("raw-vs-normalized LiDAR timeline check failed; GLIM was not run")

        if not args.skip_glim:
            run_glim(args, output_path, dump_dir, work_dir)
        else:
            print("[prep_bag] --skip-glim set; stopping after normalized bag check")

        print("[prep_bag] done")
        return 0
    except PrepError as exc:
        print(f"[prep_bag] ERROR: {exc}", file=sys.stderr)
        return 2
    except KeyboardInterrupt:
        print("[prep_bag] interrupted", file=sys.stderr)
        return 130


if __name__ == "__main__":
    raise SystemExit(main())
