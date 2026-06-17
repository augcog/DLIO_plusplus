#!/usr/bin/env python3
"""Summarize normalized DLIO bag timing and topic-contract metrics.

This is a lightweight Stage-2/reference helper. It reads an existing rosbag2
directory, counts normalized DLIO topics, computes header-stamp timing stats
for IMU/odom streams, and samples Luminar PointCloud2 per-point timestamp spans
without writing another large bag.
"""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
from statistics import median
from typing import Any

import rosbag2_py
import yaml
from nav_msgs.msg import Odometry
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import Imu, PointCloud2

from validate_luminar_timestamps import summarize_cloud


IMU_TOPIC = "/gps_p1/imu"
ODOM_TOPIC = "/gps_p1/filtered_odom"
RTK_ODOM_TOPIC = "/gps_p1/filtered_odom_rtk_fixed"
MAP_ODOM_TOPIC = "/gps_p1/filtered_odom_map"
LIDAR_TOPICS = (
    "/luminar_front/points",
    "/luminar_left/points",
    "/luminar_right/points",
)
TOPICS = (IMU_TOPIC, ODOM_TOPIC, RTK_ODOM_TOPIC, MAP_ODOM_TOPIC, *LIDAR_TOPICS)


def stamp_sec(stamp: Any) -> float:
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


def percentile(values: list[float], q: float) -> float | None:
    if not values:
        return None
    ordered = sorted(values)
    idx = min(len(ordered) - 1, max(0, int(math.ceil(q * len(ordered))) - 1))
    return ordered[idx]


def stamp_stats(stamps: list[float]) -> dict[str, Any]:
    dts = [b - a for a, b in zip(stamps, stamps[1:])]
    duration = max(0.0, stamps[-1] - stamps[0]) if stamps else 0.0
    return {
        "count": len(stamps),
        "first_stamp_s": stamps[0] if stamps else None,
        "last_stamp_s": stamps[-1] if stamps else None,
        "duration_s": duration,
        "monotonic_strict": all(dt > 0.0 for dt in dts),
        "non_positive_dt_count": sum(1 for dt in dts if dt <= 0.0),
        "rate_hz": ((len(stamps) - 1) / duration) if len(stamps) > 1 and duration > 0.0 else 0.0,
        "dt_ms_median": median(dts) * 1e3 if dts else None,
        "dt_ms_p05": percentile(dts, 0.05) * 1e3 if dts else None,
        "dt_ms_p95": percentile(dts, 0.95) * 1e3 if dts else None,
        "dt_ms_max": max(dts) * 1e3 if dts else None,
    }


def summarize_lidar(samples: list[dict[str, Any]]) -> dict[str, Any]:
    supported = [s for s in samples if s.get("supported")]
    spans = [float(s["span_ms"]) for s in supported if "span_ms" in s]
    header_deltas = [int(s["header_minus_min_point_ns"]) for s in supported if "header_minus_min_point_ns" in s]
    return {
        "sampled_count": len(samples),
        "supported_sampled_count": len(supported),
        "epoch_plausible_all_sampled": bool(supported) and all(bool(s.get("epoch_plausible")) for s in supported),
        "collapsed_count_sampled": sum(1 for s in supported if bool(s.get("collapsed"))),
        "span_ms_median": median(spans) if spans else None,
        "span_ms_min": min(spans) if spans else None,
        "span_ms_max": max(spans) if spans else None,
        "header_minus_min_point_ns_median": median(header_deltas) if header_deltas else None,
        "header_minus_min_point_ns_max_abs": max((abs(v) for v in header_deltas), default=None),
        "first_sample": samples[0] if samples else None,
    }


def open_reader(bag: Path, storage_id: str) -> rosbag2_py.SequentialReader:
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(bag), storage_id=storage_id),
        rosbag2_py.ConverterOptions(input_serialization_format="cdr", output_serialization_format="cdr"),
    )
    return reader


def read_metadata(bag: Path) -> dict[str, Any]:
    meta_path = bag / "metadata.yaml"
    if not meta_path.exists():
        return {}
    data = yaml.safe_load(meta_path.read_text()) or {}
    info = data.get("rosbag2_bagfile_information", {})
    topic_counts = {}
    topic_types = {}
    for entry in info.get("topics_with_message_count", []) or []:
        topic_meta = entry.get("topic_metadata", {})
        name = topic_meta.get("name")
        if not name:
            continue
        topic_counts[name] = int(entry.get("message_count", 0))
        topic_types[name] = topic_meta.get("type", "")
    duration_ns = int((info.get("duration") or {}).get("nanoseconds", 0))
    starting_ns = int((info.get("starting_time") or {}).get("nanoseconds_since_epoch", 0))
    return {
        "storage_identifier": info.get("storage_identifier"),
        "duration_s": duration_ns * 1e-9,
        "starting_time_ns": starting_ns,
        "message_count": int(info.get("message_count", 0)),
        "topic_counts": topic_counts,
        "topic_types": topic_types,
    }


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bag", required=True, type=Path)
    parser.add_argument("--storage-id", default="mcap")
    parser.add_argument("--json-out", type=Path)
    parser.add_argument("--max-lidar-samples", type=int, default=50)
    parser.add_argument("--lidar-sample-every", type=int, default=1000)
    parser.add_argument(
        "--max-read-messages",
        type=int,
        default=0,
        help="Stop after this many matching messages. Counts still come from metadata.yaml when available.",
    )
    args = parser.parse_args()

    metadata = read_metadata(args.bag)
    reader = open_reader(args.bag, args.storage_id)
    available = {t.name: t.type for t in reader.get_all_topics_and_types()}
    missing = [topic for topic in TOPICS if topic not in available and topic != MAP_ODOM_TOPIC]

    stamps: dict[str, list[float]] = {
        IMU_TOPIC: [],
        ODOM_TOPIC: [],
        RTK_ODOM_TOPIC: [],
        MAP_ODOM_TOPIC: [],
    }
    lidar_counts = {topic: 0 for topic in LIDAR_TOPICS}
    lidar_samples: dict[str, list[dict[str, Any]]] = {topic: [] for topic in LIDAR_TOPICS}
    sample_counts = {topic: 0 for topic in TOPICS}
    read_messages = 0
    read_truncated = False

    while reader.has_next():
        topic, data, log_time_ns = reader.read_next()
        if topic not in TOPICS:
            continue
        read_messages += 1
        sample_counts[topic] += 1
        if topic == IMU_TOPIC:
            msg = deserialize_message(data, Imu)
            stamps[topic].append(stamp_sec(msg.header.stamp))
        elif topic in (ODOM_TOPIC, RTK_ODOM_TOPIC, MAP_ODOM_TOPIC):
            msg = deserialize_message(data, Odometry)
            stamps[topic].append(stamp_sec(msg.header.stamp))
        elif topic in LIDAR_TOPICS:
            lidar_counts[topic] += 1
            should_sample = (
                len(lidar_samples[topic]) < args.max_lidar_samples
                and (lidar_counts[topic] == 1 or lidar_counts[topic] % max(1, args.lidar_sample_every) == 0)
            )
            if should_sample:
                msg = deserialize_message(data, PointCloud2)
                summary = summarize_cloud(msg, int(log_time_ns), topic, 0)
                if summary:
                    lidar_samples[topic].append(summary)
        if args.max_read_messages > 0 and read_messages >= args.max_read_messages:
            read_truncated = reader.has_next()
            break

    metadata_counts = {topic: int((metadata.get("topic_counts") or {}).get(topic, 0)) for topic in TOPICS}
    counts = {
        topic: metadata_counts.get(topic, 0) or sample_counts.get(topic, 0)
        for topic in TOPICS
    }

    result = {
        "bag": str(args.bag),
        "storage_id": args.storage_id,
        "metadata": metadata,
        "available_topics": available,
        "missing_required_topics": missing,
        "counts": counts,
        "metadata_counts": metadata_counts,
        "sample_counts": sample_counts,
        "read_messages": read_messages,
        "read_truncated": read_truncated,
        "streams": {topic: stamp_stats(values) for topic, values in stamps.items()},
        "lidar": {
            topic: {
                "count": counts[topic],
                "sample_seen_count": lidar_counts[topic],
                **summarize_lidar(lidar_samples[topic]),
            }
            for topic in LIDAR_TOPICS
        },
    }

    text = json.dumps(result, indent=2, sort_keys=True)
    if args.json_out:
        args.json_out.parent.mkdir(parents=True, exist_ok=True)
        args.json_out.write_text(text + "\n")
    print(text)


if __name__ == "__main__":
    main()
