#!/usr/bin/env python3
"""Extract a validation rosbag window and repair Luminar point timestamps.

Older prepared bags in this repo shifted PointCloud2 header/log time onto the
ROS/INS epoch but left Luminar UINT8[8] point timestamps on the raw PTP clock.
This tool creates a small reproducible validation bag by copying a requested
time window and adding the measured header-to-point offset to every Luminar
per-point timestamp.
"""

from __future__ import annotations

import argparse
import struct
from pathlib import Path

import rosbag2_py
from rclpy.serialization import deserialize_message, serialize_message
from sensor_msgs.msg import PointCloud2, PointField


DEFAULT_TOPICS = (
    "/luminar_front/points",
    "/luminar_left/points",
    "/luminar_right/points",
    "/gps_p1/imu",
    "/gps_p1/filtered_odom",
    "/gps_p1/filtered_odom_rtk_fixed",
)
LUMINAR_TOPICS = {
    "/luminar_front/points",
    "/luminar_left/points",
    "/luminar_right/points",
}
TIME_FIELD_NAMES = {"t", "time", "time_stamp", "timestamp"}


def stamp_ns(stamp) -> int:
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def open_reader(path: Path) -> rosbag2_py.SequentialReader:
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(path), storage_id="mcap"),
        rosbag2_py.ConverterOptions(
            input_serialization_format="cdr",
            output_serialization_format="cdr",
        ),
    )
    return reader


def luminar_time_field(msg: PointCloud2):
    for field in msg.fields:
        if (
            field.name in TIME_FIELD_NAMES
            and int(field.datatype) == int(PointField.UINT8)
            and int(field.count) == 8
        ):
            return field
    return None


def point_count(msg: PointCloud2) -> int:
    return int(msg.width) * int(msg.height)


def cloud_min_timestamp_ns(msg: PointCloud2, field) -> int:
    n = point_count(msg)
    if n <= 0:
        raise ValueError("empty cloud")
    data = memoryview(msg.data)
    point_step = int(msg.point_step)
    offset = int(field.offset)
    min_ts = None
    for i in range(n):
        raw = struct.unpack_from("<Q", data, i * point_step + offset)[0]
        min_ts = raw if min_ts is None else min(min_ts, raw)
    if min_ts is None:
        raise ValueError("empty cloud")
    return int(min_ts)


def shift_luminar_point_timestamps(msg: PointCloud2, shift_ns: int) -> bool:
    field = luminar_time_field(msg)
    if field is None:
        return False
    data = bytearray(msg.data)
    n = point_count(msg)
    point_step = int(msg.point_step)
    offset = int(field.offset)
    for i in range(n):
        pos = i * point_step + offset
        raw = struct.unpack_from("<Q", data, pos)[0]
        shifted = max(0, int(raw) + int(shift_ns))
        struct.pack_into("<Q", data, pos, shifted)
    msg.data = bytes(data)
    return True


def measure_shift_ns(input_bag: Path, start_ns: int, end_ns: int, topics: set[str]) -> int:
    reader = open_reader(input_bag)
    while reader.has_next():
        topic, raw, log_time_ns = reader.read_next()
        if topic not in topics or topic not in LUMINAR_TOPICS:
            continue
        if log_time_ns < start_ns or log_time_ns > end_ns:
            continue
        msg = deserialize_message(raw, PointCloud2)
        field = luminar_time_field(msg)
        if field is None or point_count(msg) == 0:
            continue
        return stamp_ns(msg.header.stamp) - cloud_min_timestamp_ns(msg, field)
    raise SystemExit("no Luminar PointCloud2 messages found in requested window")


def create_writer(output_bag: Path, topic_types: dict[str, str], topics: tuple[str, ...]) -> rosbag2_py.SequentialWriter:
    writer = rosbag2_py.SequentialWriter()
    writer.open(
        rosbag2_py.StorageOptions(uri=str(output_bag), storage_id="mcap"),
        rosbag2_py.ConverterOptions(
            input_serialization_format="cdr",
            output_serialization_format="cdr",
        ),
    )
    for idx, topic in enumerate(topics):
        msg_type = topic_types.get(topic)
        if msg_type is None:
            continue
        writer.create_topic(
            rosbag2_py.TopicMetadata(
                id=idx,
                name=topic,
                type=msg_type,
                serialization_format="cdr",
            )
        )
    return writer


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input", required=True, type=Path, help="source rosbag2 directory")
    parser.add_argument("--output", required=True, type=Path, help="output rosbag2 directory")
    parser.add_argument("--start-sec", required=True, type=float, help="window start epoch seconds")
    parser.add_argument("--duration-sec", required=True, type=float, help="window duration in seconds")
    parser.add_argument("--shift-ns", type=int, help="timestamp shift; measured from first selected cloud by default")
    parser.add_argument("--topics", nargs="+", default=list(DEFAULT_TOPICS))
    args = parser.parse_args()

    if args.output.exists():
        raise SystemExit(f"output already exists: {args.output}")

    topics = tuple(args.topics)
    topic_set = set(topics)
    start_ns = int(round(args.start_sec * 1e9))
    end_ns = start_ns + int(round(args.duration_sec * 1e9))

    shift_ns = args.shift_ns
    if shift_ns is None:
        shift_ns = measure_shift_ns(args.input, start_ns, end_ns, topic_set)

    probe = open_reader(args.input)
    topic_types = {t.name: t.type for t in probe.get_all_topics_and_types()}
    del probe

    missing = [topic for topic in topics if topic not in topic_types]
    if missing:
        raise SystemExit(f"missing topics in source bag: {missing}")

    writer = create_writer(args.output, topic_types, topics)
    reader = open_reader(args.input)
    counts = {topic: 0 for topic in topics}
    shifted_clouds = 0
    while reader.has_next():
        topic, raw, log_time_ns = reader.read_next()
        if topic not in topic_set:
            continue
        if log_time_ns < start_ns:
            continue
        if log_time_ns > end_ns:
            break

        if topic in LUMINAR_TOPICS:
            msg = deserialize_message(raw, PointCloud2)
            if shift_luminar_point_timestamps(msg, int(shift_ns)):
                raw = serialize_message(msg)
                shifted_clouds += 1
        writer.write(topic, raw, int(log_time_ns))
        counts[topic] += 1

    del writer

    print(f"input: {args.input}")
    print(f"output: {args.output}")
    print(f"window_ns: {start_ns}..{end_ns} ({args.duration_sec:.3f}s)")
    print(f"shift_ns: {shift_ns}")
    print(f"shifted_luminar_clouds: {shifted_clouds}")
    for topic in topics:
        print(f"{topic}: {counts[topic]}")


if __name__ == "__main__":
    main()
