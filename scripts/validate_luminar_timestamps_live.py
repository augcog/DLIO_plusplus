#!/usr/bin/env python3
"""Validate live Luminar PointCloud2 timestamp bytes from ROS 2 topics.

This is the live-sensor companion to validate_luminar_timestamps.py. It
subscribes to the front/left/right Luminar PointCloud2 streams, reuses the same
byte-level checks as the bag validator, and writes a JSON artifact suitable for
the validation answer report.

Run from a sourced ROS 2 Jazzy shell, for example:

    source /opt/ros/jazzy/setup.bash
    source install/setup.bash
    python3 scripts/validate_luminar_timestamps_live.py --duration 65 \
      --ptp-lock-confirmed \
      --json-out dlio_data/luminar_timestamp_validation/live_timestamp_check.json

The script cannot infer Atlas/PTP lock state from PointCloud2 alone. Use
--ptp-lock-confirmed only after checking the receiver/sensor status UI or
status topic during the capture.
"""

from __future__ import annotations

import argparse
import json
import time
from pathlib import Path
from statistics import median
from typing import Any

from validate_luminar_timestamps import (
    TOPICS,
    nearest,
    print_summary,
    schema,
    schema_key,
    stamp_ns,
    summarize_cloud,
    verdict_topic,
)


def build_live_result(
    per_topic_clouds: dict[str, list[dict[str, Any]]],
    first_schema: dict[str, tuple[tuple[str, int, int, int], ...]],
    first_schema_print: dict[str, list[dict[str, Any]]],
    topics: tuple[str, ...],
    merge_threshold_s: float,
    min_merge_checks: int,
    ptp_lock_confirmed: bool,
    ros_topics: list[tuple[str, list[str]]],
    publisher_counts: dict[str, int],
) -> dict[str, Any]:
    result: dict[str, Any] = {
        "bag": "live_ros2",
        "source": "live_ros2",
        "topics": {},
        "schema_equal": None,
        "apply_header_shift": False,
        "global_shift_ns": 0,
        "ptp_lock_confirmed": bool(ptp_lock_confirmed),
        "ptp_status": "operator_confirmed" if ptp_lock_confirmed else "not_confirmed",
        "ros_graph_topics": [{"name": name, "types": types} for name, types in ros_topics],
        "publisher_counts": publisher_counts,
    }

    schema_values = list(first_schema.values())
    result["schema_equal"] = bool(schema_values and all(s == schema_values[0] for s in schema_values))

    for topic in topics:
        clouds = sorted(per_topic_clouds.get(topic, []), key=lambda c: int(c["header_ns"]))
        if not clouds:
            result["topics"][topic] = {"cloud_count": 0, "pass": False}
            continue

        spans = [int(c["span_ns"]) for c in clouds if c.get("supported")]
        topic_result = {
            "cloud_count": len(clouds),
            "schema": first_schema_print.get(topic, []),
            "first_cloud": clouds[0],
            "example_rollover_clouds": [c for c in clouds if c.get("second_rollover_cloud")][:3],
            "span_ns_min": min(spans),
            "span_ns_median": int(median(spans)),
            "span_ns_max": max(spans),
            "span_ms_median": median([s / 1e6 for s in spans]),
            "epoch_plausible_count": sum(1 for c in clouds if c.get("epoch_plausible")),
            "collapsed_count": sum(1 for c in clouds if c.get("collapsed")),
            "second_rollover_clouds": sum(1 for c in clouds if c.get("second_rollover_cloud")),
            "large_backward_jumps_gt_500ms": sum(
                int(c.get("large_backward_jumps_gt_500ms", 0)) for c in clouds
            ),
            "backward_jumps_gt_1ms": sum(int(c.get("backward_jumps_gt_1ms", 0)) for c in clouds),
            "detector_backward_jumps_gt_1ms": sum(
                int(c.get("detector_backward_jumps_gt_1ms", 0)) for c in clouds
            ),
        }

        adjacent_second_rollovers = 0
        adjacent_backward_500ms = 0
        for prev, curr in zip(clouds, clouds[1:]):
            gap_ns = int(curr["min_ns"]) - int(prev["max_ns"])
            if int(prev["max_ns"]) // 1_000_000_000 != int(curr["min_ns"]) // 1_000_000_000:
                adjacent_second_rollovers += 1
            if gap_ns < -500_000_000:
                adjacent_backward_500ms += 1
        topic_result["adjacent_second_rollovers"] = adjacent_second_rollovers
        topic_result["adjacent_backward_jumps_gt_500ms"] = adjacent_backward_500ms
        topic_result["verdict"] = verdict_topic(topic_result)
        topic_result["pass"] = topic_result["verdict"]["pass"]
        result["topics"][topic] = topic_result

    front = sorted(per_topic_clouds.get("/luminar_front/points", []), key=lambda c: int(c["header_ns"]))
    aux_topics = [t for t in topics if t != "/luminar_front/points"]
    merge_checks = []
    merge_skipped_checks = []
    for front_cloud in front:
        near_boundary = (
            front_cloud.get("second_rollover_cloud")
            or (int(front_cloud["min_ns"]) % 1_000_000_000) < 60_000_000
            or (int(front_cloud["max_ns"]) % 1_000_000_000) > 940_000_000
        )
        if not near_boundary:
            continue
        merged = [front_cloud]
        skip_reason = ""
        aux_dts = {}
        for aux_topic in aux_topics:
            aux_clouds = sorted(per_topic_clouds.get(aux_topic, []), key=lambda c: int(c["header_ns"]))
            aux = nearest(aux_clouds, int(front_cloud["header_ns"]))
            if aux is None:
                skip_reason = f"missing nearest {aux_topic}"
                continue
            dt_ns = abs(int(aux["header_ns"]) - int(front_cloud["header_ns"]))
            aux_dts[aux_topic] = dt_ns
            if dt_ns > merge_threshold_s * 1e9:
                skip_reason = f"{aux_topic} nearest dt {dt_ns / 1e6:.3f} ms exceeds threshold"
            merged.append(aux)
        if len(merged) == 1 or skip_reason:
            merge_skipped_checks.append(
                {
                    "front_header_ns": int(front_cloud["header_ns"]),
                    "reason": skip_reason or "no aux cloud",
                    "aux_header_dt_ns": aux_dts,
                    "near_second_boundary": True,
                }
            )
            continue
        combined_span = max(int(c["max_ns"]) for c in merged) - min(int(c["min_ns"]) for c in merged)
        merge_checks.append(
            {
                "front_header_ns": int(front_cloud["header_ns"]),
                "ok": bool(combined_span <= int((merge_threshold_s + 0.12) * 1e9)),
                "aux_header_dt_ns": aux_dts,
                "combined_span_ns": int(combined_span),
                "combined_span_ms": combined_span / 1e6,
                "crosses_second": bool(
                    min(int(c["min_ns"]) for c in merged) // 1_000_000_000
                    != max(int(c["max_ns"]) for c in merged) // 1_000_000_000
                ),
                "near_second_boundary": True,
            }
        )
        if len(merge_checks) >= 10:
            break
    result["merge_checks"] = merge_checks
    result["merge_skipped_checks"] = merge_skipped_checks
    result["merge_pass"] = bool(
        result["schema_equal"]
        and len(merge_checks) >= min_merge_checks
        and all(c["ok"] for c in merge_checks)
    )
    result["overall_pass"] = bool(
        result["schema_equal"]
        and result["merge_pass"]
        and all(t.get("pass") for t in result["topics"].values())
        and result["ptp_lock_confirmed"]
    )
    return result


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--topics", nargs="+", default=list(TOPICS))
    parser.add_argument("--duration", type=float, default=65.0, help="Seconds to collect live clouds.")
    parser.add_argument("--merge-threshold", type=float, default=0.05)
    parser.add_argument(
        "--min-merge-checks",
        type=int,
        default=3,
        help="Minimum coherent near-boundary front/aux pairings required for live merge PASS.",
    )
    parser.add_argument(
        "--sample-period",
        type=float,
        default=0.2,
        help="Keep at most one non-boundary cloud per topic per this many seconds.",
    )
    parser.add_argument(
        "--boundary-window",
        type=float,
        default=0.08,
        help="Always keep clouds whose header stamp is within this many seconds of a UTC second boundary.",
    )
    parser.add_argument("--json-out", type=Path)
    parser.add_argument(
        "--ptp-lock-confirmed",
        action="store_true",
        help="Record that Atlas/PTP FIXED/locked status was verified during this live capture.",
    )
    args = parser.parse_args()

    try:
        import rclpy
        from rclpy.qos import qos_profile_sensor_data
        from sensor_msgs.msg import PointCloud2
    except ImportError as exc:
        raise SystemExit(
            "ROS 2 Python modules are not importable. Source /opt/ros/jazzy/setup.bash "
            "and install/setup.bash before running this script."
        ) from exc

    topics = tuple(args.topics)
    per_topic_clouds: dict[str, list[dict[str, Any]]] = {topic: [] for topic in topics}
    first_schema: dict[str, tuple[tuple[str, int, int, int], ...]] = {}
    first_schema_print: dict[str, list[dict[str, Any]]] = {}
    seen_counts: dict[str, int] = {topic: 0 for topic in topics}
    kept_counts: dict[str, int] = {topic: 0 for topic in topics}
    last_kept_header_ns: dict[str, int] = {}

    rclpy.init()
    node = rclpy.create_node("luminar_timestamp_live_validator")
    subscriptions = []

    def make_callback(topic: str):
        def callback(msg: PointCloud2) -> None:
            seen_counts[topic] += 1
            if topic not in first_schema:
                first_schema[topic] = schema_key(msg.fields)
                first_schema_print[topic] = schema(msg.fields)

            header_ns = stamp_ns(msg.header.stamp)
            subsec_ns = header_ns % 1_000_000_000
            boundary_ns = int(max(0.0, args.boundary_window) * 1e9)
            sample_period_ns = int(max(0.0, args.sample_period) * 1e9)
            near_boundary = subsec_ns <= boundary_ns or subsec_ns >= 1_000_000_000 - boundary_ns
            enough_gap = (
                topic not in last_kept_header_ns
                or sample_period_ns <= 0
                or header_ns - last_kept_header_ns[topic] >= sample_period_ns
            )
            if not near_boundary and not enough_gap:
                return

            summary = summarize_cloud(msg, node.get_clock().now().nanoseconds, topic, 0)
            if summary:
                per_topic_clouds[topic].append(summary)
                kept_counts[topic] += 1
                last_kept_header_ns[topic] = header_ns

        return callback

    for topic in topics:
        subscriptions.append(
            node.create_subscription(PointCloud2, topic, make_callback(topic), qos_profile_sensor_data)
        )

    start = time.monotonic()
    next_progress = start + 5.0
    try:
        while time.monotonic() - start < args.duration:
            rclpy.spin_once(node, timeout_sec=0.1)
            now = time.monotonic()
            if now >= next_progress:
                counts = ", ".join(
                    f"{topic}={kept_counts[topic]}/{seen_counts[topic]}" for topic in topics
                )
                print(f"[live_validator] elapsed={now - start:.1f}s {counts}", flush=True)
                next_progress = now + 5.0
    finally:
        ros_topics = node.get_topic_names_and_types()
        publisher_counts = {topic: len(node.get_publishers_info_by_topic(topic)) for topic in topics}
        for sub in subscriptions:
            node.destroy_subscription(sub)
        node.destroy_node()
        rclpy.shutdown()

    result = build_live_result(
        per_topic_clouds,
        first_schema,
        first_schema_print,
        topics,
        args.merge_threshold,
        args.min_merge_checks,
        args.ptp_lock_confirmed,
        ros_topics,
        publisher_counts,
    )
    result["seen_counts"] = seen_counts
    result["kept_counts"] = kept_counts
    result["sample_period_s"] = float(args.sample_period)
    result["boundary_window_s"] = float(args.boundary_window)
    result["min_merge_checks"] = int(args.min_merge_checks)
    print_summary(result)
    print("publisher_counts: " + ", ".join(f"{topic}={publisher_counts[topic]}" for topic in topics))
    print("kept_counts: " + ", ".join(f"{topic}={kept_counts[topic]}/{seen_counts[topic]}" for topic in topics))
    print(
        f"merge_skipped_checks: {len(result['merge_skipped_checks'])} "
        f"(valid_checks={len(result['merge_checks'])}, min_required={args.min_merge_checks})"
    )
    if not args.ptp_lock_confirmed:
        print(
            "\nptp_lock_confirmed: False "
            "(rerun with --ptp-lock-confirmed after checking Atlas/PTP FIXED/locked status)"
        )
    if args.json_out:
        args.json_out.parent.mkdir(parents=True, exist_ok=True)
        args.json_out.write_text(json.dumps(result, indent=2, sort_keys=True) + "\n")

    if all(result["topics"].get(topic, {}).get("cloud_count", 0) == 0 for topic in topics):
        raise SystemExit(2)
    raise SystemExit(0 if result["overall_pass"] else 1)


if __name__ == "__main__":
    main()
