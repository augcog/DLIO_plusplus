#!/usr/bin/env python3
"""Live DLIO input-adapter smoke checker.

This script subscribes to raw Atlas topics and normalized adapter outputs for a
short live hardware window, then writes a JSON report. It is intentionally a
runtime smoke gate, not a bag rewriter or offline evaluator.
"""

from __future__ import annotations

import argparse
import json
import math
import time
from pathlib import Path
from statistics import median
from typing import Any

from validate_luminar_timestamps import summarize_cloud


RAW_IMU_TOPIC = "/atlas/imu_calibrated"
RAW_POSE_TOPIC = "/atlas/pose_filtered"
IMU_TOPIC = "/gps_p1/imu"
ODOM_TOPIC = "/gps_p1/filtered_odom"
RTK_ODOM_TOPIC = "/gps_p1/filtered_odom_rtk_fixed"
MAP_ODOM_TOPIC = "/gps_p1/filtered_odom_map"
LIDAR_TOPICS = (
    "/luminar_front/points",
    "/luminar_left/points",
    "/luminar_right/points",
)


def stamp_sec(stamp: Any) -> float:
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


def stamp_ns(stamp: Any) -> int:
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def topic_stats(stamps: list[float]) -> dict[str, Any]:
    if not stamps:
        return {
            "count": 0,
            "monotonic": False,
            "duration_s": 0.0,
            "rate_hz": 0.0,
            "dt_median_s": None,
            "dt_p95_s": None,
            "dt_max_s": None,
            "non_positive_dt_count": 0,
        }
    dts = [b - a for a, b in zip(stamps, stamps[1:])]
    monotonic = all(dt > 0.0 for dt in dts)
    duration = max(0.0, stamps[-1] - stamps[0])
    rate = (len(stamps) - 1) / duration if len(stamps) > 1 and duration > 0.0 else 0.0
    out: dict[str, Any] = {
        "count": len(stamps),
        "monotonic": monotonic,
        "duration_s": duration,
        "rate_hz": rate,
        "dt_median_s": median(dts) if dts else None,
        "dt_p95_s": None,
        "dt_max_s": max(dts) if dts else None,
        "non_positive_dt_count": sum(1 for dt in dts if dt <= 0.0),
    }
    if dts:
        ordered = sorted(dts)
        out["dt_p95_s"] = ordered[min(len(ordered) - 1, int(math.ceil(0.95 * len(ordered))) - 1)]
    return out


def epoch_like(stamps: list[float]) -> bool:
    return bool(stamps) and all(1_600_000_000.0 <= s <= 2_100_000_000.0 for s in stamps)


def p1_like(stamps: list[float], threshold: float) -> bool:
    return bool(stamps) and all(0.0 < s < threshold for s in stamps)


def pass_fail(name: str, passed: bool, reason: str) -> dict[str, Any]:
    return {"name": name, "pass": bool(passed), "reason": reason}


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--duration", type=float, default=30.0)
    parser.add_argument("--json-out", type=Path, required=True)
    parser.add_argument("--expect-raw-imu-p1", action="store_true")
    parser.add_argument("--p1-like-threshold", type=float, default=100_000_000.0)
    parser.add_argument("--expected-imu-period", type=float, default=0.01)
    parser.add_argument("--imu-period-tolerance", type=float, default=0.004)
    parser.add_argument("--imu-p95-max", type=float, default=0.03)
    parser.add_argument("--min-raw-imu-count", type=int, default=100)
    parser.add_argument("--min-output-imu-count", type=int, default=100)
    parser.add_argument("--min-odom-count", type=int, default=10)
    parser.add_argument("--min-lidar-count", type=int, default=3)
    parser.add_argument("--require-rtk-fixed", action="store_true")
    parser.add_argument("--require-map-odom", action="store_true")
    parser.add_argument("--max-lidar-header-point-delta-ms", type=float, default=20.0)
    args = parser.parse_args()

    try:
        import rclpy
        from rclpy.qos import qos_profile_sensor_data
        from fusion_engine_msgs.msg import Pose
        from nav_msgs.msg import Odometry
        from sensor_msgs.msg import Imu, PointCloud2
    except ImportError as exc:
        raise SystemExit(
            "ROS 2 Python modules are not importable. Source ROS, race_common, "
            "and this workspace before running."
        ) from exc

    rclpy.init()
    node = rclpy.create_node("dlio_live_adapter_smoke_check")

    raw_imu_stamps: list[float] = []
    raw_pose_stamps: list[float] = []
    raw_pose_p1: list[float] = []
    imu_stamps: list[float] = []
    odom_stamps: list[float] = []
    rtk_stamps: list[float] = []
    map_odom_stamps: list[float] = []
    lidar_summaries: dict[str, list[dict[str, Any]]] = {topic: [] for topic in LIDAR_TOPICS}
    lidar_seen: dict[str, int] = {topic: 0 for topic in LIDAR_TOPICS}

    subscriptions = [
        node.create_subscription(Imu, RAW_IMU_TOPIC, lambda m: raw_imu_stamps.append(stamp_sec(m.header.stamp)), qos_profile_sensor_data),
        node.create_subscription(Imu, IMU_TOPIC, lambda m: imu_stamps.append(stamp_sec(m.header.stamp)), qos_profile_sensor_data),
        node.create_subscription(Odometry, ODOM_TOPIC, lambda m: odom_stamps.append(stamp_sec(m.header.stamp)), 100),
        node.create_subscription(Odometry, RTK_ODOM_TOPIC, lambda m: rtk_stamps.append(stamp_sec(m.header.stamp)), 100),
        node.create_subscription(Odometry, MAP_ODOM_TOPIC, lambda m: map_odom_stamps.append(stamp_sec(m.header.stamp)), 100),
    ]

    def pose_cb(msg: Pose) -> None:
        raw_pose_stamps.append(stamp_sec(msg.header.stamp))
        raw_pose_p1.append(float(msg.p1_time.seconds) + float(msg.p1_time.fraction_ns) * 1e-9)

    subscriptions.append(node.create_subscription(Pose, RAW_POSE_TOPIC, pose_cb, 100))

    def make_lidar_cb(topic: str):
        def cb(msg: PointCloud2) -> None:
            lidar_seen[topic] += 1
            if len(lidar_summaries[topic]) >= 20:
                return
            summary = summarize_cloud(msg, node.get_clock().now().nanoseconds, topic, 0)
            if summary:
                lidar_summaries[topic].append(summary)

        return cb

    for topic in LIDAR_TOPICS:
        subscriptions.append(
            node.create_subscription(PointCloud2, topic, make_lidar_cb(topic), qos_profile_sensor_data)
        )

    start = time.monotonic()
    next_progress = start + 5.0
    try:
        while time.monotonic() - start < args.duration:
            rclpy.spin_once(node, timeout_sec=0.1)
            now = time.monotonic()
            if now >= next_progress:
                print(
                    "[live_smoke] "
                    f"raw_imu={len(raw_imu_stamps)} imu={len(imu_stamps)} "
                    f"pose={len(raw_pose_stamps)} odom={len(odom_stamps)} "
                    + " ".join(f"{topic}={lidar_seen[topic]}" for topic in LIDAR_TOPICS),
                    flush=True,
                )
                next_progress = now + 5.0
    finally:
        publishers = {
            topic: len(node.get_publishers_info_by_topic(topic))
            for topic in [
                RAW_IMU_TOPIC,
                RAW_POSE_TOPIC,
                IMU_TOPIC,
                ODOM_TOPIC,
                RTK_ODOM_TOPIC,
                MAP_ODOM_TOPIC,
                *LIDAR_TOPICS,
            ]
        }
        for sub in subscriptions:
            node.destroy_subscription(sub)
        node.destroy_node()
        rclpy.shutdown()

    raw_imu = topic_stats(raw_imu_stamps)
    raw_pose = topic_stats(raw_pose_stamps)
    imu = topic_stats(imu_stamps)
    odom = topic_stats(odom_stamps)
    rtk = topic_stats(rtk_stamps)
    map_odom = topic_stats(map_odom_stamps)

    checks: list[dict[str, Any]] = []
    checks.append(pass_fail("raw_imu_present", raw_imu["count"] >= args.min_raw_imu_count, f"count={raw_imu['count']}"))
    checks.append(pass_fail("raw_pose_present", raw_pose["count"] >= args.min_odom_count, f"count={raw_pose['count']}"))
    checks.append(pass_fail("raw_pose_p1_present", bool(raw_pose_p1) and all(t > 0.0 for t in raw_pose_p1), f"count={len(raw_pose_p1)}"))
    if args.expect_raw_imu_p1:
        checks.append(pass_fail("raw_imu_p1_like", p1_like(raw_imu_stamps, args.p1_like_threshold), "raw IMU header.stamp is P1-like"))
    checks.append(pass_fail("output_imu_present", imu["count"] >= args.min_output_imu_count, f"count={imu['count']}"))
    checks.append(pass_fail("output_imu_epoch", epoch_like(imu_stamps), "normalized IMU stamps are ROS epoch"))
    checks.append(pass_fail("output_imu_monotonic", bool(imu["monotonic"]), f"non_positive_dt={imu['non_positive_dt_count']}"))
    imu_median = imu.get("dt_median_s")
    imu_p95 = imu.get("dt_p95_s")
    stable_imu = (
        imu_median is not None
        and abs(float(imu_median) - args.expected_imu_period) <= args.imu_period_tolerance
        and imu_p95 is not None
        and float(imu_p95) <= args.imu_p95_max
    )
    checks.append(pass_fail("output_imu_period_stable", stable_imu, f"median={imu_median} p95={imu_p95}"))
    checks.append(pass_fail("odom_present", odom["count"] >= args.min_odom_count, f"count={odom['count']}"))
    checks.append(pass_fail("odom_epoch", epoch_like(odom_stamps), "normalized odom stamps are ROS epoch"))
    checks.append(pass_fail("odom_monotonic", bool(odom["monotonic"]), f"non_positive_dt={odom['non_positive_dt_count']}"))
    checks.append(pass_fail("rtk_fixed_present", (rtk["count"] > 0) or not args.require_rtk_fixed, f"count={rtk['count']}"))
    checks.append(pass_fail("map_odom_present", (map_odom["count"] > 0) or not args.require_map_odom, f"count={map_odom['count']}"))

    lidar_results: dict[str, Any] = {}
    max_delta_ns = int(args.max_lidar_header_point_delta_ms * 1e6)
    for topic in LIDAR_TOPICS:
        summaries = lidar_summaries[topic]
        supported = [s for s in summaries if s.get("supported")]
        epoch_all = bool(supported) and all(s.get("epoch_plausible") for s in supported)
        delta_ok = bool(supported) and all(
            abs(int(s.get("header_minus_min_point_ns", 10**18))) <= max_delta_ns for s in supported
        )
        span_ok = bool(supported) and all(int(s.get("span_ns", 0)) > 1_000_000 for s in supported)
        count_ok = lidar_seen[topic] >= args.min_lidar_count
        lidar_results[topic] = {
            "seen_count": lidar_seen[topic],
            "sampled_count": len(summaries),
            "epoch_all_sampled": epoch_all,
            "header_point_delta_ok": delta_ok,
            "span_ok": span_ok,
            "first_summary": summaries[0] if summaries else None,
        }
        checks.append(pass_fail(f"{topic}_present", count_ok, f"count={lidar_seen[topic]}"))
        checks.append(pass_fail(f"{topic}_timestamp_epoch", epoch_all, "sampled point timestamps are ROS epoch"))
        checks.append(pass_fail(f"{topic}_header_point_alignment", delta_ok, f"max_abs_delta_ns={max_delta_ns}"))
        checks.append(pass_fail(f"{topic}_span_not_collapsed", span_ok, "sampled scan spans > 1 ms"))

    result = {
        "duration_s": args.duration,
        "publishers": publishers,
        "topics": {
            RAW_IMU_TOPIC: raw_imu,
            RAW_POSE_TOPIC: raw_pose,
            IMU_TOPIC: imu,
            ODOM_TOPIC: odom,
            RTK_ODOM_TOPIC: rtk,
            MAP_ODOM_TOPIC: map_odom,
        },
        "raw_pose_p1": {
            "count": len(raw_pose_p1),
            "first": raw_pose_p1[0] if raw_pose_p1 else None,
            "last": raw_pose_p1[-1] if raw_pose_p1 else None,
            "monotonic": all((b - a) > 0.0 for a, b in zip(raw_pose_p1, raw_pose_p1[1:])),
        },
        "lidar": lidar_results,
        "checks": checks,
        "overall_pass": all(c["pass"] for c in checks),
    }

    args.json_out.parent.mkdir(parents=True, exist_ok=True)
    args.json_out.write_text(json.dumps(result, indent=2, sort_keys=True) + "\n")

    for check in checks:
        status = "PASS" if check["pass"] else "FAIL"
        print(f"{status} {check['name']}: {check['reason']}")
    print(f"json_out={args.json_out}")
    raise SystemExit(0 if result["overall_pass"] else 1)


if __name__ == "__main__":
    main()
