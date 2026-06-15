#!/usr/bin/env python3
"""Validate Luminar PointCloud2 per-point timestamps in a rosbag2 bag.

This follows GLIM_GICP_Luminar_Timestamp_Validation.pdf:
- identify the PointCloud2 time field schema,
- interpret the same bytes as uint64 nanoseconds and float64 seconds,
- check scan span, epoch sanity, collapsed timestamps, and second rollovers,
- check front/left/right schema equality and merged-cloud time coherence.

Use --apply-header-shift for bags produced by the old pcap path where the
cloud header was shifted onto the ROS/INS epoch but UINT8[8] point timestamps
were left on the raw Luminar PTP clock.
"""

from __future__ import annotations

import argparse
import bisect
import json
import math
import struct
from pathlib import Path
from statistics import median
from typing import Any

import numpy as np
from rosbags.highlevel import AnyReader


TOPICS = (
    "/luminar_front/points",
    "/luminar_left/points",
    "/luminar_right/points",
)
TIME_FIELD_NAMES = ("t", "time", "time_stamp", "timestamp")
DTYPE_NAMES = {
    1: "INT8",
    2: "UINT8",
    3: "INT16",
    4: "UINT16",
    5: "INT32",
    6: "UINT32",
    7: "FLOAT32",
    8: "FLOAT64",
}


def stamp_ns(stamp: Any) -> int:
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def schema(fields: Any) -> list[dict[str, Any]]:
    return [
        {
            "name": f.name,
            "offset": int(f.offset),
            "datatype": int(f.datatype),
            "datatype_name": DTYPE_NAMES.get(int(f.datatype), str(f.datatype)),
            "count": int(f.count),
        }
        for f in fields
    ]


def schema_key(fields: Any) -> tuple[tuple[str, int, int, int], ...]:
    return tuple((f.name, int(f.offset), int(f.datatype), int(f.count)) for f in fields)


def find_field(fields: Any, names: tuple[str, ...]) -> Any | None:
    for f in fields:
        if f.name in names:
            return f
    return None


def strided_u64(msg: Any, field: Any) -> np.ndarray:
    n = int(msg.width) * int(msg.height)
    return np.ndarray(
        shape=(n,),
        dtype="<u8",
        buffer=msg.data,
        offset=int(field.offset),
        strides=(int(msg.point_step),),
    )


def strided_u8(msg: Any, offset: int) -> np.ndarray:
    n = int(msg.width) * int(msg.height)
    return np.ndarray(
        shape=(n,),
        dtype=np.uint8,
        buffer=msg.data,
        offset=offset,
        strides=(int(msg.point_step),),
    )


def raw8(msg: Any, field: Any, idx: int) -> bytes:
    off = idx * int(msg.point_step) + int(field.offset)
    return bytes(memoryview(msg.data)[off : off + 8])


def summarize_sample(msg: Any, field: Any, idx: int, shift_ns: int) -> dict[str, Any]:
    b = raw8(msg, field, idx)
    as_uint64 = struct.unpack("<Q", b)[0]
    as_double = struct.unpack("<d", b)[0]
    return {
        "index": idx,
        "raw_hex": b.hex(" "),
        "as_uint64_ns_raw": int(as_uint64),
        "as_uint64_ns_shifted": int(max(0, int(as_uint64) + int(shift_ns))),
        "as_double_sec": as_double if math.isfinite(as_double) else str(as_double),
        "as_double_ns": as_double * 1e-9 if math.isfinite(as_double) else str(as_double),
    }


def summarize_cloud(msg: Any, log_time_ns: int, topic: str, shift_ns: int) -> dict[str, Any] | None:
    tf = find_field(msg.fields, TIME_FIELD_NAMES)
    if tf is None or int(msg.width) * int(msg.height) == 0:
        return None
    if not (int(tf.datatype) == 2 and int(tf.count) == 8):
        return {
            "topic": topic,
            "log_time_ns": int(log_time_ns),
            "supported": False,
            "time_field": {
                "name": tf.name,
                "offset": int(tf.offset),
                "datatype": int(tf.datatype),
                "datatype_name": DTYPE_NAMES.get(int(tf.datatype), str(tf.datatype)),
                "count": int(tf.count),
            },
        }

    raw = strided_u64(msg, tf)
    vals = raw.astype(np.int64, copy=False) + int(shift_ns)
    vals = np.maximum(vals, 0)
    n = int(vals.size)
    first = int(vals[0])
    mid = int(vals[n // 2])
    last = int(vals[-1])
    vmin = int(vals.min())
    vmax = int(vals.max())
    span = int(vmax - vmin)
    diffs = np.diff(vals) if n > 1 else np.array([], dtype=np.int64)
    large_backwards = int(np.count_nonzero(diffs < -500_000_000))
    ms_backwards = int(np.count_nonzero(diffs < -1_000_000))

    detector_backwards = 0
    detector_field = next((f for f in msg.fields if f.name == "detector_site_id"), None)
    if detector_field is not None and int(detector_field.datatype) == 2:
        det = strided_u8(msg, int(detector_field.offset))
        for detector_id in np.unique(det):
            group_vals = vals[det == detector_id]
            if group_vals.size > 1:
                detector_backwards += int(np.count_nonzero(np.diff(group_vals) < -1_000_000))

    unique_count = int(np.unique(vals).size)
    header_ns = stamp_ns(msg.header.stamp)
    return {
        "topic": topic,
        "supported": True,
        "log_time_ns": int(log_time_ns),
        "header_ns": int(header_ns),
        "header_minus_min_point_ns": int(header_ns - vmin),
        "point_count": n,
        "point_step": int(msg.point_step),
        "is_bigendian": bool(msg.is_bigendian),
        "time_field": {
            "name": tf.name,
            "offset": int(tf.offset),
            "datatype": int(tf.datatype),
            "datatype_name": DTYPE_NAMES.get(int(tf.datatype), str(tf.datatype)),
            "count": int(tf.count),
        },
        "samples": [
            summarize_sample(msg, tf, 0, shift_ns),
            summarize_sample(msg, tf, n // 2, shift_ns),
            summarize_sample(msg, tf, n - 1, shift_ns),
        ],
        "first_ns": first,
        "mid_ns": mid,
        "last_ns": last,
        "min_ns": vmin,
        "max_ns": vmax,
        "span_ns": span,
        "span_ms": span / 1e6,
        "last_minus_first_ns": int(last - first),
        "unique_count": unique_count,
        "collapsed": bool(unique_count <= 1 or span < 1_000_000),
        "epoch_plausible": bool(1_600_000_000_000_000_000 <= vmin <= 2_100_000_000_000_000_000),
        "second_rollover_cloud": bool(vmin // 1_000_000_000 != vmax // 1_000_000_000),
        "large_backward_jumps_gt_500ms": large_backwards,
        "backward_jumps_gt_1ms": ms_backwards,
        "detector_backward_jumps_gt_1ms": detector_backwards,
    }


def verdict_topic(topic_result: dict[str, Any]) -> dict[str, Any]:
    epoch = topic_result["epoch_plausible_count"] == topic_result["cloud_count"]
    collapsed = topic_result["collapsed_count"] == 0
    no_wrap = topic_result["large_backward_jumps_gt_500ms"] == 0
    rollovers = (
        topic_result["second_rollover_clouds"] > 0
        or topic_result["adjacent_second_rollovers"] > 0
    )
    span_med = topic_result["span_ns_median"]
    plausible_span = 10_000_000 <= span_med <= 120_000_000
    return {
        "pass": bool(epoch and collapsed and no_wrap and rollovers and plausible_span),
        "epoch_all_clouds": bool(epoch),
        "not_collapsed": bool(collapsed),
        "no_large_backward_wrap": bool(no_wrap),
        "has_second_rollover_coverage": bool(rollovers),
        "scan_span_plausible": bool(plausible_span),
    }


def nearest(items: list[dict[str, Any]], key: int) -> dict[str, Any] | None:
    if not items:
        return None
    stamps = [int(x["header_ns"]) for x in items]
    idx = bisect.bisect_left(stamps, key)
    candidates = []
    if idx < len(items):
        candidates.append(items[idx])
    if idx > 0:
        candidates.append(items[idx - 1])
    if not candidates:
        return None
    return min(candidates, key=lambda x: abs(int(x["header_ns"]) - key))


def inspect_bag(
    bag: Path,
    topics: tuple[str, ...],
    max_seconds: float,
    max_clouds: int,
    apply_header_shift: bool,
    merge_threshold_s: float,
) -> dict[str, Any]:
    result: dict[str, Any] = {
        "bag": str(bag),
        "topics": {},
        "schema_equal": None,
        "apply_header_shift": bool(apply_header_shift),
        "global_shift_ns": 0,
    }

    per_topic_clouds: dict[str, list[dict[str, Any]]] = {topic: [] for topic in topics}
    first_log: dict[str, int] = {}
    first_schema: dict[str, tuple[tuple[str, int, int, int], ...]] = {}
    first_schema_print: dict[str, list[dict[str, Any]]] = {}
    raw_first_summaries: list[dict[str, Any]] = []

    with AnyReader([bag]) as reader:
        conns = [c for c in reader.connections if c.topic in topics]
        if not conns:
            raise SystemExit(f"no requested Luminar topics found in {bag}")

        # First pass: find the global header-to-point shift if requested.
        if apply_header_shift:
            for conn, log_time_ns, rawdata in reader.messages(connections=conns):
                msg = reader.deserialize(rawdata, conn.msgtype)
                summary = summarize_cloud(msg, int(log_time_ns), conn.topic, 0)
                if summary and summary.get("supported"):
                    result["global_shift_ns"] = int(summary["header_minus_min_point_ns"])
                    raw_first_summaries.append(summary)
                    break

    shift_ns = int(result["global_shift_ns"])
    with AnyReader([bag]) as reader:
        conns = [c for c in reader.connections if c.topic in topics]
        for conn, log_time_ns, rawdata in reader.messages(connections=conns):
            topic = conn.topic
            if topic not in first_log:
                first_log[topic] = int(log_time_ns)
            if max_seconds > 0 and (int(log_time_ns) - first_log[topic]) > max_seconds * 1e9:
                if all(
                    t in first_log and (len(per_topic_clouds[t]) >= max_clouds or max_clouds <= 0)
                    or (t in first_log and (int(log_time_ns) - first_log[t]) > max_seconds * 1e9)
                    for t in topics
                ):
                    break
                continue
            if max_clouds > 0 and len(per_topic_clouds[topic]) >= max_clouds:
                continue

            msg = reader.deserialize(rawdata, conn.msgtype)
            if topic not in first_schema:
                first_schema[topic] = schema_key(msg.fields)
                first_schema_print[topic] = schema(msg.fields)
            summary = summarize_cloud(msg, int(log_time_ns), topic, shift_ns)
            if summary:
                per_topic_clouds[topic].append(summary)

            if max_clouds > 0 and all(len(per_topic_clouds[t]) >= max_clouds for t in topics):
                break

    schema_values = list(first_schema.values())
    result["schema_equal"] = bool(schema_values and all(s == schema_values[0] for s in schema_values))

    for topic, clouds in per_topic_clouds.items():
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
            "large_backward_jumps_gt_500ms": sum(int(c.get("large_backward_jumps_gt_500ms", 0)) for c in clouds),
            "backward_jumps_gt_1ms": sum(int(c.get("backward_jumps_gt_1ms", 0)) for c in clouds),
            "detector_backward_jumps_gt_1ms": sum(int(c.get("detector_backward_jumps_gt_1ms", 0)) for c in clouds),
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

    front = per_topic_clouds.get("/luminar_front/points", [])
    aux_topics = [t for t in topics if t != "/luminar_front/points"]
    merge_checks = []
    for front_cloud in front:
        near_boundary = (
            front_cloud.get("second_rollover_cloud")
            or (int(front_cloud["min_ns"]) % 1_000_000_000) < 60_000_000
            or (int(front_cloud["max_ns"]) % 1_000_000_000) > 940_000_000
        )
        if not near_boundary:
            continue
        merged = [front_cloud]
        ok = True
        aux_dts = {}
        for aux_topic in aux_topics:
            aux = nearest(per_topic_clouds.get(aux_topic, []), int(front_cloud["header_ns"]))
            if aux is None:
                ok = False
                continue
            dt_ns = abs(int(aux["header_ns"]) - int(front_cloud["header_ns"]))
            aux_dts[aux_topic] = dt_ns
            if dt_ns > merge_threshold_s * 1e9:
                ok = False
            merged.append(aux)
        if len(merged) == 1:
            continue
        combined_span = max(int(c["max_ns"]) for c in merged) - min(int(c["min_ns"]) for c in merged)
        merge_checks.append(
            {
                "front_header_ns": int(front_cloud["header_ns"]),
            "ok": bool(ok and combined_span <= int((merge_threshold_s + 0.12) * 1e9)),
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
    result["merge_pass"] = bool(result["schema_equal"] and merge_checks and all(c["ok"] for c in merge_checks))
    result["overall_pass"] = bool(
        result["schema_equal"]
        and result["merge_pass"]
        and all(t.get("pass") for t in result["topics"].values())
    )
    return result


def print_summary(result: dict[str, Any]) -> None:
    mode = "corrected/header-shift view" if result["apply_header_shift"] else "raw bag bytes"
    print(f"bag: {result['bag']}")
    print(f"mode: {mode}")
    print(f"global_shift_ns: {result['global_shift_ns']}")
    print(f"schema_equal: {result['schema_equal']}")
    for topic, topic_result in result["topics"].items():
        print(f"\n{topic}")
        if not topic_result.get("cloud_count"):
            print("  no clouds")
            continue
        fc = topic_result["first_cloud"]
        print(
            "  field: {name} off={offset} type={datatype_name} count={count}, "
            "point_step={point_step}, bigendian={is_bigendian}".format(**fc["time_field"], **fc)
        )
        print(
            f"  clouds={topic_result['cloud_count']} "
            f"span_ms median={topic_result['span_ms_median']:.3f} "
            f"min/max={topic_result['span_ns_min']/1e6:.3f}/{topic_result['span_ns_max']/1e6:.3f}"
        )
        print(
            f"  epoch_plausible={topic_result['epoch_plausible_count']}/{topic_result['cloud_count']} "
            f"rollover_clouds={topic_result['second_rollover_clouds']} "
            f"adjacent_rollovers={topic_result['adjacent_second_rollovers']} "
            f"collapsed={topic_result['collapsed_count']} "
            f"large_backwards_500ms={topic_result['large_backward_jumps_gt_500ms']} "
            f"adjacent_backwards_500ms={topic_result['adjacent_backward_jumps_gt_500ms']}"
        )
        print(f"  verdict: {'PASS' if topic_result['pass'] else 'FAIL'} {topic_result['verdict']}")
        for sample in fc["samples"]:
            print(
                f"    point[{sample['index']}] raw={sample['raw_hex']} "
                f"u64_raw={sample['as_uint64_ns_raw']} "
                f"u64_shifted={sample['as_uint64_ns_shifted']} "
                f"double_sec={sample['as_double_sec']}"
            )
    print(f"\nmerge_pass: {result['merge_pass']} checks={len(result['merge_checks'])}")
    for check in result["merge_checks"][:3]:
        print(
            f"  merged rollover @ {check['front_header_ns']}: "
            f"span_ms={check['combined_span_ms']:.3f} "
            f"aux_dt_ms={[round(v/1e6, 3) for v in check['aux_header_dt_ns'].values()]} "
            f"ok={check['ok']}"
        )
    print(f"overall_pass: {result['overall_pass']}")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("bag", type=Path)
    parser.add_argument("--topics", nargs="+", default=list(TOPICS))
    parser.add_argument("--max-seconds", type=float, default=90.0)
    parser.add_argument("--max-clouds", type=int, default=0)
    parser.add_argument("--merge-threshold", type=float, default=0.05)
    parser.add_argument("--apply-header-shift", action="store_true")
    parser.add_argument("--json-out", type=Path)
    args = parser.parse_args()

    result = inspect_bag(
        args.bag,
        tuple(args.topics),
        args.max_seconds,
        args.max_clouds,
        args.apply_header_shift,
        args.merge_threshold,
    )
    print_summary(result)
    if args.json_out:
        args.json_out.parent.mkdir(parents=True, exist_ok=True)
        args.json_out.write_text(json.dumps(result, indent=2, sort_keys=True) + "\n")


if __name__ == "__main__":
    main()
