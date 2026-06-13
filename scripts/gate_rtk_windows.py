#!/usr/bin/env python3
"""Copy a prepped bag, dropping RTK-FIXED odometry inside given time windows.

Produces the mapping-side analog of the localization synthetic-RTK-denial
test: GLIM's GNSS global factors consume /gps_p1/filtered_odom_rtk_fixed, so
deleting those messages inside windows emulates RTK outages while LiDAR/IMU
data stays intact. Windows are seconds relative to the first message stamp on
the gated topic.

Usage (inside the ros2-jazzy distrobox):
    python3 scripts/gate_rtk_windows.py \
        --input  /home/dongc1/dlio_data/run_5_prepped \
        --output /home/dongc1/dlio_data/run_5_prepped_rtkdenied \
        --windows 95:135,445:495,620:650
"""

import argparse
import glob
import os
import sys

from mcap.reader import make_reader
from mcap.writer import Writer

GATED_TOPIC = "/gps_p1/filtered_odom_rtk_fixed"


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--input", required=True, help="prepped bag directory")
    ap.add_argument("--output", required=True)
    ap.add_argument("--windows", required=True, help="'a:b,c:d' seconds from first gated msg")
    ap.add_argument("--topic", default=GATED_TOPIC)
    args = ap.parse_args()

    wins = [tuple(map(float, w.split(":"))) for w in args.windows.split(",")]
    files = sorted(glob.glob(os.path.join(args.input, "*.mcap")))
    if not files:
        sys.exit(f"no .mcap in {args.input}")
    os.makedirs(args.output, exist_ok=True)
    out_path = os.path.join(args.output, os.path.basename(args.output) + "_0.mcap")

    n_drop = n_keep = 0
    first_stamp = None
    with open(out_path, "wb") as fo:
        w = Writer(fo)
        w.start("ros2")
        sch_map, ch_map = {}, {}
        for f in files:
            with open(f, "rb") as fi:
                r = make_reader(fi)
                for schema, channel, msg in r.iter_messages():
                    if schema.id not in sch_map:
                        sch_map[schema.id] = w.register_schema(schema.name, schema.encoding, schema.data)
                    if channel.id not in ch_map:
                        ch_map[channel.id] = w.register_channel(
                            channel.topic, channel.message_encoding, sch_map[schema.id])
                    if channel.topic == args.topic:
                        t = msg.log_time * 1e-9
                        if first_stamp is None:
                            first_stamp = t
                        rel = t - first_stamp
                        if any(a <= rel <= b for a, b in wins):
                            n_drop += 1
                            continue
                        n_keep += 1
                    w.add_message(ch_map[channel.id], msg.log_time, msg.data, msg.publish_time)
        w.finish()

    print(f"dropped {n_drop} / kept {n_keep} msgs on {args.topic}")
    # rosbag2 needs metadata.yaml; reuse the input's and fix name + counts are
    # not strictly validated by glim_rosbag/ros2 bag info for playback via mcap
    import yaml
    with open(os.path.join(args.input, "metadata.yaml")) as f:
        meta = yaml.safe_load(f)
    info = meta["rosbag2_bagfile_information"]
    info["relative_file_paths"] = [os.path.basename(out_path)]
    if "files" in info:
        for fent in info["files"]:
            fent["path"] = os.path.basename(out_path)
        info["files"] = info["files"][:1]
    for t in info.get("topics_with_message_count", []):
        if t["topic_metadata"]["name"] == args.topic:
            t["message_count"] = n_keep
    with open(os.path.join(args.output, "metadata.yaml"), "w") as f:
        yaml.safe_dump(meta, f)
    print(f"wrote {args.output}")


if __name__ == "__main__":
    main()
