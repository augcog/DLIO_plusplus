#!/usr/bin/env python3
"""Reference Python merger for Luminar Iris PCAP data and ROS 2 bags.

The production mapping path uses the faster C++ `glim_pcap_rosbag`
executable. Keep this script as a debug/reference fallback for validating
packet decoding, PointCloud2 layout, and timestamp alignment behavior.
"""

# Copyright 2025 Jeff Liu

import argparse
import subprocess
import heapq
from tqdm import tqdm
from rclpy.serialization import serialize_message
import struct as _s
import math
import time
from collections import defaultdict
from std_msgs.msg import Header
from sensor_msgs.msg import PointCloud2, PointField
import rosbag2_py

# ------ CONSTANTS -------#

# AV-24 Luminars IPs and topics
LIDARS = {
    "10.42.37.20": "/luminar_front/points",
    "10.42.37.22": "/luminar_right/points",
    "10.42.37.21": "/luminar_left/points",
}

# Per-lidar destination UDP ports used by ros2_iris_driver config
LIDAR_DST_PORTS = {
    "10.42.37.20": 4370,
    "10.42.37.21": 4371,
    "10.42.37.22": 4372,
}

# AV-24 Luminars frame IDs
FRAME_IDS = {
    "10.42.37.20": "luminar_front",
    "10.42.37.22": "luminar_right",
    "10.42.37.21": "luminar_left",
}

# Iris point cloud data stream source UDP port
IRIS_DATA_UDP_SRCPORT = 4371


# Note: Define PointField list for Luminar Iris PC2 format
PC2_FIELDS = [
    PointField(name="timestamp", offset=0,  datatype=PointField.UINT8,  count=8),
    PointField(name="x", offset=8,  datatype=PointField.FLOAT32, count=1),
    PointField(name="y", offset=12,  datatype=PointField.FLOAT32, count=1),
    PointField(name="z", offset=16,  datatype=PointField.FLOAT32, count=1),
    PointField(name="reflectance", offset=20, datatype=PointField.FLOAT32, count=1),

    PointField(name="return_index", offset=24, datatype=PointField.UINT8,  count=1),
    PointField(name="last_return_index", offset=25, datatype=PointField.UINT8,  count=1),
    PointField(name="sensor_id", offset=26, datatype=PointField.UINT8, count=1),

    PointField(name="azimuth",   offset=32, datatype=PointField.FLOAT32, count=1),
    PointField(name="elevation", offset=36, datatype=PointField.FLOAT32, count=1),
    PointField(name="depth",     offset=40, datatype=PointField.FLOAT32, count=1),

    PointField(name="line_index",     offset=44, datatype=PointField.UINT16, count=1),
    PointField(name="frame_index",    offset=46, datatype=PointField.UINT8,  count=1),
    PointField(name="detector_site_id", offset=47, datatype=PointField.UINT8, count=1),
    PointField(name="scan_checkpoint", offset=48, datatype=PointField.UINT8, count=1),
    PointField(name="existence_probability_percent", offset=49, datatype=PointField.UINT8, count=1),
    PointField(name="data_qualifier",  offset=50, datatype=PointField.UINT8, count=1),
    PointField(name="blockage_level",  offset=51, datatype=PointField.UINT8, count=1),
]
POINT_STEP = 56

# ------ MAIN MERGER OBJECTS -------#

class PCAPStreamer:
    def __init__(self, pcap_path):
        self.pcap_path = pcap_path

        self.packets_by_ip_count = defaultdict(int)

    def stream_one_pcap(self, start_epoch=None, end_epoch=None):
        parts = ["udp"]
        parts.append(f"udp.srcport=={IRIS_DATA_UDP_SRCPORT}")
        ip_port_or = " || ".join([
            f"(ip.src=={ip} && udp.dstport=={port})"
            for ip, port in LIDAR_DST_PORTS.items()
        ])
        parts.append(f"({ip_port_or})")
        if start_epoch is not None:
            parts.append(f"frame.time_epoch>={float(start_epoch):.9f}")
        if end_epoch is not None:
            parts.append(f"frame.time_epoch<={float(end_epoch):.9f}")
        disp = " && ".join(parts)
        print("[tshark filter]:", disp)

        tshark_command = [
            "tshark", "-n", "-l",
            "-r", self.pcap_path,
            "-Y", disp,
            "-T", "fields",
            "-E", "separator=,",
            "-e", "frame.time_epoch",
            "-e", "ip.src",
            "-e", "udp.length",
            "-e", "data.data"
        ]
        proc = subprocess.Popen(tshark_command, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
        try:
            for line in proc.stdout:
                cols = line.rstrip("\n").split(",", 3)
                if len(cols) < 4 or cols[3] == "":
                    continue
                t_epoch, ip_src, udp_len, payload_hex = cols
                # tshark's data.data is colon-delimited (e.g., "aa:bb:cc")
                # bytes.fromhex expects a hex string without separators.
                payload_hex = payload_hex.replace(":", "")
                yield t_epoch, ip_src, udp_len, payload_hex
        finally:
            if proc.poll() is None:
                proc.terminate()
                try:
                    proc.communicate(timeout=0.2)
                except subprocess.TimeoutExpired:
                    proc.kill()

    def get_capture_epoch_bounds(self):
        try:
            proc = subprocess.run(
                ["capinfos", "-a", "-e", "-S", self.pcap_path],
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True,
                check=False,
            )
        except Exception:
            return None, None

        first_epoch = None
        last_epoch = None
        for line in proc.stdout.splitlines():
            line = line.strip()
            if line.startswith("First packet time:"):
                try:
                    first_epoch = float(line.split(":", 1)[1].strip())
                except ValueError:
                    first_epoch = None
            elif line.startswith("Last packet time:"):
                try:
                    last_epoch = float(line.split(":", 1)[1].strip())
                except ValueError:
                    last_epoch = None
        if proc.returncode != 0 and first_epoch is not None and last_epoch is not None:
            # capinfos can return non-zero for slightly malformed/truncated files but still prints bounds.
            print("[merge] capinfos returned non-zero; using parsed packet time bounds from its output")
        return first_epoch, last_epoch

class IrisPacketHeader:
    __slots__ = ("packet_version_major", "packet_version_minor", "packet_version_patch",
                "packet_sequence", "num_rays", "frame_sequence", "ptp_timestamp",
                "sensor_id", "data_qualifier", "reserved")
    def __init__(self):
        pass

    def __str__(self):
        return (f"IrisPacketHeader(version={self.packet_version_major}."
                f"{self.packet_version_minor}.{self.packet_version_patch}, "
                f"packet_sequence={self.packet_sequence}, num_rays={self.num_rays}, "
                f"frame_sequence={self.frame_sequence}, ptp_timestamp={self.ptp_timestamp}, "
                f"sensor_id={self.sensor_id}, data_qualifier={self.data_qualifier}, "
                f"reserved={self.reserved})")
    
class ScanAssemblerPC2:
    def __init__(self, point_fields, point_step=56, inactivity_sec=0.05):
        self.fields = point_fields
        self.point_step = point_step
        self.inactivity_sec = inactivity_sec

        self.active = {}
        self.last_key_by_active = {}
        self.last_key_by_src = {}


    def new_state(self, key, src_ip, t_epoch, hdr):
        return {
            "key": key,
            "src_ip": src_ip,
            "first_time": t_epoch,
            "last_time": t_epoch,
            "frame_seq": int(hdr.frame_sequence) & 0xFF,
            "ptp_sec": int(hdr.ptp_timestamp),
            "dq": int(hdr.data_qualifier) & 0xFF,
            "sensor_id": int(hdr.sensor_id) & 0xFF,
            "pkt_drops": 0,
            "points": []  # each: (ts64, x, y, z, refl, ret_idx, last_ret_idx, sensor_id,
                          #        az, el, depth, line_idx, frame_idx, detector_id,
                          #        scan_checkpoint, exist_prob, dq, blockage)
        }
    
    def touch(self, st, t_epoch):
        st["last_time"] = max(st["last_time"], t_epoch)

    def finalize(self, key):
        st = self.active.pop(key)
        msg = self.create_pc2(st)
        return (msg, st["first_time"], st["last_time"]) if msg else None
    
    def maybe_timeout(self, now):
        done = []
        for k, st in list(self.active.items()):
            if (now - st["last_time"]) >= self.inactivity_sec:
                result = self.finalize(k)
                if result is not None:
                    done.append(result)
        return done
    
    def feed(self, t_epoch, src_ip, hdr, points_iter):
        completed = []

        k = scan_key(src_ip, hdr)
        prev = self.last_key_by_src.get(src_ip)
        if prev is not None and prev != k:
            if prev in self.active:
                completed.append(self.finalize(prev))
        self.last_key_by_src[src_ip] = k

        st = self.active.get(k)
        if st is None:
            st = self.new_state(k, src_ip, t_epoch, hdr)
            self.active[k] = st

        for (ts64, x, y, z, refl, depth, ret_idx, last_retr,
             sensor_id, az, el, line_idx, frame_idx, detector_id,
             scan_checkpoint, exist_prob, dq, blockage) in points_iter:
            st["points"].append((
                int(ts64), float(x), float(y), float(z), float(refl),
                int(ret_idx) & 0xFF, int(last_retr) & 0xFF, int(sensor_id) & 0xFF,
                float(az), float(el), float(depth),
                int(line_idx) & 0xFFFF, int(frame_idx) & 0xFF, int(detector_id) & 0xFF,
                int(scan_checkpoint) & 0xFF, int(exist_prob) & 0xFF,
                int(dq) & 0xFF, int(blockage) & 0xFF
            ))
        self.touch(st, t_epoch)
        completed.extend(self.maybe_timeout(now=t_epoch))
        return completed
    
    def flush_all(self):
        out = []
        for k in list(self.active.keys()):
            out.append(self.finalize(k))
        return out

    def create_pc2(self, st):
        if not st["points"]:
            return None
        first_ts_ns = min(p[0] for p in st["points"])
        sec, nsec = divmod(int(first_ts_ns), 1_000_000_000)
        header = Header()
        header.frame_id = FRAME_IDS.get(st["src_ip"])
        header.stamp.sec = int(sec)
        header.stamp.nanosec = int(nsec)

        n = len(st["points"])
        row_step = self.point_step * n
        buf = bytearray(row_step)

        OFF = {
            "timestamp": 0, # uint8[8] as <Q
            "x": 8, "y": 12, "z": 16, # <f
            "reflectance": 20, # <f
            "return_index": 24, # u8
            "last_return_index": 25, # u8
            "sensor_id": 26, # u8
            # padding 27-31
            "azimuth": 32, "elevation": 36, "depth": 40, # <f
            "line_index": 44, # <H
            "frame_index": 46, # u8
            "detector_site_id": 47, # u8
            "scan_checkpoint": 48, # u8
            "existence_probability_percent": 49, # u8
            "data_qualifier": 50, # u8
            "blockage_level": 51, # u8
        }

        for i, p in enumerate(st["points"]):
            (ts64, x, y, z, refl,
             ret_idx, last_ret_idx, sensor_id,
             az, el, depth,
             line_idx, frame_idx, detector_id,
             scan_chk, exist, dq, block) = p

            o = i * self.point_step
            _s.pack_into('<Q',   buf, o + OFF["timestamp"], ts64)
            _s.pack_into('<fff', buf, o + OFF["x"], float(x), float(y), float(z))
            _s.pack_into('<f',   buf, o + OFF["reflectance"], float(refl))
            buf[o + OFF["return_index"]] = ret_idx
            buf[o + OFF["last_return_index"]] = last_ret_idx
            buf[o + OFF["sensor_id"]] = sensor_id
            _s.pack_into('<fff', buf, o + OFF["azimuth"], float(az), float(el), float(depth))
            _s.pack_into('<H',   buf, o + OFF["line_index"], int(line_idx))
            buf[o + OFF["frame_index"]] = frame_idx
            buf[o + OFF["detector_site_id"]] = detector_id
            buf[o + OFF["scan_checkpoint"]] = scan_chk
            buf[o + OFF["existence_probability_percent"]] = exist
            buf[o + OFF["data_qualifier"]] = dq
            buf[o + OFF["blockage_level"]] = block

        msg = PointCloud2(
            header=header,
            height=1, width=n,
            fields=self.fields, is_bigendian=False,
            point_step=self.point_step, row_step=row_step,
            data=bytes(buf), is_dense=False
        )
        return msg

class LidarPCAPMerger:
    def __init__(self, pcap_path, rosbag_in, rosbag_out, inactivity_sec=0.05):
        self.pcap_path = pcap_path
        self.rosbag_in = rosbag_in
        self.rosbag_out = rosbag_out
        self.inactivity_sec = inactivity_sec

        self.ptp_to_ros_shift_ns = None
        self.capture_anchor_epoch_sec = None
        self.unix_to_bag_offset_ns = None
        self.freq_log_interval_sec = 1.0
        self.scan_stats = defaultdict(lambda: {"count": 0, "first_ns": None, "last_ns": None})
        self.scan_rate_window = defaultdict(list)
        self.last_freq_log_wall = time.monotonic()
        self.scan_reorder_lookahead_ns = int(200e6)  # 200 ms

    def _record_scan(self, topic, t_ns):
        st = self.scan_stats[topic]
        st["count"] += 1
        if st["first_ns"] is None:
            st["first_ns"] = int(t_ns)
        st["last_ns"] = int(t_ns)

        window = self.scan_rate_window[topic]
        window.append(int(t_ns))
        cutoff = int(t_ns) - int(1e9)
        while window and window[0] < cutoff:
            window.pop(0)

    def _maybe_log_live_rates(self):
        now = time.monotonic()
        if (now - self.last_freq_log_wall) < self.freq_log_interval_sec:
            return
        self.last_freq_log_wall = now

        rates = []
        for topic in sorted(self.scan_rate_window.keys()):
            window = self.scan_rate_window[topic]
            if len(window) >= 2:
                dt = (window[-1] - window[0]) / 1e9
                hz = (len(window) - 1) / dt if dt > 0 else 0.0
            else:
                hz = 0.0
            rates.append(f"{topic}: {hz:.2f} Hz")
        if rates:
            print("[merge freq] " + " | ".join(rates))

    def _log_final_rates(self):
        print("[merge freq] final summary:")
        for topic in sorted(self.scan_stats.keys()):
            st = self.scan_stats[topic]
            count = st["count"]
            if count >= 2 and st["first_ns"] is not None and st["last_ns"] is not None:
                dt = (st["last_ns"] - st["first_ns"]) / 1e9
                hz = (count - 1) / dt if dt > 0 else 0.0
            else:
                hz = 0.0
            print(f"  {topic}: count={count}, avg_hz={hz:.2f}")

    def _choose_scan_epoch_window(self, streamer, bag_start_ns, bag_end_ns):
        tail_ns = int(self.inactivity_sec * 1e9)
        bag_start_epoch = bag_start_ns / 1e9
        bag_end_epoch = bag_end_ns / 1e9
        bag_end_epoch_with_tail = (bag_end_ns + tail_ns) / 1e9
        bag_duration_epoch = (bag_end_ns - bag_start_ns + tail_ns) / 1e9

        pcap_start_epoch, pcap_end_epoch = streamer.get_capture_epoch_bounds()
        if pcap_start_epoch is None or pcap_end_epoch is None:
            print("[merge] could not read PCAP epoch bounds; using bag time window filter")
            return bag_start_epoch, bag_end_epoch_with_tail, False, None, None

        overlap_start = max(bag_start_epoch, pcap_start_epoch)
        overlap_end = min(bag_end_epoch_with_tail, pcap_end_epoch)
        overlaps = overlap_start <= overlap_end
        if overlaps:
            # Use the strict intersection so we only parse packets present in both time ranges.
            trim_overlap_end = min(bag_end_epoch, pcap_end_epoch)
            trim_start_ns = int(overlap_start * 1e9)
            trim_end_ns = int(trim_overlap_end * 1e9)
            return overlap_start, overlap_end, False, trim_start_ns, trim_end_ns

        # Fallback for bags whose timeline is offset from wall-clock epoch:
        # anchor at PCAP start and keep only bag-duration worth of packets.
        fallback_start = pcap_start_epoch
        fallback_end = min(pcap_end_epoch, pcap_start_epoch + bag_duration_epoch)
        print(
            "[merge] bag/pcap epochs do not overlap; "
            f"using duration-limited PCAP window {fallback_start:.9f}..{fallback_end:.9f}"
        )
        return fallback_start, fallback_end, True, None, None

    def merge(self):
        reader, writer = open_rosbags(self.rosbag_in, self.rosbag_out)
        meta = reader.get_metadata()
        src_topics = {tm.name: tm for tm in reader.get_all_topics_and_types()}

        topic_names = set(src_topics.keys())
        bag_start_ns = meta.starting_time.nanoseconds

        for ip, topic in LIDARS.items():
            if topic not in topic_names:
                clone_or_create_topic(writer, src_topics, topic, 'sensor_msgs/msg/PointCloud2')
                topic_names.add(topic)

        

        streamer = PCAPStreamer(self.pcap_path)
        assembler = ScanAssemblerPC2(PC2_FIELDS, POINT_STEP, inactivity_sec=self.inactivity_sec)

        total_bag_duration_ns = meta.duration.nanoseconds
        bag_end_ns = bag_start_ns + total_bag_duration_ns

        scan_start_epoch, scan_end_epoch, remap_to_bag_start, trim_start_ns, trim_end_ns = self._choose_scan_epoch_window(
            streamer, bag_start_ns, bag_end_ns
        )
        scan_iter = self.scan_generator(
            streamer,
            assembler,
            scan_start_epoch,
            scan_end_epoch,
            bag_start_ns if remap_to_bag_start else None,
        )
        if trim_start_ns is not None and trim_end_ns is not None:
            print(f"[merge] trimming bag messages to overlap window ns [{trim_start_ns}, {trim_end_ns}]")
        next_scan = next(scan_iter, None)
        scan_heap = []


        total_bag_messages = meta.message_count


        p_msgs = tqdm(total=total_bag_messages, unit='msg', desc='Processing bag (messages)')
        p_scans = tqdm(unit='scan', desc='LiDAR scans added')

        try:
            while reader.has_next():
                topic, data, t = reader.read_next()
                scan_read_horizon_ns = t + self.scan_reorder_lookahead_ns
                while next_scan is not None and next_scan[0] <= scan_read_horizon_ns:
                    heapq.heappush(scan_heap, next_scan)
                    next_scan = next(scan_iter, None)

                while scan_heap and scan_heap[0][0] <= t:
                    t_ns_scan, topic_scan, pc2 = heapq.heappop(scan_heap)

                    if topic_scan not in topic_names:
                        clone_or_create_topic(writer, src_topics, topic_scan, 'sensor_msgs/msg/PointCloud2')
                        topic_names.add(topic_scan)

                    writer.write(topic_scan, serialize_message(pc2), int(t_ns_scan))
                    self._record_scan(topic_scan, t_ns_scan)
                    self._maybe_log_live_rates()
                    p_scans.update(1)

                if trim_start_ns is None or trim_start_ns <= t <= trim_end_ns:
                    writer.write(topic, data, t)
                p_msgs.update(1)

            while next_scan is not None:
                heapq.heappush(scan_heap, next_scan)
                next_scan = next(scan_iter, None)

            while scan_heap:
                t_ns_scan, topic_scan, pc2 = heapq.heappop(scan_heap)

                if topic_scan not in topic_names:
                    clone_or_create_topic(writer, src_topics, topic_scan, 'sensor_msgs/msg/PointCloud2')
                    topic_names.add(topic_scan)

                writer.write(topic_scan, serialize_message(pc2), int(t_ns_scan))
                self._record_scan(topic_scan, t_ns_scan)
                self._maybe_log_live_rates()
                p_scans.update(1)

        finally:
            p_msgs.close()
            p_scans.close()

        print(f"[merge] wrote merged bag to {self.rosbag_out}")
        self._log_final_rates()

    def scan_generator(self, streamer, assembler, scan_start_epoch, scan_end_epoch, force_anchor_ns=None):
        for t_epoch, ip_src, udp_len, payload_hex in streamer.stream_one_pcap(
            start_epoch=scan_start_epoch,
            end_epoch=scan_end_epoch,
        ):
            if self.capture_anchor_epoch_sec is None:
                self.capture_anchor_epoch_sec = float(t_epoch)
            topic = LIDARS.get(ip_src)
            if topic is None or not payload_hex:
                continue

            payload = bytes.fromhex(payload_hex)
            hdr, points_iter = parse_iris_payload(payload)
            if hdr is None or points_iter is None:
                continue

            for result in assembler.feed(float(t_epoch), ip_src, hdr, points_iter):
                if result is None:
                    continue

                pc2, first_time, last_time = result

                scan_ptp_ns = pc2.header.stamp.sec * 1_000_000_000 + pc2.header.stamp.nanosec

                if self.ptp_to_ros_shift_ns is None:
                    if force_anchor_ns is not None:
                        # If bag and PCAP epochs differ, anchor first scan to bag start.
                        self.ptp_to_ros_shift_ns = int(force_anchor_ns) - scan_ptp_ns
                    else:
                        # Anchor once using capture time for the first completed scan.
                        # Subsequent scans use PTP deltas to avoid network jitter.
                        scan_unix_ns = int(last_time * 1e9)
                        self.ptp_to_ros_shift_ns = scan_unix_ns - scan_ptp_ns

                t_out = scan_ptp_ns + self.ptp_to_ros_shift_ns
                pc2.header.stamp.sec = t_out // 1_000_000_000
                pc2.header.stamp.nanosec = t_out % 1_000_000_000
                yield (t_out, topic, pc2)

        for result in assembler.flush_all():
            if result is None:
                continue
            pc2, first_time, last_time = result

            topic = next((LIDARS[ip] for ip, fid in FRAME_IDS.items() if fid == pc2.header.frame_id), None)

            if topic is None:
                topic = list(LIDARS.values())[0]

            scan_ptp_ns = pc2.header.stamp.sec * 1_000_000_000 + pc2.header.stamp.nanosec

            if self.ptp_to_ros_shift_ns is None:
                self.ptp_to_ros_shift_ns = 0

            t_out = scan_ptp_ns + self.ptp_to_ros_shift_ns

            pc2.header.stamp.sec = t_out // 1_000_000_000
            pc2.header.stamp.nanosec = t_out % 1_000_000_000

            yield (t_out, topic, pc2)

# ------ HELPERS (PARSING) -------#

def q2_14_to_float(x16):
    x16 = x16 & 0xFFFF
    if x16 & 0x8000:
        x16 -= 0x10000
    return x16 / (1 << 14)

def uq12_12_to_float(u):
    return u / float(1 << 12)

def uq1_15_to_float(u):
    return u / float(1 << 15)

def read_bits(b, bit_ofs, nbits):
    byte_ofs = bit_ofs // 8
    bit_in_byte = bit_ofs % 8
    nbytes = (bit_in_byte + nbits + 7) // 8
    chunk = int.from_bytes(b[byte_ofs:byte_ofs+nbytes], "little", signed=False)
    chunk >>= bit_in_byte
    return chunk & ((1 << nbits) - 1)

def parse_iris_payload(payload):
    if len(payload) < 16:
        return None, None

    # TODO: Don't bother parsing the useless fields
    hdr = IrisPacketHeader()
    # SECTION 2.1 OF THE LUMINAR IRIS DATA OUTPUT SPECIFICATION
    # iris packet header, incrementally
    # For information on type, see output specification (e.g Q2.18)
    # packet version major 8 bits
    hdr.packet_version_major = read_bits(payload, 0, 8)
    # packet version minor 8 bits
    hdr.packet_version_minor = read_bits(payload, 8, 8)
    # packet version patch 8 bits
    hdr.packet_version_patch = read_bits(payload, 16, 8)
    # packet sequence 8 bits
    hdr.packet_sequence = read_bits(payload, 24, 8)
    # num rays 8 bits
    hdr.num_rays = read_bits(payload, 32, 8)
    # frame sequence 8 bits
    hdr.frame_sequence = read_bits(payload, 40, 8)
    # ptp timestamp 48 bits
    hdr.ptp_timestamp = read_bits(payload, 48, 48)
    # sensor id 8 bits
    hdr.sensor_id = read_bits(payload, 96, 8)
    # data qualifier 8 bits
    hdr.data_qualifier = read_bits(payload, 104, 8)
    # reserved 16 bits
    hdr.reserved = read_bits(payload, 112, 16)  # end bit = 128

    # Skip non-data packets that match IP/UDP filter.
    if hdr.packet_version_major != 1 or hdr.packet_version_minor != 3:
        return None, None

    bit = 128
    payload_bits = len(payload) * 8

    # Parsing each ray
    def iter_points():
        nonlocal bit
        for _ in range(hdr.num_rays):
            if (bit + 128) > payload_bits:
                return
            # SECTION 2.2 OF THE LUMINAR IRIS DATA OUTPUT SPECIFICATION
            # Parse ray header (2 64 bit words per header)
            # azimuth angle radians 16 bits Q2.14
            azimuth_a_q2_14 = read_bits(payload, bit, 16)
            # elevation angle radians 16 bits Q2.14
            elevation_a_q2_14 = read_bits(payload, bit + 16, 16)
            # ptp timestamp offset nanoseconds 32 bits Unsigned
            ptp_timestamp_offset = read_bits(payload, bit + 32, 32)
            # scan checkpoint uncertain 8 bits undetermined
            scan_checkpoint = read_bits(payload, bit + 64, 8)
            # ray sequence 4 bits unsigned
            ray_sequence = read_bits(payload, bit + 72, 4)
            # number of returns 4 bits unsigned
            num_returns = read_bits(payload, bit + 76, 4)
            # detector number 1 bit 0 or 1 unsigned
            detector_number = read_bits(payload, bit + 80, 1)
            # blockage number 4 bits unsigned
            blockage_number = read_bits(payload, bit + 81, 4)
            # RESERVED BITS [85-86]
            # line number 9 bits unsigned
            line_number = read_bits(payload, bit + 87, 9)
            # RESERVED BITS [96-127]

            bit += 128
            az = q2_14_to_float(azimuth_a_q2_14)
            el = q2_14_to_float(elevation_a_q2_14)
            cos_el = math.cos(el)
            sin_el = math.sin(el)
            cos_az = math.cos(az)
            sin_az = math.sin(az)

            base_ns = (hdr.ptp_timestamp * 1_000_000_000) + ptp_timestamp_offset

            for r_idx in range(num_returns):
                if (bit + 64) > payload_bits:
                    return
                # SECTION 2.3 OF THE LUMINAR IRIS DATA OUTPUT SPECIFICATION
                # Parse return data (1 64 bit word per return)
                # existence probability [0, 255] / 100 
                existence_prob = read_bits(payload, bit, 8)
                # range uq12.12 24 bits
                range_uq12_12 = read_bits(payload, bit + 8, 24)
                # reflectance uq1.15 16 bits
                reflectance_uq1_15 = read_bits(payload, bit + 32, 16)
                # retro artifact 0/1 1 bit
                retro_artifact = read_bits(payload, bit + 48, 1)
                # range wrap artifact 0/1 1 bit
                range_wrap_artifact = read_bits(payload, bit + 49, 1)
                # RESERVED BIT [50]
                # min range artifact 0/1 1 bit
                min_range_artifact = read_bits(payload, bit + 51, 1)
                # crosstalk artifact 0/1 1 bit
                crosstalk_artifact = read_bits(payload, bit + 52, 1)
                # low reflectance artifact 0/1 1 bit
                low_reflectance_artifact = read_bits(payload, bit + 53, 1)
                # RESERVED BITS [54-63]
                bit += 64

                range_ = uq12_12_to_float(range_uq12_12)
                reflectance = uq1_15_to_float(reflectance_uq1_15)
                x = range_ * cos_el * cos_az
                y = range_ * cos_el * sin_az
                z = range_ * sin_el

                yield (base_ns, x, y, z, reflectance, range_, r_idx, max(0, num_returns - 1),
                       hdr.sensor_id, az, el, line_number, hdr.frame_sequence, detector_number,
                       scan_checkpoint, existence_prob, hdr.data_qualifier, blockage_number)
    return hdr, iter_points()

def scan_key(src_ip, hdr):
    return (src_ip,
            int(hdr.sensor_id) & 0xFF,
            int(hdr.frame_sequence) & 0xFF)

# ------ HELPERS (ROSBAG I/O) -------#
def get_rosbag_options_read(path, serialization_format='cdr'):
    storage_options = rosbag2_py.StorageOptions(uri=path, storage_id='mcap')
    converter_options = rosbag2_py.ConverterOptions(
        input_serialization_format=serialization_format,
        output_serialization_format=serialization_format)
    return storage_options, converter_options

def get_rosbag_options_write(path, serialization_format='cdr'):
    storage_options = rosbag2_py.StorageOptions(uri=path, storage_id='mcap', max_bagfile_size=10_000_000_000)
    converter_options = rosbag2_py.ConverterOptions(
        input_serialization_format=serialization_format,
        output_serialization_format=serialization_format)
    return storage_options, converter_options

def open_rosbags(bag_path_input, bag_path_output):
    storage_in, conv_in = get_rosbag_options_read(bag_path_input)
    storage_out, conv_out = get_rosbag_options_write(bag_path_output)
    reader = rosbag2_py.SequentialReader()
    writer = rosbag2_py.SequentialWriter()
    reader.open(storage_in, conv_in)
    src_topics = { tm.name: tm for tm in reader.get_all_topics_and_types() }
    writer.open(storage_out, conv_out)
    for src_tm in reader.get_all_topics_and_types():
        if src_tm.type != 'luminar_iris_msgs/msg/SensorHealth':
            clone_or_create_topic(writer, src_topics, src_tm.name, src_tm.type)
    return reader, writer

def clone_or_create_topic(writer, src_topics, name, type_):
    src = src_topics.get(name)
    if src and getattr(src, "type", None) == type_:
        tm = rosbag2_py.TopicMetadata(
            name=src.name,
            type=src.type,
            serialization_format=src.serialization_format,
            offered_qos_profiles=getattr(src, "offered_qos_profiles", "") or "",
            type_description_hash=getattr(src, "type_description_hash", "") or ""
        )
        writer.create_topic(tm)
        return

    # Feel free to adjust QoS as needed
    luminar_qos = (
        "- history: 1\n"
        "  depth: 5\n"
        "  reliability: 2\n"
        "  durability: 2\n"
        "  deadline:\n"
        "    sec: 9223372036\n"
        "    nsec: 854775807\n"
        "  lifespan:\n"
        "    sec: 9223372036\n"
        "    nsec: 854775807\n"
        "  liveliness: 1\n"
        "  liveliness_lease_duration:\n"
        "    sec: 9223372036\n"
        "    nsec: 854775807\n"
        "  avoid_ros_namespace_conventions: false"
    )
    tm = rosbag2_py.TopicMetadata(
        name=name,
        type=type_,
        serialization_format="cdr",
        offered_qos_profiles=luminar_qos,
        type_description_hash=""
    )
    writer.create_topic(tm)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Merge Luminar PCAP files with ROS2 bag")
    parser.add_argument('pcap_path', type=str, help='Path to the PCAP file')
    parser.add_argument('rosbag', type=str, help='Path to the ROS2 bag folder')
    parser.add_argument('output', type=str, help='Path to the output merged ROS2 bag file')
    args = parser.parse_args()

    LidarPCAPMerger(args.pcap_path, args.rosbag, args.output, inactivity_sec=0.05).merge()
