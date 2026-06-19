#!/usr/bin/env python3
"""Extract Point One FusionEngine IMUOutput measurement times from a PCAP.

The output CSV can be passed to dlio_input_adapter as an IMU P1 sidecar for
raw-live replay:

    scripts/extract_p1_imu_output_times.py ins.pcap -o imu_output_p1.csv
    scripts/run_localization_replay.sh --raw-live \
      --adapter-imu-p1-sidecar imu_output_p1.csv ...

CSV columns:
  pcap_time        packet capture time in ROS/UNIX epoch seconds
  p1_time          IMUOutput.p1_time measurement time in P1 seconds
  sequence_number  FusionEngine sequence number
  payload_size     FusionEngine payload size
  transport/src/dst/debug fields for audit
"""

from __future__ import annotations

import argparse
import csv
import math
import socket
import struct
import sys
import zlib
from collections import Counter
from pathlib import Path


SYNC = b"\x2e\x31"
HEADER_SIZE = 24
IMU_OUTPUT = 11000


def pcap_packets(path: Path):
    with path.open("rb") as handle:
        header = handle.read(24)
        if len(header) != 24:
            raise RuntimeError(f"{path} is too small to be a pcap")
        magic = header[:4]
        if magic == b"\xd4\xc3\xb2\xa1":
            endian, scale = "<", 1e-6
        elif magic == b"\xa1\xb2\xc3\xd4":
            endian, scale = ">", 1e-6
        elif magic == b"\x4d\x3c\xb2\xa1":
            endian, scale = "<", 1e-9
        elif magic == b"\xa1\xb2\x3c\x4d":
            endian, scale = ">", 1e-9
        else:
            raise RuntimeError(f"unsupported pcap magic: {magic.hex()}")

        packet_index = 0
        while True:
            packet_header = handle.read(16)
            if not packet_header:
                break
            if len(packet_header) != 16:
                raise RuntimeError("truncated pcap packet header")
            ts_sec, ts_frac, caplen, _origlen = struct.unpack(endian + "IIII", packet_header)
            frame = handle.read(caplen)
            if len(frame) != caplen:
                raise RuntimeError("truncated pcap packet payload")
            yield packet_index, ts_sec + ts_frac * scale, frame
            packet_index += 1


def ip_to_text(raw: bytes) -> str:
    return socket.inet_ntoa(raw)


def network_payloads(path: Path):
    for packet_index, pcap_time, frame in pcap_packets(path):
        if len(frame) < 14:
            continue
        eth_type = struct.unpack_from("!H", frame, 12)[0]
        offset = 14
        if eth_type == 0x8100 and len(frame) >= 18:
            eth_type = struct.unpack_from("!H", frame, 16)[0]
            offset = 18
        if eth_type != 0x0800 or len(frame) < offset + 20:
            continue

        ip = frame[offset:]
        ihl = (ip[0] & 0x0F) * 4
        if len(ip) < ihl:
            continue
        total_len = struct.unpack_from("!H", ip, 2)[0]
        proto = ip[9]
        src_ip = ip_to_text(ip[12:16])
        dst_ip = ip_to_text(ip[16:20])
        transport = ip[ihl:total_len]

        if proto == 17:
            if len(transport) < 8:
                continue
            src_port, dst_port, udp_len, _checksum = struct.unpack_from("!HHHH", transport, 0)
            payload = transport[8:udp_len]
            if payload:
                yield {
                    "packet_index": packet_index,
                    "pcap_time": pcap_time,
                    "transport": "udp",
                    "src": f"{src_ip}:{src_port}",
                    "dst": f"{dst_ip}:{dst_port}",
                    "payload": payload,
                    "tcp_seq": None,
                }
        elif proto == 6:
            if len(transport) < 20:
                continue
            src_port, dst_port, seq = struct.unpack_from("!HHI", transport, 0)
            data_offset = (transport[12] >> 4) * 4
            if len(transport) < data_offset:
                continue
            payload = transport[data_offset:]
            if payload:
                yield {
                    "packet_index": packet_index,
                    "pcap_time": pcap_time,
                    "transport": "tcp",
                    "src": f"{src_ip}:{src_port}",
                    "dst": f"{dst_ip}:{dst_port}",
                    "payload": payload,
                    "tcp_seq": seq,
                }


class FusionEngineParser:
    def __init__(self):
        self.buffer = bytearray()

    def feed(self, data: bytes):
        self.buffer.extend(data)
        out = []
        while True:
            sync = self.buffer.find(SYNC)
            if sync < 0:
                if len(self.buffer) > 1:
                    del self.buffer[:-1]
                return out
            if sync > 0:
                del self.buffer[:sync]
            if len(self.buffer) < HEADER_SIZE:
                return out

            if self.buffer[2] != 0 or self.buffer[3] != 0:
                del self.buffer[0]
                continue

            payload_size = struct.unpack_from("<I", self.buffer, 16)[0]
            if payload_size > 1 << 20:
                del self.buffer[0]
                continue
            total_size = HEADER_SIZE + payload_size
            if len(self.buffer) < total_size:
                return out

            message = bytes(self.buffer[:total_size])
            stored_crc = struct.unpack_from("<I", message, 4)[0]
            computed_crc = zlib.crc32(message[8:]) & 0xFFFFFFFF
            if computed_crc != stored_crc:
                del self.buffer[0]
                continue

            out.append(
                {
                    "message_version": message[9],
                    "message_type": struct.unpack_from("<H", message, 10)[0],
                    "sequence_number": struct.unpack_from("<I", message, 12)[0],
                    "payload_size": payload_size,
                    "payload": message[HEADER_SIZE:],
                }
            )
            del self.buffer[:total_size]


def new_stream_state():
    return {"parser": FusionEngineParser(), "next_seq": None, "gaps": 0}


def feed_transport(state, packet):
    payload = packet["payload"]
    if packet["transport"] != "tcp":
        return state["parser"].feed(payload)

    seq = packet["tcp_seq"]
    next_seq = state["next_seq"]
    if next_seq is None:
        state["next_seq"] = seq + len(payload)
        return state["parser"].feed(payload)
    if seq < next_seq:
        overlap = next_seq - seq
        if overlap >= len(payload):
            return []
        payload = payload[overlap:]
        state["next_seq"] = next_seq + len(payload)
        return state["parser"].feed(payload)
    if seq > next_seq:
        state["gaps"] += 1
        state["parser"] = FusionEngineParser()
    state["next_seq"] = seq + len(payload)
    return state["parser"].feed(payload)


def extract_imu_output(path: Path, max_messages: int | None = None):
    streams = {}
    rows = []
    message_counts = Counter()
    gap_count = 0

    for packet in network_payloads(path):
        key = (packet["transport"], packet["src"], packet["dst"])
        state = streams.setdefault(key, new_stream_state())
        for message in feed_transport(state, packet):
            message_counts[message["message_type"]] += 1
            if message["message_type"] != IMU_OUTPUT:
                continue
            payload = message["payload"]
            if len(payload) < 8:
                continue
            seconds, fraction_ns = struct.unpack_from("<II", payload, 0)
            if seconds == 0xFFFFFFFF or fraction_ns == 0xFFFFFFFF:
                continue
            rows.append(
                {
                    "pcap_time": f"{packet['pcap_time']:.9f}",
                    "p1_time": f"{seconds + fraction_ns * 1e-9:.9f}",
                    "sequence_number": message["sequence_number"],
                    "payload_size": message["payload_size"],
                    "transport": packet["transport"],
                    "src": packet["src"],
                    "dst": packet["dst"],
                    "packet_index": packet["packet_index"],
                }
            )
            if max_messages is not None and len(rows) >= max_messages:
                gap_count = sum(s["gaps"] for s in streams.values())
                return rows, message_counts, gap_count

    gap_count = sum(s["gaps"] for s in streams.values())
    return rows, message_counts, gap_count


def summarize(rows):
    if len(rows) < 2:
        return "not enough IMUOutput rows for dt stats"
    p1 = [float(row["p1_time"]) for row in rows]
    cap = [float(row["pcap_time"]) for row in rows]
    p1_dt = [b - a for a, b in zip(p1, p1[1:]) if math.isfinite(b - a)]
    cap_dt = [b - a for a, b in zip(cap, cap[1:]) if math.isfinite(b - a)]

    def stats(values):
        values = sorted(values)
        mid = len(values) // 2
        p95 = values[int(0.95 * (len(values) - 1))]
        return (
            f"median={values[mid] * 1e3:.3f}ms "
            f"p95={p95 * 1e3:.3f}ms "
            f"min={values[0] * 1e3:.3f}ms "
            f"max={values[-1] * 1e3:.3f}ms"
        )

    return (
        f"IMUOutput rows={len(rows)}\n"
        f"  p1_dt: {stats(p1_dt)}\n"
        f"  capture_dt: {stats(cap_dt)}"
    )


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("pcap", type=Path)
    parser.add_argument("-o", "--output", type=Path, required=True)
    parser.add_argument("--max-messages", type=int)
    args = parser.parse_args()

    rows, counts, gap_count = extract_imu_output(args.pcap, args.max_messages)
    if not rows:
        print("no IMU_OUTPUT messages found", file=sys.stderr)
        return 1

    args.output.parent.mkdir(parents=True, exist_ok=True)
    with args.output.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)

    print(summarize(rows))
    print(f"top FusionEngine message types: {counts.most_common(10)}")
    print(f"tcp_gap_resets: {gap_count}")
    print(f"wrote: {args.output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
