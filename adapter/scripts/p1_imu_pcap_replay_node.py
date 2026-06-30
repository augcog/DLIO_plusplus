#!/usr/bin/env python3
"""Replay Point One FusionEngine IMU_OUTPUT from a PCAP as a ROS IMU stream.

This is an online source: it reads the PCAP sequentially, paces packets from
their capture timestamps, parses FusionEngine messages as bytes arrive, and
publishes each IMU_OUTPUT with its real P1 measurement timestamp.
"""

from __future__ import annotations

import socket
import struct
import threading
import time
import zlib
from collections import Counter
from pathlib import Path
from typing import Dict, Iterable, Iterator, List, Optional, Tuple

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import Imu


SYNC = b"\x2e\x31"
HEADER_SIZE = 24
IMU_OUTPUT = 11000
IMU_OUTPUT_STRUCT = struct.Struct("<II" + "d" * 12)


class PcapError(RuntimeError):
    pass


def pcap_packets(path: Path) -> Iterator[Tuple[int, float, bytes]]:
    with path.open("rb") as handle:
        header = handle.read(24)
        if len(header) != 24:
            raise PcapError(f"{path} is too small to be a pcap")

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
            raise PcapError(f"unsupported pcap magic: {magic.hex()}")

        packet_index = 0
        while True:
            packet_header = handle.read(16)
            if not packet_header:
                break
            if len(packet_header) != 16:
                raise PcapError("truncated pcap packet header")

            ts_sec, ts_frac, caplen, _origlen = struct.unpack(endian + "IIII", packet_header)
            frame = handle.read(caplen)
            if len(frame) != caplen:
                raise PcapError("truncated pcap packet payload")

            yield packet_index, ts_sec + ts_frac * scale, frame
            packet_index += 1


def ip_to_text(raw: bytes) -> str:
    return socket.inet_ntoa(raw)


def network_payloads(path: Path) -> Iterator[Dict[str, object]]:
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
    def __init__(self) -> None:
        self.buffer = bytearray()

    def feed(self, data: bytes) -> List[Dict[str, object]]:
        self.buffer.extend(data)
        out: List[Dict[str, object]] = []

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


class TcpStreamState:
    def __init__(self) -> None:
        self.parser = FusionEngineParser()
        self.next_seq: Optional[int] = None
        self.gaps = 0

    def feed(self, seq: Optional[int], payload: bytes) -> List[Dict[str, object]]:
        if seq is None:
            return self.parser.feed(payload)

        if self.next_seq is None:
            self.next_seq = seq + len(payload)
            return self.parser.feed(payload)

        if seq < self.next_seq:
            overlap = self.next_seq - seq
            if overlap >= len(payload):
                return []
            payload = payload[overlap:]
            self.next_seq = self.next_seq + len(payload)
            return self.parser.feed(payload)

        if seq > self.next_seq:
            self.gaps += 1
            self.parser = FusionEngineParser()

        self.next_seq = seq + len(payload)
        return self.parser.feed(payload)


class P1ImuPcapReplay(Node):
    def __init__(self) -> None:
        super().__init__("p1_imu_pcap_replay")

        pcap_path_text = str(self.declare_parameter("pcap_path", "").value)
        self.pcap_path = Path(pcap_path_text)
        self.output_topic = str(self.declare_parameter("output_topic", "/atlas/imu_calibrated").value)
        self.clock_topic = str(self.declare_parameter("clock_topic", "/clock").value)
        self.frame_id = str(self.declare_parameter("frame_id", "cg").value)
        self.pace_mode = str(self.declare_parameter("pace_mode", "clock").value).lower()
        self.play_rate = float(self.declare_parameter("play_rate", 1.0).value)
        self.max_clock_lag_sec = float(self.declare_parameter("max_clock_lag_sec", 1.0).value)
        self.max_messages = int(self.declare_parameter("max_messages", 0).value)

        if not pcap_path_text:
            raise RuntimeError("pcap_path parameter is required")
        if not self.pcap_path.exists():
            raise RuntimeError(f"pcap_path does not exist: {self.pcap_path}")
        if self.pace_mode not in {"clock", "wall", "none"}:
            raise RuntimeError("pace_mode must be one of: clock, wall, none")
        if self.play_rate <= 0.0:
            raise RuntimeError("play_rate must be > 0")

        self.pub = self.create_publisher(Imu, self.output_topic, 100)
        self.clock_cond = threading.Condition()
        self.current_clock: Optional[float] = None
        self.shutdown_requested = False

        if self.pace_mode == "clock":
            clock_qos = QoSProfile(
                history=HistoryPolicy.KEEP_LAST,
                depth=10,
                reliability=ReliabilityPolicy.BEST_EFFORT,
                durability=DurabilityPolicy.VOLATILE,
            )
            self.clock_sub = self.create_subscription(
                Clock, self.clock_topic, self.clock_callback, clock_qos)
        else:
            self.clock_sub = None

        self.get_logger().info(
            f"PCAP IMU replay ready: pcap={self.pcap_path} output={self.output_topic} "
            f"pace_mode={self.pace_mode}"
        )
        self.worker = threading.Thread(target=self.run_replay, name="p1_imu_pcap_replay", daemon=True)
        self.worker.start()

    def destroy_node(self) -> bool:
        self.shutdown_requested = True
        with self.clock_cond:
            self.clock_cond.notify_all()
        if self.worker.is_alive():
            self.worker.join(timeout=2.0)
        return super().destroy_node()

    def clock_callback(self, msg: Clock) -> None:
        sec = float(msg.clock.sec) + float(msg.clock.nanosec) * 1e-9
        with self.clock_cond:
            self.current_clock = sec
            self.clock_cond.notify_all()

    def wait_for_clock(self) -> Optional[float]:
        with self.clock_cond:
            while self.current_clock is None and not self.shutdown_requested and rclpy.ok():
                self.clock_cond.wait(timeout=0.1)
            return self.current_clock

    def wait_until_packet_time(self, packet_time: float) -> bool:
        if self.pace_mode == "none":
            return True

        if self.pace_mode == "wall":
            if not hasattr(self, "_wall_start"):
                self._wall_start = time.monotonic()
                self._pcap_start = packet_time
            elapsed_pcap = (packet_time - self._pcap_start) / self.play_rate
            target = self._wall_start + max(0.0, elapsed_pcap)
            while not self.shutdown_requested and rclpy.ok():
                remaining = target - time.monotonic()
                if remaining <= 0.0:
                    return True
                time.sleep(min(remaining, 0.05))
            return False

        now = self.wait_for_clock()
        if now is None:
            return False
        if packet_time < now - self.max_clock_lag_sec:
            return False

        with self.clock_cond:
            while not self.shutdown_requested and rclpy.ok():
                now = self.current_clock
                if now is not None and packet_time <= now:
                    return True
                self.clock_cond.wait(timeout=0.05)
        return False

    def run_replay(self) -> None:
        streams: Dict[Tuple[str, str, str], TcpStreamState] = {}
        counts: Counter[int] = Counter()
        published = 0
        skipped_late_packets = 0

        try:
            for packet in network_payloads(self.pcap_path):
                if self.shutdown_requested or not rclpy.ok():
                    break

                packet_time = float(packet["pcap_time"])
                if not self.wait_until_packet_time(packet_time):
                    skipped_late_packets += 1
                    continue

                key = (str(packet["transport"]), str(packet["src"]), str(packet["dst"]))
                state = streams.setdefault(key, TcpStreamState())
                payload = packet["payload"]
                assert isinstance(payload, (bytes, bytearray))
                messages = state.feed(packet["tcp_seq"], bytes(payload))
                for message in messages:
                    msg_type = int(message["message_type"])
                    counts[msg_type] += 1
                    if msg_type != IMU_OUTPUT:
                        continue

                    imu = self.decode_imu_output(message["payload"])
                    if imu is None:
                        continue
                    self.pub.publish(imu)
                    published += 1
                    if self.max_messages > 0 and published >= self.max_messages:
                        self.get_logger().info(f"Reached max_messages={self.max_messages}; stopping.")
                        raise StopIteration

        except StopIteration:
            pass
        except Exception as exc:
            self.get_logger().error(f"PCAP IMU replay failed: {exc}")
        finally:
            gap_count = sum(stream.gaps for stream in streams.values())
            self.get_logger().info(
                f"PCAP IMU replay done: imu_published={published} "
                f"skipped_late_packets={skipped_late_packets} tcp_gap_resets={gap_count} "
                f"top_message_types={counts.most_common(8)}"
            )
            rclpy.try_shutdown()

    def decode_imu_output(self, payload: object) -> Optional[Imu]:
        if not isinstance(payload, (bytes, bytearray)):
            return None
        if len(payload) < IMU_OUTPUT_STRUCT.size:
            return None

        values = IMU_OUTPUT_STRUCT.unpack_from(payload, 0)
        p1_seconds = int(values[0])
        p1_fraction_ns = int(values[1])
        if p1_seconds == 0xFFFFFFFF or p1_fraction_ns == 0xFFFFFFFF:
            return None

        accel = values[2:5]
        accel_std = values[5:8]
        gyro = values[8:11]
        gyro_std = values[11:14]

        msg = Imu()
        msg.header.stamp.sec = p1_seconds
        msg.header.stamp.nanosec = p1_fraction_ns
        msg.header.frame_id = self.frame_id

        msg.linear_acceleration.x = accel[0]
        msg.linear_acceleration.y = accel[1]
        msg.linear_acceleration.z = accel[2]
        msg.angular_velocity.x = gyro[0]
        msg.angular_velocity.y = gyro[1]
        msg.angular_velocity.z = gyro[2]

        msg.linear_acceleration_covariance[0] = accel_std[0] * accel_std[0]
        msg.linear_acceleration_covariance[4] = accel_std[1] * accel_std[1]
        msg.linear_acceleration_covariance[8] = accel_std[2] * accel_std[2]
        msg.angular_velocity_covariance[0] = gyro_std[0] * gyro_std[0]
        msg.angular_velocity_covariance[4] = gyro_std[1] * gyro_std[1]
        msg.angular_velocity_covariance[8] = gyro_std[2] * gyro_std[2]
        return msg


def main() -> None:
    rclpy.init()
    node = P1ImuPcapReplay()
    try:
        rclpy.spin(node)
    except (ExternalShutdownException, KeyboardInterrupt):
        pass
    finally:
        try:
            node.destroy_node()
        except KeyboardInterrupt:
            pass


if __name__ == "__main__":
    main()
