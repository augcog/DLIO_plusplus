#!/usr/bin/env python3
"""Republish Odometry with an explicit map-frame translation.

This small bridge is intended for replay datasets whose recorded INS odometry
uses the same ENU axes as a localization map but a different documented height
or origin convention.  The translation is explicit at the command line; no
site-specific offset is embedded in the tool.
"""

from __future__ import annotations

import argparse
import math
from typing import Sequence

import rclpy
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)


class OdomOffsetBridge(Node):
    def __init__(
        self,
        input_topic: str,
        output_topic: str,
        offset: tuple[float, float, float],
        frame_id: str,
    ) -> None:
        super().__init__("odom_offset_bridge")
        self._offset = offset
        self._frame_id = frame_id
        self._publisher = self.create_publisher(Odometry, output_topic, 50)
        input_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=200,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self._subscription = self.create_subscription(
            Odometry, input_topic, self._callback, input_qos
        )
        self._count = 0
        self.get_logger().info(
            f"Republishing {input_topic} -> {output_topic}; "
            f"map translation={offset}, frame_id={frame_id!r}"
        )

    def _callback(self, msg: Odometry) -> None:
        msg.pose.pose.position.x += self._offset[0]
        msg.pose.pose.position.y += self._offset[1]
        msg.pose.pose.position.z += self._offset[2]
        if self._frame_id:
            msg.header.frame_id = self._frame_id
        self._publisher.publish(msg)
        self._count += 1
        if self._count % 10000 == 0:
            self.get_logger().info(f"Published {self._count} translated samples")


def parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input-topic", required=True)
    parser.add_argument("--output-topic", required=True)
    parser.add_argument(
        "--offset",
        type=float,
        nargs=3,
        metavar=("X", "Y", "Z"),
        required=True,
        help="translation added to every odometry position, in map meters",
    )
    parser.add_argument(
        "--frame-id",
        default="map",
        help="replacement header.frame_id; pass an empty string to preserve it",
    )
    args = parser.parse_args(argv)
    if not all(math.isfinite(value) for value in args.offset):
        parser.error("--offset values must be finite")
    if not args.input_topic.startswith("/") or not args.output_topic.startswith("/"):
        parser.error("--input-topic and --output-topic must be absolute ROS topics")
    if args.input_topic == args.output_topic:
        parser.error("input and output topics must differ")
    return args


def main(argv: Sequence[str] | None = None) -> int:
    args = parse_args(argv)
    rclpy.init()
    node = OdomOffsetBridge(
        args.input_topic,
        args.output_topic,
        tuple(args.offset),
        args.frame_id,
    )
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except Exception:
        # SIGINT can invalidate the rclpy context while the executor is
        # rebuilding its wait set, which Jazzy reports as RCLError rather than
        # ExternalShutdownException. Suppress only that shutdown race.
        if rclpy.ok():
            raise
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
