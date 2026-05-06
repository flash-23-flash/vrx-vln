#!/usr/bin/env python3
"""Publish one instruction message to /vln/instruction_text."""

from __future__ import annotations

import sys
import time
from typing import List

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class InstructionPublisher(Node):
    def __init__(self, text: str, topic: str) -> None:
        super().__init__("publish_instruction")
        self._pub = self.create_publisher(String, topic, 10)
        self._text = text
        self._timer = self.create_timer(0.5, self._once)
        self._sent = False

    def _once(self) -> None:
        if self._sent:
            return
        self._pub.publish(String(data=self._text))
        self.get_logger().info(f"Published instruction: {self._text}")
        self._sent = True
        self.destroy_timer(self._timer)


def main(args: List[str] | None = None) -> None:
    argv = args if args is not None else sys.argv
    text = "Go to waypoint A"
    topic = "/vln/instruction_text"
    if len(argv) >= 2:
        text = argv[1]
    if len(argv) >= 3:
        topic = argv[2]

    rclpy.init(args=argv)
    node = InstructionPublisher(text=text, topic=topic)
    start = time.time()
    try:
        while rclpy.ok() and time.time() - start < 2.0:
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        try:
            node.destroy_node()
        except BaseException:
            pass
        if rclpy.ok():
            rclpy.shutdown()
