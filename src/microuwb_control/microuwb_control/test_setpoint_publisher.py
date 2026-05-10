#!/usr/bin/env python3
"""Publishes /microuwb/setpoint (PoseStamped) at 10 Hz.

--test hover   : constant (2.5, 2.0, 1.5)
--test step    : (2.5, 2.0, 1.5) for 5 s, then (3.5, 2.0, 1.5)
--test square  : 4 corners at 1.0 m altitude, 10 s per leg
"""
import sys
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped

HOVER = (2.5, 2.0, 1.5)
STEP_A = (2.5, 2.0, 1.5)
STEP_B = (3.5, 2.0, 1.5)
SQUARE = [
    (1.0, 1.0, 1.0),
    (4.0, 1.0, 1.0),
    (4.0, 3.0, 1.0),
    (1.0, 3.0, 1.0),
]


class SetpointPublisher(Node):
    def __init__(self, mode: str) -> None:
        super().__init__("test_setpoint_publisher")
        self._mode = mode
        self._pub = self.create_publisher(PoseStamped, "/microuwb/setpoint", 10)
        self._t = 0.0
        self._leg = 0
        self.create_timer(0.1, self._tick)
        self.get_logger().info(f"Setpoint publisher started — mode={mode}")

    def _tick(self) -> None:
        self._t += 0.1
        if self._mode == "hover":
            xyz = HOVER
        elif self._mode == "step":
            xyz = STEP_A if self._t < 5.0 else STEP_B
        elif self._mode == "square":
            leg_idx = int(self._t / 10.0) % len(SQUARE)
            xyz = SQUARE[leg_idx]
        else:
            return

        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "world"
        msg.pose.position.x = float(xyz[0])
        msg.pose.position.y = float(xyz[1])
        msg.pose.position.z = float(xyz[2])
        msg.pose.orientation.w = 1.0
        self._pub.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    mode = "hover"
    for i, a in enumerate(sys.argv):
        if a == "--test" and i + 1 < len(sys.argv):
            mode = sys.argv[i + 1]
    if mode not in ("hover", "step", "square"):
        print(f"Unknown test mode '{mode}', defaulting to hover")
        mode = "hover"
    node = SetpointPublisher(mode)
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
