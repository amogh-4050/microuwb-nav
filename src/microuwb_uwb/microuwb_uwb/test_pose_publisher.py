"""Test pose publisher — standalone figure-8 trajectory for uwb_range_simulator testing.

Publishes /drone/ground_truth_pose at 50 Hz.
Trajectory: figure-8 (lemniscate) in x-y centered on (2.5, 2.0), radius 1.5m,
z sinusoidal between 0.5m and 2.0m.  Period: 30 seconds, loops indefinitely.
"""

import math
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped


_CX, _CY = 2.5, 2.0
_RADIUS   = 1.5
_PERIOD   = 30.0        # seconds per figure-8 cycle
_RATE_HZ  = 50.0


class TestPosePublisher(Node):
    def __init__(self) -> None:
        super().__init__("test_pose_publisher")
        self._pub = self.create_publisher(PoseStamped, "/drone/ground_truth_pose", 10)
        self._t0  = time.monotonic()
        self.create_timer(1.0 / _RATE_HZ, self._timer_cb)
        self.get_logger().info(
            f"Test pose publisher: figure-8 cx={_CX} cy={_CY} r={_RADIUS}m period={_PERIOD}s"
        )

    def _timer_cb(self) -> None:
        elapsed = (time.monotonic() - self._t0) % _PERIOD
        omega   = 2.0 * math.pi / _PERIOD

        # Lemniscate-style figure-8 in x-y plane
        x = _CX + _RADIUS * math.sin(omega * elapsed)
        y = _CY + (_RADIUS / 2.0) * math.sin(2.0 * omega * elapsed)

        # z oscillates sinusoidally between 0.5 m and 2.0 m
        z = 1.25 + 0.75 * math.sin(omega * elapsed)

        msg = PoseStamped()
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        msg.pose.position.x = x
        msg.pose.position.y = y
        msg.pose.position.z = z
        # Identity orientation (no rotation needed for a point-mass drone placeholder)
        msg.pose.orientation.w = 1.0

        self._pub.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = TestPosePublisher()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
