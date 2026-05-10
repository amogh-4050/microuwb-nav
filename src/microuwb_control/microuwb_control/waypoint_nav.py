"""Waypoint navigation node — step 8.

Publishes a sequence of 3D setpoints to /microuwb/setpoint (PoseStamped).
Advances to the next waypoint only after the drone arrives within
arrival_radius_m AND dwells there for dwell_s seconds.

Trajectory sequence (loops forever):
  Square (4 pts) → Circle (12 pts) → Triangle (3 pts) → repeat

All waypoints are generated from YAML params and clamped to safe inner bounds
so they stay within the UWB grid (wall avoidance by construction).
"""

from __future__ import annotations

import math
import time

import numpy as np

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped

# Safe inner bounds — 0.5m margin from UWB grid limits and room walls
_XY_MIN = np.array([0.5, 0.5])
_XY_MAX = np.array([4.5, 3.5])
_Z_MIN, _Z_MAX = 0.4, 2.5

# Shape boundary indices in the flat waypoint list
_SHAPE_NAMES = ["cuboid", "circle"]


def _clamp_wp(wp: np.ndarray) -> np.ndarray:
    out = wp.copy()
    out[:2] = np.clip(out[:2], _XY_MIN, _XY_MAX)
    out[2]  = float(np.clip(out[2], _Z_MIN, _Z_MAX))
    return out


def _build_cuboid(z_low: float, z_high: float) -> list[np.ndarray]:
    # 6 of 8 cuboid corners — skips two to avoid redundant edges
    return [_clamp_wp(np.array(p)) for p in [
        [1.2, 1.0, z_low],
        [3.8, 1.0, z_low],
        [3.8, 3.0, z_low],
        [3.8, 3.0, z_high],
        [1.2, 3.0, z_high],
        [1.2, 1.0, z_high],
    ]]


def _build_circle(z: float, n: int, radius: float) -> list[np.ndarray]:
    cx, cy = 2.5, 2.0
    wps = []
    for i in range(n):
        angle = 2.0 * math.pi * i / n
        wps.append(_clamp_wp(np.array([
            cx + radius * math.cos(angle),
            cy + radius * math.sin(angle),
            z,
        ])))
    return wps


def _build_triangle(z: float) -> list[np.ndarray]:
    return [_clamp_wp(np.array(p)) for p in [
        [1.0, 0.8, z],
        [4.0, 0.8, z],
        [2.5, 3.2, z],
    ]]


class WaypointNav(Node):
    def __init__(self) -> None:
        super().__init__("waypoint_nav")

        self.declare_parameter("cruise_z",        1.3)
        self.declare_parameter("cuboid_z_low",     1.0)
        self.declare_parameter("cuboid_z_high",    1.6)
        self.declare_parameter("arrival_radius_m", 0.30)
        self.declare_parameter("dwell_s",          2.0)
        self.declare_parameter("circle_points",    12)
        self.declare_parameter("circle_radius_m",  1.4)
        self.declare_parameter("publish_hz",       5.0)

        g = lambda n: self.get_parameter(n).get_parameter_value()  # noqa: E731
        z_low   = g("cuboid_z_low").double_value
        z_high  = g("cuboid_z_high").double_value
        z_circ  = g("cruise_z").double_value
        self._r = g("arrival_radius_m").double_value
        self._d = g("dwell_s").double_value
        n_circ  = g("circle_points").integer_value
        r_circ  = g("circle_radius_m").double_value
        hz      = g("publish_hz").double_value

        cub = _build_cuboid(z_low, z_high)
        ci  = _build_circle(z_circ, n_circ, r_circ)

        # Sequence: cuboid → circle → repeat (no triangle)
        self._shape_starts = [0, len(cub)]
        self._waypoints    = cub + ci
        self._total        = len(self._waypoints)

        self.get_logger().info(
            f"Waypoints loaded: {len(cub)} cuboid + {len(ci)} circle "
            f"= {self._total} total.  z_low={z_low:.2f}m z_high={z_high:.2f}m"
        )

        self._idx:          int   = 0
        self._dwell_start:  float | None = None
        self._pos:          np.ndarray | None = None

        self.create_subscription(PoseStamped, "/drone/ground_truth_pose", self._pos_cb, 10)
        self._pub = self.create_publisher(PoseStamped, "/microuwb/setpoint", 10)
        self.create_timer(1.0 / hz, self._tick)

    # ── Callbacks ─────────────────────────────────────────────────────────────

    def _pos_cb(self, msg: PoseStamped) -> None:
        p = msg.pose.position
        self._pos = np.array([p.x, p.y, p.z])

    def _tick(self) -> None:
        if self._pos is None:
            return

        wp = self._waypoints[self._idx]
        self._publish_wp(wp)

        dist = float(np.linalg.norm(wp - self._pos))
        if dist < self._r:
            if self._dwell_start is None:
                self._dwell_start = time.monotonic()
            elif time.monotonic() - self._dwell_start >= self._d:
                self._advance()
        else:
            self._dwell_start = None  # drone drifted — reset dwell

    def _advance(self) -> None:
        old_idx = self._idx
        self._idx = (self._idx + 1) % self._total
        self._dwell_start = None

        old_shape = self._shape_name(old_idx)
        new_shape = self._shape_name(self._idx)
        transition = f" ── shape: {old_shape} → {new_shape}" if old_shape != new_shape else ""
        self.get_logger().info(
            f"wp {old_idx:02d}/{self._total - 1} reached → advancing to {self._idx:02d}{transition}"
        )

    def _shape_name(self, idx: int) -> str:
        for i, start in enumerate(reversed(self._shape_starts)):
            if idx >= start:
                return _SHAPE_NAMES[len(self._shape_starts) - 1 - i]
        return "unknown"

    def _publish_wp(self, wp: np.ndarray) -> None:
        msg = PoseStamped()
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        msg.pose.position.x = float(wp[0])
        msg.pose.position.y = float(wp[1])
        msg.pose.position.z = float(wp[2])
        msg.pose.orientation.w = 1.0
        self._pub.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = WaypointNav()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
