"""Trilateration verification node.

Compares /microuwb/position_estimate against /drone/ground_truth_pose using
nearest-timestamp matching (±50ms tolerance, ring buffer of last 200 samples).
Logs running error statistics (median, p95, max) over the last 100 matched
pairs every 2 seconds.
"""

from __future__ import annotations

import collections
import math
import time
from typing import Deque

import numpy as np

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped

_MATCH_TOL_S   = 0.050   # timestamp match tolerance
_WINDOW        = 100     # error window for rolling stats
_BUFFER_SIZE   = 200     # ring buffer depth for each topic
_PRINT_PERIOD  = 2.0     # seconds between stats printout


def _stamp_to_sec(stamp) -> float:
    return stamp.sec + stamp.nanosec * 1e-9


def _dist(a: PoseStamped, b: PoseStamped) -> float:
    pa, pb = a.pose.position, b.pose.position
    return math.sqrt((pa.x - pb.x)**2 + (pa.y - pb.y)**2 + (pa.z - pb.z)**2)


class VerifyTrilateration(Node):
    def __init__(self) -> None:
        super().__init__("verify_trilateration")

        # Ring buffers keyed by timestamp (seconds, float)
        self._gt_buf:  Deque[PoseStamped] = collections.deque(maxlen=_BUFFER_SIZE)
        self._est_buf: Deque[PoseStamped] = collections.deque(maxlen=_BUFFER_SIZE)

        # Rolling error window
        self._errors: Deque[float] = collections.deque(maxlen=_WINDOW)
        self._total_matched = 0

        self.create_subscription(PoseStamped, "/drone/ground_truth_pose",
                                 self._gt_cb, 10)
        self.create_subscription(PoseStamped, "/microuwb/position_estimate",
                                 self._est_cb, 10)

        self._last_print = time.monotonic()
        self.get_logger().info("Verify-trilateration node ready — waiting for data…")

    # ── Callbacks ────────────────────────────────────────────────────────────

    def _gt_cb(self, msg: PoseStamped) -> None:
        self._gt_buf.append(msg)

    def _est_cb(self, msg: PoseStamped) -> None:
        self._est_buf.append(msg)
        self._try_match(msg)
        self._maybe_print()

    # ── Matching ─────────────────────────────────────────────────────────────

    def _try_match(self, est: PoseStamped) -> None:
        if not self._gt_buf:
            return
        t_est = _stamp_to_sec(est.header.stamp)

        # Find nearest ground-truth by timestamp
        best: PoseStamped | None = None
        best_dt = float("inf")
        for gt in self._gt_buf:
            dt = abs(_stamp_to_sec(gt.header.stamp) - t_est)
            if dt < best_dt:
                best_dt = dt
                best = gt

        if best_dt > _MATCH_TOL_S:
            return  # no match within tolerance

        err = _dist(best, est)
        self._errors.append(err)
        self._total_matched += 1

    # ── Stats printout ───────────────────────────────────────────────────────

    def _maybe_print(self) -> None:
        now = time.monotonic()
        if now - self._last_print < _PRINT_PERIOD:
            return
        self._last_print = now

        if len(self._errors) < 5:
            self.get_logger().info(
                f"[verify] waiting for matches… ({self._total_matched} so far)"
            )
            return

        arr = np.array(self._errors)
        median = float(np.median(arr))
        p95    = float(np.percentile(arr, 95))
        maximum = float(arr.max())

        # PASS / WARN indicators
        med_ok = "PASS" if median < 0.30 else "WARN"
        p95_ok = "PASS" if p95    < 1.00 else "WARN"

        self.get_logger().info(
            f"[verify] n={self._total_matched} | "
            f"median={median*100:.1f}cm [{med_ok}]  "
            f"p95={p95*100:.1f}cm [{p95_ok}]  "
            f"max={maximum*100:.1f}cm  "
            f"(window={len(arr)})"
        )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = VerifyTrilateration()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
