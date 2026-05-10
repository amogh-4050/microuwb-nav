"""Trilateration + Kalman filter verification node.

Compares /microuwb/position_estimate and /microuwb/position_filtered against
/drone/ground_truth_pose using nearest-timestamp matching (±50 ms tolerance,
ring buffer of last 200 samples). Logs rolling error statistics for BOTH
streams side by side every 2 seconds, with a per-LOS/NLOS breakdown to
validate Huber robust loss effectiveness.
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
from microuwb_msgs.msg import UWBRangeArray

_MATCH_TOL_S   = 0.050   # timestamp match tolerance
_WINDOW        = 100     # rolling error window
_BUFFER_SIZE   = 200     # ring buffer depth per topic
_PRINT_PERIOD  = 2.0     # seconds between stats printout


def _stamp_to_sec(stamp) -> float:
    return stamp.sec + stamp.nanosec * 1e-9


def _dist(a: PoseStamped, b: PoseStamped) -> float:
    pa, pb = a.pose.position, b.pose.position
    return math.sqrt((pa.x - pb.x)**2 + (pa.y - pb.y)**2 + (pa.z - pb.z)**2)


def _match_gt(
    gt_buf: Deque[PoseStamped],
    msg: PoseStamped,
) -> float | None:
    """Return error (m) vs nearest ground-truth, or None if no match within tolerance."""
    if not gt_buf:
        return None
    t = _stamp_to_sec(msg.header.stamp)
    best: PoseStamped | None = None
    best_dt = float("inf")
    for gt in gt_buf:
        dt = abs(_stamp_to_sec(gt.header.stamp) - t)
        if dt < best_dt:
            best_dt = dt
            best = gt
    if best_dt > _MATCH_TOL_S:
        return None
    return _dist(best, msg)


class VerifyTrilateration(Node):
    def __init__(self) -> None:
        super().__init__("verify_trilateration")

        self._gt_buf: Deque[PoseStamped] = collections.deque(maxlen=_BUFFER_SIZE)
        self._ranges_buf: Deque[UWBRangeArray] = collections.deque(maxlen=_BUFFER_SIZE)

        # Trilateration stream — overall
        self._trilat_errors: Deque[float] = collections.deque(maxlen=_WINDOW)
        self._trilat_matched = 0
        self._all_trilat_errors: list[float] = []

        # Trilateration stream — split by LOS/NLOS context
        self._trilat_los_errors: list[float] = []    # cycles where all anchors were LOS
        self._trilat_nlos_errors: list[float] = []   # cycles with ≥1 NLOS anchor

        # Kalman filter stream
        self._kf_errors: Deque[float] = collections.deque(maxlen=_WINDOW)
        self._kf_matched = 0
        self._all_kf_errors: list[float] = []

        self.create_subscription(PoseStamped, "/drone/ground_truth_pose",
                                 self._gt_cb, 10)
        self.create_subscription(UWBRangeArray, "/microuwb/ranges",
                                 self._ranges_cb, 10)
        self.create_subscription(PoseStamped, "/microuwb/position_estimate",
                                 self._trilat_cb, 10)
        self.create_subscription(PoseStamped, "/microuwb/position_filtered",
                                 self._kf_cb, 10)

        self._last_print = time.monotonic()
        self.get_logger().info("Verify node ready — waiting for data…")

    # ── Callbacks ────────────────────────────────────────────────────────────

    def _gt_cb(self, msg: PoseStamped) -> None:
        self._gt_buf.append(msg)

    def _ranges_cb(self, msg: UWBRangeArray) -> None:
        self._ranges_buf.append(msg)

    def _trilat_cb(self, msg: PoseStamped) -> None:
        err = _match_gt(self._gt_buf, msg)
        if err is not None:
            self._trilat_errors.append(err)
            self._trilat_matched += 1
            self._all_trilat_errors.append(err)
            # Attribute error to LOS-only or mixed-NLOS bucket
            has_nlos = self._has_nlos_at(msg)
            if has_nlos is True:
                self._trilat_nlos_errors.append(err)
            elif has_nlos is False:
                self._trilat_los_errors.append(err)
        self._maybe_print()

    def _kf_cb(self, msg: PoseStamped) -> None:
        err = _match_gt(self._gt_buf, msg)
        if err is not None:
            self._kf_errors.append(err)
            self._kf_matched += 1
            self._all_kf_errors.append(err)
        self._maybe_print()

    def _has_nlos_at(self, msg: PoseStamped) -> bool | None:
        """Return True if nearest ranges msg had ≥1 NLOS anchor, False if all-LOS, None if no match."""
        if not self._ranges_buf:
            return None
        t = _stamp_to_sec(msg.header.stamp)
        best_dt = float("inf")
        best_rng = None
        for rng in self._ranges_buf:
            dt = abs(_stamp_to_sec(rng.header.stamp) - t)
            if dt < best_dt:
                best_dt = dt
                best_rng = rng
        if best_dt > _MATCH_TOL_S or best_rng is None:
            return None
        return any(not r.line_of_sight for r in best_rng.ranges)

    # ── Stats printout ───────────────────────────────────────────────────────

    def _maybe_print(self) -> None:
        now = time.monotonic()
        if now - self._last_print < _PRINT_PERIOD:
            return
        self._last_print = now

        t_n = len(self._trilat_errors)
        k_n = len(self._kf_errors)

        if t_n < 5 and k_n < 5:
            self.get_logger().info(
                f"[verify] waiting… trilat={self._trilat_matched} matched, "
                f"kalman={self._kf_matched} matched"
            )
            return

        def _fmt(errors: Deque[float], total: int) -> str:
            if len(errors) < 5:
                return f"{'--':>6}cm  {'--':>6}cm  (n={total})"
            arr = np.array(errors)
            med = float(np.median(arr)) * 100
            p95 = float(np.percentile(arr, 95)) * 100
            med_tag = "PASS" if med < 30.0 else "WARN"
            p95_tag = "PASS" if p95 < 100.0 else "WARN"
            return (
                f"median={med:5.1f}cm [{med_tag}]  "
                f"p95={p95:5.1f}cm [{p95_tag}]  "
                f"(n={total})"
            )

        self.get_logger().info(
            f"[verify] trilat  {_fmt(self._trilat_errors, self._trilat_matched)}"
        )
        # LOS/NLOS breakdown — shows Huber loss effectiveness
        n_los  = len(self._trilat_los_errors)
        n_nlos = len(self._trilat_nlos_errors)
        if n_los >= 5:
            los_med = float(np.median(self._trilat_los_errors[-_WINDOW:])) * 100
            self.get_logger().info(
                f"[verify]   └─ all-LOS cycles:  median={los_med:5.1f}cm  (n={n_los})"
            )
        if n_nlos >= 5:
            nlos_med = float(np.median(self._trilat_nlos_errors[-_WINDOW:])) * 100
            self.get_logger().info(
                f"[verify]   └─ ≥1 NLOS cycles:  median={nlos_med:5.1f}cm  (n={n_nlos})  [Huber down-weighted]"
            )
        self.get_logger().info(
            f"[verify] kalman  {_fmt(self._kf_errors, self._kf_matched)}"
        )


def _print_histogram(node: VerifyTrilateration) -> None:
    bins = [(0, 20), (20, 50), (50, 100), (100, 200), (200, float("inf"))]
    for label, errors in [("trilat", node._all_trilat_errors), ("kalman", node._all_kf_errors)]:
        if not errors:
            print(f"[histogram] {label}: no data collected")
            continue
        arr = np.array(errors) * 100.0  # convert m → cm
        n = len(arr)
        print(f"[histogram] {label} full-run (n={n}):")
        for lo, hi in bins:
            count = int(np.sum((arr >= lo) & (arr < hi)))
            hi_str = f"{hi:.0f}" if hi != float("inf") else "∞"
            print(f"  {lo:>3}–{hi_str:>4} cm : {count:5d} ({count / n * 100:5.1f}%)")
        print(
            f"  median={np.median(arr):.1f}cm  "
            f"p95={np.percentile(arr, 95):.1f}cm  "
            f"p99={np.percentile(arr, 99):.1f}cm  "
            f"max={arr.max():.1f}cm"
        )

    # Huber effectiveness: compare all-LOS vs ≥1-NLOS trilat cycles
    print("\n[Huber breakdown — trilateration]")
    for label, errors in [("all-LOS cycles", node._trilat_los_errors),
                           ("≥1 NLOS cycles", node._trilat_nlos_errors)]:
        if len(errors) < 5:
            print(f"  {label}: insufficient data (n={len(errors)})")
            continue
        arr = np.array(errors) * 100.0
        print(
            f"  {label} (n={len(arr)}): "
            f"median={np.median(arr):.1f}cm  "
            f"p95={np.percentile(arr, 95):.1f}cm"
        )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = VerifyTrilateration()
    try:
        rclpy.spin(node)
    finally:
        _print_histogram(node)
        node.destroy_node()
        rclpy.shutdown()
