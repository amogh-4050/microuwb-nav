"""Trilateration node.

Consumes /microuwb/ranges (UWBRangeArray) and publishes a per-cycle 3D
position estimate via weighted nonlinear least-squares with Huber robust loss.
NLOS measurements are included but down-weighted when their residuals exceed
f_scale_m — no hard drop. No temporal filtering — that's step 5 (KF).

Math reference: Core Electronics tutorial linearization as warm-start, then
scipy.optimize.least_squares (TRF + Huber) with residual weighting by
1/sqrt(variance).
"""

from __future__ import annotations

import numpy as np
from scipy.optimize import least_squares

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy,
)

from geometry_msgs.msg import PoseStamped
from microuwb_msgs.msg import AnchorArray, UWBRangeArray

# Room bounds for sanity clamping [min, max] per axis
_ROOM_MIN = np.array([0.0, 0.0, 0.0])
_ROOM_MAX = np.array([5.0, 4.0, 3.0])


class TrilaterationNode(Node):
    def __init__(self) -> None:
        super().__init__("trilateration_node")

        self.declare_parameter("min_valid_ranges", 4)
        self.declare_parameter("nonlinear_max_iter", 50)
        self.declare_parameter("room_center", [2.5, 2.0, 1.5])
        self.declare_parameter("weight_by_variance", True)
        self.declare_parameter("f_scale_m", 0.30)

        self._min_valid   = self.get_parameter("min_valid_ranges").get_parameter_value().integer_value
        self._max_nfev    = self.get_parameter("nonlinear_max_iter").get_parameter_value().integer_value
        self._room_center = np.array(
            self.get_parameter("room_center").get_parameter_value().double_array_value
        )
        self._weight      = self.get_parameter("weight_by_variance").get_parameter_value().bool_value
        self._f_scale     = self.get_parameter("f_scale_m").get_parameter_value().double_value

        self._anchor_positions: dict[int, np.ndarray] = {}
        self._last_estimate: np.ndarray | None = None
        self._skipped_cycles = 0
        self._nlos_included_cycles = 0

        transient_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
        )
        self.create_subscription(AnchorArray, "/microuwb/anchors",
                                 self._anchors_cb, transient_qos)
        self.create_subscription(UWBRangeArray, "/microuwb/ranges",
                                 self._ranges_cb, 10)

        self._pub = self.create_publisher(PoseStamped, "/microuwb/position_estimate", 10)
        self.get_logger().info("Trilateration node ready.")

    # ── Callbacks ────────────────────────────────────────────────────────────

    def _anchors_cb(self, msg: AnchorArray) -> None:
        self._anchor_positions = {
            a.id: np.array([a.position.x, a.position.y, a.position.z])
            for a in msg.anchors
        }
        self.get_logger().info(
            f"Anchor map updated: {sorted(self._anchor_positions.keys())}"
        )

    def _ranges_cb(self, msg: UWBRangeArray) -> None:
        if not self._anchor_positions:
            return

        # Include all valid ranges — NLOS measurements are down-weighted by Huber
        # loss in _solve() when their residuals exceed f_scale_m, rather than dropped.
        measurements: list[tuple[np.ndarray, float, float]] = []
        n_nlos = 0
        for r in msg.ranges:
            if r.anchor_id not in self._anchor_positions:
                continue
            if not r.line_of_sight:
                n_nlos += 1
            measurements.append((
                self._anchor_positions[r.anchor_id],
                float(r.range_m),
                max(float(r.range_variance), 1e-6),  # guard against zero
            ))

        if len(measurements) < self._min_valid:
            self.get_logger().debug(
                f"Only {len(measurements)} valid ranges (need {self._min_valid}) — skipping cycle."
            )
            self._skipped_cycles += 1
            return

        if n_nlos > 0:
            self._nlos_included_cycles += 1
            self.get_logger().debug(
                f"{n_nlos} NLOS anchor(s) included via Huber weighting (f_scale={self._f_scale:.2f}m)"
            )

        estimate = self._solve(measurements)
        if estimate is None:
            return

        # Sanity clamp — catches solver divergence
        clamped = np.clip(estimate, _ROOM_MIN, _ROOM_MAX)
        if not np.allclose(clamped, estimate, atol=0.01):
            self.get_logger().warn(
                f"Estimate {estimate.tolist()} outside room bounds — clamped.",
                throttle_duration_sec=2.0,
            )
        self._last_estimate = clamped

        out = PoseStamped()
        out.header.stamp    = msg.header.stamp
        out.header.frame_id = "map"
        out.pose.position.x = float(clamped[0])
        out.pose.position.y = float(clamped[1])
        out.pose.position.z = float(clamped[2])
        # Identity quaternion — trilateration gives no attitude
        out.pose.orientation.w = 1.0
        self._pub.publish(out)

    # ── Solver ───────────────────────────────────────────────────────────────

    def _solve(
        self,
        measurements: list[tuple[np.ndarray, float, float]],
    ) -> np.ndarray | None:
        anchors   = np.array([m[0] for m in measurements])   # (N, 3)
        ranges    = np.array([m[1] for m in measurements])   # (N,)
        variances = np.array([m[2] for m in measurements])   # (N,)

        # ── Linear warm-start (Core Electronics tutorial linearization) ──────
        # Reference anchor = index 0
        p_linear = self._linear_lstsq(anchors, ranges)

        # ── Nonlinear refinement (Levenberg-Marquardt) ───────────────────────
        x0 = self._last_estimate if self._last_estimate is not None else p_linear

        weights = 1.0 / np.sqrt(variances) if self._weight else np.ones(len(variances))

        def residuals(p: np.ndarray) -> np.ndarray:
            diffs = anchors - p[np.newaxis, :]          # (N, 3)
            predicted = np.sqrt(np.sum(diffs ** 2, axis=1))  # (N,)
            return weights * (predicted - ranges)

        try:
            # TRF required for Huber loss (method='lm' ignores loss parameter).
            result = least_squares(
                residuals, x0,
                method="trf",
                loss="huber",
                f_scale=self._f_scale,
                max_nfev=self._max_nfev,
            )
            return result.x
        except Exception as exc:  # noqa: BLE001 — fall back, don't crash
            self.get_logger().warn(f"Nonlinear solver failed ({exc}); using linear estimate.")
            return p_linear

    @staticmethod
    def _linear_lstsq(anchors: np.ndarray, ranges: np.ndarray) -> np.ndarray:
        """Linearized trilateration (tutorial math).

        Uses anchor[0] as reference. Builds A (N-1, 3) and b (N-1,) then
        solves via numpy least-squares (handles N > 3 overconstrained case).
        """
        n = len(anchors)
        if n < 2:
            # Can't linearize; return midpoint of available anchors
            return anchors.mean(axis=0)

        a0, r0 = anchors[0], ranges[0]
        A = np.zeros((n - 1, 3))
        b = np.zeros(n - 1)

        for k in range(1, n):
            ak, rk = anchors[k], ranges[k]
            A[k - 1] = 2.0 * (ak - a0)
            b[k - 1] = (
                (ak[0]**2 - a0[0]**2)
                + (ak[1]**2 - a0[1]**2)
                + (ak[2]**2 - a0[2]**2)
                + r0**2 - rk**2
            )

        result, _, _, _ = np.linalg.lstsq(A, b, rcond=None)
        return result


def main(args=None) -> None:
    rclpy.init(args=args)
    node = TrilaterationNode()
    try:
        rclpy.spin(node)
    finally:
        print(
            f"[trilateration] shutdown: {node._skipped_cycles} skipped cycles "
            f"(valid ranges < {node._min_valid}), "
            f"{node._nlos_included_cycles} cycles with ≥1 NLOS anchor included via Huber"
        )
        node.destroy_node()
        rclpy.shutdown()
