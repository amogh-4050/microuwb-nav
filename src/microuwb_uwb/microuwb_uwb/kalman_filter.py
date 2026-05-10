"""Kalman filter node (step 5).

Subscribes to /microuwb/position_estimate (PoseStamped from trilateration)
and publishes /microuwb/position_filtered (PoseStamped) after a 6D
constant-velocity linear Kalman filter.

Math follows Core Electronics tutorial SimpleKalman3D structure verbatim.
"""

from __future__ import annotations

import numpy as np

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped


def _stamp_to_sec(stamp) -> float:
    return stamp.sec + stamp.nanosec * 1e-9


class KalmanFilterNode(Node):
    def __init__(self) -> None:
        super().__init__("kalman_filter")

        self.declare_parameter("process_noise_q", 0.12)
        self.declare_parameter("measurement_noise_r", 1.1)
        self.declare_parameter("dt_nominal_s", 0.10)
        self.declare_parameter("dt_min_s", 0.05)
        self.declare_parameter("dt_max_s", 0.20)
        self.declare_parameter("reset_gap_s", 0.5)
        self.declare_parameter("initial_position_variance", 1.0)
        self.declare_parameter("initial_velocity_variance", 1.0)

        self._Q        = self.get_parameter("process_noise_q").get_parameter_value().double_value
        self._R        = self.get_parameter("measurement_noise_r").get_parameter_value().double_value
        self._dt_min   = self.get_parameter("dt_min_s").get_parameter_value().double_value
        self._dt_max   = self.get_parameter("dt_max_s").get_parameter_value().double_value
        self._reset_gap = self.get_parameter("reset_gap_s").get_parameter_value().double_value
        self._pos_var  = self.get_parameter("initial_position_variance").get_parameter_value().double_value
        self._vel_var  = self.get_parameter("initial_velocity_variance").get_parameter_value().double_value

        # Filter state
        self._x: np.ndarray | None = None   # [x, y, z, vx, vy, vz]
        self._P: np.ndarray | None = None   # 6×6 covariance
        self._last_stamp: float | None = None

        # H: maps 6D state → 3D position measurement
        self._H = np.zeros((3, 6))
        self._H[0, 0] = 1.0
        self._H[1, 1] = 1.0
        self._H[2, 2] = 1.0

        self.create_subscription(
            PoseStamped, "/microuwb/position_estimate", self._cb, 10
        )
        self._pub = self.create_publisher(
            PoseStamped, "/microuwb/position_filtered", 10
        )
        self.get_logger().info(
            f"Kalman filter node ready (Q={self._Q}, R={self._R})."
        )

    # ── Initialization ────────────────────────────────────────────────────────

    def _init_state(self, msg: PoseStamped) -> None:
        p = msg.pose.position
        self._x = np.array([p.x, p.y, p.z, 0.0, 0.0, 0.0])
        self._P = np.diag([self._pos_var] * 3 + [self._vel_var] * 3)
        self._last_stamp = _stamp_to_sec(msg.header.stamp)

    # ── Callback ──────────────────────────────────────────────────────────────

    def _cb(self, msg: PoseStamped) -> None:
        t = _stamp_to_sec(msg.header.stamp)
        z = np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z])

        if self._x is None:
            self._init_state(msg)
            return

        dt_raw = t - self._last_stamp

        if dt_raw > self._reset_gap:
            self.get_logger().warn(
                f"KF reset: gap={dt_raw:.3f}s > {self._reset_gap}s | "
                f"last_stamp={self._last_stamp:.3f} | "
                f"last_pos=[{self._x[0]:.3f}, {self._x[1]:.3f}, {self._x[2]:.3f}]"
            )
            self._init_state(msg)
            return

        dt = float(np.clip(dt_raw, self._dt_min, self._dt_max))
        self._last_stamp = t

        self._x, self._P = self._predict_update(self._x, self._P, z, dt)

        out = PoseStamped()
        out.header.stamp    = msg.header.stamp
        out.header.frame_id = "map"
        out.pose.position.x = float(self._x[0])
        out.pose.position.y = float(self._x[1])
        out.pose.position.z = float(self._x[2])
        out.pose.orientation.w = 1.0
        self._pub.publish(out)

    # ── Filter ────────────────────────────────────────────────────────────────

    def _predict_update(
        self,
        x: np.ndarray,
        P: np.ndarray,
        z: np.ndarray,
        dt: float,
    ) -> tuple[np.ndarray, np.ndarray]:
        # Transition matrix — constant-velocity model
        F = np.eye(6)
        F[0, 3] = dt
        F[1, 4] = dt
        F[2, 5] = dt

        # Prediction
        x_pred = np.empty(6)
        x_pred[0:3] = x[0:3] + x[3:6] * dt
        x_pred[3:6] = x[3:6]
        P_pred = F @ P @ F.T + self._Q * np.eye(6)

        # Update — Core Electronics tutorial structure verbatim
        H = self._H
        y = z - H @ x_pred                             # innovation (3D)
        S = H @ P_pred @ H.T + self._R * np.eye(3)    # innovation covariance
        K = P_pred @ H.T @ np.linalg.inv(S)            # Kalman gain (6×3)

        # Standard KF update — handles both position and velocity optimally
        x_new = x_pred + K @ y
        P_new = (np.eye(6) - K @ H) @ P_pred

        return x_new, P_new


def main(args=None) -> None:
    rclpy.init(args=args)
    node = KalmanFilterNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
