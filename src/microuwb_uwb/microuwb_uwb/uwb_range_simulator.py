"""UWB range simulator node.

Interpolates a pre-computed Sionna CIR HDF5 table at the drone's current position
and publishes DW3000-realistic noisy range measurements to all 5 anchors at 10 Hz.
"""

import numpy as np
import h5py
from scipy.ndimage import map_coordinates

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile, DurabilityPolicy, ReliabilityPolicy, HistoryPolicy
)

from geometry_msgs.msg import PoseStamped
from microuwb_msgs.msg import AnchorArray, UWBRange, UWBRangeArray


class UWBRangeSimulator(Node):
    def __init__(self) -> None:
        super().__init__("uwb_range_simulator")

        # ── Parameters ──────────────────────────────────────────────────────
        self.declare_parameter("cir_table_path", "")
        self.declare_parameter("update_rate_hz", 10.0)
        self.declare_parameter("sigma_clock_m", 0.02)
        self.declare_parameter("per_anchor_bias_seed", 42)
        self.declare_parameter("per_anchor_bias_range_m", 0.05)
        self.declare_parameter("enable_nlos_extra_noise", False)
        self.declare_parameter("nlos_extra_noise_sigma_m", 0.10)
        self.declare_parameter("nlos_extra_noise_probability", 0.20)
        self.declare_parameter("dropout_probability", 0.01)

        cir_path = self.get_parameter("cir_table_path").get_parameter_value().string_value
        rate     = self.get_parameter("update_rate_hz").get_parameter_value().double_value
        self._sigma_clock   = self.get_parameter("sigma_clock_m").get_parameter_value().double_value
        self._bias_range    = self.get_parameter("per_anchor_bias_range_m").get_parameter_value().double_value
        self._nlos_enabled  = self.get_parameter("enable_nlos_extra_noise").get_parameter_value().bool_value
        self._nlos_sigma    = self.get_parameter("nlos_extra_noise_sigma_m").get_parameter_value().double_value
        self._nlos_prob     = self.get_parameter("nlos_extra_noise_probability").get_parameter_value().double_value
        self._dropout_prob  = self.get_parameter("dropout_probability").get_parameter_value().double_value

        # ── Load HDF5 table ──────────────────────────────────────────────────
        if not cir_path:
            self.get_logger().fatal("Parameter cir_table_path is not set — cannot start.")
            raise RuntimeError("cir_table_path not set")

        self.get_logger().info(f"Loading CIR table from {cir_path} …")
        self._load_table(cir_path)

        # ── Per-anchor bias (sampled once at startup) ────────────────────────
        seed = self.get_parameter("per_anchor_bias_seed").get_parameter_value().integer_value
        rng  = np.random.default_rng(seed)
        self._bias = rng.uniform(-self._bias_range, self._bias_range, size=self._n_anchors)
        self.get_logger().info(f"Per-anchor biases [m]: {np.round(self._bias, 4).tolist()}")

        # ── RNG for runtime noise (not seeded — intentionally random) ────────
        self._rng = np.random.default_rng()

        # ── State ────────────────────────────────────────────────────────────
        self._last_pose: PoseStamped | None = None
        self._anchors_ok = False

        # ── Subscribers ──────────────────────────────────────────────────────
        transient_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
        )
        self.create_subscription(AnchorArray, "/microuwb/anchors",
                                 self._anchors_cb, transient_qos)
        self.create_subscription(PoseStamped, "/drone/ground_truth_pose",
                                 self._pose_cb, 10)

        # ── Publisher ────────────────────────────────────────────────────────
        self._pub = self.create_publisher(UWBRangeArray, "/microuwb/ranges", 10)

        # ── Timer ────────────────────────────────────────────────────────────
        self.create_timer(1.0 / rate, self._timer_cb)
        self.get_logger().info(f"UWB range simulator ready at {rate} Hz.")

    # ── HDF5 load ────────────────────────────────────────────────────────────

    def _load_table(self, path: str) -> None:
        with h5py.File(path, "r") as f:
            attrs = f.attrs
            self._grid_origin     = np.array(attrs["grid_origin"],     dtype=np.float64)
            self._grid_resolution = np.array(attrs["grid_resolution"], dtype=np.float64)
            self._grid_shape      = np.array(attrs["grid_shape"],      dtype=np.int32)

            anchor_ids = f["anchor_ids"][...]
            self._n_anchors = len(anchor_ids)

            self._range_m   = []
            self._variance  = []
            self._fp_power  = []
            self._los       = []

            for aid in sorted(anchor_ids):
                grp = f["per_anchor"][f"anchor_{aid}"]
                self._range_m.append(np.array(grp["range_m"],             dtype=np.float32))
                self._variance.append(np.array(grp["variance_m2"],        dtype=np.float32))
                self._fp_power.append(np.array(grp["first_path_power_db"], dtype=np.float32))
                self._los.append(np.array(grp["los"],                      dtype=bool))

        nx, ny, nz = self._grid_shape
        ox, oy, oz = self._grid_origin
        rx, ry, rz = self._grid_resolution
        self._grid_min = self._grid_origin
        self._grid_max = self._grid_origin + (self._grid_shape - 1) * self._grid_resolution

        self.get_logger().info(
            f"CIR table loaded: {self._n_anchors} anchors, grid {nx}×{ny}×{nz}, "
            f"bounds x=[{ox:.2f},{self._grid_max[0]:.2f}] "
            f"y=[{oy:.2f},{self._grid_max[1]:.2f}] "
            f"z=[{oz:.2f},{self._grid_max[2]:.2f}]"
        )

    # ── Callbacks ────────────────────────────────────────────────────────────

    def _anchors_cb(self, msg: AnchorArray) -> None:
        n = len(msg.anchors)
        if n != self._n_anchors:
            self.get_logger().warn(
                f"/microuwb/anchors has {n} anchors but HDF5 has {self._n_anchors} — mismatch!"
            )
        else:
            self.get_logger().info(f"Received {n} anchors — range simulator active.")
        self._anchors_ok = True

    def _pose_cb(self, msg: PoseStamped) -> None:
        self._last_pose = msg

    # ── Timer ─────────────────────────────────────────────────────────────────

    def _timer_cb(self) -> None:
        if self._last_pose is None:
            return
        if not self._anchors_ok:
            return

        p = self._last_pose.pose.position
        pos = np.array([p.x, p.y, p.z], dtype=np.float64)

        # Bounds check
        if np.any(pos < self._grid_min) or np.any(pos > self._grid_max):
            self.get_logger().warn(
                f"Drone position {pos.tolist()} is outside grid bounds "
                f"{self._grid_min.tolist()} … {self._grid_max.tolist()} — skipping cycle.",
                throttle_duration_sec=2.0,
            )
            return

        # Float grid coordinates (same order as array axes: x→ax0, y→ax1, z→ax2)
        float_coords = (pos - self._grid_origin) / self._grid_resolution

        now = self.get_clock().now().to_msg()
        array_msg = UWBRangeArray()
        array_msg.header.stamp    = now
        array_msg.header.frame_id = "map"

        for i in range(self._n_anchors):
            # ── Dropout ────────────────────────────────────────────────────
            if self._rng.random() < self._dropout_prob:
                continue

            # ── Trilinear interpolation (range, variance, first-path power) ──
            coords = float_coords.reshape(3, 1)  # shape (3, 1) for map_coordinates
            clean_range = float(map_coordinates(self._range_m[i],  coords, order=1, mode="nearest")[0])
            variance    = float(map_coordinates(self._variance[i], coords, order=1, mode="nearest")[0])
            fp_power    = float(map_coordinates(self._fp_power[i], coords, order=1, mode="nearest")[0])

            # NaN → drone is inside a furniture voxel or adjacent to one
            if np.isnan(clean_range):
                continue

            # ── LOS: nearest-neighbour (binary, cannot interpolate booleans) ──
            nn = np.clip(np.round(float_coords).astype(int),
                         [0, 0, 0], self._grid_shape - 1)
            los = bool(self._los[i][nn[0], nn[1], nn[2]])

            # ── Runtime noise ──────────────────────────────────────────────
            noisy = clean_range + self._sigma_clock * self._rng.standard_normal() + self._bias[i]

            # NLOS extra noise (disabled by default; flip enable_nlos_extra_noise in YAML)
            if self._nlos_enabled and not los:
                if self._rng.random() < self._nlos_prob:
                    noisy += self._nlos_sigma * self._rng.standard_normal()

            # ── Build message ──────────────────────────────────────────────
            rng_msg = UWBRange()
            rng_msg.header.stamp    = now
            rng_msg.header.frame_id = "map"
            rng_msg.anchor_id       = i
            rng_msg.range_m         = float(noisy)
            rng_msg.range_variance  = float(variance)
            rng_msg.line_of_sight   = los
            rng_msg.first_path_power = float(fp_power)
            # rx_power filled from first_path_power (total_rx_power not yet in table schema)
            rng_msg.rx_power        = float(fp_power)

            array_msg.ranges.append(rng_msg)

        if array_msg.ranges:
            self._pub.publish(array_msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = UWBRangeSimulator()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
