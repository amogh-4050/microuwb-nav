"""Cascaded PID flight controller — step 7.

Outer position loop (10 Hz, triggered by /drone/ground_truth_pose or KF):
  position error → desired roll/pitch + total thrust command.

Inner attitude loop (100 Hz timer):
  attitude error → torques → 4-rotor mixer.

Pre-arm (first arm_delay_s seconds):
  Attitude stabilisation only — keeps drone flat on the ground.
  No position tracking.

On arm:
  setpoint is frozen to current drone position (position hold).
  Move by adjusting setpoint_x/y/z via rqt_reconfigure, or by
  publishing to /microuwb/setpoint (PoseStamped).
"""
from __future__ import annotations

import math

import numpy as np

import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import FloatingPointRange, ParameterDescriptor, SetParametersResult

from geometry_msgs.msg import PoseStamped, Wrench
from sensor_msgs.msg import Imu
from std_msgs.msg import Float64MultiArray
from visualization_msgs.msg import Marker
from rclpy.qos import qos_profile_sensor_data


_G           = 9.81
_ARM         = 0.04
_MAX_ROTOR_N = 0.20
_MAX_TORQUE  = 0.016


def _quat_to_euler(q) -> tuple[float, float, float]:
    sinr = 2.0 * (q.w * q.x + q.y * q.z)
    cosr = 1.0 - 2.0 * (q.x**2 + q.y**2)
    roll = math.atan2(sinr, cosr)

    sinp = 2.0 * (q.w * q.y - q.z * q.x)
    pitch = math.copysign(math.pi / 2, sinp) if abs(sinp) >= 1.0 else math.asin(sinp)

    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y**2 + q.z**2)
    yaw = math.atan2(siny, cosy)

    return roll, pitch, yaw


def _stamp_s(stamp) -> float:
    return stamp.sec + stamp.nanosec * 1e-9


def _wrap_pi(a: float) -> float:
    return (a + math.pi) % (2.0 * math.pi) - math.pi


class FlightController(Node):
    def __init__(self) -> None:
        super().__init__("flight_controller")

        self._declare_params()
        self._cache_gains()
        self.add_on_set_parameters_callback(self._params_cb)

        # ── Outer-loop state ──────────────────────────────────────────────
        # Setpoint starts at default; overwritten with current pos on arm
        self._setpoint = np.array([2.5, 2.0, 1.5])
        self._pos:      np.ndarray | None = None
        self._vel       = np.zeros(3)
        self._prev_t:   float | None = None

        self._desired_roll  = 0.0
        self._desired_pitch = 0.0
        self._desired_yaw   = 0.0
        self._thrust_cmd    = self._hover_thrust

        self._int_xy = np.zeros(2)
        self._int_z  = 0.0

        # ── Inner-loop state ──────────────────────────────────────────────
        self._roll = self._pitch = self._yaw = 0.0
        self._roll_rate = self._pitch_rate = self._yaw_rate = 0.0
        self._int_roll = self._int_pitch = self._int_yaw = 0.0

        # ── Subscribers ───────────────────────────────────────────────────
        use_gt = self.declare_parameter("use_ground_truth", True) \
                     .get_parameter_value().bool_value
        pos_topic = "/drone/ground_truth_pose" if use_gt else "/microuwb/position_filtered"
        self.get_logger().info(f"Position source: {pos_topic}")
        self.create_subscription(PoseStamped, pos_topic, self._pos_cb, 10)
        self.create_subscription(Imu, "/microuwb/imu", self._imu_cb, qos_profile_sensor_data)
        self.create_subscription(PoseStamped, "/microuwb/setpoint", self._sp_cb, 10)

        # ── Publishers ────────────────────────────────────────────────────
        # Single Wrench on base_link (link frame): force.z=thrust, torque.xyz=attitude
        self._pub_body   = self.create_publisher(Wrench,            "/microuwb/body_wrench",      10)
        self._pub_debug  = self.create_publisher(Float64MultiArray, "/microuwb/controller_debug", 10)
        self._pub_marker = self.create_publisher(Marker,            "/microuwb/setpoint_marker",  10)
        self.create_timer(0.5, self._publish_setpoint_marker)   # 2 Hz

        # ── Arming delay ─────────────────────────────────────────────────
        self._armed = False
        arm_delay = self.declare_parameter("arm_delay_s", 5.0) \
                        .get_parameter_value().double_value
        self._arm_timer = self.create_timer(arm_delay, self._arm)

        # ── 100 Hz control timer ──────────────────────────────────────────
        self.create_timer(0.01, self._control_cb)
        self.get_logger().info(
            f"Flight controller ready — arming in {arm_delay:.0f} s "
            f"(hover_thrust={self._hover_thrust:.3f} N)."
        )

    def _arm(self) -> None:
        self._armed = True
        self._arm_timer.cancel()   # one-shot — use reset_drone.sh to respawn
        # freeze setpoint to current position so drone holds in place on arm
        if self._pos is not None:
            self._setpoint = self._pos.copy()
            # sync rqt_reconfigure sliders to current position
            self.set_parameters([
                rclpy.parameter.Parameter("setpoint_x", rclpy.Parameter.Type.DOUBLE, float(self._pos[0])),
                rclpy.parameter.Parameter("setpoint_y", rclpy.Parameter.Type.DOUBLE, float(self._pos[1])),
                rclpy.parameter.Parameter("setpoint_z", rclpy.Parameter.Type.DOUBLE, float(self._pos[2])),
            ])
        self.get_logger().info(
            f"ARMED — holding ({self._setpoint[0]:.2f}, "
            f"{self._setpoint[1]:.2f}, {self._setpoint[2]:.2f}). "
            "Use rqt_reconfigure setpoint_x/y/z sliders to move."
        )

    # ── Parameter plumbing ────────────────────────────────────────────────────

    def _fdesc(self, text: str, lo: float, hi: float) -> ParameterDescriptor:
        return ParameterDescriptor(
            description=text,
            floating_point_range=[FloatingPointRange(from_value=lo, to_value=hi, step=0.0)],
        )

    def _declare_params(self) -> None:
        P = self.declare_parameter
        d = self._fdesc
        # PID gains
        P("kp_att_roll",  0.0064,  d("Roll attitude P gain",   0.0, 1.0))
        P("kd_att_roll",  0.00085, d("Roll attitude D gain",   0.0, 0.1))
        P("ki_att_roll",  0.0,     d("Roll attitude I gain",   0.0, 0.1))
        P("kp_att_pitch", 0.0064,  d("Pitch attitude P gain",  0.0, 1.0))
        P("kd_att_pitch", 0.00085, d("Pitch attitude D gain",  0.0, 0.1))
        P("ki_att_pitch", 0.0,     d("Pitch attitude I gain",  0.0, 0.1))
        P("kp_att_yaw",   0.01,    d("Yaw attitude P gain",    0.0, 1.0))
        P("kd_att_yaw",   0.002,   d("Yaw attitude D gain",    0.0, 0.1))
        P("ki_att_yaw",   0.0,     d("Yaw attitude I gain",    0.0, 0.1))
        P("kp_pos_xy",    0.4,     d("XY position P gain",     0.0, 5.0))
        P("kd_pos_xy",    0.4,     d("XY position D gain",     0.0, 5.0))
        P("ki_pos_xy",    0.08,    d("XY position I gain",     0.0, 1.0))
        P("kp_pos_z",     3.0,     d("Z position P gain",      0.0, 10.0))
        P("kd_pos_z",     1.5,     d("Z position D gain",      0.0, 10.0))
        P("ki_pos_z",     0.25,    d("Z position I gain",      0.0, 2.0))
        P("hover_thrust", 0.57,    d("Hover thrust total (N)", 0.1, 0.8))
        P("max_tilt_rad", 0.15,    d("Max tilt clamp (rad)",   0.0, 1.0))
        P("max_accel",    0.5,     d("Max XY accel (m/s²)",    0.0, 5.0))
        P("lookahead_s",  0.15,   d("XY predictive lookahead (s)", 0.0, 0.5))
        # Waypoint sliders — shown in rqt_reconfigure, move drone on change
        P("setpoint_x",   2.5,     d("Target X (m)",           0.5, 4.5))
        P("setpoint_y",   2.0,     d("Target Y (m)",           0.5, 3.5))
        P("setpoint_z",   1.5,     d("Target Z (m)",           0.3, 2.5))

    def _cache_gains(self) -> None:
        g = lambda n: self.get_parameter(n).get_parameter_value().double_value  # noqa: E731
        self._kp_att_roll  = g("kp_att_roll")
        self._kd_att_roll  = g("kd_att_roll")
        self._ki_att_roll  = g("ki_att_roll")
        self._kp_att_pitch = g("kp_att_pitch")
        self._kd_att_pitch = g("kd_att_pitch")
        self._ki_att_pitch = g("ki_att_pitch")
        self._kp_att_yaw   = g("kp_att_yaw")
        self._kd_att_yaw   = g("kd_att_yaw")
        self._ki_att_yaw   = g("ki_att_yaw")
        self._kp_pos_xy    = g("kp_pos_xy")
        self._kd_pos_xy    = g("kd_pos_xy")
        self._ki_pos_xy    = g("ki_pos_xy")
        self._kp_pos_z     = g("kp_pos_z")
        self._kd_pos_z     = g("kd_pos_z")
        self._ki_pos_z     = g("ki_pos_z")
        self._hover_thrust = g("hover_thrust")
        self._max_tilt_rad = g("max_tilt_rad")
        self._max_accel    = g("max_accel")
        self._lookahead    = g("lookahead_s")

    def _params_cb(self, params) -> SetParametersResult:
        for p in params:
            n, v = p.name, p.value
            if   n == "kp_att_roll":  self._kp_att_roll  = v
            elif n == "kd_att_roll":  self._kd_att_roll  = v
            elif n == "ki_att_roll":  self._ki_att_roll  = v; self._int_roll  = 0.0
            elif n == "kp_att_pitch": self._kp_att_pitch = v
            elif n == "kd_att_pitch": self._kd_att_pitch = v
            elif n == "ki_att_pitch": self._ki_att_pitch = v; self._int_pitch = 0.0
            elif n == "kp_att_yaw":   self._kp_att_yaw   = v
            elif n == "kd_att_yaw":   self._kd_att_yaw   = v
            elif n == "ki_att_yaw":   self._ki_att_yaw   = v; self._int_yaw   = 0.0
            elif n == "kp_pos_xy":    self._kp_pos_xy    = v
            elif n == "kd_pos_xy":    self._kd_pos_xy    = v
            elif n == "ki_pos_xy":    self._ki_pos_xy    = v; self._int_xy[:] = 0.0
            elif n == "kp_pos_z":     self._kp_pos_z     = v
            elif n == "kd_pos_z":     self._kd_pos_z     = v
            elif n == "ki_pos_z":     self._ki_pos_z     = v; self._int_z     = 0.0
            elif n == "hover_thrust": self._hover_thrust = v; self._thrust_cmd = v
            elif n == "max_tilt_rad": self._max_tilt_rad = v
            elif n == "max_accel":    self._max_accel    = v
            elif n == "lookahead_s":  self._lookahead    = v
            elif n == "setpoint_x":   self._setpoint[0]  = v
            elif n == "setpoint_y":   self._setpoint[1]  = v
            elif n == "setpoint_z":   self._setpoint[2]  = v
        return SetParametersResult(successful=True)

    # ── Subscriber callbacks ──────────────────────────────────────────────────

    def _sp_cb(self, msg: PoseStamped) -> None:
        p = msg.pose.position
        self._setpoint = np.array([p.x, p.y, p.z])

    def _imu_cb(self, msg: Imu) -> None:
        self._roll, self._pitch, self._yaw = _quat_to_euler(msg.orientation)
        av = msg.angular_velocity
        self._roll_rate  = av.x
        self._pitch_rate = av.y
        self._yaw_rate   = av.z

    def _pos_cb(self, msg: PoseStamped) -> None:
        t   = _stamp_s(msg.header.stamp)
        pos = np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z])

        dt_int = 0.1
        if self._pos is not None and self._prev_t is not None:
            dt_raw = t - self._prev_t
            if 0.005 < dt_raw < 0.5:
                self._vel  = (pos - self._pos) / dt_raw
                dt_int = dt_raw

        self._prev_t = t
        self._pos    = pos
        if self._armed:
            self._outer_loop(dt=dt_int)

    # ── Outer position loop (10 Hz) ───────────────────────────────────────────

    def _outer_loop(self, dt: float) -> None:
        err_z  = self._setpoint[2] - self._pos[2]

        # Only correct XY once airborne — floor contact causes tiny tilts that
        # the low-rate XY loop can't counter, leading to horizontal drift.
        if self._pos[2] > 0.3:
            # Predict where drone will be after attitude loop lag, brake against that.
            predicted_pos_xy = self._pos[:2] + self._vel[:2] * self._lookahead
            err_xy = self._setpoint[:2] - predicted_pos_xy
            raw_accel = (
                self._kp_pos_xy * err_xy
                + self._kd_pos_xy * (-self._vel[:2])
                + self._ki_pos_xy * self._int_xy
            )
            accel = np.clip(raw_accel, -self._max_accel, self._max_accel)
            near_wall = (
                self._pos[0] < 0.6 or self._pos[0] > 4.4 or
                self._pos[1] < 0.6 or self._pos[1] > 3.4
            )
            if np.all(np.abs(raw_accel) <= self._max_accel) and not near_wall:
                self._int_xy += err_xy * dt
            elif near_wall:
                self._int_xy[:] = 0.0   # flush windup when pinned against a wall
        else:
            accel = np.zeros(2)
            self._int_xy[:] = 0.0   # reset so integral doesn't wind up on the floor

        # Gazebo drone body-X points in world +X, so positive pitch (nose up) = −X thrust.
        # Flip sign vs. standard aerospace: positive accel[0] → positive pitch to go +X.
        self._desired_pitch = float(np.clip( accel[0] / _G,
                                             -self._max_tilt_rad, self._max_tilt_rad))
        self._desired_roll  = float(np.clip(-accel[1] / _G,
                                             -self._max_tilt_rad, self._max_tilt_rad))

        raw_thrust = (
            self._hover_thrust
            + self._kp_pos_z * err_z
            + self._kd_pos_z * (-self._vel[2])
            + self._ki_pos_z * self._int_z
        )
        self._thrust_cmd = float(np.clip(raw_thrust, 0.1, 0.8))

        if 0.1 < raw_thrust < 0.8:
            self._int_z += err_z * dt

    # ── Inner attitude loop (100 Hz) ─────────────────────────────────────────

    def _attitude_torques(self, desired_roll: float, desired_pitch: float
                          ) -> tuple[float, float, float]:
        """Returns (roll_t, pitch_t, yaw_t) in body frame N·m."""
        dt = 0.01

        roll_err = desired_roll - self._roll
        raw_rt = (self._kp_att_roll * roll_err
                  + self._kd_att_roll * (-self._roll_rate)
                  + self._ki_att_roll * self._int_roll)
        roll_t = float(np.clip(raw_rt, -_MAX_TORQUE, _MAX_TORQUE))
        if abs(raw_rt) <= _MAX_TORQUE:
            self._int_roll += roll_err * dt

        pitch_err = desired_pitch - self._pitch
        raw_pt = (self._kp_att_pitch * pitch_err
                  + self._kd_att_pitch * (-self._pitch_rate)
                  + self._ki_att_pitch * self._int_pitch)
        pitch_t = float(np.clip(raw_pt, -_MAX_TORQUE, _MAX_TORQUE))
        if abs(raw_pt) <= _MAX_TORQUE:
            self._int_pitch += pitch_err * dt

        yaw_err = _wrap_pi(self._desired_yaw - self._yaw)
        raw_yt = (self._kp_att_yaw * yaw_err
                  + self._kd_att_yaw * (-self._yaw_rate)
                  + self._ki_att_yaw * self._int_yaw)
        yaw_t = float(np.clip(raw_yt, -_MAX_TORQUE, _MAX_TORQUE))
        if abs(raw_yt) <= _MAX_TORQUE:
            self._int_yaw += yaw_err * dt

        return roll_t, pitch_t, yaw_t

    def _publish_wrench(self, thrust: float, roll_t: float, pitch_t: float, yaw_t: float) -> None:
        msg = Wrench()
        msg.force.z  = thrust   # body Z = up when level, tilts with drone (correct physics)
        msg.torque.x = roll_t   # body roll  — positive = left wing up
        msg.torque.y = pitch_t  # body pitch — positive = nose up
        msg.torque.z = yaw_t    # body yaw
        self._pub_body.publish(msg)

    def _control_cb(self) -> None:
        if not self._armed:
            # Gravity cancel + attitude stab from t=0, no position data needed.
            roll_t, pitch_t, yaw_t = self._attitude_torques(0.0, 0.0)
            self._publish_wrench(self._hover_thrust, roll_t, pitch_t, yaw_t)
            return

        if self._pos is None:
            return

        roll_t, pitch_t, yaw_t = self._attitude_torques(
            self._desired_roll, self._desired_pitch
        )
        self._publish_wrench(self._thrust_cmd, roll_t, pitch_t, yaw_t)

        dbg = Float64MultiArray()
        dbg.data = [
            float(self._setpoint[0]), float(self._setpoint[1]), float(self._setpoint[2]),
            float(self._pos[0]),      float(self._pos[1]),      float(self._pos[2]),
            self._roll, self._pitch, self._yaw,
            self._desired_roll, self._desired_pitch,
            roll_t, pitch_t, yaw_t, self._thrust_cmd,
        ]
        self._pub_debug.publish(dbg)


    def _publish_setpoint_marker(self) -> None:
        # Green when drone is within 0.3 m of setpoint (holding), yellow when navigating.
        m = Marker()
        m.header.stamp    = self.get_clock().now().to_msg()
        m.header.frame_id = "map"
        m.ns     = "setpoint"
        m.id     = 0
        m.type   = Marker.SPHERE
        m.action = Marker.ADD
        m.pose.position.x = float(self._setpoint[0])
        m.pose.position.y = float(self._setpoint[1])
        m.pose.position.z = float(self._setpoint[2])
        m.pose.orientation.w = 1.0
        m.scale.x = m.scale.y = m.scale.z = 0.12   # 12 cm sphere

        near = (self._pos is not None and
                float(np.linalg.norm(self._setpoint - self._pos)) < 0.30)
        m.color.a = 0.85
        m.color.r = 0.0
        m.color.g = 1.0
        m.color.b = 0.0 if near else 0.6   # pure green = arrived, teal = navigating
        self._pub_marker.publish(m)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = FlightController()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
