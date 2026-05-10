#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from gazebo_msgs.srv import SetEntityState
from gazebo_msgs.msg import EntityState
from std_msgs.msg import Empty


X_BOUNDS = (0.0, 5.0)
Y_BOUNDS = (0.0, 4.0)
Z_BOUNDS = (-0.1, 2.85)   # -0.1 allows ground contact; 2.85 stops before 3m ceiling
SPAWN = (2.5, 1.0, 0.010)
THROTTLE_S = 2.0
GRACE_S = 6.0              # don't respawn during arm delay + initial liftoff


class RespawnGuard(Node):
    def __init__(self) -> None:
        super().__init__("respawn_guard")
        self._start_time = self.get_clock().now().nanoseconds * 1e-9
        self._last_respawn = 0.0
        self._cli = self.create_client(SetEntityState, "/gazebo/set_entity_state")
        self.create_subscription(
            PoseStamped, "/drone/ground_truth_pose", self._pose_cb, 10
        )
        self.create_subscription(Empty, "/drone/reset", self._manual_reset_cb, 10)
        self.get_logger().info(
            "Respawn guard active — publish to /drone/reset (Empty) for manual respawn"
        )

    def _manual_reset_cb(self, _: Empty) -> None:
        self.get_logger().info("Manual reset requested")
        self._do_teleport()

    def _pose_cb(self, msg: PoseStamped) -> None:
        x = msg.pose.position.x
        y = msg.pose.position.y
        z = msg.pose.position.z

        out_of_bounds = (
            not (X_BOUNDS[0] <= x <= X_BOUNDS[1])
            or not (Y_BOUNDS[0] <= y <= Y_BOUNDS[1])
            or not (Z_BOUNDS[0] <= z <= Z_BOUNDS[1])
        )
        if not out_of_bounds:
            return

        now = self.get_clock().now().nanoseconds * 1e-9
        if now - self._start_time < GRACE_S:
            return
        if now - self._last_respawn < THROTTLE_S:
            return
        self.get_logger().warn(
            f"Drone out of bounds ({x:.2f},{y:.2f},{z:.2f}) — teleporting to spawn"
        )
        self._do_teleport()

    def _do_teleport(self) -> None:
        now = self.get_clock().now().nanoseconds * 1e-9
        self._last_respawn = now
        if not self._cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().error("set_entity_state service unavailable")
            return
        req = SetEntityState.Request()
        req.state = EntityState()
        req.state.name = "microuwb_drone"
        req.state.reference_frame = "world"
        req.state.pose.position.x = SPAWN[0]
        req.state.pose.position.y = SPAWN[1]
        req.state.pose.position.z = SPAWN[2]
        req.state.pose.orientation.w = 1.0
        self._cli.call_async(req)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = RespawnGuard()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
