"""Ground-truth relay: converts /drone/odom (nav_msgs/Odometry from p3d plugin)
to /drone/ground_truth_pose (geometry_msgs/PoseStamped) so all downstream
nodes keep working without message-type changes.
"""

from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data


class GroundTruthRelay(Node):
    def __init__(self) -> None:
        super().__init__("gt_relay")
        # p3d plugin publishes BEST_EFFORT — must match or the subscription silently drops
        self.create_subscription(Odometry, "/drone/odom", self._cb, qos_profile_sensor_data)
        self._pub = self.create_publisher(PoseStamped, "/drone/ground_truth_pose", 10)
        self.get_logger().info("gt_relay: /drone/odom → /drone/ground_truth_pose")

    def _cb(self, msg: Odometry) -> None:
        out = PoseStamped()
        out.header = msg.header
        out.pose = msg.pose.pose
        self._pub.publish(out)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = GroundTruthRelay()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
