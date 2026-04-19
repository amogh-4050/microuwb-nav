import os
import yaml

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, HistoryPolicy
from ament_index_python.packages import get_package_share_directory

from geometry_msgs.msg import Point
from microuwb_msgs.msg import Anchor, AnchorArray


class AnchorPublisher(Node):
    def __init__(self):
        super().__init__('anchor_publisher')

        self.declare_parameter('config_file', '')
        config_file = self.get_parameter('config_file').get_parameter_value().string_value

        if not config_file:
            pkg = get_package_share_directory('microuwb_bringup')
            config_file = os.path.join(pkg, 'config', 'anchors.yaml')

        with open(config_file, 'r') as f:
            data = yaml.safe_load(f)

        # TransientLocal = latched: late subscribers receive the last message
        qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
        )
        pub = self.create_publisher(AnchorArray, '/microuwb/anchors', qos)

        msg = AnchorArray()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        for a in data['anchors']:
            anchor = Anchor()
            anchor.id = int(a['id'])
            anchor.name = str(a['name'])
            anchor.position = Point(x=float(a['x']), y=float(a['y']), z=float(a['z']))
            msg.anchors.append(anchor)

        pub.publish(msg)
        self.get_logger().info(f'Published {len(msg.anchors)} UWB anchors on /microuwb/anchors')


def main(args=None):
    rclpy.init(args=args)
    node = AnchorPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
