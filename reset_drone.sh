#!/bin/bash
source /home/avg_shilp_kid/microuwb_nav_ws/install/setup.bash
ros2 topic pub --once /drone/reset std_msgs/msg/Empty "{}"
