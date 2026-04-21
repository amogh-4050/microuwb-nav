import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    bringup_pkg = get_package_share_directory("microuwb_bringup")

    test_traj_arg = DeclareLaunchArgument(
        "test_trajectory",
        default_value="false",
        description="If true, start the figure-8 test pose publisher alongside the simulator.",
    )

    world_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_pkg, "launch", "world.launch.py")
        ),
    )

    uwb_simulator = Node(
        package="microuwb_uwb",
        executable="uwb_range_simulator",
        name="uwb_range_simulator",
        output="screen",
        parameters=[os.path.join(bringup_pkg, "config", "uwb_simulator.yaml")],
    )

    test_pose_publisher = Node(
        package="microuwb_uwb",
        executable="test_pose_publisher",
        name="test_pose_publisher",
        output="screen",
        condition=IfCondition(LaunchConfiguration("test_trajectory")),
    )

    trilateration = Node(
        package="microuwb_uwb",
        executable="trilateration_node",
        name="trilateration_node",
        output="screen",
    )

    verify = Node(
        package="microuwb_uwb",
        executable="verify_trilateration",
        name="verify_trilateration",
        output="screen",
        condition=IfCondition(LaunchConfiguration("test_trajectory")),
    )

    return LaunchDescription([
        test_traj_arg,
        world_launch,
        uwb_simulator,
        test_pose_publisher,
        trilateration,
        verify,
    ])
