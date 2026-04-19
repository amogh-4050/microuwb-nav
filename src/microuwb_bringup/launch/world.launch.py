import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    ceiling_arg   = DeclareLaunchArgument('ceiling',   default_value='true')
    furniture_arg = DeclareLaunchArgument('furniture', default_value='true')

    gazebo_pkg = get_package_share_directory('microuwb_gazebo')
    room_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_pkg, 'launch', 'room.launch.py')
        ),
        launch_arguments={
            'ceiling':   LaunchConfiguration('ceiling'),
            'furniture': LaunchConfiguration('furniture'),
        }.items(),
    )

    anchor_publisher = Node(
        package='microuwb_uwb',
        executable='anchor_publisher',
        name='anchor_publisher',
        output='screen',
    )

    return LaunchDescription([
        ceiling_arg,
        furniture_arg,
        room_launch,
        anchor_publisher,
    ])
