import os
import tempfile

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def _gazebo_with_world(context, *args, **kwargs):
    ceiling = LaunchConfiguration('ceiling').perform(context)
    furniture = LaunchConfiguration('furniture').perform(context)

    pkg = get_package_share_directory('microuwb_gazebo')
    xacro_path = os.path.join(pkg, 'worlds', 'room.world.xacro')

    doc = xacro.process_file(xacro_path, mappings={
        'ceiling': ceiling,
        'furniture': furniture,
    })

    tmpdir = tempfile.mkdtemp(prefix='microuwb_')
    world_path = os.path.join(tmpdir, 'room.world')
    with open(world_path, 'w') as f:
        f.write(doc.toxml())

    gazebo_ros_pkg = get_package_share_directory('gazebo_ros')
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_pkg, 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={'world': world_path, 'verbose': 'true'}.items(),
    )
    return [gazebo_launch]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('ceiling',   default_value='true',
                              description='Include ceiling in the room world'),
        DeclareLaunchArgument('furniture', default_value='true',
                              description='Include furniture models'),
        OpaqueFunction(function=_gazebo_with_world),
    ])
