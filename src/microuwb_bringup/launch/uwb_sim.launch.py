import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    bringup_pkg     = get_package_share_directory("microuwb_bringup")
    description_pkg = get_package_share_directory("microuwb_description")
    rviz_pkg        = get_package_share_directory("microuwb_rviz")

    test_traj_arg = DeclareLaunchArgument(
        "test_trajectory",
        default_value="false",
        description="Start figure-8 test pose publisher (step-4/5 mode).",
    )
    use_drone_arg = DeclareLaunchArgument(
        "use_drone",
        default_value="false",
        description="Spawn physical drone model and gt_relay instead of test_pose_publisher.",
    )
    test_mode_arg = DeclareLaunchArgument(
        "test_mode",
        default_value="none",
        description="Setpoint test mode when use_drone=true: hover|step|square|none.",
    )
    ceiling_arg = DeclareLaunchArgument(
        "ceiling",
        default_value="false",
        description="Include ceiling in world (default false for visibility).",
    )
    waypoint_nav_arg = DeclareLaunchArgument(
        "waypoint_nav",
        default_value="false",
        description="Run autonomous waypoint sequencer (square→circle→triangle loop).",
    )
    rviz_arg = DeclareLaunchArgument(
        "rviz",
        default_value="false",
        description="Launch RViz2 with pre-built display config.",
    )
    furniture_arg = DeclareLaunchArgument(
        "furniture",
        default_value="true",
        description="Include furniture obstacles in world.",
    )

    set_model_path = SetEnvironmentVariable(
        name="GAZEBO_MODEL_PATH",
        value=[
            os.path.join(description_pkg, "models"),
            ":",
            os.environ.get("GAZEBO_MODEL_PATH", ""),
        ],
    )

    world_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_pkg, "launch", "world.launch.py")
        ),
        launch_arguments={
            "ceiling":   LaunchConfiguration("ceiling"),
            "furniture": LaunchConfiguration("furniture"),
        }.items(),
    )

    uwb_simulator = Node(
        package="microuwb_uwb",
        executable="uwb_range_simulator",
        name="uwb_range_simulator",
        output="screen",
        parameters=[os.path.join(bringup_pkg, "config", "uwb_simulator.yaml")],
    )

    # Only start when test_trajectory=true AND use_drone=false
    test_pose_publisher = Node(
        package="microuwb_uwb",
        executable="test_pose_publisher",
        name="test_pose_publisher",
        output="screen",
        condition=IfCondition(
            PythonExpression([
                "'", LaunchConfiguration("test_trajectory"), "' == 'true'",
                " and '", LaunchConfiguration("use_drone"), "' == 'false'",
            ])
        ),
    )

    spawn_drone = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        name="spawn_drone",
        arguments=[
            "-file", os.path.join(description_pkg, "models", "microuwb_drone", "model.sdf"),
            "-entity", "microuwb_drone",
            "-x", "2.5", "-y", "1.0", "-z", "0.010",
        ],
        output="screen",
        condition=IfCondition(LaunchConfiguration("use_drone")),
    )

    gt_relay = Node(
        package="microuwb_uwb",
        executable="gt_relay",
        name="gt_relay",
        output="screen",
        condition=IfCondition(LaunchConfiguration("use_drone")),
    )

    trilateration = Node(
        package="microuwb_uwb",
        executable="trilateration_node",
        name="trilateration_node",
        output="screen",
        parameters=[os.path.join(bringup_pkg, "config", "trilateration.yaml")],
    )

    kalman_filter = Node(
        package="microuwb_uwb",
        executable="kalman_filter",
        name="kalman_filter",
        output="screen",
        parameters=[os.path.join(bringup_pkg, "config", "kalman_filter.yaml")],
    )

    verify = Node(
        package="microuwb_uwb",
        executable="verify_trilateration",
        name="verify_trilateration",
        output="screen",
        condition=IfCondition(LaunchConfiguration("test_trajectory")),
    )

    # Step 7 control nodes — only when use_drone=true AND test_mode!=none
    _drone_and_test = PythonExpression([
        "'", LaunchConfiguration("use_drone"), "' == 'true'",
        " and '", LaunchConfiguration("test_mode"), "' != 'none'",
    ])

    flight_controller = Node(
        package="microuwb_control",
        executable="flight_controller",
        name="flight_controller",
        output="screen",
        parameters=[os.path.join(bringup_pkg, "config", "flight_controller.yaml")],
        condition=IfCondition(LaunchConfiguration("use_drone")),
    )

    respawn_guard = Node(
        package="microuwb_control",
        executable="respawn_guard",
        name="respawn_guard",
        output="screen",
        condition=IfCondition(LaunchConfiguration("use_drone")),
    )

    test_setpoint_publisher = Node(
        package="microuwb_control",
        executable="test_setpoint_publisher",
        name="test_setpoint_publisher",
        output="screen",
        arguments=["--test", LaunchConfiguration("test_mode")],
        condition=IfCondition(_drone_and_test),
    )

    # Identity transform so RViz fixed frame "map" resolves against Gazebo's "world"
    map_to_world_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="map_world_tf",
        arguments=["0", "0", "0", "0", "0", "0", "world", "map"],
    )

    _drone_and_wp = PythonExpression([
        "'", LaunchConfiguration("use_drone"), "' == 'true'",
        " and '", LaunchConfiguration("waypoint_nav"), "' == 'true'",
    ])

    waypoint_nav_node = Node(
        package="microuwb_control",
        executable="waypoint_nav",
        name="waypoint_nav",
        output="screen",
        parameters=[os.path.join(bringup_pkg, "config", "waypoint_nav.yaml")],
        condition=IfCondition(_drone_and_wp),
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", os.path.join(rviz_pkg, "config", "uwb_sim.rviz")],
        output="screen",
        condition=IfCondition(LaunchConfiguration("rviz")),
    )

    return LaunchDescription([
        test_traj_arg,
        use_drone_arg,
        test_mode_arg,
        ceiling_arg,
        furniture_arg,
        waypoint_nav_arg,
        rviz_arg,
        set_model_path,
        map_to_world_tf,
        world_launch,
        uwb_simulator,
        test_pose_publisher,
        spawn_drone,
        gt_relay,
        trilateration,
        kalman_filter,
        verify,
        flight_controller,
        respawn_guard,
        test_setpoint_publisher,
        waypoint_nav_node,
        rviz_node,
    ])
