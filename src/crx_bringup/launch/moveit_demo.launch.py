from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="true",
            description="Use simulated (Gazebo) clock.",
            choices=["true"],
        ),
        DeclareLaunchArgument("headless", default_value="false", choices=["true", "false"],
                              description="Disable the Gazebo GUI."),
        DeclareLaunchArgument("rviz", default_value="true", choices=["true", "false"],
                              description="Start RViz."),
        DeclareLaunchArgument(
            "world", default_value=PathJoinSubstitution(
                [FindPackageShare("gazebo_ros"), "worlds", "empty.world"]),
            description="Gazebo Classic world file."),
    ]

    gazebo_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("crx_bringup"), "/launch/gazebo_sim.launch.py"]
        ),
        launch_arguments={
            "headless": LaunchConfiguration("headless"),
            "world": LaunchConfiguration("world"),
        }.items(),
    )

    moveit_config = (
        MoveItConfigsBuilder("crx10ia_l", package_name="crx_moveit_config")
        .to_moveit_configs()
    )

    move_group = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            {"use_sim_time": ParameterValue(LaunchConfiguration("use_sim_time"), value_type=bool)},
        ],
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        condition=IfCondition(LaunchConfiguration("rviz")),
        arguments=["-d", str(moveit_config.package_path / "config/moveit.rviz")],
        parameters=[
            moveit_config.to_dict(),
            {"use_sim_time": ParameterValue(LaunchConfiguration("use_sim_time"), value_type=bool)},
        ],
    )

    return LaunchDescription(declared_arguments + [gazebo_sim, move_group, rviz])
