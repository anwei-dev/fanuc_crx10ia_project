from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="true",
            description="Use simulated (Gazebo) clock.",
        ),
    ]

    gazebo_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("crx_bringup"), "/launch/gazebo_sim.launch.py"]
        ),
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
            {"use_sim_time": LaunchConfiguration("use_sim_time")},
        ],
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", str(moveit_config.package_path / "config/moveit.rviz")],
        parameters=[
            moveit_config.to_dict(),
            {"use_sim_time": LaunchConfiguration("use_sim_time")},
        ],
    )

    return LaunchDescription(declared_arguments + [gazebo_sim, move_group, rviz])
