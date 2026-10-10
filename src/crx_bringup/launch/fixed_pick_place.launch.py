"""Start the cube scene and fixed A-to-B pick-and-place program."""
import os

from ament_index_python.packages import get_package_prefix
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    arguments = [
        DeclareLaunchArgument("headless", default_value="false", choices=["true", "false"],
                              description="Disable the Gazebo GUI."),
        DeclareLaunchArgument("rviz", default_value="true", choices=["true", "false"],
                              description="Start RViz."),
        DeclareLaunchArgument("run_demo", default_value="true", choices=["true", "false"],
                              description="Execute the fixed cube A-to-B grasp program."),
        DeclareLaunchArgument("world", default_value=PathJoinSubstitution(
            [FindPackageShare("crx_manipulation"), "worlds", "pick_place.world"]),
            description="Gazebo Classic world file."),
    ]
    plugin_path = SetEnvironmentVariable("GAZEBO_PLUGIN_PATH", [
        os.path.join(get_package_prefix("crx_manipulation"), "lib"), os.pathsep,
        EnvironmentVariable("GAZEBO_PLUGIN_PATH", default_value=""),
    ])
    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution(
            [FindPackageShare("crx_bringup"), "launch", "moveit_demo.launch.py"])),
        launch_arguments={
            "headless": LaunchConfiguration("headless"),
            "rviz": LaunchConfiguration("rviz"),
            "world": LaunchConfiguration("world"),
        }.items(),
    )
    moveit_config = MoveItConfigsBuilder(
        "crx10ia_l", package_name="crx_moveit_config").to_moveit_configs()
    grasp_program = Node(
        package="crx_manipulation", executable="fixed_pick_place", name="fixed_pick_place",
        output="screen", condition=IfCondition(LaunchConfiguration("run_demo")),
        parameters=[moveit_config.to_dict(), PathJoinSubstitution(
            [FindPackageShare("crx_manipulation"), "config", "fixed_pick_place.yaml"])],
    )
    return LaunchDescription(arguments + [plugin_path, simulation, grasp_program])
