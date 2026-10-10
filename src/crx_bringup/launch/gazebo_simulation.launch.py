"""Start the simulation, then activate controllers after a successful spawn."""

from functools import partial
import os

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, EmitEvent, IncludeLaunchDescription, LogInfo,
    RegisterEventHandler, SetEnvironmentVariable,
)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command, EnvironmentVariable, FindExecutable, LaunchConfiguration,
    PathJoinSubstitution, PythonExpression,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def on_spawn_exit(event, context, *, controllers):
    if event.returncode != 0:
        reason = f"Robot spawn failed (exit {event.returncode}); controllers were not started."
        return [LogInfo(msg=f"ERROR: {reason}"), EmitEvent(event=Shutdown(reason=reason))]
    return [controllers]


def on_controllers_exit(event, context):
    if event.returncode != 0:
        reason = f"Controller activation failed (exit {event.returncode})."
        return [LogInfo(msg=f"ERROR: {reason}"), EmitEvent(event=Shutdown(reason=reason))]
    return []


def generate_launch_description():
    description_share = get_package_share_directory("crx_description")
    arguments = [
        DeclareLaunchArgument("model", default_value="crx10ia_l",
                              description="Gazebo entity name (not a robot variant)."),
        DeclareLaunchArgument("headless", default_value="false", choices=["true", "false"],
                              description="Disable the Gazebo GUI."),
        DeclareLaunchArgument("world", default_value=PathJoinSubstitution(
            [FindPackageShare("gazebo_ros"), "worlds", "empty.world"]),
            description="Gazebo Classic world file."),
    ]
    for name in ("x", "y", "z", "R", "P", "Y"):
        arguments.append(DeclareLaunchArgument(
            name, default_value="0", description="Gazebo spawn pose; use defaults with MoveIt."))

    # Preserve paths provided by the caller. Resolve them at launch execution time.
    environment = [
        SetEnvironmentVariable("GAZEBO_MODEL_PATH", [
            os.path.dirname(description_share), os.pathsep,
            EnvironmentVariable("GAZEBO_MODEL_PATH", default_value=""),
        ]),
        SetEnvironmentVariable("GAZEBO_RESOURCE_PATH", [
            description_share, os.pathsep,
            EnvironmentVariable("GAZEBO_RESOURCE_PATH", default_value=""),
        ]),
    ]
    description = Command([
        PathJoinSubstitution([FindExecutable(name="xacro")]), " ",
        PathJoinSubstitution([FindPackageShare("crx_control"), "urdf", "crx10ia_l_gazebo.xacro"]),
    ])
    state_publisher = Node(
        package="robot_state_publisher", executable="robot_state_publisher", output="screen",
        parameters=[{"robot_description": ParameterValue(description, value_type=str),
                     "use_sim_time": True}],
    )
    controllers = Node(
        package="controller_manager", executable="spawner", output="screen",
        arguments=["joint_state_broadcaster", "manipulator_controller", "gripper_controller",
                   "--controller-manager-timeout", "60"],
    )
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([FindPackageShare("gazebo_ros"), "/launch/gazebo.launch.py"]),
        launch_arguments={
            "gui": PythonExpression(["'false' if '", LaunchConfiguration("headless"),
                                     "' == 'true' else 'true'"]),
            "world": LaunchConfiguration("world"),
        }.items(),
    )
    spawn_arguments = ["-topic", "robot_description", "-entity", LaunchConfiguration("model")]
    for name in ("x", "y", "z", "R", "P", "Y"):
        spawn_arguments.extend([f"-{name}", LaunchConfiguration(name)])
    spawn = Node(
        package="gazebo_ros", executable="spawn_entity.py", output="screen",
        arguments=spawn_arguments + ["-timeout", "60"],
    )
    return LaunchDescription(arguments + environment + [
        state_publisher, gazebo,
        RegisterEventHandler(OnProcessExit(
            target_action=spawn, on_exit=partial(on_spawn_exit, controllers=controllers))),
        RegisterEventHandler(OnProcessExit(
            target_action=controllers, on_exit=on_controllers_exit)),
        spawn,
    ])
