import os

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    RegisterEventHandler,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument(
            "use_mock_hardware",
            default_value="true",
            description="Use mock hardware instead of the real FANUC controller.",
        ),
        DeclareLaunchArgument(
            "robot_ip",
            default_value="192.168.150.2",
            description="IP address of the FANUC robot controller.",
        ),
        DeclareLaunchArgument(
            "read_only",
            default_value="false",
            description="Read-only mode (joint states only).",
        ),
        DeclareLaunchArgument(
            "use_rmi",
            default_value="false",
            description="Use RMI instead of Ethernet/IP for communication.",
        ),
    ]

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare("crx_control"), "urdf", "crx10ia_l_control.xacro"]
            ),
            " use_mock_hardware:=",
            LaunchConfiguration("use_mock_hardware"),
            " robot_ip:=",
            LaunchConfiguration("robot_ip"),
            " read_only:=",
            LaunchConfiguration("read_only"),
            " use_rmi:=",
            LaunchConfiguration("use_rmi"),
        ]
    )

    robot_description = {"robot_description": robot_description_content}

    robot_controllers = PathJoinSubstitution(
        [FindPackageShare("crx_control"), "config", "ros2_controllers.yaml"]
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description],
    )

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, robot_controllers],
        output="screen",
    )

    load_joint_state_broadcaster = ExecuteProcess(
        cmd=["ros2", "control", "load_controller", "--set-state", "active",
             "joint_state_broadcaster"],
        output="screen",
    )

    load_manipulator_controller = ExecuteProcess(
        cmd=["ros2", "control", "load_controller", "--set-state", "active",
             "manipulator_controller"],
        output="screen",
    )

    return LaunchDescription(
        declared_arguments
        + [
            robot_state_publisher,
            control_node,
            RegisterEventHandler(
                event_handler=OnProcessExit(
                    target_action=control_node,
                    on_exit=[load_joint_state_broadcaster],
                )
            ),
            RegisterEventHandler(
                event_handler=OnProcessExit(
                    target_action=load_joint_state_broadcaster,
                    on_exit=[load_manipulator_controller],
                )
            ),
        ]
    )
