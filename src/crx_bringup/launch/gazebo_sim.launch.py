import os

from launch import LaunchDescription
from launch.actions import RegisterEventHandler, DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration, PythonExpression

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


ARGUMENTS = [
    DeclareLaunchArgument('name', default_value='', description='NAME_SPACE'),
    DeclareLaunchArgument('model', default_value='crx10ia_l', description='ROBOT_MODEL'),
    DeclareLaunchArgument('headless', default_value='false', description='Headless Gazebo (no GUI)'),
    DeclareLaunchArgument('x', default_value='0', description='Location x on Gazebo'),
    DeclareLaunchArgument('y', default_value='0', description='Location y on Gazebo'),
    DeclareLaunchArgument('z', default_value='0', description='Location z on Gazebo'),
    DeclareLaunchArgument('R', default_value='0', description='Location Roll on Gazebo'),
    DeclareLaunchArgument('P', default_value='0', description='Location Pitch on Gazebo'),
    DeclareLaunchArgument('Y', default_value='0', description='Location Yaw on Gazebo'),
]

ENVIRONMENT = [SetEnvironmentVariable(name='GAZEBO_MODEL_PATH', value=os.path.dirname(get_package_share_directory("crx_description")))]


def generate_launch_description():
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [
                    FindPackageShare("crx_control"),
                    "urdf",
                    "crx10ia_l_gazebo.xacro",
                ]
            ),
        ]
    )

    robot_description = {"robot_description": ParameterValue(robot_description_content, value_type=str)}

    node_robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description, {"use_sim_time": True}],
    )

    # Spawner waits for the controller manager created by the Gazebo plugin.
    spawn_controllers = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "manipulator_controller",
                   "gripper_controller", "--controller-manager-timeout", "60"],
        output="screen",
    )

    gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([FindPackageShare("gazebo_ros"), '/launch/gazebo.launch.py']),
                launch_arguments={"gui": PythonExpression(["'false' if '", LaunchConfiguration('headless'), "'.lower() == 'true' else 'true'"])}.items()
             )

    gz_spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                        arguments=[
                            "-topic",
                            "robot_description",
                            "-entity",
                            LaunchConfiguration('model'),
                            "-x",
                            LaunchConfiguration('x'),
                            "-y",
                            LaunchConfiguration('y'),
                            "-z",
                            LaunchConfiguration('z'),
                            "-R",
                            LaunchConfiguration('R'),
                            "-P",
                            LaunchConfiguration('P'),
                            "-Y",
                            LaunchConfiguration('Y'),
                            "-timeout", "60",
                        ],
                        output='screen')

    return LaunchDescription(ENVIRONMENT + ARGUMENTS + [
        node_robot_state_publisher,
        gazebo,
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=gz_spawn_entity,
                on_exit=[spawn_controllers],
            )
        ),
        gz_spawn_entity,
    ])
