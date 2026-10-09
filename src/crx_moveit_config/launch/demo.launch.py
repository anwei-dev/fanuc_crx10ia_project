from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    moveit_config = (
        MoveItConfigsBuilder("crx10ia_l", package_name="crx_moveit_config")
        .to_moveit_configs()
    )

    # 不启动 robot_state_publisher，Gazebo 那边已经启动

    move_group = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            {"use_sim_time": True},
        ],
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", str(moveit_config.package_path / "config/moveit.rviz")],
        parameters=[
            moveit_config.to_dict(),          # ← 加这个
            {"use_sim_time": True},
        ],
    )

    return LaunchDescription([move_group, rviz])
