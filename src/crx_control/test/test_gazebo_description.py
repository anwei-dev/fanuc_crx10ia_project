"""Exercise the raw parameter parsing used by Gazebo Classic's control plugin."""

from pathlib import Path
import subprocess
import xml.etree.ElementTree as ET

import yaml

from rclpy.utilities import remove_ros_args


def test_gazebo_description_parameter_override():
    model = Path(__file__).resolve().parents[1] / "urdf/crx10ia_l_gazebo.xacro"
    description = subprocess.check_output(["xacro", str(model)], text=True)
    # URDF/XML validity alone is insufficient. The Gazebo plugin forwards the
    # whole document as a raw ROS parameter override, including XML comments.
    assert remove_ros_args(args=[
        "gazebo_ros2_control", "--ros-args", "--param",
        "robot_description:=" + description,
    ]) == ["gazebo_ros2_control"]


def test_joint_state_broadcaster_publishes_independent_model_joints():
    package = Path(__file__).resolve().parents[1]
    description = subprocess.check_output(
        ["xacro", str(package / "urdf/crx10ia_l_gazebo.xacro")], text=True
    )
    robot = ET.fromstring(description)
    independent = {
        joint.get("name") for joint in robot.findall("joint")
        if joint.get("type") != "fixed" and joint.find("mimic") is None
    }
    config = yaml.safe_load((package / "config/ros2_controllers.yaml").read_text())
    broadcaster = config["joint_state_broadcaster"]["ros__parameters"]
    assert set(broadcaster["joints"]) == independent
    # Humble ignores the joint filter if the interface list is empty.
    assert set(broadcaster["interfaces"]) == {"position", "velocity"}
