"""Exercise the raw parameter parsing used by Gazebo Classic's control plugin."""

from pathlib import Path
import subprocess
import struct
import xml.etree.ElementTree as ET

import yaml
import numpy as np
from ament_index_python.packages import get_package_share_directory

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


def test_fixed_base_collision_clears_ground():
    package = Path(__file__).resolve().parents[1]
    robot = ET.fromstring(subprocess.check_output(
        ["xacro", str(package / "urdf/crx10ia_l_gazebo.xacro")], text=True
    ))
    joint = robot.find("joint[@name='world_to_base']")
    assert joint.get("type") == "fixed"
    assert joint.find("parent").get("link") == "world"
    assert joint.find("child").get("link") == "base_link"

    def pose(origin):
        import math
        result = np.eye(4)
        if origin is None:
            return result
        roll, pitch, yaw = map(float, origin.get("rpy", "0 0 0").split())
        cr, sr = math.cos(roll), math.sin(roll)
        cp, sp = math.cos(pitch), math.sin(pitch)
        cy, sy = math.cos(yaw), math.sin(yaw)
        result[:3, :3] = [[cy*cp, cy*sp*sr-sy*cr, cy*sp*cr+sy*sr],
                         [sy*cp, sy*sp*sr+cy*cr, sy*sp*cr-cy*sr],
                         [-sp, cp*sr, cp*cr]]
        result[:3, 3] = list(map(float, origin.get("xyz", "0 0 0").split()))
        return result

    collisions = robot.findall("link[@name='base_link']/collision")
    assert collisions, "Keep the base collision geometry enabled"
    for collision in collisions:
        mesh = collision.find("geometry/mesh")
        uri = mesh.get("filename")
        assert uri.startswith("package://")
        name, relative = uri[len("package://"):].split("/", 1)
        data = (Path(get_package_share_directory(name)) / relative).read_bytes()
        count = struct.unpack_from("<I", data, 80)[0]
        assert len(data) == 84 + count * 50, "Expected a binary STL"
        dtype = np.dtype([("normal", "<f4", (3,)), ("vertices", "<f4", (3, 3)),
                          ("attribute", "<u2")])
        vertices = np.frombuffer(data, dtype=dtype, offset=84)["vertices"].reshape(-1, 3)
        vertices = vertices * np.array(list(map(float, mesh.get("scale", "1 1 1").split())))
        frame = pose(joint.find("origin")) @ pose(collision.find("origin"))
        world_vertices = vertices @ frame[:3, :3].T + frame[:3, 3]
        assert world_vertices[:, 2].min() > 0.0001, "Fixed base penetrates or touches ground"
