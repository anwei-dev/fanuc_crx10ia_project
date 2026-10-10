"""Geometric contracts for the eye-in-hand camera, without starting ROS nodes."""
from pathlib import Path
import math
import subprocess
import xml.etree.ElementTree as ET

import numpy as np
import pytest


@pytest.fixture(scope="module")
def robot():
    model = Path(__file__).resolve().parents[1] / "urdf/crx10ia_l_gazebo.xacro"
    return ET.fromstring(subprocess.check_output(["xacro", str(model)], text=True))


def transform(origin):
    xyz = [float(v) for v in origin.get("xyz", "0 0 0").split()]
    r, p, y = [float(v) for v in origin.get("rpy", "0 0 0").split()]
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    matrix = np.eye(4)
    matrix[:3, :3] = [[cy*cp, cy*sp*sr-sy*cr, cy*sp*cr+sy*sr],
                     [sy*cp, sy*sp*sr+cy*cr, sy*sp*cr-cy*sr],
                     [-sp, cp*sr, cp*cr]]
    matrix[:3, 3] = xyz
    return matrix


def camera_in_gripper(robot):
    mount = transform(robot.find("joint[@name='camera_mount_joint']/origin"))
    body = transform(robot.find("joint[@name='camera_joint']/origin"))
    optical = transform(robot.find("joint[@name='camera_optical_joint']/origin"))
    sensor = transform(robot.find("joint[@name='camera_sensor_joint']/origin"))
    return mount @ body @ sensor @ optical


def test_tcp_and_full_opening_inside_camera_view(robot):
    optical = camera_in_gripper(robot)
    tcp = transform(robot.find("joint[@name='gripper-tcp']/origin"))[:3, 3]
    camera = robot.find("gazebo[@reference='camera_sensor_frame']/sensor/camera")
    near, far = float(camera.findtext("clip/near")), float(camera.findtext("clip/far"))
    half_width = math.tan(float(camera.findtext("horizontal_fov"))/2)
    aspect = int(camera.findtext("image/height"))/int(camera.findtext("image/width"))
    for offset in [-0.0425, 0., 0.0425]:
        point = np.linalg.inv(optical) @ np.r_[tcp + [offset, 0, 0], 1.]
        assert near < point[2] < far
        assert abs(point[0]/point[2]) < half_width
        assert abs(point[1]/point[2]) < half_width*aspect
        if offset == 0.:
            assert np.linalg.norm(point[:2]) < 1e-9


def test_gazebo_sensor_and_optical_frame_agree(robot):
    sensor_frame = transform(robot.find("joint[@name='camera_sensor_joint']/origin"))
    optical = sensor_frame @ transform(robot.find("joint[@name='camera_optical_joint']/origin"))
    pose = [float(v) for v in robot.findtext("gazebo[@reference='camera_sensor_frame']/sensor/pose").split()]
    sensor_origin = ET.Element("origin", xyz=" ".join(map(str, pose[:3])), rpy=" ".join(map(str, pose[3:])))
    sensor = sensor_frame @ transform(sensor_origin)
    np.testing.assert_allclose(sensor[:3, 3], optical[:3, 3], atol=1e-12)
    np.testing.assert_allclose(sensor[:3, 0], optical[:3, 2], atol=1e-12)


def test_camera_and_bracket_stay_outside_finger_motion_plane(robot):
    # All finger axes are parallel to gripper Y, so their local mesh Y bounds
    # remain unchanged throughout opening. Check conservative box projections.
    from ament_index_python.packages import get_package_share_directory
    assets = Path(get_package_share_directory("crx_description")) / "meshes/robotiq/collision/2f_85"
    dtype = np.dtype([("normal", "<f4", (3,)), ("vertices", "<f4", (3, 3)), ("attribute", "<u2")])
    finger_min_y = min(np.frombuffer(p.read_bytes()[84:], dtype=dtype)["vertices"][:, :, 1].min()
                       for p in assets.glob("*.stl") if p.stem != "robotiq_base")
    mount = transform(robot.find("joint[@name='camera_mount_joint']/origin"))
    body = mount @ transform(robot.find("joint[@name='camera_joint']/origin"))
    for link, frame in [("camera_mount_link", mount), ("camera_link", body)]:
        for collision in robot.findall(f"link[@name='{link}']/collision"):
            size = np.array([float(v) for v in collision.find("geometry/box").get("size").split()])
            pose = frame @ transform(collision.find("origin"))
            max_y = pose[1, 3] + np.abs(pose[1, :3]) @ (size/2)
            assert finger_min_y - max_y > .015
