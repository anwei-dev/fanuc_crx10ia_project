"""Keep the fixed program and its separately authored Gazebo scene consistent."""
from pathlib import Path
import xml.etree.ElementTree as ET

import yaml

PACKAGE = Path(__file__).resolve().parents[1]
CONFIG = yaml.safe_load((PACKAGE / "config/fixed_pick_place.yaml").read_text())["fixed_pick_place"]["ros__parameters"]
WORLD = ET.parse(PACKAGE / "worlds/pick_place.world").getroot().find("world")


def test_spawned_cube_matches_program():
    cube = WORLD.find("model[@name='demo_cube']")
    position = list(map(float, cube.findtext("pose").split()[:3]))
    assert all(abs(a - b) < 0.001 for a, b in zip(position, CONFIG["point_a"]))
    for tag in ("collision", "visual"):
        dimensions = list(map(float, cube.findtext(f"link/{tag}/geometry/box/size").split()))
        assert dimensions == [CONFIG["cube_size"]] * 3


def test_a_and_b_are_supported_by_table():
    top = WORLD.find("model[@name='work_table']/link/collision[@name='top_collision']")
    center = list(map(float, top.findtext("pose").split()[:3]))
    dimensions = list(map(float, top.findtext("geometry/box/size").split()))
    half_cube = CONFIG["cube_size"] / 2
    for name in ("point_a", "point_b"):
        point = CONFIG[name]
        for axis in (0, 1):
            assert abs(point[axis] - center[axis]) + half_cube < dimensions[axis] / 2
        assert abs(point[2] - half_cube - (center[2] + dimensions[2] / 2)) < 1e-6
