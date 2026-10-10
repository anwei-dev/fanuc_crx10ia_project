"""Check launch composition without starting Gazebo or ROS processes."""

import importlib.util
from pathlib import Path
from types import SimpleNamespace

import pytest
from launch import LaunchContext
from launch.actions import DeclareLaunchArgument, EmitEvent, IncludeLaunchDescription, SetEnvironmentVariable
from launch.utilities import normalize_to_list_of_substitutions, perform_substitutions
from launch_ros.actions import Node


def load_launch(name):
    path = Path(__file__).resolve().parents[1] / "launch" / name
    spec = importlib.util.spec_from_file_location(name.replace(".", "_"), path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def prepare(name, **overrides):
    description = load_launch(name).generate_launch_description()
    context = LaunchContext()
    context.launch_configurations.update(overrides)
    for action in description.entities:
        if isinstance(action, DeclareLaunchArgument):
            action.execute(context)
    return description, context


@pytest.mark.parametrize("enabled", ["true", "false"])
def test_demo_forwards_world_and_controls_rviz(enabled):
    world = "/tmp/custom scene.world"
    description, context = prepare("moveit_demo.launch.py", rviz=enabled,
                                   headless="true", world=world)
    include = next(a for a in description.entities if isinstance(a, IncludeLaunchDescription))
    arguments = {name: perform_substitutions(context, normalize_to_list_of_substitutions(value))
                 for name, value in include.launch_arguments}
    assert arguments == {"headless": "true", "world": world}
    rviz = next(a for a in description.entities if isinstance(a, Node) and a.condition is not None)
    assert rviz.condition.evaluate(context) == (enabled == "true")


@pytest.mark.parametrize("headless,gui", [("true", "false"), ("false", "true")])
def test_gazebo_gui_and_world_arguments(headless, gui):
    description, context = prepare("gazebo_sim.launch.py", headless=headless,
                                   world="/tmp/table.world")
    include = next(a for a in description.entities if isinstance(a, IncludeLaunchDescription))
    arguments = {name: perform_substitutions(context, normalize_to_list_of_substitutions(value))
                 for name, value in include.launch_arguments}
    assert arguments == {"gui": gui, "world": "/tmp/table.world"}


def test_gazebo_preserves_existing_resource_paths():
    description, context = prepare("gazebo_sim.launch.py")
    context.environment["GAZEBO_MODEL_PATH"] = "/tmp/external_models"
    context.environment["GAZEBO_RESOURCE_PATH"] = "/tmp/external_materials"
    for action in description.entities:
        if isinstance(action, SetEnvironmentVariable):
            action.execute(context)
    assert context.environment["GAZEBO_MODEL_PATH"].split(":")[-1] == "/tmp/external_models"
    assert context.environment["GAZEBO_RESOURCE_PATH"].split(":")[-1] == "/tmp/external_materials"
    assert "crx_description" in context.environment["GAZEBO_RESOURCE_PATH"]


def test_spawn_failure_shuts_down_instead_of_starting_controllers():
    module = load_launch("gazebo_sim.launch.py")
    controller = object()
    success = module.on_spawn_exit(SimpleNamespace(returncode=0), None, controllers=controller)
    assert success == [controller]
    failure = module.on_spawn_exit(SimpleNamespace(returncode=1), None, controllers=controller)
    assert controller not in failure
    assert any(isinstance(action, EmitEvent) for action in failure)


def test_controller_activation_failure_shuts_down():
    module = load_launch("gazebo_sim.launch.py")
    assert module.on_controllers_exit(SimpleNamespace(returncode=0), None) == []
    assert any(isinstance(action, EmitEvent) for action in
               module.on_controllers_exit(SimpleNamespace(returncode=1), None))
