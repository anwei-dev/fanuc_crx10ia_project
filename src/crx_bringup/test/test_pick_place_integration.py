"""Opt-in Gazebo test: reject remote grasps, move A to B, verify release.

Run after building and sourcing the overlay:
  CRX_RUN_GAZEBO_TEST=1 python3 -m pytest src/crx_bringup/test/test_pick_place_integration.py -s
A working DISPLAY is required by the RGB-D sensor even with the GUI disabled.
"""
import math
import os
from pathlib import Path
import signal
import subprocess
import time

import pytest


@pytest.mark.skipif(os.environ.get("CRX_RUN_GAZEBO_TEST") != "1",
                    reason="Set CRX_RUN_GAZEBO_TEST=1 to run Gazebo integration")
def test_fixed_pick_place(tmp_path, monkeypatch):
    # The test owns a separate ROS domain, Gazebo port, and child process groups.
    monkeypatch.setenv("ROS_DOMAIN_ID", "93")
    monkeypatch.setenv("ROS_LOCALHOST_ONLY", "1")
    monkeypatch.setenv("GAZEBO_MASTER_URI", "http://127.0.0.1:11493")
    monkeypatch.setenv("ROS_LOG_DIR", str(tmp_path / "ros_log"))
    monkeypatch.setenv("LIBGL_ALWAYS_SOFTWARE", "1")
    import rclpy
    import yaml
    from ament_index_python.packages import get_package_share_directory
    from control_msgs.action import FollowJointTrajectory
    from gazebo_msgs.srv import GetEntityState
    from moveit_configs_utils import MoveItConfigsBuilder
    from rclpy.action import ActionClient
    from std_srvs.srv import SetBool

    def stop(process):
        if process is None or process.poll() is not None:
            return
        os.killpg(process.pid, signal.SIGINT)
        try:
            process.wait(timeout=15)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGKILL)
            process.wait()

    def call(client, request):
        future = client.call_async(request)
        rclpy.spin_until_future_complete(node, future, timeout_sec=10)
        assert future.done(), "Service timed out"
        return future.result()

    share = Path(get_package_share_directory("crx_manipulation"))
    config = yaml.safe_load((share / "config/fixed_pick_place.yaml").read_text())["fixed_pick_place"]["ros__parameters"]
    model = MoveItConfigsBuilder("crx10ia_l", package_name="crx_moveit_config").to_moveit_configs().to_dict()
    params = tmp_path / "robot_parameters.yaml"
    params.write_text(yaml.safe_dump({"fixed_pick_place": {"ros__parameters": model}}))
    sim_log = tmp_path / "simulation.log"
    program_log = tmp_path / "program.log"
    rclpy.init()
    node = rclpy.create_node("pick_place_integration_test")
    program = None
    with sim_log.open("w") as output, program_log.open("w") as program_output:
        simulation = subprocess.Popen([
            "ros2", "launch", "crx_bringup", "fixed_pick_place.launch.py",
            "headless:=true", "rviz:=false", "run_demo:=false",
        ], stdout=output, stderr=subprocess.STDOUT, start_new_session=True)
        try:
            grasp = node.create_client(SetBool, "/demo_cube/set_grasp")
            state = node.create_client(GetEntityState, "/demo/get_entity_state")
            controller = ActionClient(node, FollowJointTrajectory,
                                      "/gripper_controller/follow_joint_trajectory")
            assert grasp.wait_for_service(timeout_sec=60), sim_log.read_text()
            assert state.wait_for_service(timeout_sec=60), sim_log.read_text()
            assert controller.wait_for_server(timeout_sec=60), sim_log.read_text()
            # An open gripper, far from A, must never pull the cube toward it.
            rejected = call(grasp, SetBool.Request(data=True))
            assert not rejected.success, rejected.message
            before = call(state, GetEntityState.Request(name="demo_cube", reference_frame="world"))
            assert before.success
            assert math.dist([before.state.pose.position.x, before.state.pose.position.y,
                              before.state.pose.position.z], config["point_a"]) < config["position_tolerance"]
            program = subprocess.Popen([
                "ros2", "run", "crx_manipulation", "fixed_pick_place", "--ros-args",
                "--params-file", str(params),
                "--params-file", str(share / "config/fixed_pick_place.yaml"),
            ], stdout=program_output, stderr=subprocess.STDOUT, start_new_session=True)
            program.wait(timeout=300)
            assert program.returncode == 0, program_log.read_text()
            assert "PICK_PLACE_SUCCESS" in program_log.read_text()
            final = call(state, GetEntityState.Request(name="demo_cube", reference_frame="world"))
            assert final.success
            actual = [final.state.pose.position.x, final.state.pose.position.y, final.state.pose.position.z]
            assert math.dist(actual, config["point_b"]) < config["position_tolerance"]
            # After release and retreat, a new grasp request must fail again.
            assert not call(grasp, SetBool.Request(data=True)).success
            print(program_log.read_text())
            print(f"Verified B position {actual}; logs in {tmp_path}")
        finally:
            stop(program)
            stop(simulation)
            node.destroy_node()
            rclpy.shutdown()
