#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>
#include <gazebo_msgs/srv/get_entity_state.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/collision_detection/collision_matrix.h>
#include <moveit_msgs/srv/get_planning_scene.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/planning_scene.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <std_srvs/srv/set_bool.hpp>

#include <chrono>
#include <cmath>
#include <future>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

using namespace std::chrono_literals;
using Trajectory = control_msgs::action::FollowJointTrajectory;

namespace crx_manipulation
{
class FixedPickPlace
{
public:
  explicit FixedPickPlace(const rclcpp::Node::SharedPtr & node)
  : node_(node), arm_(node, "manipulator")
  {
    a_ = parameter<std::vector<double>>("point_a", {0.65, -0.20, 0.42});
    b_ = parameter<std::vector<double>>("point_b", {0.65, 0.20, 0.42});
    size_ = parameter<double>("cube_size", 0.04);
    approach_ = parameter<double>("approach_height", 0.18);
    offset_ = parameter<double>("tcp_above_cube", 0.005);
    open_ = parameter<double>("gripper_open", 0.0);
    closed_ = parameter<double>("gripper_grasp", 0.45);
    tolerance_ = parameter<double>("position_tolerance", 0.015);
    if (a_.size() != 3 || b_.size() != 3) { throw std::runtime_error("A/B must be XYZ vectors"); }
    arm_.setPoseReferenceFrame("world");
    arm_.setEndEffectorLink("tcp");
    arm_.setPlannerId("RRTConnect");
    arm_.setPlanningTime(10.0);
    arm_.setNumPlanningAttempts(10);
    arm_.setMaxVelocityScalingFactor(parameter<double>("velocity_scaling", 0.15));
    arm_.setMaxAccelerationScalingFactor(parameter<double>("acceleration_scaling", 0.15));
    arm_.setGoalPositionTolerance(0.002);
    arm_.setGoalOrientationTolerance(0.01);
    gripper_ = rclcpp_action::create_client<Trajectory>(node_,
      "/gripper_controller/follow_joint_trajectory");
    arm_action_ = rclcpp_action::create_client<Trajectory>(node_,
      "/manipulator_controller/follow_joint_trajectory");
    grasp_ = node_->create_client<std_srvs::srv::SetBool>("/demo_cube/set_grasp");
    planning_scene_ = node_->create_client<moveit_msgs::srv::GetPlanningScene>("/get_planning_scene");
    state_ = node_->create_client<gazebo_msgs::srv::GetEntityState>("/demo/get_entity_state");
  }

  void run()
  {
    RCLCPP_INFO(node_->get_logger(), "Waiting for the simulation and trajectory controllers...");
    require(gripper_->wait_for_action_server(60s) && arm_action_->wait_for_action_server(60s) &&
            grasp_->wait_for_service(60s) && state_->wait_for_service(60s) &&
            planning_scene_->wait_for_service(60s), "Simulation not ready");
    arm_.startStateMonitor();
    require(static_cast<bool>(arm_.getCurrentState(15.0)), "No current robot state");
    require(distance(cube_pose().position, a_) < tolerance_,
            "Cube is not at A; restart the demo world before running again");
    setup_scene();
    gripper(open_);
    move(a_, approach_, "Move above A");
    allow_cube_contact(true);
    move(a_, 0.0, "Descend to grasp");
    gripper(closed_);
    set_grasp(true);
    // Applying the scene synchronously avoids planning before the attachment exists.
    attach_scene(true);
    move(a_, approach_, "Lift cube");
    auto lifted = cube_pose();
    require(lifted.position.z > a_[2] + approach_ * 0.6,
            "Cube did not lift with the gripper");
    move(b_, approach_, "Carry cube above B");
    move(b_, 0.0, "Lower cube to B");
    // Remove the constraint at the support surface, then open the fingers.
    set_grasp(false);
    attach_scene(false);
    gripper(open_);
    move(b_, approach_, "Retreat from B");
    allow_cube_contact(false);
    std::this_thread::sleep_for(1s);
    const auto final = cube_pose();
    const double error = distance(final.position, b_);
    require(error < tolerance_, "Cube final position is outside the B tolerance");
    RCLCPP_INFO(node_->get_logger(),
      "PICK_PLACE_SUCCESS cube=(%.4f, %.4f, %.4f), B error=%.4f m",
      final.position.x, final.position.y, final.position.z, error);
  }

private:
  template<typename T>
  T parameter(const std::string & name, T fallback)
  {
    if (!node_->has_parameter(name)) { return node_->declare_parameter<T>(name, fallback); }
    return node_->get_parameter(name).get_value<T>();
  }

  static void require(bool condition, const std::string & message)
  {
    if (!condition) { throw std::runtime_error(message); }
  }

  template<typename Future>
  auto wait(Future & future, std::chrono::seconds timeout, const std::string & operation)
  {
    require(future.wait_for(timeout) == std::future_status::ready, operation + " timed out");
    return future.get();
  }

  static double distance(const geometry_msgs::msg::Point & p, const std::vector<double> & q)
  {
    return std::sqrt(std::pow(p.x - q[0], 2) + std::pow(p.y - q[1], 2) + std::pow(p.z - q[2], 2));
  }

  static std::vector<std::string> touch_links()
  {
    return {"robotiq_85_left_finger_tip_link", "robotiq_85_right_finger_tip_link"};
  }

  geometry_msgs::msg::Pose cube_pose()
  {
    auto request = std::make_shared<gazebo_msgs::srv::GetEntityState::Request>();
    request->name = "demo_cube";
    request->reference_frame = "world";
    auto future = state_->async_send_request(request);
    const auto response = wait(future, 10s, "Query cube pose");
    require(response->success, "Gazebo could not find demo_cube");
    return response->state.pose;
  }

  moveit_msgs::msg::CollisionObject cube(const geometry_msgs::msg::Pose & pose)
  {
    moveit_msgs::msg::CollisionObject object;
    object.header.frame_id = "world";
    object.id = "demo_cube";
    shape_msgs::msg::SolidPrimitive box;
    box.type = box.BOX;
    box.dimensions = {size_, size_, size_};
    object.primitives = {box};
    object.primitive_poses = {pose};
    object.operation = object.ADD;
    return object;
  }

  void setup_scene()
  {
    auto table = cube(geometry_msgs::msg::Pose());
    table.id = "work_table";
    // A conservative solid volume covers both tabletop and support.
    table.primitives[0].dimensions = {0.8, 0.9, 0.4};
    table.primitive_poses[0].position.x = 0.65;
    table.primitive_poses[0].position.z = 0.2;
    table.primitive_poses[0].orientation.w = 1.0;
    require(scene_.applyCollisionObjects({table, cube(cube_pose())}), "Cannot initialize planning scene");
  }

  void allow_cube_contact(bool allow)
  {
    moveit_msgs::msg::PlanningScene diff;
    diff.is_diff = true;
    diff.robot_state.is_diff = true;
    auto request = std::make_shared<moveit_msgs::srv::GetPlanningScene::Request>();
    request->components.components = moveit_msgs::msg::PlanningSceneComponents::ALLOWED_COLLISION_MATRIX;
    auto future = planning_scene_->async_send_request(request);
    const auto current = wait(future, 10s, "Read collision matrix");
    collision_detection::AllowedCollisionMatrix matrix(current->scene.allowed_collision_matrix);
    // Preserve all existing mechanical-link exemptions from the SRDF.
    matrix.setEntry("demo_cube", "work_table", true);
    for (const auto & link : touch_links()) { matrix.setEntry("demo_cube", link, allow); }
    matrix.getMessage(diff.allowed_collision_matrix);
    require(scene_.applyPlanningScene(diff), "Cannot update allowed cube contact");
  }

  void attach_scene(bool attach)
  {
    moveit_msgs::msg::PlanningScene diff;
    diff.is_diff = true;
    diff.robot_state.is_diff = true;
    moveit_msgs::msg::AttachedCollisionObject object;
    object.link_name = "tcp";
    object.touch_links = touch_links();
    object.object = cube(cube_pose());
    object.object.operation = attach ? object.object.ADD : object.object.REMOVE;
    diff.robot_state.attached_collision_objects = {object};
    // MoveIt removes the world object automatically when attaching it.
    if (!attach) { diff.world.collision_objects = {cube(cube_pose())}; }
    require(scene_.applyPlanningScene(diff), "Cannot synchronize attached cube in MoveIt");
  }

  void move(const std::vector<double> & point, double height, const char * label)
  {
    RCLCPP_INFO(node_->get_logger(), "%s", label);
    geometry_msgs::msg::Pose pose;
    pose.position.x = point[0];
    pose.position.y = point[1];
    pose.position.z = point[2] + offset_ + height;
    pose.orientation.w = 0.0;
    pose.orientation.x = 1.0;  // Tool Z points down; fingers close along world X.
    arm_.setStartStateToCurrentState();
    arm_.setPoseTarget(pose);
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    require(arm_.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS,
            std::string(label) + ": planning failed");
    require(arm_.execute(plan) == moveit::core::MoveItErrorCode::SUCCESS,
            std::string(label) + ": execution failed");
  }

  void gripper(double position)
  {
    RCLCPP_INFO(node_->get_logger(), "Gripper target %.3f rad", position);
    Trajectory::Goal goal;
    goal.trajectory.joint_names = {"robotiq_85_left_knuckle_joint"};
    trajectory_msgs::msg::JointTrajectoryPoint point;
    point.positions = {position};
    point.velocities = {0.0};
    point.time_from_start.sec = 3;
    goal.trajectory.points = {point};
    auto sending = gripper_->async_send_goal(goal);
    const auto handle = wait(sending, 10s, "Gripper goal acceptance");
    require(static_cast<bool>(handle), "Gripper goal rejected");
    auto result = gripper_->async_get_result(handle);
    if (result.wait_for(30s) != std::future_status::ready) {
      gripper_->async_cancel_goal(handle);
      throw std::runtime_error("Gripper trajectory timed out");
    }
    const auto response = result.get();
    require(response.code == rclcpp_action::ResultCode::SUCCEEDED && response.result &&
            response.result->error_code == Trajectory::Result::SUCCESSFUL,
            "Gripper trajectory failed");
  }

  void set_grasp(bool attach)
  {
    auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
    request->data = attach;
    auto future = grasp_->async_send_request(request);
    const auto response = wait(future, 10s, "Physical grasp constraint");
    require(response->success, response->message);
  }

  rclcpp::Node::SharedPtr node_;
  moveit::planning_interface::MoveGroupInterface arm_;
  moveit::planning_interface::PlanningSceneInterface scene_;
  rclcpp_action::Client<Trajectory>::SharedPtr gripper_, arm_action_;
  rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr grasp_;
  rclcpp::Client<gazebo_msgs::srv::GetEntityState>::SharedPtr state_;
  rclcpp::Client<moveit_msgs::srv::GetPlanningScene>::SharedPtr planning_scene_;
  std::vector<double> a_, b_;
  double size_, approach_, offset_, open_, closed_, tolerance_;
};
}  // namespace crx_manipulation

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("fixed_pick_place",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  std::thread spinner([&executor] { executor.spin(); });
  int status = 0;
  try {
    crx_manipulation::FixedPickPlace(node).run();
  } catch (const std::exception & error) {
    RCLCPP_ERROR(node->get_logger(), "PICK_PLACE_FAILED: %s", error.what());
    status = 1;
  }
  executor.cancel();
  spinner.join();
  rclcpp::shutdown();
  return status;
}
