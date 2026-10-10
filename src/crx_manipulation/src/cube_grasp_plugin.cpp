// A gated physical constraint for this deterministic simulation demonstration.
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo_ros/node.hpp>
#include <std_srvs/srv/set_bool.hpp>

#include <algorithm>
#include <cmath>
#include <chrono>
#include <condition_variable>
#include <mutex>
#include <string>

namespace crx_manipulation
{
class CubeGraspPlugin : public gazebo::ModelPlugin
{
public:
  void Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf) override
  {
    model_ = model;
    cube_ = model->GetLink("cube_link");
    node_ = gazebo_ros::Node::Get(sdf);
    service_ = node_->create_service<std_srvs::srv::SetBool>(
      "set_grasp", [this](const std_srvs::srv::SetBool::Request::SharedPtr request,
                          std_srvs::srv::SetBool::Response::SharedPtr response) {
        std::unique_lock<std::mutex> lock(mutex_);
        requested_attach_ = request->data;
        pending_ = true;
        if (!finished_.wait_for(lock, std::chrono::seconds(3), [this] { return !pending_; })) {
          pending_ = false;  // A paused simulation must not execute a timed-out request later.
          response->success = false;
          response->message = "No physics update within 3 seconds; request cancelled.";
          return;
        }
        response->success = success_;
        response->message = message_;
      });
    update_ = gazebo::event::Events::ConnectWorldUpdateBegin([this](const gazebo::common::UpdateInfo &) { Update(); });
    RCLCPP_INFO(node_->get_logger(), "Grasp assistance ready at /demo_cube/set_grasp");
  }

private:
  static double box_gap(const ignition::math::AxisAlignedBox & a,
                        const ignition::math::AxisAlignedBox & b)
  {
    double squared = 0.0;
    for (int axis = 0; axis < 3; ++axis) {
      const double gap = std::max({0.0, a.Min()[axis] - b.Max()[axis],
                                  b.Min()[axis] - a.Max()[axis]});
      squared += gap * gap;
    }
    return std::sqrt(squared);
  }

  void Update()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!pending_) { return; }
    success_ = false;
    if (!requested_attach_) {
      if (joint_) {
        joint_->Detach();
        joint_->Fini();
        joint_.reset();
      }
      success_ = true;
      message_ = "Released; cube is governed by gravity and contact again.";
    } else if (joint_) {
      success_ = true;
      message_ = "Cube is already grasped.";
    } else {
      const auto robot = model_->GetWorld()->ModelByName("crx10ia_l");
      const auto parent = robot ? robot->GetLink("link_6") : nullptr;
      const auto left = robot ? robot->GetLink("robotiq_85_left_finger_tip_link") : nullptr;
      const auto right = robot ? robot->GetLink("robotiq_85_right_finger_tip_link") : nullptr;
      const auto knuckle = robot ? robot->GetJoint("robotiq_85_left_knuckle_joint") : nullptr;
      if (!cube_ || !parent || !left || !right || !knuckle) {
        message_ = "Robot, cube, or gripper links are not ready.";
      } else if (knuckle->Position(0) < 0.40 || knuckle->Position(0) > 0.52) {
        message_ = "Gripper must be at the 40 mm cube grasp opening.";
      } else if (box_gap(left->BoundingBox(), cube_->BoundingBox()) > 0.003 ||
                 box_gap(right->BoundingBox(), cube_->BoundingBox()) > 0.003) {
        message_ = "Cube must be within 3 mm of both fingertip collision bounds.";
      } else {
        // Load uses the current poses. No object pose is changed or teleported.
        joint_ = model_->GetWorld()->Physics()->CreateJoint("fixed", model_);
        joint_->SetName("cube_grasp_joint");
        joint_->Load(parent, cube_, ignition::math::Pose3d::Zero);
        joint_->Init();
        success_ = true;
        message_ = "Physical grasp constraint created.";
      }
    }
    RCLCPP_INFO(node_->get_logger(), "%s", message_.c_str());
    pending_ = false;
    finished_.notify_all();
  }

  gazebo::physics::ModelPtr model_;
  gazebo::physics::LinkPtr cube_;
  gazebo::physics::JointPtr joint_;
  gazebo::event::ConnectionPtr update_;
  gazebo_ros::Node::SharedPtr node_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr service_;
  std::mutex mutex_;
  std::condition_variable finished_;
  bool pending_{false}, requested_attach_{false}, success_{false};
  std::string message_;
};
GZ_REGISTER_MODEL_PLUGIN(CubeGraspPlugin)
}  // namespace crx_manipulation
