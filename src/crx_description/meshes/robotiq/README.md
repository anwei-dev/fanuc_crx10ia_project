# Robotiq 2F-85 assets

Source: `gripper/ros2_robotiq_gripper-main.zip`, archive commit
`a74d007d8f2f06dc6a503ad21038ba869d4999a6`, PickNik Robotics,
https://github.com/PickNikRobotics/ros2_robotiq_gripper .

The 2F-85 visual/collision meshes are copied unchanged. The accompanying
`urdf/gripper/robotiq_2f_85.xacro` is adapted from the archive's
`robotiq_description/urdf/robotiq_2f_85_macro.urdf.xacro`: hardware options
and driver includes are removed, mesh URLs are local, and contact settings
use Gazebo Classic extensions. See LICENSE for the upstream BSD-3-Clause license.
