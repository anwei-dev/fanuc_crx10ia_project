# CRX-10iA/L + Robotiq 2F-85 仿真

已使用 `gripper/ros2_robotiq_gripper-main.zip` 的 2F-85 模型。
只复制几何、惯性和关节数据，保留上游 BSD 许可证；没有引入串口、真实夹爪驱动或激活控制器。
适用当前 ROS 2 Humble + Gazebo Classic 环境。

## 修改文件

| 文件（相对 src） | 内容 |
| --- | --- |
| `crx_description/urdf/gripper/robotiq_2f_85.xacro`（新增） | 夹爪模型、一个主动关节和五个 mimic 关节、指尖接触参数 |
| `crx_description/meshes/robotiq/`（新增） | 2F-85 可视化/碰撞网格、来源说明与 LICENSE |
| `crx_description/urdf/crx10ia_l/crx10ia_l_macro.xacro` | 安装夹爪、增加 tool_mount，移动 tcp，保留相机安装位置 |
| `crx_description/package.xml` | 补充引入资源的 BSD-3-Clause 许可 |
| `crx_description/launch/view_robot.launch.py` | 显式将 robot_description 作为字符串传递 |
| `crx_control/urdf/robotiq_gazebo_control.xacro`（新增） | Gazebo 控制接口和五个联动关系 |
| `crx_control/urdf/crx_gazebo.xacro` | 将夹爪加入现有 GazeboSystem，并修正限位表达式缺少 `$` 的问题 |
| `crx_control/urdf/crx10ia_l_gazebo.xacro` | 将机械臂底座固定到 world，防止仿真底座自由运动 |
| `crx_control/config/ros2_controllers.yaml` | 增加 position_controllers/GripperActionController |
| `crx_control/package.xml` | 声明 gripper_controllers 依赖 |
| `crx_bringup/launch/gazebo_sim.launch.py` | 修正模型话题参数、显式字符串参数、自动加载三个控制器 |
| `crx_bringup/launch/robot_bringup.launch.py` | 仅修正共享模型参数的字符串类型；未扩展真实设备能力 |
| `crx_moveit_config/config/crx10ia_l.urdf.xacro` | 与 Gazebo 使用同一模型 |
| `crx_moveit_config/config/crx10ia_l.srdf` | 增加 gripper 规划组、末端执行器、open/closed 姿态和机械连接处的碰撞豁免 |
| `crx_moveit_config/config/moveit_controllers.yaml` | 绑定夹爪 GripperCommand action，清理原有重复键 |
| `crx_moveit_config/config/joint_limits.yaml` | 夹爪速度和加速度限制 |
| `crx_moveit_config/config/ompl_planning.yaml` | 夹爪规划组使用 RRTConnect |

现有 CMakeLists.txt 已安装整个 urdf、meshes、config 和 launch 目录，无需修改。
`moveit_demo.launch.py` 已组合 Gazebo 与 MoveIt，无需增加单独夹爪启动文件。

## 安装坐标与控制值

安装链为 `link_6 → flange → tool_mount → robotiq_85_base_link → tcp`。
`tool_mount` 保留原 TCP 的朝向；新 `tcp` 位于夹爪基座局部 Z 轴前方 0.145 m。
这是固定抓取参考点，手指实际高度会随开度变化，可按抓取任务调整 `gripper-tcp` 的 origin。
相机继续位于原安装位置（父坐标系改名为 tool_mount），不随新 TCP 前移。

主动关节为 `robotiq_85_left_knuckle_joint`，0 rad 表示张开，0.7929 rad 表示闭合。
控制命令的 position 是关节角度，**不是以米表示的指间距离**。
其余五个关节由 Gazebo mimic 参数联动，不需要单独控制器。
位置控制模式的 max_effort 不等同于物理夹持力调节；本次没有实现力控或物体附着插件。

## 构建与启动

```bash
cd /home/anl/git/bot/my_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select crx_description crx_control crx_moveit_config crx_bringup
source install/setup.bash
ros2 launch crx_bringup moveit_demo.launch.py
```

只看模型：`ros2 launch crx_bringup view_robot.launch.py`。
只启动仿真：`ros2 launch crx_bringup gazebo_sim.launch.py`。
无 Gazebo 界面：在后一条命令后加 `headless:=true`。

在另一个已 source 工作空间的终端检查控制器：

```bash
ros2 control list_controllers
```

应看到 joint_state_broadcaster、manipulator_controller、gripper_controller 均为 active。

闭合：

```bash
ros2 action send_goal /gripper_controller/gripper_cmd control_msgs/action/GripperCommand "{command: {position: 0.7929, max_effort: 50.0}}"
```

张开：

```bash
ros2 action send_goal /gripper_controller/gripper_cmd control_msgs/action/GripperCommand "{command: {position: 0.0, max_effort: 50.0}}"
```

在 RViz MotionPlanning 中选择 `gripper` 规划组及 `open` / `closed` 姿态，执行 Plan & Execute。
机械臂组仍是 `manipulator`，规划末端仍名为 `tcp`，但位置已前移。

## 验证范围

四个包已在 `/tmp/crx-gripper-check` 中独立构建通过；URDF 展开和 check_urdf 通过；
Gazebo 与 MoveIt 展开模型一致，网格路径与五个 mimic 的方向已检查。
启动参数的字符串类型已做离线求值检查。

已在隔离的 Gazebo 实际运行验证：三个控制器激活成功；夹爪闭合与张开均返回
reached_goal；五个联动关节跟随正确；开合期间机械臂六轴保持控制器激活后的姿态。
Gazebo 内部导出的联动关节状态名称带 `_mimic` 后缀，因此 joint_state_broadcaster
显式仅发布六个机械臂关节和夹爪主动关节的位置、速度。联动关节仍在 Gazebo 中运动；
robot_state_publisher 和 MoveIt 使用 URDF mimic 关系推导联动状态。
已联合启动 Gazebo 与 move_group 验证，关节状态只包含七个独立关节，
不存在 `Joint ... not found in model` 报错，夹爪开合和机械臂保持正常。
MoveIt 开合规划、自碰撞矩阵及带物体抓取尚未完成运行验收。

## 控制插件参数解析报错修复

若日志出现 `gazebo_ros2_control: parser error Couldn't parse parameter override rule`
且随后模型瘫软，是控制插件未能启动。本次定位到新增 TCP 的 XML 注释含有
冒号加空格，被插件内部的 ROS 参数解析器当作 YAML 内容。已改写该注释。
`ParameterValue(..., value_type=str)` 只处理 launch 参数层，不能解决插件内部再次解析的问题。

新增 `crx_control/test/test_gazebo_description.py`，并在 `crx_control/CMakeLists.txt`
与 `package.xml` 中注册测试及依赖。测试将展开后的整份 URDF 送入实际的 ROS 参数解析器，
避免只验证 XML 格式而漏掉这类错误。
