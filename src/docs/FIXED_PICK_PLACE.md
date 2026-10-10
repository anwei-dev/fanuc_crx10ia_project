# 固定立方体抓取搬运

使用 CRX-10iA/L + Robotiq 2F-85，将 40 mm、50 g 的红色立方体从桌面 A 点搬运到 B 点。
程序不订阅图像或点云，使用固定世界坐标；原有相机仍随机器人启动。

| 项目 | 数值 |
| --- | --- |
| 桌面高度 | 0.400 m |
| A 点（立方体中心） | (0.65, -0.20, 0.42) m |
| B 点（立方体中心） | (0.65, 0.20, 0.42) m |
| 接近与抬升高度 | 0.18 m |
| 夹爪抓取角 | 0.45 rad，匹配约 40 mm 开口 |
| TCP 高于立方体中心 | 5 mm |
| 放置中心误差容限 | 15 mm |

A 点有绿色桌面标记，B 点有蓝色标记。夹爪以竖直向下姿态抓取。

## 构建与运行

在工作空间根目录执行：

```bash
source /opt/ros/humble/setup.bash
bash src/scripts/dev.sh build
bash src/scripts/dev.sh pick
```

独立搬运入口包含场景和自动搬运，直接启动命令：

```bash
source install/setup.bash
ros2 launch crx_bringup cube_pick_place.launch.py
```

关闭 Gazebo GUI 和 RViz：

```bash
bash src/scripts/dev.sh pick headless:=true rviz:=false
```

只启动场景用于观察，不执行搬运：

```bash
bash src/scripts/dev.sh pick run_demo:=false
```

此入口自行启动 Gazebo、MoveIt 和控制器，使用它前先关闭同一 ROS 域中的旧仿真。
完成后仿真窗口继续保留，便于查看放置结果。再次执行完整演示时关闭并重新启动场景，
让立方体回到 A 点；程序会拒绝在立方体不位于 A 点时直接重跑。

依赖新增 `gazebo_dev`、`moveit_ros_planning_interface` 等，完整依赖见 package.xml。
若环境尚未安装，按项目 README 使用 rosdep 安装。

## 执行流程

1. 等待轨迹 Action、夹持服务及关节状态就绪，确认立方体在 A 点。
2. 将桌面和立方体加入 MoveIt PlanningScene，张开夹爪。
3. 运动至 A 点上方，允许立方体与两侧指尖接触，下降到抓取姿态。
4. 用三秒轨迹闭合到抓取开度，申请仿真物理附着。
5. 同步 MoveIt 的附着物体，抬升并确认 Gazebo 中立方体实际升起。
6. 搬运至 B 点上方，下降到放置姿态。
7. 解除物理附着并同步规划场景，张开夹爪，退回 B 点上方。
8. 恢复指尖与立方体的碰撞检查，检查 Gazebo 实际中心是否落在 B 点容差内。

机械臂由 MoveIt 规划，通过 `/manipulator_controller/follow_joint_trajectory` 执行；
夹爪通过 `/gripper_controller/follow_joint_trajectory` 执行。
碰撞矩阵修改保留 SRDF 中已有的机械连接豁免，桌面与机器人之间始终进行碰撞检查。
任务失败时输出 `PICK_PLACE_FAILED` 并返回非零状态；成功输出 `PICK_PLACE_SUCCESS` 和实际放置误差。

## 抓取辅助的范围

`libcrx_cube_grasp.so` 加载在示例立方体上，通过 `/demo_cube/set_grasp`
（`std_srvs/srv/SetBool`）建立或解除 Gazebo 物理固定关节。
建立约束前检查机器人与指尖连杆存在、夹爪角度在 0.40～0.52 rad 内，且立方体与
左右指尖碰撞包围盒的间距均不超过 3 mm。约束建立保留当时相对位姿，不移动或瞬移物体。

这是用于固定抓取流程的**附着辅助仿真**，位置门限基于碰撞包围盒，
不代表检测了真实双侧接触力，也不验证摩擦力是否足够。
释放后移除约束，立方体重新由重力与桌面接触支撑。
MoveIt 的附着状态仅用于规划，Gazebo 物理关节才负责物体随夹爪运动。

物理修改在 Gazebo 更新线程执行。仿真暂停导致三秒内无物理更新时，服务取消该请求并返回失败。

## 文件与参数

- `crx_manipulation/src/fixed_pick_place.cpp`：固定抓取程序与结果验证。
- `crx_manipulation/src/cube_grasp_plugin.cpp`：受限物理附着服务。
- `crx_manipulation/config/fixed_pick_place.yaml`：A/B、开度、运动速度、接近高度与容差。
- `crx_manipulation/worlds/pick_place.world`：桌面、A/B 标记、立方体和状态查询插件。
- `crx_bringup/launch/cube_pick_place.launch.py`：场景与程序的组合启动。

改变 A 点、立方体尺寸或桌面时，同时更新 YAML 和 world；更改 B 点时同步更新桌面标记。
当前模型、角度门限和场景针对 40 mm 立方体设计，不作为任意物体的通用抓取策略。

## 回归验证

常规检查包含场景与配置一致性测试：

```bash
bash src/scripts/dev.sh check
```

完整 Gazebo 测试默认跳过，可在构建并加载 overlay 后单独运行：

```bash
source install/setup.bash
CRX_RUN_GAZEBO_TEST=1 PYTHONDONTWRITEBYTECODE=1 python3 -m pytest src/crx_bringup/test/test_pick_place_integration.py -s
```

测试使用独立 ROS 域 93、Gazebo 端口 11493，验证远距离/张开状态拒绝附着、实际搬运到 B，
以及松开退回后不能重新附着。结束时关闭测试启动的进程。
测试日志保存于 pytest 临时目录。无 GUI 模式仍需要 RGB-D 传感器的图形渲染环境。

## 已完成的实测

在 ROS 2 Humble / Gazebo Classic 11 的隔离仿真中，集成测试已完成远距离张开拒绝附着、
抓取、抬升、搬运、放置、退回与释放后再次拒绝附着的检查。
一次实测最终中心为 `(0.649414, 0.202552, 0.419999)` m，相对 B 点误差约 2.6 mm。
这是一次固定场景运行结果，不代表多场景抓取成功率或纯摩擦抓取性能。
