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

## 实现过程与问题复盘

这次任务从已经能运行的机械臂、夹爪和 MoveIt 仿真出发，目标是先完成固定坐标的
“接近、夹取、搬运、放置、退回”流程，不引入视觉识别和真实硬件通信。
调试的关键是同时检查三个层面：MoveIt 是否认为运动合法，控制器是否完成轨迹，
Gazebo 中物体是否真的随夹爪移动。三者中的任何一个成功，都不能单独代表搬运成功。

下面将实际调试中出现的失败与为完成流程增加的配套设计分别说明。
日志摘录来自本次开发留下的运行记录，文件名和接口名称按当前版本描述。

### 1. 先把固定抓取场景和坐标关系确定下来

**需要解决的问题：** Gazebo 的场景、程序中的目标坐标以及 MoveIt 的碰撞场景分别维护。
如果只在 Gazebo 放置立方体，MoveIt 不会自动知道桌面和立方体的存在；如果把物体中心
直接作为 TCP 位置，也未必能让两侧指尖落在合适的夹持高度。

**处理过程：**

1. 在 `pick_place.world` 中创建桌面和动态立方体。立方体边长 0.04 m，桌面高度
   0.40 m，因此静置时立方体中心高度为 `0.40 + 0.04 / 2 = 0.42 m`。
2. 在 `fixed_pick_place.yaml` 中定义 A/B 坐标、接近高度、TCP 偏移和夹爪开度，
   程序统一使用 `world` 坐标系，以 `tcp` 作为末端目标连杆。
3. 抓取位置在物体中心上方增加 0.005 m 的 TCP 偏移，用于匹配现有 TCP 与指尖夹持区的
   几何关系；接近和抬升目标再增加 0.18 m。
4. 程序启动后查询 Gazebo 中立方体的实际位姿，并将它作为碰撞物加入 PlanningScene；
   同时添加覆盖桌面及支撑区域的保守实体碰撞盒。

**验证方式：** `test_scene_geometry.py` 检查 world 中的初始位置、碰撞尺寸和视觉尺寸
与程序配置一致，并检查 A/B 两点都在桌面范围内、立方体底面高度与桌面一致。
运行时还要求物体确实位于 A 点，防止沿用上一次搬运后的场景继续执行。

### 2. 第一阶段失败：到 A 点上方的运动规划失败

早期运行在 `Move above A` 阶段停止，客户端记录：

```text
MoveGroupInterface::plan() failed or timeout reached
PICK_PLACE_FAILED: Move above A: planning failed
```

**定位与修复：** 检查末端目标的坐标系、TCP 和姿态初始化。向下抓取采用绕 X 轴
旋转 180° 的四元数，最终显式设置 `(x, y, z, w) = (1, 0, 0, 0)`。
不能只修改 `x` 而沿用已有的 `w = 1`；这样得到的四元数不是单位四元数，
也不是所需的向下姿态。当前 `move()` 每次创建新的目标 Pose，并明确设置 `w = 0`
和 `x = 1`，使工具 Z 轴向下、夹爪沿世界 X 方向闭合。

同时核对规划器名称：本项目 `ompl_planning.yaml` 中的配置键为 `RRTConnect`，
程序使用 `setPlannerId("RRTConnect")`，不套用其他项目中的
`RRTConnectkConfigDefault` 名称。规划时间设为 10 秒，尝试次数设为 10，
速度和加速度缩放均为 0.15。

上述日志本身只说明规划失败，不能单凭这一行认定是 IK、姿态或碰撞中的哪一种原因。
姿态、配置名称和后续碰撞检查需要分别核对；增加规划时间也不能修复非法目标。
修正后运行能够通过 `Move above A` 并进入下降阶段。

### 3. 第二阶段失败：允许抓取接触后，下降规划反而失败

下一次运行已经完成 A 点上方的运动，但在 `Descend to grasp` 阶段失败。
服务端出现：

```text
Skipping invalid start state (invalid state)
There are no valid initial states!
PICK_PLACE_FAILED: Descend to grasp: planning failed
```

**原因：** 抓取必须允许指尖接触物体，因此程序需要修改允许碰撞矩阵
（Allowed Collision Matrix，ACM）。早期更新方式没有保留完整的已有矩阵，
使 SRDF 中对机械连接连杆的碰撞豁免丢失。这样即使机器人没有发生新的物理碰撞，
MoveIt 也可能把当前状态判为自碰撞，导致规划器找不到合法起始状态。

**解决方法：** 将矩阵更新改成“读取当前矩阵，再修改指定碰撞对”：

1. 通过 `/get_planning_scene` 请求当前 `ALLOWED_COLLISION_MATRIX`。
2. 基于返回值构建 `collision_detection::AllowedCollisionMatrix`，保留原有条目。
3. 只修改立方体与左右指尖的碰撞许可，并允许立方体与桌面接触。
4. 将结果作为 PlanningScene 差量同步提交，不重建整套机器人碰撞规则。
5. 放置退回后重新禁止立方体与指尖接触；立方体与桌面的接触许可继续保留。

桌面与机械臂之间的碰撞检查始终保留。不能为了让规划成功而全局关闭碰撞检测。
修复后下降动作可以完成，流程推进到闭合夹爪和物体附着。

### 4. 第三阶段失败：物理夹持成功，但 MoveIt 附着同步失败

一次集成测试完成了接近、下降及 `Gripper target 0.450 rad`，随后报错：

```text
PICK_PLACE_FAILED: Cannot synchronize attached cube in MoveIt
Tried to remove world object 'demo_cube', but it does not exist in this scene.
```

**原因：** MoveIt 在处理附着物体的 ADD 操作时，会将同名物体从世界碰撞对象中移除。
早期更新又显式提交了一次世界对象 REMOVE，造成重复移除，场景同步返回失败。
此时 Gazebo 的物理附着已经建立，但规划场景没有成功完成状态切换，程序不能继续抬升。

**解决方法：** 在 `attach_scene()` 中明确两个方向的更新语义：

| 阶段 | MoveIt 更新 | Gazebo 更新 |
| --- | --- | --- |
| 抓取后 | 添加附着物体，指定 `tcp` 和指尖 `touch_links`；不再重复删除世界对象 | 建立机械臂末端与立方体的固定关节 |
| 放置时 | 移除附着物体，并按 Gazebo 实际位姿重新添加世界碰撞物 | 解除固定关节，让物体恢复重力和接触作用 |

场景使用同步的 `applyPlanningScene()` 提交，并检查返回值。
确认附着同步成功后才规划抬升，避免异步发布后立即规划导致的状态竞争。
后续完整运行通过该阶段，并完成抬升、搬运、放置和退回。

### 5. 夹爪闭合与物体随动：明确两套附着机制的职责

**需要解决的问题：** 夹爪关节到达目标角度，并不说明物体已经被稳定抓住；
MoveIt 的附着物体只用于碰撞检查，也不会自动在 Gazebo 中产生抓取力。

本次采用受限的物理附着辅助，让固定场景的搬运流程可以验证。当前实现没有证明
纯摩擦夹持失败，也没有完成纯摩擦抓取的稳定性评估，因此不把这项设计描述为
已经解决了所有接触动力学问题。

**具体实现：**

- 夹爪通过 `FollowJointTrajectory` 执行三秒闭合轨迹，目标关节为
  `robotiq_85_left_knuckle_joint`，抓取角度为 0.45 rad。等待 Action 成功返回后，
  才请求物理附着；被拒绝、超时或执行失败都会中止流程。
- `CubeGraspPlugin` 检查机器人、物体、指尖和关节是否存在，要求夹爪关节角度位于
  0.40～0.52 rad，并要求物体与两侧指尖包围盒的距离分别不超过 3 mm。
- 条件满足后，在 `link_6` 与 `cube_link` 之间创建 Gazebo 固定关节。
  建立关节使用当时位姿，不通过修改物体坐标将物体吸到末端。
- MoveIt 侧另行添加附着碰撞物，使搬运规划考虑物体的体积。

两侧包围盒距离只是几何门限，不是接触力反馈，也不能证明真实双侧接触。
集成测试验证了远处且张开的夹爪请求会被拒绝；它没有分别覆盖每一个门限的边界条件。

### 6. 启动、线程和时序问题的处理

这些是实现时加入的配套措施，不都对应一次已记录的运行故障。

**启动就绪：** launch 启动节点后，控制器、服务和关节状态未必已经可用。
程序分别等待机械臂与夹爪 Action、抓取服务、Gazebo 位姿服务、PlanningScene 服务，
并确认能获取当前机器人状态。每一步运动都从当前状态开始规划。
底层仿真入口在机器人生成成功后才加载控制器，生成或控制器激活失败时关闭启动流程。
这避免了只用固定延时推测系统已经就绪。

**回调处理：** 主线程执行搬运并等待 Action/服务结果；另一个线程运行 ROS executor，
持续处理状态、反馈和响应。否则等待结果时若没有线程处理回调，程序可能一直等不到完成。

**物理线程：** 插件的 ROS 服务回调只提交附着/释放请求，Gazebo 世界更新回调负责
创建或销毁物理关节，避免直接从服务线程修改物理世界。请求通过互斥锁和条件变量同步。
若三秒内没有物理更新，服务返回失败并取消待处理请求，防止暂停仿真时请求超时，
恢复后却突然执行旧的夹持命令。暂停取消机制目前由代码实现，现有集成测试没有单独测试它。

**插件与图形环境：** 搬运入口在启动 Gazebo 前将插件库目录加入 `GAZEBO_PLUGIN_PATH`，
同时保留调用者已有路径。`headless:=true` 仅关闭 Gazebo GUI，机器人 RGB-D 相机仍运行，
因此集成测试仍要求可用的图形渲染环境。

### 7. 放置完成不能只检查机械臂到达 B 点

**需要解决的问题：** 如果只检查机械臂轨迹返回成功，即使物体没有被带起、途中掉落
或释放后位置错误，也可能被误判为搬运完成。

**当前验证链路：**

1. 开始时查询 Gazebo 物体实际位置，确认与 A 点距离小于 0.015 m。
2. 抬升后再次查询，要求立方体中心高度超过 `A.z + 0.6 × approach_height`，
   当前阈值为 0.528 m。物体未实际升起则立即失败。
3. 在 B 点降低到放置高度，先解除物理约束并同步 MoveIt 场景，再张开夹爪、退回。
4. 等待一秒后查询 Gazebo 实际位置，以三维欧氏距离检查与 B 点的误差是否小于 0.015 m。
5. 集成测试还在释放退回后再次申请附着，确认此时请求被拒绝。

这些检查验证了初始、抬升和最终状态，没有连续监测整段搬运轨迹，也没有测量夹持力。
所有关键步骤检查返回值；异常统一输出 `PICK_PLACE_FAILED` 并以非零状态结束。
当前失败处理会停止任务，但没有自动执行恢复动作或保证清理已经建立的附着约束；
调试失败后应重启搬运场景，再执行下一次测试。

### 8. 通过隔离测试确认修复，并拆分启动入口

验证时使用独立 ROS 域 93 和 Gazebo 端口 11493，避免已有仿真节点影响控制器发现和服务调用。
测试先以 `run_demo:=false` 启动搬运场景，再单独运行抓取程序，分别保留仿真和程序日志，
便于区分规划服务端报错、控制器执行失败和插件拒绝附着。

最终一次完整实测通过：

```text
Move above A
Descend to grasp
Gripper target 0.450 rad
Lift cube
Carry cube above B
Lower cube to B
Gripper target 0.000 rad
Retreat from B
PICK_PLACE_SUCCESS cube=(0.6494, 0.2026, 0.4200), B error=0.0026 m
```

这说明该次固定场景运行中，规划、轨迹执行、物理附着、场景同步和最终位置检查共同完成。
它不代表经过重复统计的成功率，也不说明不同尺寸、不同质量或随机位置的物体均能抓取。
以上是抓取功能开发阶段的实测记录；后续 launch 拆分和改名完成了构建与启动配置检查，
没有在每次文件改名后重新执行完整 Gazebo 搬运。

启动入口最终按功能分开：`gazebo_moveit.launch.py` 保留通用仿真与规划，
`cube_pick_place.launch.py` 负责加载搬运场景、配置插件路径和运行搬运节点。
后者包含前者，不重复维护机器人或 MoveIt 配置。静态启动测试检查通用入口不会启动搬运程序，
并检查搬运入口的默认场景、自动执行开关和插件路径保留逻辑。

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
