# 开发约定与扩展边界

## 当前维护范围

维护 `crx_bringup/moveit_demo.launch.py` 和 `gazebo_sim.launch.py` 两个仿真入口。
当前以单机械臂、固定世界基座为范围，不引入真实设备适配、多机器人命名空间或插件框架。
保留的纯软件 mock 模型和 MoveIt Setup Assistant 生成入口不视为本项目的验证入口。
`gazebo_sim` 中原先未生效的 `name` 参数已移除；
`model` 表示 Gazebo 实体名称，并不切换机械臂型号。

依赖方向为 bringup → moveit_config → control → description；应用包依赖所需的下层包，
不要让 description 或 control 反过来依赖感知与业务任务。

## 后续模块

下面是规划边界，尚未创建这些 ROS 包。每个包在出现第一项可运行功能时再创建。

| 拟新增包 | 负责 | 边界 |
| --- | --- | --- |
| `crx_perception` | RGB-D 同步、分割、三维定位 | 不直接发送运动命令 |
| `crx_manipulation` | 抓取候选、场景更新、MTC 规划与执行 | 不承担业务分拣决策 |
| `crx_tasks` | 行为树、任务状态、重试与取消 | 调用操作能力，不重复规划实现 |
| `crx_simulation` | 场景资源、物体生成、实验重置 | 仿真真值与在线感知输入分离 |
| `crx_evaluation` | 批量实验、指标、结果报告 | 允许使用真值评估，不向策略泄漏真值 |

自定义消息确实需要跨包共享时再增加 `crx_interfaces`；在此之前优先复用标准消息。
不提前创建空节点、空 Action 或无法执行的测试占位。

## 数据约定

- 几何量使用米、弧度、秒。机械臂全局参考为 `world`，规划末端为 `tcp`。
- 图像与点云携带采样时间和原始 frame；使用观测时刻的 TF，不把最新 TF 当作历史 TF。
- 相机光学坐标系为 `camera_optical_frame`，Z 向前、X 向右、Y 向下。
- 仿真节点使用 `/clock`。超时需要明确按仿真时间还是墙上时间计算，暂停仿真不能误报算法失败。
- 夹爪控制量是主动关节角，范围约 `0..0.7929 rad`，不是米；后续如使用开口宽度，须做显式几何映射。
- 感知结果应包括对象 ID、类别、置信度、带时间戳的位姿和尺寸；目标关联策略由 perception 管理。
- 长时间任务使用 ROS Action，支持反馈、取消、超时；不在服务回调中阻塞等待完整任务。
- 配置放入所属包的 YAML 或 launch 参数，避免把个人路径、设备地址和实验数字写入业务代码。

## 现有运行接口

| 类型 | 名称 |
| --- | --- |
| 关节状态 | `/joint_states` |
| 机械臂轨迹 Action | `/manipulator_controller/follow_joint_trajectory` |
| 夹爪轨迹 Action | `/gripper_controller/follow_joint_trajectory` |
| RGB / 内参 | `/camera/image_raw`、`/camera/camera_info` |
| 深度 / 内参 | `/camera/depth/image_raw`、`/camera/depth/camera_info` |
| 点云 | `/camera/points` |
| MoveIt 规划组 | `manipulator`、`gripper` |

物体进入 MoveIt PlanningScene 和物体在 Gazebo 中被实际夹持是不同机制，须分别实现和验证。
目前仅确认轨迹控制与夹爪联动，不宣称已实现物体抓取。

## 开发与验证流程

1. 修改对应职责的包，不把业务逻辑放进 launch 文件。
2. 使用 `bash src/scripts/dev.sh check` 构建并运行回归测试。
3. 按改动选择实际仿真验证，记录命令、环境和观察结果。
4. 提交前检查 `git diff --check` 和 `git status --short`，只提交任务相关源文件。

构建脚本固定选择当前四个包。新增应用包时同步更新 `scripts/dev.sh` 的包列表。
全新 checkout 的安装不能依赖空目录；新目录包含实际资源后，再加入 CMake 安装规则。
依赖变更同步更新 package.xml，避免只在个人环境中手动安装后漏记。

`dev.sh check` 可作为未来 CI 的统一检查命令。当前没有配置远端 CI 或容器镜像；
优先维护本地可复现的构建与测试，后续接入 CI 时复用此入口。

## 实验记录

后续每次运行至少记录 Git 提交号、未提交改动状态、场景种子、配置、物体类别、
首次抓取结果、最终任务结果、重试次数、规划耗时与失败原因。
报告同时记录仿真时间、墙上时间和运行机器，避免把实时因子不同的实验直接比较。

大文件放在 `src/artifacts/` 或 `src/datasets/`，默认不进 Git。
训练集与测试集按场景或 episode 划分；仿真真值只用于标签和评测。
小型、经过筛选的测试夹具单独放在所属包的 `test/fixtures/` 中管理。

## 当前限制

- 相机数据未接入占据图，Gazebo 场景不会自动同步至 MoveIt。
- 尚无自动抓取、抓取成功判定、物体附着机制或失败恢复。
- 单独启动 Gazebo 可以改变机器人生成位姿；组合 MoveIt 的受支持基线是默认零位姿。
- `headless:=true rviz:=false` 关闭窗口，但 RGB-D 渲染仍依赖图形环境。
- 启动契约测试不启动节点，不覆盖运行时传感器、物理接触或控制器状态。
