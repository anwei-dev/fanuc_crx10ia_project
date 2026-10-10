# CRX 视觉抓取仿真项目

当前基线为 ROS 2 Humble + Gazebo Classic + MoveIt 2，使用 FANUC CRX-10iA/L、
Robotiq 2F-85 和末端 RGB-D 相机。当前已实现模型、控制器、运动规划接入和相机数据发布。
已提供固定坐标的立方体抓取搬运示例。视觉识别、视觉驱动的自主抓取、分拣与失败恢复属于后续开发内容。

本目录是工作空间中的 `src/`，以下命令从工作空间根目录执行。
此文档描述仿真项目的维护入口，项目仅包含仿真与可视化功能。

## 环境与构建

使用 Ubuntu 22.04 / ROS 2 Humble。当前保持 Gazebo Classic 仿真基线，
不要混用其他 ROS 发行版或 Gazebo Sim 的插件。依赖以各包的 `package.xml` 为准。

```bash
source /opt/ros/humble/setup.bash
# rosdep 已初始化时执行；此命令会通过系统包管理器安装依赖。
rosdep install --from-paths src/crx_description src/crx_control src/crx_moveit_config src/crx_bringup src/crx_manipulation --ignore-src -r -y --rosdistro humble
bash src/scripts/dev.sh build
```

脚本使用现有工作空间的 `build/`、`install/`、`log/`，可从任意目录调用。
也可以选择独立输出目录，避免影响日常安装：

```bash
CRX_OUTPUT_DIR=/tmp/crx-check bash src/scripts/dev.sh check
```

## 启动

```bash
# Gazebo + MoveIt + RViz，默认空场景，不自动执行搬运
bash src/scripts/dev.sh demo

# 无 Gazebo GUI、无 RViz，适合批量实验
bash src/scripts/dev.sh demo headless:=true rviz:=false

# 只启动 Gazebo 与控制器
bash src/scripts/dev.sh sim

# 固定立方体 A 到 B 抓取搬运（自带桌面场景）
bash src/scripts/dev.sh pick

# 使用自定义 Gazebo Classic 场景
bash src/scripts/dev.sh demo world:=/absolute/path/to/table.world
```

无 GUI 模式仍会渲染 RGB-D 相机，运行环境需要具备对应的图形渲染支持。

对应的 ROS 启动命令：

```bash
source install/setup.bash
ros2 launch crx_bringup moveit_demo.launch.py
ros2 launch crx_bringup moveit_demo.launch.py --show-args
```

`headless` 只控制 Gazebo GUI，`rviz` 单独控制 RViz。
`moveit_demo.launch.py` 启动通用仿真与规划环境；
`fixed_pick_place.launch.py` 加载立方体场景并自动搬运，使用 `run_demo:=false` 可仅查看搬运场景和手动规划。
组合仿真入口统一使用仿真时钟，`use_sim_time` 仅接受 `true`。
Gazebo 单独入口的位姿参数用于独立仿真实验；组合 MoveIt 时应保持零位姿，
否则模型生成位置与 MoveIt 的世界坐标可能不一致。
自定义 world 只改变 Gazebo 场景，其中的障碍物不会自动进入 MoveIt PlanningScene。

## 工程结构

| 目录 | 职责 |
| --- | --- |
| `crx_description` | 几何、惯性、相机、夹爪和视觉材质 |
| `crx_control` | Gazebo 控制接口、控制器参数及模型回归测试 |
| `crx_moveit_config` | 规划组、碰撞矩阵、运动学、规划器与执行映射 |
| `crx_bringup` | 启动编排、启动参数和失败退出策略 |
| `crx_manipulation` | 固定抓取程序、示例场景和受限物理附着插件 |
| `scripts` | 本地构建、测试、运行入口 |
| `docs` | 模块边界、开发约定与已有验证记录 |

硬件结构在 description 定义，控制参数在 control 定义，规划参数在 moveit_config
定义；bringup 只做组合，不复制机器人配置。模型生成成功后才加载控制器，
生成或控制器加载失败时退出本次启动，避免留下部分启动的系统。
Gazebo 材质与模型搜索路径会保留调用者原有设置。

## 检查

```bash
bash src/scripts/dev.sh check  # 构建并执行所有已注册测试
bash src/scripts/dev.sh test   # 已构建后的测试
```

检查覆盖机器人描述参数解析、独立关节发布、底座离地间隙、相机几何，以及启动参数、
资源路径、启动失败分支以及固定搬运场景与程序配置一致性。它们不需要启动 ROS 节点或 Gazebo，也不能替代实际仿真验收。
修改物理参数、接触模型、规划或控制行为后，需要补充针对性的运行验证。

当前没有启用 MoveIt 三维占据图更新器；旧 Kinect 示例配置已清除。
相机话题存在并不代表已经实现视觉避障。物体识别、PlanningScene 更新与自过滤将在
感知阶段统一实现。

## 文档

- [固定立方体抓取搬运](docs/FIXED_PICK_PLACE.md)
- [开发约定和后续模块边界](docs/DEVELOPMENT.md)
- [夹爪控制与仿真验证](GRIPPER_SIMULATION.md)
- [相机安装与坐标说明](docs/CAMERA_INSTALLATION.md)
- [高频抖动分析](docs/GAZEBO_JITTER_ANALYSIS.md)

第三方模型来源及许可见对应资源目录中的 README / LICENSE；不要移除这些文件。
