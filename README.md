# FANUC CRX-10iA/L ROS 2 Workspace

ROS 2 (Humble) workspace for the FANUC CRX-10iA/L collaborative robot, supporting
RViz visualization, Gazebo simulation, and MoveIt 2 motion planning with
ros2_control simulation controllers. See [the project guide](src/README.md)
for development commands and supported simulation entry points.

## Packages

| Package | Layer | Responsibility |
|---------|-------|----------------|
| `crx_description` | Description | URDF/Xacro geometry, meshes and RViz config |
| `crx_control` | Control | Gazebo ros2_control integration and controller config |
| `crx_moveit_config` | Planning | MoveIt 2 SRDF, kinematics, planners and controller mapping |
| `crx_bringup` | Orchestration | Simulation and visualization launch orchestration |

Dependency flow (single direction):

```
crx_description  ->  crx_control  ->  crx_moveit_config  ->  crx_bringup
```

## Build

```bash
cd <workspace>
colcon build --symlink-install
source install/setup.bash
```

## Usage

### 1. Visualize the robot (RViz + joint_state_publisher_gui)

```bash
ros2 launch crx_bringup view_robot.launch.py
```

### 2. Gazebo simulation

```bash
ros2 launch crx_bringup gazebo_sim.launch.py headless:=false
```

### 3. MoveIt 2 planning with Gazebo

```bash
ros2 launch crx_bringup moveit_demo.launch.py
```
