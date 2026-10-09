# FANUC CRX-10iA/L ROS 2 Workspace

ROS 2 (Humble) workspace for the FANUC CRX-10iA/L collaborative robot, supporting
RViz visualization, Gazebo simulation, MoveIt 2 motion planning, and real-hardware
control via ros2_control.

## Packages

| Package | Layer | Responsibility |
|---------|-------|----------------|
| `crx_description` | Description | URDF/Xacro geometry, meshes and RViz config |
| `crx_control` | Control | ros2_control hardware interfaces (real / mock / Gazebo) and controller config |
| `crx_moveit_config` | Planning | MoveIt 2 SRDF, kinematics, planners and controller mapping |
| `crx_bringup` | Orchestration | Unified launch files for all use-cases |

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

### 4. Real robot or mock hardware

```bash
# Mock hardware (no robot required)
ros2 launch crx_bringup robot_bringup.launch.py use_mock_hardware:=true

# Real robot (Ethernet/IP)
ros2 launch crx_bringup robot_bringup.launch.py \
    use_mock_hardware:=false robot_ip:=192.168.150.2
```
