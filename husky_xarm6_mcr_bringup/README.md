# Husky xArm6 MCR Bringup

This package provides comprehensive launch files to bring up the complete Husky xArm6 MCR system in simulation.

## Architecture

The bringup launch orchestrates the following components in sequence:

1. **Gazebo Environment** - Starts Ignition Gazebo with the selected world
2. **Robot Spawning & Control** - Spawns the robot URDF and starts ros2_control controllers
3. **MoveIt** - Brings up the move_group node for motion planning
4. **RViz** (optional) - Launches RViz for visualization

## Usage

### Basic Launch (Apple Orchard World)

```bash
ros2 launch husky_xarm6_mcr_bringup bringup.launch.py
```

This will:
- Start Gazebo with the apple orchard world
- Spawn the Husky xArm6 robot
- Start platform and manipulator controllers
- Launch MoveIt move_group
- Open RViz for visualization

### Launch with Empty World

```bash
ros2 launch husky_xarm6_mcr_bringup bringup.launch.py world:=empty
```

### Launch without RViz

```bash
ros2 launch husky_xarm6_mcr_bringup bringup.launch.py use_rviz:=false
```

### Custom World File

```bash
ros2 launch husky_xarm6_mcr_bringup bringup.launch.py world:=/path/to/custom.sdf
```

## Launch Arguments

| Argument | Default | Description |
|----------|---------|-------------|
| `world` | `apple_orchard` | World to load: "apple_orchard", "empty", or path to .sdf file |
| `use_rviz` | `true` | Whether to launch RViz |
| `manipulator_prefix` | `xarm6_` | Prefix for manipulator joint names |
| `platform_prefix` | `a200_` | Prefix for platform joint names/frames |
| `use_sim_time` | `true` | Use simulation time |

## Component Packages

This bringup package depends on and launches:

- **husky_xarm6_mcr_gz** - Gazebo worlds and models
- **husky_xarm6_mcr_control** - Robot spawning, ros2_control, and ros_gz_bridge
- **husky_xarm6_mcr_moveit_config** - MoveIt configuration and planning
- **husky_xarm6_mcr_description** - Robot URDF/SRDF descriptions

## Topics and Services

After launch, the system exposes:

### Control Topics
- `/platform_velocity_controller/cmd_vel_unstamped` - Platform velocity commands
- `/xarm6_traj_controller/joint_trajectory` - Manipulator trajectory commands

### State Topics
- `/joint_states` - Combined joint states from all controllers
- `/tf`, `/tf_static` - Transform tree

### MoveIt Services
- `/move_group/plan_kinematics_path` - Motion planning service
- Various other MoveIt services for trajectory execution, scene updates, etc.

## Troubleshooting

### Gazebo doesn't start
Ensure you've sourced the workspace after building:
```bash
source install/setup.bash
```

### Controllers fail to spawn
Check that Gazebo is fully initialized before controllers spawn. The launch uses event handlers to sequence spawning.

### MoveIt errors
Verify that the robot_description matches between ros2_control and MoveIt. Both should use the same URDF with `sim:=true`.

## Development

To modify individual components:
- **Gazebo worlds**: Edit files in `husky_xarm6_mcr_gz`
- **Controller configuration**: Edit `husky_xarm6_mcr_control/config/controllers.yaml`
- **MoveIt planning**: Edit configs in `husky_xarm6_mcr_moveit_config/config/`
- **Robot model**: Edit URDF in `husky_xarm6_mcr_description/urdf/`
