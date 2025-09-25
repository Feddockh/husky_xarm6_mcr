husky_xarm6_mcr_control
=======================

This package provides a minimal ros2_control configuration and launch files for the Husky + XArm6 robot used in the `husky_xarm6_mcr` workspace.

Files added:
- `config/arm_controllers.yaml` : controller definitions for the manipulator
- `launch/ros2_control.launch.py` : starts `ros2_control_node`, `robot_state_publisher`, and spawners
- `launch/bringup.launch.py` : convenience include for the control launch

Usage:

Run bringup (after building workspace):

```bash
source install/setup.bash
ros2 launch husky_xarm6_mcr_control bringup.launch.py
```
