# Manipulation Workspace Data

This directory stores learned manipulation workspace files for the NBV planner.

## Workflow

### 1. Learn Manipulation Workspace

Learn the reachable workspace by sampling random joint configurations:

```bash
# Basic learning (10,000 samples, no visualization)
ros2 launch husky_xarm6_mcr_nbv_planner nbv_volumetric_planner_demo.launch.py \
    learn_workspace:=true

# Custom number of samples
ros2 launch husky_xarm6_mcr_nbv_planner nbv_volumetric_planner_demo.launch.py \
    learn_workspace:=true \
    num_samples:=5000

# With visualization (slower but shows progress in RViz)
ros2 launch husky_xarm6_mcr_nbv_planner nbv_volumetric_planner_demo.launch.py \
    learn_workspace:=true \
    visualize_learning:=true \
    visualization_topic:=workspace_visualization
```

This will:
- Sample N random valid joint configurations (default: 10,000)
- Check collision validity for each
- Compute forward kinematics to get end-effector positions
- Discretize positions into voxels (default: 2cm resolution)
- Optionally visualize each valid position as a green sphere in RViz
- Save to `data/workspace_YYYYMMDD_HHMMSS.bin`

**Note:** 
- Learning takes several minutes (depends on num_samples)
- Visualization slows down learning but provides visual feedback
- Subscribe to the visualization topic in RViz to see progress

### 2. Load Saved Workspace

Load a previously learned workspace (much faster):

```bash
ros2 launch husky_xarm6_mcr_nbv_planner nbv_volumetric_planner_demo.launch.py \
    learn_workspace:=false
```

By default, this automatically loads the most recent timestamped workspace file.

## File Format

- Binary format (`.bin`)
- Contains:
  - Voxel size (double, 8 bytes)
  - Number of voxels (size_t, 8 bytes)
  - Voxel keys (uint64_t each, 8 bytes)

## Tips

- Learn workspace once for your robot configuration
- Workspace is specific to:
  - Robot kinematics
  - Joint limits
  - Collision geometry
  - Voxel resolution
- Re-learn if you change robot configuration or environment
- Files are stored in `install/share/husky_xarm6_mcr_nbv_planner/data/`
- With `--symlink-install`, files are actually in the source `data/` directory
