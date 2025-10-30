# Quick Start Guide: Tree Scanning with NBV Planning

This guide walks you through using the NBV planner for autonomous tree scanning.

## Prerequisites

- Husky xArm6 MCR simulation running
- Camera system publishing point clouds
- MoveIt configured and running

## Step 1: Build the Package

```bash
# Install Open3D if not already installed
pip3 install open3d

cd ~/ros2_ws
colcon build --packages-select husky_xarm6_mcr_planning
source install/setup.bash
```

## Step 2: Configure for Your Tree

Edit `config/nbv_planner.yaml`:

```yaml
nbv_planner:
  ros__parameters:
    # Set tree location
    target_x: 2.0    # meters from robot base
    target_y: 0.0
    target_z: 1.5    # height of tree center
    target_radius: 1.0  # approximate tree size
    
    # More samples = better coverage, slower planning
    num_samples: 50
```

## Step 3: Launch Your System

### Terminal 1: Start simulation and MoveIt
```bash
ros2 launch husky_xarm6_mcr_bringup <your_bringup_launch>
```

### Terminal 2: Start camera system
```bash
ros2 launch firefly-ros2-wrapper-bringup <your_camera_launch>
```

### Terminal 3: Start NBV planner
```bash
ros2 launch husky_xarm6_mcr_planning nbv_planning.launch.py
```

## Step 4: Visualize in RViz

1. Open RViz if not already open
2. Add these displays:
   - **MarkerArray** on topic `/nbv_candidates` (candidate views - green arrows)
   - **MarkerArray** on topic `/nbv_best` (best view - red arrow)
   - **PointCloud2** on topic `/merged_pointcloud` (observed points)

## Step 5: Monitor Progress

Watch the terminal output:
```
[nbv_planner]: Best view selected with gain: 1234.00
[nbv_planner]: Exploration progress: 45.2%
```

The system will automatically:
1. Sample candidate viewpoints around the tree
2. Evaluate information gain for each
3. Select the best view
4. Plan and execute motion (when integrated with execution)

## Understanding the Outputs

### Volumetric Map
- **Unknown**: Not yet observed (white/gray in visualization)
- **Free**: Observed empty space (transparent)
- **Occupied**: Tree branches and obstacles (colored)

### Information Gain
- Higher gain = more new information from that view
- Gain decreases as tree becomes fully scanned
- Stop when exploration progress > 95%

## Tuning Parameters

### If planning is too slow:
- Reduce `num_samples` (e.g., 30)
- Increase `resolution` (e.g., 0.1 = 10cm voxels)
- Reduce `max_depth` in octree (e.g., 6 instead of 8)

### If coverage is incomplete:
- Increase `num_samples` (e.g., 100)
- Decrease `camera_distance` (get closer to tree)
- Check `camera_max_range` matches your sensors

### If views are infeasible:
- Adjust `target_x/y/z` to be within reach
- Check robot workspace limits
- Verify MoveIt is properly configured

## Integration with Execution

Currently, the planner selects views but doesn't execute automatically. To add execution:

### Option A: Manual execution
```python
# In your code
best_view = planner.plan_iteration()
if best_view:
    result = moveit_interface.plan_to_pose(best_view)
```

### Option B: Automatic execution loop
Modify `NBVPlannerNode.planning_callback()`:
```python
def planning_callback(self):
    best_view = self.planner.plan_iteration()
    if best_view:
        # Execute motion
        result = self.moveit_interface.plan_to_pose(best_view)
        if result:
            self.get_logger().info('Motion executed successfully')
        else:
            self.get_logger().warn('Motion planning failed')
```

### Option C: Service interface
Create a service to trigger planning on demand:
```bash
ros2 service call /plan_next_view std_srvs/srv/Trigger
```

## Adding Base Motion

To scan a larger tree, you can move the base:

1. Sample base positions around tree
2. For each base position, sample arm views
3. Evaluate combined base+arm motion cost
4. Execute base motion, then arm motion

See `README.md` section "Adding Base Motion" for implementation details.

## Extracting the 3D Tree Model

After scanning, the accumulated point cloud is automatically saved. You can also export it manually:

```bash
# Point clouds are auto-saved to /tmp/volumetric_map_*.ply every 100 updates

# Or record the full point cloud topic during scanning
ros2 bag record /merged_pointcloud

# Load and process with Open3D
python3 << EOF
import open3d as o3d
pcd = o3d.io.read_point_cloud('/tmp/volumetric_map_100.ply')
o3d.visualization.draw_geometries([pcd])
EOF

# Or use point cloud processing tools:
# - Open3D for filtering, reconstruction, meshing
# - CloudCompare for visualization and analysis
# - PCL for advanced processing
```

## Troubleshooting

### "No point cloud received"
- Check camera system is running: `ros2 topic echo /merged_pointcloud`
- Verify topic name matches in config

### "No feasible views found"
- Tree location may be out of reach
- Check with: `ros2 topic echo /tf` (look for base->tree transform)
- Adjust `target_x/y/z` to be within ~0.7m of robot

### "MoveGroup action server not found"
- Ensure MoveIt is running
- Check: `ros2 action list | grep move_action`
- Source your workspace: `source ~/ros2_ws/install/setup.bash`

### Planning takes too long
- Reduce `num_samples` in config
- Increase voxel `resolution`
- Use `use_octomap: true` for efficiency (requires octomap-python)

## Next Steps

1. **Test in simulation**: Verify basic functionality
2. **Tune parameters**: Adjust for your specific tree size and camera specs
3. **Add execution**: Integrate motion execution
4. **Implement stopping criteria**: Stop when tree is fully scanned
5. **Add learning**: Try learning-based planners from templates

## Example Complete Workflow

```bash
# 1. Start everything
tmux new-session -d -s tree_scan
tmux send-keys -t tree_scan "ros2 launch husky_xarm6_mcr_bringup sim.launch.py" C-m
tmux split-window -t tree_scan
tmux send-keys -t tree_scan "sleep 10 && ros2 launch firefly-ros2-wrapper-bringup cameras.launch.py" C-m
tmux split-window -t tree_scan
tmux send-keys -t tree_scan "sleep 15 && ros2 launch husky_xarm6_mcr_planning nbv_planning.launch.py" C-m
tmux attach -t tree_scan

# 2. Monitor in another terminal
ros2 topic hz /nbv_candidates
ros2 topic echo /nbv_planner/progress

# 3. When done (95% explored)
ros2 bag record -a -o tree_scan_data

# 4. Process results
# Use your favorite point cloud processing pipeline
```

## Questions?

See `README.md` for detailed documentation or contact the Kantor Lab.
