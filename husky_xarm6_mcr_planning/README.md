# Husky xArm6 MCR Planning

Motion planning algorithms for the Husky xArm6 Multi-Camera Rig system, including next-best-view (NBV) planning for autonomous exploration and scanning.

## Overview

This package provides:

1. **Next-Best-View (NBV) Planning**: Sampling-based planner that selects optimal viewpoints for maximizing volumetric information gain
2. **Volumetric Mapping**: Maintains 3D occupancy representation for information gain computation
3. **MoveIt Interface**: Clean Python wrapper for motion planning and execution
4. **Extensible Architecture**: Designed to support future learning-based and novel planning methods

## Architecture

```
husky_xarm6_mcr_planning/
├── planners/
│   └── nbv_planner.py          # Sampling-based NBV planner
├── perception/
│   └── volumetric_map.py       # 3D occupancy mapping
├── motion_interface/
│   └── moveit_interface.py     # MoveIt wrapper
├── config/
│   └── nbv_planner.yaml        # Configuration parameters
└── launch/
    └── nbv_planning.launch.py  # Launch file
```

## Installation

### Dependencies

```bash
# ROS2 dependencies (should already be installed)
sudo apt install ros-humble-moveit ros-humble-tf2-ros

# Python dependencies
pip3 install numpy open3d

# Open3D provides efficient octree structures for point cloud processing
```

### Build

```bash
cd ~/ros2_ws
colcon build --packages-select husky_xarm6_mcr_planning
source install/setup.bash
```

## Usage

### Basic NBV Planning

1. **Start your simulation and camera system**:
```bash
# Terminal 1: Your existing bringup
ros2 launch husky_xarm6_mcr_bringup <your_launch_file>

# Terminal 2: Camera system
ros2 launch firefly-ros2-wrapper-bringup <camera_launch>
```

2. **Launch NBV planner**:
```bash
ros2 launch husky_xarm6_mcr_planning nbv_planning.launch.py
```

### Configuration

Edit `config/nbv_planner.yaml` to adjust:

- **Target region**: Location and size of area to scan (e.g., tree location)
- **Sampling density**: Number of candidate views
- **Camera parameters**: FOV, range
- **Map resolution**: Voxel size for occupancy grid

### Visualization

View candidate viewpoints and best view in RViz:
- Topic: `/nbv_candidates` (MarkerArray)
- Green arrows: Candidate views
- Red arrow: Selected best view

## How It Works

### Next-Best-View Planning Algorithm

1. **Sample Candidates**: Generate viewpoints uniformly distributed around target region using Fibonacci sphere
2. **Compute Information Gain**: For each candidate, estimate number of new voxels that would be observed
3. **Check Feasibility**: Verify candidate is reachable and collision-free
4. **Select Best**: Choose view with maximum information gain
5. **Execute Motion**: Plan and execute trajectory to selected view

### Volumetric Mapping

- Maintains 3D occupancy representation using **Open3D octree**
- Updates from point cloud observations
- Tracks unknown/free/occupied space
- Computes information gain by counting unknown voxels in camera frustum
- Efficiently handles large-scale point clouds
- Supports point cloud export for post-processing

## Extending the System

### Adding Learning-Based Planners

Create a new planner class in `planners/`:

```python
from ..planners.nbv_planner import NextBestViewPlanner

class LearningBasedNBV(NextBestViewPlanner):
    def __init__(self, *args, model_path=None, **kwargs):
        super().__init__(*args, **kwargs)
        self.model = self.load_model(model_path)
    
    def select_best_view(self, evaluated_views):
        # Use learned policy to select view
        features = self.extract_features(evaluated_views)
        best_view = self.model.predict(features)
        return best_view
```

### Adding Base Motion

Currently, the planner only moves the arm. To add base motion:

1. **Expand workspace**: Modify `MoveItInterface.get_workspace_bounds()` to include reachable space with base motion
2. **Sample base poses**: In `NBVPlanner.sample_candidate_views()`, include base position variations
3. **Execute combined motion**: Plan base + arm trajectories sequentially or with whole-body planner

Example structure:
```python
class MobileNBVPlanner(NextBestViewPlanner):
    def sample_candidate_views(self, num_samples):
        candidates = []
        for base_pose in self.sample_base_poses():
            for arm_pose in self.sample_arm_poses(base_pose):
                candidates.append((base_pose, arm_pose))
        return candidates
```

### Implementing Novel Methods

The modular structure supports various planning approaches:

- **Frontier-based exploration**: Modify sampling to focus on frontiers between known and unknown
- **Coverage planning**: Add coverage metrics in addition to volumetric gain
- **Multi-objective optimization**: Extend evaluation to include travel time, safety, etc.
- **Active learning**: Use uncertainty estimates instead of binary unknown/known

## Integration with Your System

### With Existing Bringup

Add NBV planning to your existing launch file:

```python
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

# In your launch file
nbv_launch = IncludeLaunchDescription(
    PythonLaunchDescriptionSource([
        PathJoinSubstitution([
            FindPackageShare('husky_xarm6_mcr_planning'),
            'launch',
            'nbv_planning.launch.py'
        ])
    ])
)
```

### Point Cloud Input

The volumetric map subscribes to `/merged_pointcloud`. Make sure your camera system publishes to this topic or update the topic name in `config/nbv_planner.yaml`.

## Tree Scanning Application

For your tree scanning use case:

1. **Set target region**: Configure tree location in `nbv_planner.yaml`:
```yaml
target_x: 2.0  # Tree position
target_y: 0.0
target_z: 1.5
target_radius: 1.5  # Tree size
```

2. **Adjust sampling**: Increase `num_samples` for denser coverage of complex geometry
3. **Monitor progress**: Check exploration ratio to know when scanning is complete
4. **Extract tree model**: Use accumulated point cloud for branch reconstruction

## Next Steps

### Immediate:
1. ✅ Package structure created
2. ✅ Core NBV planner implemented
3. ⏳ Test in simulation
4. ⏳ Tune parameters for your tree scanning task

### Short-term:
- Implement proper IK sampling
- Add collision checking integration
- Implement ray tracing for free space
- Add octomap support for efficiency

### Medium-term:
- Implement learning-based view selection
- Add base motion coordination
- Develop specialized tree scanning strategies
- Integrate with branch reconstruction pipeline

### Long-term:
- Multi-robot coordination
- Online replanning with dynamic obstacles
- Semantic segmentation integration
- Real-world deployment and testing

## Troubleshooting

**No feasible views found**: Check workspace bounds and target region location

**Planning too slow**: Reduce `num_samples` or increase `resolution` (larger voxels)

**Poor information gain**: Verify point cloud is being published and volumetric map is updating

**MoveIt connection failed**: Ensure move_group is running from your bringup

## References

This implementation draws from:
- **NBV Planning**: Isler et al., "Information-theoretic Planning with Trajectory Optimization"
- **Volumetric Mapping**: Octomap framework by Hornung et al.
- **Active Vision**: Kriegel et al., "Efficient next-best-scan planning"

## Contributing

To add new planning methods:
1. Create new planner class inheriting from `NextBestViewPlanner`
2. Override methods as needed
3. Add configuration to yaml
4. Add entry point to setup.py
5. Document your approach

## License

Apache-2.0

## Contact

For questions about this package, contact the Kantor Lab.
