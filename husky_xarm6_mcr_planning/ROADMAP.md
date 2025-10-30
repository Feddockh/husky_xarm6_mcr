# Development Roadmap

Your path from current state to advanced tree scanning system.

## Where You Are Now ✅

- ✅ Husky xArm6 simulation working
- ✅ MoveIt configured for arm control
- ✅ Multi-camera rig publishing images and depth
- ✅ Point cloud generation from cameras
- ✅ **NEW: NBV planning package created**

## Phase 1: Basic NBV Planning (1-2 weeks)

### Goals
- [ ] Build and test NBV planner in simulation
- [ ] Verify point cloud integration
- [ ] Execute first autonomous scan of a simulated tree
- [ ] Visualize candidates and selected views in RViz

### Steps
```bash
# 1. Build the package
cd ~/ros2_ws
colcon build --packages-select husky_xarm6_mcr_planning
source install/setup.bash

# 2. Test individual components
ros2 run husky_xarm6_mcr_planning volumetric_map_node

# 3. Launch with simulation
ros2 launch husky_xarm6_mcr_planning nbv_planning.launch.py

# 4. Tune parameters for your tree
# Edit config/nbv_planner.yaml
```

### Key Milestones
- [ ] Volumetric map updates from point clouds
- [ ] Candidate views sampled around target
- [ ] Information gain computed correctly
- [ ] Feasible views selected
- [ ] Visualization working in RViz

### Expected Challenges
- **Point cloud topic mismatch**: Update topic name in config
- **MoveIt connection**: Ensure action server is running
- **Workspace limits**: Adjust target region to be reachable
- **Slow planning**: Reduce num_samples initially

## Phase 2: Integration & Execution (2-3 weeks)

### Goals
- [ ] Integrate NBV planner with your existing bringup
- [ ] Implement automatic motion execution
- [ ] Add exploration stopping criteria
- [ ] Collect complete tree scans

### Tasks

#### 2.1 Execution Integration
```python
# In nbv_planner.py, modify planning_callback():
def planning_callback(self):
    if self.get_exploration_progress() < 0.95:
        best_view = self.planner.plan_iteration()
        if best_view:
            result = self.moveit_interface.plan_to_pose(best_view)
            if result:
                self.get_logger().info('View executed')
            else:
                self.get_logger().warn('Planning failed, retry')
    else:
        self.get_logger().info('Exploration complete!')
```

#### 2.2 Complete Launch File
```python
# Create: husky_xarm6_mcr_bringup/launch/complete_scanning.launch.py
def generate_launch_description():
    return LaunchDescription([
        # Simulation
        IncludeLaunchDescription(...bringup...),
        # Cameras
        IncludeLaunchDescription(...cameras...),
        # NBV Planning
        IncludeLaunchDescription(...nbv_planning...),
    ])
```

#### 2.3 Data Collection
```bash
# Record scanning session
ros2 bag record -a -o tree_scan_$(date +%Y%m%d_%H%M%S)
```

### Key Milestones
- [ ] One-command launch of entire system
- [ ] Automatic scanning without manual intervention
- [ ] Complete tree coverage achieved
- [ ] Point cloud data recorded for analysis

### Success Metrics
- Exploration progress reaches >90%
- Tree branches visible from multiple angles
- No collision during execution
- Total scan time < 10 minutes

## Phase 3: Optimization & Refinement (2-4 weeks)

### Goals
- [ ] Tune for optimal performance
- [ ] Add travel cost minimization
- [ ] Implement base motion (if needed)
- [ ] Compare different sampling strategies

### Experiments to Run

#### 3.1 Parameter Optimization
Test different configurations:
- num_samples: [20, 50, 100, 200]
- resolution: [0.03, 0.05, 0.10]
- camera_distance: [1.0, 1.5, 2.0]

Measure:
- Planning time
- Execution time
- Coverage completeness
- Number of views needed

#### 3.2 Multi-Objective Scoring
```python
def evaluate_view(self, pose):
    info_gain = self.map.compute_information_gain(pose)
    travel_dist = self.compute_travel_distance(pose)
    smoothness = self.compute_motion_smoothness(pose)
    
    score = (
        1.0 * info_gain +
        -0.3 * travel_dist +
        0.2 * smoothness
    )
    return score
```

Test different weight combinations.

#### 3.3 Base Motion Extension
If arm workspace is limiting:
```python
# Sample base positions
base_samples = [
    (1.0, 0.0), (1.5, 0.5), (1.5, -0.5),  # Front positions
    (2.0, 0.0), (2.0, 0.5), (2.0, -0.5),  # Farther positions
]

for base_pos in base_samples:
    # Move base
    # Sample arm views
    # Evaluate combined
```

### Key Milestones
- [ ] Optimized parameters documented
- [ ] Travel cost reduces redundant motion
- [ ] Base motion working (if implemented)
- [ ] Comparison study complete

## Phase 4: Learning-Based Methods (1-3 months)

### Goals
- [ ] Collect training dataset
- [ ] Train imitation learning model
- [ ] Train RL agent
- [ ] Compare learned vs heuristic

### 4.1 Data Collection
```bash
# Run 100 scanning episodes
for i in {1..100}; do
    ros2 launch husky_xarm6_mcr_planning nbv_planning.launch.py \
        collect_data:=true \
        output:=data/episode_$i
    # Randomize tree location, size
done
```

Collect:
- States (volumetric map, robot pose)
- Actions (selected views)
- Rewards (information gained)
- Outcomes (coverage achieved)

### 4.2 Imitation Learning
```python
# Train model to mimic your best planner
python train_imitation.py \
    --data_dir data/ \
    --model_arch mlp \
    --epochs 100
```

### 4.3 Reinforcement Learning
```python
# Train RL agent
python train_rl.py \
    --algorithm ppo \
    --env tree_scanning \
    --episodes 1000
```

### 4.4 Evaluation
Compare methods:
- Sampling-based NBV (baseline)
- Imitation learning
- Reinforcement learning

Metrics:
- Average coverage
- Average time
- Success rate
- Robustness to tree variation

### Key Milestones
- [ ] Dataset of 100+ scans collected
- [ ] Imitation model trained and evaluated
- [ ] RL agent trained and evaluated
- [ ] Results paper-worthy

### Expected Timeline
- Data collection: 1 week
- Model training: 2-4 weeks
- Evaluation: 1 week
- Analysis: 1-2 weeks

## Phase 5: Novel Methods & Research (3-6 months)

### Potential Directions

#### 5.1 Semantic-Guided NBV
- Train branch segmentation model
- Use segmentation to predict information gain
- Focus on tree-specific features

#### 5.2 Multi-Tree Coordination
- Plan paths between multiple trees
- Optimize orchard scanning
- Multi-robot coordination

#### 5.3 Active Reconstruction
- Real-time surface reconstruction
- Guide views to complete surfaces
- Quality metrics for branch models

#### 5.4 Adaptive Sampling
- Learn optimal sampling distribution
- Dense samples near branches
- Sparse samples in open space

### Research Questions
- Can learning reduce scan time by 50%?
- What's the minimum number of views needed?
- How does tree complexity affect performance?
- Can we transfer to real-world trees?

### Publication Targets
- ICRA, IROS, RSS (robotics venues)
- ICCV, CVPR (with vision component)
- Field Robotics venues (real-world deployment)

## Timeline Summary

```
Now           Phase 1       Phase 2       Phase 3         Phase 4           Phase 5
  |------------|------------|-------------|---------------|------------------|
  0            2 weeks      5 weeks       9 weeks         4 months          10 months

Current       Basic NBV     Integrated    Optimized       Learning          Research
              working       system        performance     methods           contributions
```

## Critical Path

**Must do first:**
1. ✅ Package created
2. Build and test in simulation
3. Integrate with existing system
4. Collect first complete scan

**After basics work:**
5. Optimize parameters
6. Add travel cost
7. Consider base motion

**If time/interest:**
8. Collect training data
9. Train learning models
10. Novel research directions

## Resources Needed

### Computing
- Simulation: Your current workstation
- Training: GPU recommended for RL/DL (can use lab resources)
- Data storage: ~1GB per 100 scans

### Time Investment
- Phase 1-2: ~1 hour/day for 4 weeks
- Phase 3: ~2 hours/day for 3 weeks
- Phase 4-5: Depends on research goals

### Skills to Develop
- ROS2 Python programming (you have this)
- MoveIt motion planning (learning this)
- Point cloud processing (PCL, Open3D)
- Deep learning (if pursuing Phase 4+)

## Next Immediate Actions

### This Week
1. [ ] Build the package
2. [ ] Fix any build errors
3. [ ] Test volumetric_map_node standalone
4. [ ] Test nbv_planner_node standalone

### Next Week
5. [ ] Launch full system together
6. [ ] Visualize in RViz
7. [ ] First autonomous view selection
8. [ ] Record point cloud data

### Following Week
9. [ ] Tune parameters for your tree
10. [ ] Complete first full scan
11. [ ] Analyze results
12. [ ] Document findings

## Questions to Answer Along the Way

- What's the optimal number of samples for your tree size?
- How does voxel resolution affect quality vs speed?
- Is arm reach sufficient or do you need base motion?
- What's the minimum scan time achievable?
- How consistent are results across different trees?

## Success Criteria

### Phase 1 Success
- NBV planner runs without errors
- Candidate views generated and visualized
- Information gain computed from point clouds

### Phase 2 Success
- Complete autonomous scan achieved
- Tree fully covered from multiple views
- No manual intervention required

### Phase 3 Success
- Parameter set documented and justified
- Improvement over naive scanning shown
- System ready for real-world consideration

### Phase 4 Success
- Learning method outperforms heuristic
- Results publishable
- Code and data available for reproduction

## Getting Unstuck

If you hit roadblocks:
1. Check the troubleshooting in QUICK_START.md
2. Review examples in README.md
3. Test components in isolation
4. Ask in lab meetings
5. Reach out to Kantor Lab for help

Good luck! You have a solid foundation to build on.
