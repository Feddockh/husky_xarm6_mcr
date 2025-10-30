# Motion Planning Strategies for Tree Scanning

Overview of different approaches you can implement with this framework.

## 1. Sampling-Based NBV (Implemented)

**Status**: ✅ Implemented in `nbv_planner.py`

### How It Works
1. Sample viewpoints uniformly around target (Fibonacci sphere)
2. Compute volumetric information gain for each
3. Filter infeasible views (out of reach, collisions)
4. Select view with maximum gain

### Pros
- Simple and interpretable
- No training required
- Proven effective for exploration

### Cons
- May not find optimal long-term strategy
- Greedy: doesn't plan ahead
- Doesn't learn from experience

### Best For
- Quick prototyping
- Baseline comparisons
- Small to medium exploration tasks

### Parameters to Tune
```yaml
num_samples: 50        # More = better coverage, slower
camera_distance: 1.5   # Adjust for tree size
```

## 2. Frontier-Based Exploration

**Status**: 🔨 Template provided, needs implementation

### How It Works
1. Identify frontiers (boundaries between known and unknown)
2. Sample views near frontiers
3. Select frontier with highest information potential
4. Consider travel cost to frontier

### Implementation
```python
class FrontierNBVPlanner(NextBestViewPlanner):
    def sample_candidate_views(self):
        # Detect frontiers in volumetric map
        frontiers = self.map.detect_frontiers()
        
        # Sample views near frontiers
        candidates = []
        for frontier in frontiers:
            views = self.sample_views_near(frontier)
            candidates.extend(views)
        
        return candidates
```

### Pros
- Efficient exploration
- Natural for tree scanning (branches create frontiers)
- Reduces redundant observations

### Cons
- Requires frontier detection
- May miss small details

### Best For
- Large-scale exploration
- Complex branching structures
- When you want to minimize total distance

## 3. Coverage-Based Planning

**Status**: 📝 Design concept

### How It Works
1. Model tree surface as collection of patches
2. Track coverage of each patch
3. Select views that cover most uncovered patches
4. Use set cover approximation

### Implementation Strategy
```python
class CoverageNBVPlanner(NextBestViewPlanner):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.surface_patches = self.discretize_target_surface()
        self.coverage_map = {}
    
    def compute_coverage_gain(self, pose):
        # Which patches would this view cover?
        visible_patches = self.get_visible_patches(pose)
        uncovered = [p for p in visible_patches 
                     if not self.coverage_map.get(p, False)]
        return len(uncovered)
```

### Pros
- Guarantees full coverage
- Good for inspection tasks
- Quantifiable completeness

### Cons
- Requires surface model or estimation
- May take more views than information-based

### Best For
- Quality inspection
- When you need 100% coverage guarantee
- Smooth surfaces (less occlusion)

## 4. Learning-Based NBV

**Status**: 🔨 Template in `learning_nbv_template.py`

### Approach A: Imitation Learning

Learn from optimal planner demonstrations.

```python
# Collect expert demonstrations
expert = OptimalNBVPlanner()  # Or your best heuristic
learner = ImitationLearningNBV(expert_planner=expert)

for episode in range(num_episodes):
    demo = learner.collect_demonstration()

# Train neural network to mimic expert
learner.train_from_demonstrations(learner.demonstrations)
```

**Pros**: Fast training, stable, good for complex features
**Cons**: Limited by expert performance

### Approach B: Reinforcement Learning

Learn optimal policy through exploration.

```python
planner = ReinforcementLearningNBV()

for episode in range(num_episodes):
    state = get_initial_state()
    while not done:
        action = planner.select_action(state, epsilon=0.1)
        next_state, reward = execute_action(action)
        planner.collect_training_data(state, action, reward, next_state)
        
        if len(planner.replay_buffer) > batch_size:
            planner.update_policy()
```

**Pros**: Can exceed human/heuristic performance
**Cons**: Requires many training episodes, less stable

### Approach C: Learning Information Gain Model

Learn to predict information gain without ray tracing.

```python
class LearnedGainPredictor:
    def predict_gain(self, pose, map_state):
        # Neural network predicts gain directly
        # Much faster than volumetric computation
        features = self.extract_features(pose, map_state)
        predicted_gain = self.model(features)
        return predicted_gain
```

**Pros**: Very fast evaluation, scalable
**Cons**: Requires training data from real evaluations

### Training Data Collection

```bash
# Collect training data while using standard NBV
ros2 launch husky_xarm6_mcr_planning nbv_planning.launch.py collect_data:=true

# Generates dataset: poses, map states, information gains
# Use for supervised learning
```

### Best For
- When you'll do many similar scanning tasks
- Want to optimize for specific tree types
- Can afford offline training time
- Want faster online planning

## 5. Multi-Objective Optimization

**Status**: 📝 Design concept

### How It Works
Optimize multiple objectives simultaneously:
- Information gain (maximize)
- Travel distance (minimize)
- Motion smoothness (maximize)
- Energy consumption (minimize)
- Safety margin (maximize)

### Implementation
```python
class MultiObjectiveNBV(NextBestViewPlanner):
    def evaluate_candidates(self, candidates):
        results = []
        for pose in candidates:
            # Compute multiple objectives
            info_gain = self.map.compute_information_gain(pose)
            travel_cost = self.compute_travel_distance(pose)
            smoothness = self.compute_smoothness(pose)
            safety = self.compute_safety_margin(pose)
            
            # Weighted sum or Pareto optimality
            score = (
                1.0 * info_gain +
                -0.5 * travel_cost +
                0.3 * smoothness +
                0.2 * safety
            )
            results.append((pose, score))
        return results
```

### Pros
- Balances multiple concerns
- More practical for real deployment
- Can optimize for specific constraints

### Cons
- Tuning weights is challenging
- May not maximize any single objective

### Best For
- Real-world deployment
- When you have hard constraints (time, energy)
- Multi-robot coordination

## 6. Path Planning with NBV

**Status**: 📝 Advanced concept

### How It Works
Instead of planning next single view, plan sequence of views.

```python
class PathPlanningNBV:
    def plan_view_sequence(self, horizon=5):
        # Plan next N views together
        # Use RRT*, A*, or trajectory optimization
        
        best_path = None
        best_score = -inf
        
        for _ in range(num_samples):
            path = self.sample_view_path(length=horizon)
            score = self.evaluate_path(path)
            if score > best_score:
                best_score = score
                best_path = path
        
        return best_path
```

### Pros
- Better long-term planning
- Can avoid getting stuck in local optima
- More efficient exploration

### Cons
- Much more computationally expensive
- Harder to implement
- May be overkill for simple tasks

### Best For
- Large exploration tasks
- When online planning time is available
- Multi-robot scenarios

## 7. Mobile Base + Arm Planning

**Status**: 📝 Extension of current system

### How It Works
Coordinate base motion with arm motion for larger workspace.

```python
class MobileNBVPlanner(NextBestViewPlanner):
    def sample_candidate_views(self):
        candidates = []
        
        # Sample base positions
        for base_x in np.linspace(-2, 2, 10):
            for base_y in np.linspace(-2, 2, 10):
                base_pose = (base_x, base_y, 0)
                
                # For each base, sample arm views
                arm_views = super().sample_candidate_views()
                
                for arm_pose in arm_views:
                    # Combined base+arm pose
                    combined = (base_pose, arm_pose)
                    candidates.append(combined)
        
        return candidates
    
    def execute_view(self, base_pose, arm_pose):
        # Move base first
        self.move_base(base_pose)
        # Then move arm
        self.moveit.plan_to_pose(arm_pose)
```

### Pros
- Much larger workspace
- Can scan large trees completely
- More flexibility

### Cons
- More complex planning
- Longer execution time
- Need to ensure base stability

### Best For
- Large trees or orchards
- When arm reach is limiting
- Multi-tree scanning

## 8. Semantic-Guided NBV

**Status**: 🔬 Research direction

### How It Works
Use semantic segmentation to guide exploration.

```python
class SemanticNBV(NextBestViewPlanner):
    def __init__(self, *args, segmentation_model, **kwargs):
        super().__init__(*args, **kwargs)
        self.segmentation = segmentation_model
    
    def compute_information_gain(self, pose):
        # Predict what this view would see
        predicted_view = self.render_view(pose)
        segmentation = self.segmentation(predicted_view)
        
        # Higher gain for views that see branches
        branch_pixels = np.sum(segmentation == BRANCH_CLASS)
        
        # Also consider novelty
        geometric_gain = super().compute_information_gain(pose)
        
        return branch_pixels * geometric_gain
```

### Pros
- Task-specific optimization
- Can focus on interesting features (branches)
- Reduces scanning time

### Cons
- Requires trained segmentation model
- More complex implementation
- Depends on model quality

### Best For
- When you know what you're looking for
- Have semantic annotations
- Want task-specific efficiency

## Recommendation for Your Tree Scanning

### Phase 1: Get Started (Now)
✅ Use **Sampling-Based NBV** (already implemented)
- Test with your simulation
- Tune parameters for your trees
- Establish baseline performance

### Phase 2: Optimize (Near-term)
🎯 Add **Multi-Objective** considerations:
- Balance information gain with travel distance
- Consider motion smoothness for stability
- Add stopping criteria (95% coverage)

Optional: Implement **Mobile Base** motion if arm reach insufficient

### Phase 3: Learn (Medium-term)
🧠 Try **Learning-Based** approaches:
- Collect data from Phase 1 runs
- Train imitation learning model
- Compare against baseline

### Phase 4: Specialize (Long-term)
🌳 **Semantic-Guided** for tree-specific optimization:
- Train branch detection model
- Focus on branch structures
- Optimize for orchard applications

## Getting Help

For implementing any of these approaches:
1. Start from the appropriate template
2. Refer to the main README.md
3. Check academic papers (references in README)
4. Contact Kantor Lab for specific questions

## Code Examples Repository

Consider creating a separate examples package:
```
husky_xarm6_mcr_planning_examples/
├── frontier_nbv_example.py
├── coverage_planning_example.py
├── rl_training_example.py
└── mobile_base_example.py
```

This keeps experimental methods separate from core framework.
